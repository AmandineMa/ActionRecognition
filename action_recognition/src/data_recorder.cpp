/**
 * \file data_recorder.cpp
 * \brief This ROS node aims to record data in a file.

 * \author Amandine M.
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <fstream>
#include <string>
#include <stdio.h>
#include <sys/types.h>
#include <fcntl.h>

#include "action_recognition/common.hpp"
#include "toaster_msgs/HumanListStamped.h"
#include "toaster_msgs/FactList.h"
#include "action_recognition/FeatureMatrix.hpp"

namespace bf = boost::filesystem;

/* -------- Structure ---------- */

/**
 * \brief Structure for tf frames, containing the target, the source
 * and the type of the SensorFeatureVector
 */
struct Frame{
  std::string target_frame;
  std::string source_frame;
  SensorFeatureVectorType type;

  Frame(){}
  Frame(std::string targ_frame, std::string src_frame, SensorFeatureVectorType t):
    target_frame(targ_frame), source_frame(src_frame), type(t){}
};

/* -------- Global variables ---------- */

/** \brief ROS node shared between the different functions of the file */
ros::NodeHandle* node_;
/** \brief Vector of all the frames to get at each time t during the recording*/
std::vector<std::vector<Frame> > transforms_array_;
/** \brief Vector containing the number of frames for each human limb 
 * For example, v={1, 2} means there are 1 frame for the limb0 (hand)
 * and 2 frames for the limb1 (head) <br>
 * This can be determined by hand in the code or thanks to the ROS params
 */
std::vector<int> tf_nb_per_limb_;
/** \brief Vector containing the visible state of each object. <br>
 * An object can be retrieved via its index, which can be retrieved via the #objects_map
 * with the name of the object
 */
std::vector<bool> object_visible_;
/** \brief Map of {object name, object index}<br>
 * Aim to access to object information contained in vectors
 */
std::map<std::string, int> objects_map_;
/** \brief Hand pose updated to each #human_list_callback */
geometry_msgs::Pose hand_pose_;
/** \brief Head pose updated to each #human_list_callback */
geometry_msgs::Pose head_pose_;

/* -------- Function declarations -------- */
/**
 * \brief Initialization of all the Frames objets with the ROS params 
 * 
 * The transforms_array vector is filled
 */
void initialize_tf2_frames(void);
/** \brief Callback to get messages from the humanList topic */
void human_list_callback(const toaster_msgs::HumanListStamped::ConstPtr& msg);
/** \brief Callback to get messages from the factList topic */
void is_visible_callback(const toaster_msgs::FactList::ConstPtr& msg);

/** \brief Main function that runs the ROS node to record one file */
int main(int argc, char** argv){
  // ROS node initialization
  ros::init(argc, argv, "data_record");
  ros::NodeHandle node;
  node_ = &node;
  ros::Rate rate(30.0); 

  // To measure time between to loops
  uint64_t time_prev;
  uint64_t time_now;
  std::vector<uint64_t> time_diff_vector;

  // Path root to save the file
  std::string path_root;
  node.getParam("recorder/path_root", path_root);
  // Path to record file
  std::string data_file_name;  
  node.getParam("recorder/data_file_name", data_file_name);  
  // To signify if the map->pose transforms are wanted
  bool transform_limb_map;
  node.getParam("tf_frames/transform_limb_map", transform_limb_map);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::Subscriber human_list_sub_ = node.subscribe("/pdg/humanList", 1, human_list_callback);
  ros::Subscriber is_visible_sub = node.subscribe("/move3d_facts/factList", 1, is_visible_callback);
 
  // To the write name to the new file
  // If the file name given through the params is "action", it checks if there are already existing "action" files
  // If there are, it counts how many. If there are 2, the new file will be names "action3.txt"
  int count_file = 0;
  bf::directory_iterator end_it;
  for(bf::directory_iterator file_it(path_root); file_it != end_it; file_it++){
    bf::path file_path = file_it->path();
    if(bf::is_regular_file(file_path) && !tools::is_hidden(file_path)){  
      std::string file_name = tools::get_file_name(file_path);
      std::string label_name = file_name.substr(0, file_name.find_last_of("_"));
      if(label_name.compare(data_file_name) == 0 && bf::extension(file_path).compare(".txt") == 0)
        count_file += 1;
    }
  }

  std::string seg_file_name;
  seg_file_name = data_file_name+"_"+std::to_string(count_file)+".xml"; //C++11
  data_file_name = data_file_name+"_"+std::to_string(count_file)+".txt"; //C++11
  std::ofstream data_file(path_root+data_file_name);
  std::ofstream write_seg(path_root+seg_file_name);

  initialize_tf2_frames(); 
 
  // Iniatialization of the terminal so that it can listen to input from keyboard when the node is running
  char c;
  int n, tem;
  int count = 0;
  tem = fcntl(0, F_GETFL, 0);
  fcntl (0, F_SETFL, (tem | O_NDELAY));
  bool first = true;

  // Listen to tf2 broadcaster and write the frames in file
  std::vector<Frame>::iterator it;
  FeatureMatrix fm;
  while (node.ok()){
    ros::spinOnce();
    try{ 
      // New time diff between 2 loops is added to the vector
      time_now = ros::Time::now().toNSec();
      if(!first)
        time_diff_vector.push_back(time_now - time_prev);
      first = false;
      time_prev = time_now;

      // 1 feature vector for 1 loop, added to the feature matrix
     fm.new_feature_vector();
     // Iteration through the array of transforms, 1 iteration for 1 limb
      for(std::vector<std::vector<Frame> >::iterator it_tf_array = transforms_array_.begin(); 
          it_tf_array != transforms_array_.end(); it_tf_array++){
        
        geometry_msgs::Pose* limb_pose;
        // To get the index on which the iterator is
        switch(it_tf_array - transforms_array_.begin()){
          case 0:
            limb_pose = &hand_pose_;
            break;
          case 1:
            limb_pose = &head_pose_;
            break;
          default:
            break;
        }

        // If the transform map->limb is needed
        if(transform_limb_map){

          std::vector<float> vector;
          vector.push_back(limb_pose->position.x);
          vector.push_back(limb_pose->position.y);
          vector.push_back(limb_pose->position.z);
          vector.push_back(limb_pose->orientation.x);
          vector.push_back(limb_pose->orientation.y);
          vector.push_back(limb_pose->orientation.z);
          vector.push_back(limb_pose->orientation.w);

          fm.add_sensor_feature_vector(vector); 
        }
        // To iterate through the different transformations to compute between limb and objects
        for(it = it_tf_array->begin(); it != it_tf_array->end(); it++){
          geometry_msgs::TransformStamped transformStamped; 
          // Transform map->object
          transformStamped = tfBuffer.lookupTransform("map", 
                                                      it->source_frame, 
                                                      ros::Time(0));
          tf2::Transform element_transform;
          tf2::Transform result_transform;
          tf2::Transform limb_transform;
          // geometry_msgs to tf2::Transform for the transformStamped
          tf2::fromMsg(transformStamped.transform, element_transform);
          // geometry_msgs to tf2::Transform for the limb_pose
          tf2::fromMsg(*limb_pose, limb_transform);
          // Compute the transform limb->object
          result_transform = element_transform.inverseTimes(limb_transform);

          // Add the transform to a vector (which will be added as sensor feature vector)
          std::vector<float> vector;
          vector.push_back(result_transform.getOrigin().x());
          vector.push_back(result_transform.getOrigin().y());
          vector.push_back(result_transform.getOrigin().z());

          // Add the flag isVisible and compute the distance between the head and the object
          // (to do before the addition of the quaternion to vector so vector.end() can be used)
          if(limb_pose == &head_pose_){
            bool isVisible = object_visible_[objects_map_[it->source_frame]];
            fm.add_flag(isVisible);
            if(isVisible){
              fm.add_flag(std::sqrt(std::inner_product(vector.begin(), vector.end(), vector.begin(), 0.0)));
            }else{
              fm.add_flag(0);
            }
          }

          // Add the quaternion of the transform to the vector if needed
          if(it->type == SensorFeatureVectorType::SensorFeatureVectorExtended){
            vector.push_back(result_transform.getRotation().x());
            vector.push_back(result_transform.getRotation().y());
            vector.push_back(result_transform.getRotation().z());
            vector.push_back(result_transform.getRotation().w());
          }
          // Add vector as sensor feature vector to the feature vector
          fm.add_sensor_feature_vector(vector);         
        }
      }
      count++;
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    // Write in file when an enter key input is detected, to signal a new segmentation 
    // ( do not press other key)
    n = read(0, &c, 1);
    if (n > 0)
      write_seg << "new segmentation at vector " << count << " " << "\n";

    rate.sleep();
  }
  fm.write_to_file(data_file, FeatureFileFormat::lab);
  // Close before exit node
  double sum = std::accumulate(time_diff_vector.begin(), time_diff_vector.end(), 0.0);
  double mean = sum / time_diff_vector.size();
  fcntl(0, F_SETFL, tem);
  write_seg.close();
  std::cout << mean << std::endl;
  return 0;

}

void initialize_tf2_frames(void){
  int transforms_nb;
  int i = 0;
  std::string n = std::to_string(i); //C++11
  while(node_->getParam("tf_frames/transforms_nb/limb"+n, transforms_nb)){
    tf_nb_per_limb_.push_back(transforms_nb);
    i++; 
    n = std::to_string(i);
  }

  std::string path = "tf_frames/transforms/nb";
  std::string target_frame;
  std::string source_frame;
  int type; 
  int count = 0;
  int count_head = 0;
  for(std::vector<int>::iterator it = tf_nb_per_limb_.begin(); it != tf_nb_per_limb_.end(); ++it){
    transforms_array_.emplace_back();
    for(int c = 0; c < *it; c++){
      std::string n = std::to_string(count++); //C++11
      node_->getParam(path+n+"/targetFrame", target_frame);
      node_->getParam(path+n+"/sourceFrame", source_frame);
      node_->getParam(path+n+"/type", type);
      transforms_array_.back().emplace_back(target_frame, source_frame, static_cast<SensorFeatureVectorType>(type));
      if(target_frame == "sonof/HERAKLES_HUMAN1/head"){
        objects_map_[source_frame]=count_head;
        count_head++;
      }
    }
  }
  object_visible_.assign(objects_map_.size(), false);
}


void human_list_callback(const toaster_msgs::HumanListStamped::ConstPtr& msg){
  if (!msg->humanList.empty())
  {
    for (unsigned int i = 0; i < msg->humanList.size(); ++i)
    {
      if(msg->humanList[i].meAgent.meEntity.id=="HERAKLES_HUMAN1")
        for( unsigned int j = 0 ; j < msg->humanList[i].meAgent.skeletonJoint.size() ; ++j )
        {
          if(msg->humanList[i].meAgent.skeletonNames[j]=="head")
            head_pose_=msg->humanList[i].meAgent.skeletonJoint[j].meEntity.pose;
          if(msg->humanList[i].meAgent.skeletonNames[j]=="rightHand")
            hand_pose_=msg->humanList[i].meAgent.skeletonJoint[j].meEntity.pose; 
        }
    }
  }
}

void is_visible_callback(const toaster_msgs::FactList::ConstPtr& msg){
  std::fill(object_visible_.begin(), object_visible_.end(), false);

  if(!msg->factList.empty()){
    for (unsigned int i = 0; i < msg->factList.size(); ++i){
      if(msg->factList[i].property == "isVisibleBy"){
        if(msg->factList[i].targetId =="HERAKLES_HUMAN1"){
          for(std::map<std::string, int>::iterator objects_map_it = objects_map_.begin(); 
              objects_map_it != objects_map_.end(); objects_map_it++){
            if(msg->factList[i].subjectId == objects_map_it->first)
              object_visible_[objects_map_it->second]=true;
          }
        }
      }
    }
  }
}
