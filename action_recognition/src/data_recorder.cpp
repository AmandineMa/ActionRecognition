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

/**
 * \brief Structure for tf_frames/transforms params
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
std::vector<Frame> tf_frames_array;   
std::vector<std::vector<Frame> > transforms_array;
std::vector<int> tf_nb_per_limb;
std::vector<bool> object_visible;
std::map<std::string, int> objects_map;
ros::NodeHandle* node_;
geometry_msgs::Pose hand_pose_,head_pose_ ;
/* -------- Function declarations -------- */
void initialize_tf2_frames(void);
int getch();
void humanListCallback(const toaster_msgs::HumanListStamped::ConstPtr& msg);
void is_visible_callback(const toaster_msgs::FactList::ConstPtr& msg);

/* ------- main ------- */
int main(int argc, char** argv){
  // ROS node initialization
  ros::init(argc, argv, "data_record");
  ros::NodeHandle node;
  node_ = &node;
  ros::Rate rate(30.0); 

  uint64_t time_prev;
  uint64_t time_now;
  std::vector<uint64_t> time_diff_vector;

  std::string path_root;
  node.getParam("recorder/path_root", path_root);

  // Get frames from tf2 initialization
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::Subscriber human_list_sub_ = node.subscribe("/pdg/humanList", 1, humanListCallback);
  ros::Subscriber is_visible_sub = node.subscribe("/move3d_facts/factList", 1, is_visible_callback);
  geometry_msgs::TransformStamped transformStamped; 
  std::string data_file_name;  
  std::string seg_file_name;
  bool transform_limb_map;
  node.getParam("recorder/data_file_name", data_file_name);  
  node.getParam("tf_frames/transform_limb_map", transform_limb_map);


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
  
  seg_file_name = data_file_name+"_"+std::to_string(count_file)+".xml"; //C++11
  data_file_name = data_file_name+"_"+std::to_string(count_file)+".txt"; //C++11
  std::ofstream data_file(path_root+data_file_name);
  std::ofstream write_seg(path_root+seg_file_name);

  initialize_tf2_frames(); 
 
  // Get inputs from terminal initialization
  char c;
  int n, tem;
  int count = 0;
  int count_frame = 0;
  tem = fcntl(0, F_GETFL, 0);
  fcntl (0, F_SETFL, (tem | O_NDELAY));
  bool first = true;
  // Listen to tf2 broadcaster and write the frames in file
  std::vector<Frame>::iterator it;
  FeatureMatrix fm;
  while (node.ok()){
    ros::spinOnce();
    try{ 

      time_now = ros::Time::now().toNSec();
      if(!first)
        time_diff_vector.push_back(time_now - time_prev);
      first = false;
      time_prev = time_now;

     fm.new_feature_vector();
      for(std::vector<std::vector<Frame> >::iterator it_tf_array = transforms_array.begin(); 
          it_tf_array != transforms_array.end(); it_tf_array++){
        
        geometry_msgs::Pose* limb_pose;
        switch(it_tf_array - transforms_array.begin()){ //iterator on index n
          case 0:
            limb_pose = &hand_pose_;
            break;
          case 1:
            limb_pose = &head_pose_;
            break;
          default:
            break;
        }

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
        for(it = it_tf_array->begin(); it != it_tf_array->end(); it++){

          transformStamped = tfBuffer.lookupTransform("map", 
                                                      it->source_frame, 
                                                      ros::Time(0));
          tf2::Transform element_transform;
          tf2::Transform result_transform;
          tf2::Transform limb_transform;
          tf2::fromMsg(transformStamped.transform, element_transform);
          tf2::fromMsg(*limb_pose, limb_transform);

          result_transform = element_transform.inverseTimes(limb_transform);

          std::vector<float> vector;
          vector.push_back(result_transform.getOrigin().x());
          vector.push_back(result_transform.getOrigin().y());
          vector.push_back(result_transform.getOrigin().z());

          if(limb_pose == &head_pose_){
            bool isVisible = object_visible[objects_map[it->source_frame]];
            fm.add_flag(isVisible);
            if(isVisible)
              fm.add_flag(std::sqrt(std::inner_product(vector.begin(), vector.end(), vector.begin(), 0.0)));
            else
              fm.add_flag(0);
          }

          if(it->type == SensorFeatureVectorType::SensorFeatureVectorExtended){
            vector.push_back(result_transform.getRotation().x());
            vector.push_back(result_transform.getRotation().y());
            vector.push_back(result_transform.getRotation().z());
            vector.push_back(result_transform.getRotation().w());
          }

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
    tf_nb_per_limb.push_back(transforms_nb);
    i++; 
    n = std::to_string(i);
  }

  std::string path = "tf_frames/transforms/nb";
  std::string target_frame;
  std::string source_frame;
  int type; 
  int count = 0;
  int count_head = 0;
  for(std::vector<int>::iterator it = tf_nb_per_limb.begin(); it != tf_nb_per_limb.end(); ++it){
    transforms_array.emplace_back();
    for(int c = 0; c < *it; c++){
      std::string n = std::to_string(count++); //C++11
      node_->getParam(path+n+"/targetFrame", target_frame);
      node_->getParam(path+n+"/sourceFrame", source_frame);
      node_->getParam(path+n+"/type", type);
      transforms_array.back().emplace_back(target_frame, source_frame, static_cast<SensorFeatureVectorType>(type));
      if(target_frame == "sonof/HERAKLES_HUMAN1/head"){
        objects_map[source_frame]=count_head;
        count_head++;
      }
    }
  }
  object_visible.assign(objects_map.size(), false);
}


void humanListCallback(const toaster_msgs::HumanListStamped::ConstPtr& msg){
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
  std::fill(object_visible.begin(), object_visible.end(), false);

  if(!msg->factList.empty()){
    for (unsigned int i = 0; i < msg->factList.size(); ++i){
      if(msg->factList[i].property == "isVisibleBy"){
        if(msg->factList[i].targetId =="HERAKLES_HUMAN1"){
          for(std::map<std::string, int>::iterator objects_map_it = objects_map.begin(); 
              objects_map_it != objects_map.end(); objects_map_it++){
            if(msg->factList[i].subjectId == objects_map_it->first)
              object_visible[objects_map_it->second]=true;
          }
        }
      }
    }
  }
}
