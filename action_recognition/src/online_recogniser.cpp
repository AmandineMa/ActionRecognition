/**
 * \file online_recogniser.cpp
 * \brief This ROS node aims to do online human action recognition from trained HMMs.<br>
 * It is multithreaded, with one thread doing recognition with HTK, the other one getting
 * the tf data and writing them in a feature matrix. <br>
 * It works with sliding window. Recognition of a window of x seconds
 * with data stored in a feature matrix.  Then it removes a few samples for the feature matrix
 * and keep the rest of the data as "memory". Then new samples are added to the feature matrix.
 * Then it recognizes it, then it removes the first samples and so on.
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
#include <thread>
#include <condition_variable>
#include <atomic>
#include <boost/regex.hpp>

#include "action_recognition/common.hpp"
#include "toaster_msgs/HumanListStamped.h"
#include "toaster_msgs/FactList.h"
#include "action_recognition/FeatureMatrixD.hpp"
#include "action_recognition/Setup.hpp"

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
std::vector<std::vector<Frame> > transforms_array_;
std::vector<int> tf_nb_per_limb_;
ros::NodeHandle* node_;
geometry_msgs::Pose hand_pose_,head_pose_ ; 
FeatureMatrixD fm_;
bool is_ready(false);
std::mutex mutex_;
std::condition_variable cv_;

const float RATE = 30.0;
const int SAMPLE_NUMBER_TH = 5*RATE;
const int N_SAMPLE_REMOVE = 1*RATE;

/* -------- Function declarations -------- */
void initialize_tf2_frames(void);
void humanListCallback(const toaster_msgs::HumanListStamped::ConstPtr& msg);

Setup setup_init(void);
void set_env(Setup setup);

// Thread doing the recognition
void recogniser(void){
  int normalization_type;
  std::string tmp_dir;
  node_->getParam("recogniser/normalization", normalization_type); 
  node_->getParam("recogniser/tmp_dir", tmp_dir); 
 
  Setup setup(setup_init());

  set_env(setup);


  while(ros::ok()){
 
    // Wait for the feature matrix to be ready to be recognized
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, []{return is_ready;});

    fm_.normalize(static_cast<NormalizationType>(normalization_type));
   
    std::string tmp_file_name = tmp_dir+"tmp_data.dat";
    std::ofstream tmp_file(tmp_file_name);
    fm_.write_to_file(tmp_file);
    
    // Remove the n first feature vectors from the matrix
    fm_.pop_feature_vectors(N_SAMPLE_REMOVE);
    is_ready = false;
    // Free the lock
    lock.unlock();

    // Recognition with HTK. The recognised action is written in ROS_INFO
    std::string command = "HVite -A -T 1 -H "+setup.hmmsdef_path
      +" -w "+setup.grammar_net_path
      +" "+setup.dict_path
      +" "+setup.labels_list_path
      +" "+tmp_file_name;
    std::string output = tools::execute_command(command);
    boost::smatch match;
    boost::regex reg_exp("\\n([a-z]+\\s)+");
    boost::regex_search(output, match, reg_exp);
    std::istringstream iss(match[0]);
    std::string action;
    // Get the last action recognised by HVite
    while(iss >> action){}     
    ROS_INFO("%s", action.c_str());
    //ROS_INFO("%s", output.c_str());

  }
}

/* ------- main ------- */
int main(int argc, char** argv){
  // ROS node initialization
  ros::init(argc, argv, "data_record");
  ros::NodeHandle node;
  node_ = &node;
  ros::Rate rate(RATE); 

  std::string path_root;
  node.getParam("recorder/path_root", path_root);

  // Get frames from tf2 initialization
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::Subscriber human_list_sub_ = node.subscribe("/pdg/humanList", 1, humanListCallback);
  //ros::Subscriber is_visible_sub = node.subscribe("/move3d_facts/factList", 1, is_visible_callback);
 
  bool transform_limb_map;
  node.getParam("tf_frames/transform_limb_map", transform_limb_map);
  
  initialize_tf2_frames(); 
 
  // Listen to tf2 broadcaster and write the frames in file
  std::vector<Frame>::iterator it;

  // Init the recogniser thread
  std::thread recogniser_thread(recogniser);
  uint64_t time_prev;
  uint64_t time_now; bool first = true;
  std::vector<uint64_t> time_diff_vector;

  while (node.ok()){

    ros::spinOnce();

    try{  
      // New time diff between 2 loops is added to the vector
      time_now = ros::Time::now().toNSec();
      if(!first){
        time_diff_vector.push_back(time_now - time_prev);
        //std::cout << time_diff_vector.back() << std::endl;
      }
      first = false;
      time_prev = time_now;

      // 1 feature vector for 1 loop, added to the feature matrix
      fm_.new_feature_vector();

      // Iteration through the array of transforms, 1 iteration for 1 limb
      for(std::vector<std::vector<Frame> >::iterator it_tf_array = transforms_array_.begin(); 
          it_tf_array != transforms_array_.end(); it_tf_array++){
        
        geometry_msgs::Pose* limb_pose; 
        // To get the index on which the iterator is
        switch(it_tf_array - transforms_array_.begin()){ //iterator on index n
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

          // vector.push_back(32);
          // vector.push_back(32);
          // vector.push_back(32);
          // vector.push_back(32);
          // vector.push_back(32);
          // vector.push_back(32);
          // vector.push_back(32);
          fm_.add_sensor_feature_vector(vector); 

        }  
        // To iterate through the different transformations to compute between limb and objects
        for(it = it_tf_array->begin(); it != it_tf_array->end(); it++){
          // Transform map->object 
          geometry_msgs::TransformStamped transformStamped; 
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

          // vector.push_back(32);
          // vector.push_back(32);
          // vector.push_back(32);
 
          // Add the quaternion of the transform to the vector if needed
          if(it->type == SensorFeatureVectorType::SensorFeatureVectorExtended){
            vector.push_back(result_transform.getRotation().x());
            vector.push_back(result_transform.getRotation().y());
            vector.push_back(result_transform.getRotation().z());
            vector.push_back(result_transform.getRotation().w());

            // vector.push_back(32);
            // vector.push_back(32);
            // vector.push_back(32);
            // vector.push_back(32);
          }

          fm_.add_sensor_feature_vector(vector);         
        }
      }

      // lock is initialized but without locking the mutex
      std::unique_lock<std::mutex> lock(mutex_, std::defer_lock);
      // Try to get the lock for the mutex, if it cannot do it
      // it means than the mutex is already locked by the other thread
      // If so, it just re-loop to get a new sample
      if(lock.try_lock()){
        if(fm_.get_samples_number() > SAMPLE_NUMBER_TH){
          is_ready = true;
          cv_.notify_one();
        }
      }

    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    rate.sleep();
  }
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
    }
  }
}

Setup setup_init(void){
  std::string path_root;
  std::string path_segmentation;
  std::string path_data;
  node_->getParam("file_setup/path_root", path_root);
  node_->getParam("file_setup/path_data", path_data);
  node_->getParam("file_setup/path_segmentation", path_segmentation);
  Setup setup(path_root,path_data, path_segmentation);
  setup.hmmsdef_path = setup.output_path+"hmmsdef";
  return setup;
}

void set_env(Setup setup){
  std::string HTK_conf_file_name = setup.root_path+"HTK_conf_file";
  std::ofstream HTK_conf_file(HTK_conf_file_name);
  HTK_conf_file << "NATURALREADORDER = TRUE" << "\n" << "NATURALWRITEORDER = TRUE" << "\n"
                << "FORCEOUT = TRUE" << "\n";
  HTK_conf_file.close();
  setenv("HCONFIG",const_cast<char*>(HTK_conf_file_name.c_str()),true);
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
