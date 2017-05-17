#include <ros/ros.h>
#include <ros/console.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>
#include <string>
#include <stdio.h>
#include <cstdlib>

#include "action_recognition/SensorFeatureVector.hpp" // TDODO: to change, here for size
#include "action_recognition/SensorFeatureVectorExtended.hpp" // TDODO: to change, here for size

struct Frame{
  std::string target_frame;
  std::string source_frame;
  int type;

  Frame(){}
  Frame(std::string targ_frame, std::string src_frame, int t):target_frame(targ_frame), source_frame(src_frame), type(t){}
};

std::vector<Frame> tf_frames_array;
ros::NodeHandle* node_;


void initialize_tf2_frames(void);

int main(int argc, char** argv){
  ros::init(argc, argv, "data_record");

  ros::NodeHandle node;
  node_ = &node;

  ros::Rate rate(10.0); 

  std::string path_root = "/home/amayima/catkin_ws/src/ActionRecognition/test_data_handler/";

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  initialize_tf2_frames();    

  geometry_msgs::TransformStamped transformStamped; 
  std::string data_file_name;
  node.getParam("setup/data_file_name", data_file_name);
  std::ofstream data_file(path_root+data_file_name);
  std::vector<Frame>::iterator it;
  data_file << "<Data>\n";
  while (node.ok()){
      
    try{
      data_file << "<FeatVect>";
      for(it = tf_frames_array.begin(); it != tf_frames_array.end(); it++){
        transformStamped = tfBuffer.lookupTransform(it->target_frame, 
                                                    it->source_frame, 
                                                    ros::Time(0));
        if(it->type == SENSOR_FEATURE_VECTOR_EXTENDED_SIZE){
          data_file << "<SensFeatExt>" << 
            transformStamped.transform.translation.x << " " << 
            transformStamped.transform.translation.y << " " << 
            transformStamped.transform.translation.z << " " << 
            transformStamped.transform.rotation.x << " " << 
            transformStamped.transform.rotation.y << " " << 
            transformStamped.transform.rotation.z << " " <<
            transformStamped.transform.rotation.w <<
            "</SensFeatExt>";
        }else if(it->type == SENSOR_FEATURE_VECTOR_SIZE){
          data_file << "<SensFeat>" << 
            transformStamped.transform.translation.x << " " << 
            transformStamped.transform.translation.y << " " << 
            transformStamped.transform.translation.z <<
            "</SensFeat>";
        }
      }
      data_file << "</FeatVect>\n";
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    rate.sleep();
  }

  data_file << "</Data>";
  data_file.close();

  return 0;

}

void initialize_tf2_frames(void){
  int transforms_nb;
  node_->getParam("tf_frames/transforms_nb", transforms_nb);
  std::string path = "tf_frames/transforms/nb";
  std::string target_frame;
  std::string source_frame;
  int type;
  for(int i = 0; i < transforms_nb; i++){
    std::string n = std::to_string(i); //C++11
    node_->getParam(path+n+"/targetFrame", target_frame);
    node_->getParam(path+n+"/sourceFrame", source_frame);
    node_->getParam(path+n+"/type", type);
    tf_frames_array.emplace_back(target_frame, source_frame, type);
  }
}
