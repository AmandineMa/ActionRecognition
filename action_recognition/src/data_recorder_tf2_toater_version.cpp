#include <ros/ros.h>
#include <ros/console.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <fstream>
#include <string>
#include <stdio.h>
#include <sys/types.h>
#include <fcntl.h>

#include "action_recognition/common.hpp"

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
ros::NodeHandle* node_;

/* -------- Function declarations -------- */
void initialize_tf2_frames(void);
int getch();

/* ------- main ------- */
int main(int argc, char** argv){
  // ROS node initialization
  ros::init(argc, argv, "data_record");
  ros::NodeHandle node;
  node_ = &node;
  ros::Rate rate(10.0); 

  std::string path_root = "/home/amayima/catkin_ws/src/ActionRecognition/test_data_handler/";

  // Get frames from tf2 initialization
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped; 
  std::string data_file_name;  
  std::string seg_file_name;
  node.getParam("setup/data_file_name", data_file_name);  
  node.getParam("setup/seg_file_name", seg_file_name);
  std::ofstream data_file(path_root+data_file_name);
  std::ofstream write_seg(path_root+seg_file_name);

  initialize_tf2_frames(); 
 
  // Get inputs from terminal initialization
  char c;
  int n, tem;
  int count = 0;
  tem = fcntl(0, F_GETFL, 0);
  fcntl (0, F_SETFL, (tem | O_NDELAY));

  // Listen to tf2 broadcaster and write the frames in file
  std::vector<Frame>::iterator it;
  data_file << "<Data>\n";
  while (node.ok()){
      
    try{
        data_file << "<FeatVect>"; 
        for(it = tf_frames_array.begin(); it != tf_frames_array.end(); it++){
          transformStamped = tfBuffer.lookupTransform(it->source_frame, 
                                                      it->target_frame, 
                                                      ros::Time(0));
          if(it->type == SensorFeatureVectorType::SensorFeatureVectorExtended){
            data_file << "<SensFeatExt>" << 
              transformStamped.transform.translation.x << " " << 
              transformStamped.transform.translation.y << " " << 
              transformStamped.transform.translation.z << " " << 
              transformStamped.transform.rotation.x << " " << 
              transformStamped.transform.rotation.y << " " << 
              transformStamped.transform.rotation.z << " " <<
              transformStamped.transform.rotation.w <<
              "</SensFeatExt>";
          }else if(it->type == SensorFeatureVectorType::SensorFeatureVector){
            data_file << "<SensFeat>" << 
              transformStamped.transform.translation.x << " " << 
              transformStamped.transform.translation.y << " " << 
              transformStamped.transform.translation.z <<
              "</SensFeat>";
          }
        }
        data_file << "</FeatVect>\n";
        count++;
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    // Write in file when an keyboard input is detected, to signal a new segmentation
    n = read(0, &c, 1);
    if (n > 0){
      write_seg << "new segmentation at vector " << count << "\n";
    }

    rate.sleep();
  }

  // Close before exit node
  fcntl(0, F_SETFL, tem);
  write_seg.close();
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
    tf_frames_array.emplace_back(target_frame, source_frame, static_cast<SensorFeatureVectorType>(type));
  }
}


