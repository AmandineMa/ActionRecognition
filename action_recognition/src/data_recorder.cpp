#include <ros/ros.h>
#include <ros/console.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <fstream>
#include <string>
#include <stdio.h>
#include <sys/types.h>
#include <fcntl.h>

#include "action_recognition/common.hpp"
#include "toaster_msgs/HumanListStamped.h"


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
geometry_msgs::Pose hand_pose_,head_pose_ ;
/* -------- Function declarations -------- */
void initialize_tf2_frames(void);
int getch();

void humanListCallback(const toaster_msgs::HumanListStamped::ConstPtr& msg)
{
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

/* ------- main ------- */
int main(int argc, char** argv){
  // ROS node initialization
  ros::init(argc, argv, "data_record");
  ros::NodeHandle node;
  node_ = &node;
  ros::Rate rate(10.0); 

  std::string path_root;
  node.getParam("recorder/path_root", path_root);

  // Get frames from tf2 initialization
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::Subscriber human_list_sub_ = node.subscribe("/pdg/humanList", 1, humanListCallback);
  geometry_msgs::TransformStamped transformStamped; 
  std::string data_file_name;  
  std::string seg_file_name;
  node.getParam("recorder/data_file_name", data_file_name);  
  node.getParam("recorder/seg_file_name", seg_file_name);
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

  // Listen to tf2 broadcaster and write the frames in file
  std::vector<Frame>::iterator it;
  data_file << "<Data>\n";
  while (node.ok()){
    ros::spinOnce();
    try{
      data_file << "<FeatVect>"; 
      for(it = tf_frames_array.begin(); it != tf_frames_array.end(); it++){
        transformStamped = tfBuffer.lookupTransform("map", 
                                                    it->source_frame, 
                                                    ros::Time(0));
        tf2::Transform element_transform;
        tf2::Transform result_transform;
        tf2::Transform limb_transform;
        tf2::fromMsg(transformStamped.transform, element_transform);
        tf2::fromMsg(hand_pose_, limb_transform);
        //if(count_frame < tf_frames_array.size()/2)
        result_transform = element_transform.inverseTimes(limb_transform);
        // else
        //result_transform = element_transform.inverseTimes(head_pose_);
        if(it->type == SensorFeatureVectorType::SensorFeatureVectorExtended){
          data_file << "<SensFeatExt>" << 
            result_transform.getOrigin().x() << " " << 
            result_transform.getOrigin().y() << " " << 
            result_transform.getOrigin().z() << " " << 
            result_transform.getRotation().x() << " " << 
            result_transform.getRotation().y() << " " << 
            result_transform.getRotation().z() << " " <<
            result_transform.getRotation().w() <<
            "</SensFeatExt>";
        }else if(it->type == SensorFeatureVectorType::SensorFeatureVector){
          data_file << "<SensFeat>" << 
            result_transform.getOrigin().x() << " " << 
            result_transform.getOrigin().y() << " " << 
            result_transform.getOrigin().z() <<
              "</SensFeat>";
        }
        count_frame++;
      }
      count_frame = 0;
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


