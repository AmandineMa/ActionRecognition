#include <ros/ros.h>
#include <ros/console.h>
#include <boost/filesystem.hpp>
#include <fstream>
#include "action_recognition/common.hpp"

namespace bf = boost::filesystem;

int main(int argc, char** argv){
  // ROS node initialization
  ros::init(argc, argv, "data_record");
  ros::NodeHandle node;
  ros::Rate rate(10.0);
  std::string dir_path = "/home/amayima/catkin_ws/src/ActionRecognition/test_data_handler/data/set1/";  
  std::string new_dir_path = "/home/amayima/catkin_ws/src/ActionRecognition/test_data_handler/data/set1_new/";
 bf::directory_iterator end_it;
for(bf::directory_iterator file_it(dir_path); file_it != end_it; file_it++){
   bf::path file_path = file_it->path();
   int i = 0;
  if(bf::is_regular_file(file_path) && !tools::is_hidden(file_path)){ 
    std::ifstream data_file(file_path.c_str());
    std::ofstream new_data_file(new_dir_path+tools::get_file_name(file_path.c_str())+".txt");
    std::string line;
    while(std::getline(data_file, line)){
      if(line.find("</FeatVect>") != std::string::npos){
        if(!(i % 2)){
          std::string new_line = line.substr(0, line.rfind("</FeatVect>"));
          line = new_line;        
        } 
        else
          line = line+"\n";
        i++;
      }
      new_data_file << line;
    }
    data_file.close();
    new_data_file.close();
  }
 }

}
