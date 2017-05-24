#include <ros/ros.h>
#include <ros/console.h>
#include <boost/filesystem.hpp>
#include <fstream>

#include "action_recognition/common.hpp"

namespace bf = boost::filesystem;

int main(int argc, char** argv){

// ROS node initialization
  ros::init(argc, argv, "generator");
  ros::NodeHandle node;
  std::string data_dir;
  std::string seg_dir;
  node.getParam("generator/data_path", data_dir);
  node.getParam("generator/seg_path", seg_dir);
  bf::directory_iterator end_it;
  bf::directory_iterator action_it(data_dir);

  // Iterate directories (1 directory = 1 general activity)
  for(action_it; action_it != end_it; action_it++){
  
    bf::path path_action_dir = action_it->path();

    // If the iterator is on a directory and that directory is not empty
    if( bf::is_directory(path_action_dir) && !tools::is_hidden(path_action_dir)){
      
      // Iterate files of the directory (1 file = 1 data example of the general activity)
      for(bf::directory_iterator file_it(path_action_dir); file_it != end_it; file_it++){
 
        bf::path file_path = file_it->path();
        if(bf::is_regular_file(file_path) && !tools::is_hidden(file_path)){ 
          std::ofstream file(seg_dir+"/"+tools::get_last_dir_name(file_path)+"/"+tools::get_file_name(file_path)+".xml");
          int i = 0;
          std::string line;
          std::ifstream f(file_path.c_str());
          while(std::getline(f, line)){
            ++i;
          }
          std::string file_name = tools::get_file_name(file_path);
          std::string label_name = file_name.substr(0, file_name.find_last_of("_"));
          file << "<MotionLabeling>\n<MotionLabel name=\""
               << label_name << "\" startPoint=\"1\" "
               << "endPoint=\"" << i-2 << "\"/>\n</MotionLabeling>";

        }

      }

    }

  }
  return 0;
}
