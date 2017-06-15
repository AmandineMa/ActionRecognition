#include <ros/ros.h>
#include <ros/console.h>
#include <boost/filesystem.hpp>
#include <fstream>
#include <boost/regex.hpp>
#include <queue>

#include "action_recognition/common.hpp"

namespace bf = boost::filesystem;

std::string filling(std::string label_name, int &c){

  std::string x;
  if(label_name.compare("pointing")==0){
    if(c == 0){
      x = "pointing";
      c = 0;
    }
  }

  if(label_name.compare("pick_place")==0){
    if(c == 0){
      x = "pick";c++;
    }else{ 
      x = "place";
      c = 0;
    }
  }

  if(label_name.compare("pick_place_h")==0){
    if(c == 0){
      x = "pick";c++;
    }else{ 
      x = "place";
      c = 0;
    }
  }

  if(label_name.compare("pick_place_w")==0){
    if(c == 0){
      x = "pick";c++;
    }else{ 
      x = "place";
      c = 0;
    }
  }

  if(label_name.compare("scenario1")==0){
    if(c == 0){
      x = "sil";c++;
    }else if(c == 1){
      x = "walk";c++;
    }else if(c == 2){
      x = "pick";c++;
    }else if(c == 3){
      x = "place";c++;
    }else{ 
      x = "sil";
      c = 0;
    }
    
  }
  if(label_name.compare("scenario1bis")==0){
    if(c == 0){
      x = "sil";c++;
    }else if(c == 1){
      x = "walk";c++;
    }else if(c == 2){
      x = "pick";c++;
    }else if(c == 3){
      x = "walk";c++;
    }else if(c == 4){
      x = "place";c++;
    }else{ 
      x = "sil";
      c = 0;
    }
  }
  if(label_name.compare("scenario2")==0){
    if(c == 0){
      x = "sil";c++;
    }else if(c == 1){
      x = "walk";c++;
    }else if(c == 2){
      x = "point";c++;
    }else if(c == 3){
      x = "walk";c++;
    }else{ 
      x = "sil";
      c = 0;
    }
  }
  if(label_name.compare("scenario4")==0){
    if(c == 0){
      x = "sil";c++;
    }else if(c == 1){
      x = "walk";c++;
    }else if(c == 2){
      x = "pretend_pick";c++;
    }else if(c == 3){
      x = "pick";c++;
    }else{ 
      x = "sil";
      c = 0;
    }
  }

  return x;
}

int main(int argc, char** argv){

  // ROS node initialization
  ros::init(argc, argv, "generator");
  ros::NodeHandle node;
  std::string data_dir;
  std::string seg_dir;  
  std::string pre_seg_dir;
  node.getParam("generator/data_path", data_dir);
  node.getParam("generator/seg_path", seg_dir);
  node.getParam("generator/pre_seg_path", pre_seg_dir);
  bf::directory_iterator end_it;

  if (bf::create_directory(seg_dir))
    std::cout<<"segmentation directory created"<<std::endl;

  std::string data_path = data_dir;

  // If the iterator is on a directory and that directory is not empty
  if( bf::is_directory(data_path) && !tools::is_hidden(data_path)){
    int count_file = 0;int c=0;
    // Iterate files of the directory (1 file = 1 data example of the general activity)
    for(bf::directory_iterator file_it(data_path); file_it != end_it; file_it++){
 
      bf::path file_path = file_it->path();
      if(bf::is_regular_file(file_path) && !tools::is_hidden(file_path)){ 
        std::ofstream file(seg_dir+tools::get_file_name(file_path)+".xml");
        // Count line number in data file
        int i = 0;
        std::string line;
        std::ifstream f(file_path.c_str());
        while(std::getline(f, line)){
          ++i;
        }
        i=i-2;
        std::ifstream pre_seg_file(pre_seg_dir+tools::get_file_name(file_path)+".xml");
        std::string file_name = tools::get_file_name(file_path);
        std::string label_name = file_name.substr(0, file_name.find_last_of("_"));
        std::string pre_seg_line;
        boost::smatch match;
        boost::regex reg_exp("\\d+");
        int match_prev;
        std::queue<int> num_queue;
        int count_line = 0;
        while(std::getline(pre_seg_file, pre_seg_line)){ 
          if(boost::regex_search(pre_seg_line, match, reg_exp)){
            num_queue.push(stoi(match[0]));
          }
          count_line++;
        }
          
        if((label_name.compare("scenario1")==0 && count_line != 4) 
           || (label_name.compare("scenario1bis")==0 && count_line != 5)
           || (label_name.compare("scenario2")==0 && count_line != 4) 
           || (label_name.compare("scenario4")==0 && count_line != 4)
           || (label_name.compare("pick_place_h")==0 && count_line != 2)
           || (label_name.compare("pick_place_w")==0 && count_line != 2)
           || (label_name.compare("pick_place")==0 && count_line != 1))
          std::cout << "error in file " << file_name << std::endl;


        file << "<MotionLabeling>\n";
        if(label_name.compare("pointing")!=0){
          while((num_queue.size()!=1 && count_line == 2) || (!num_queue.empty() && count_line==1)){
            
            file << "<MotionLabel name=\"";
             
            file << filling(label_name, c);
            if(c == 1){
              file <<"\" startPoint=\""<< 1;
              file <<"\" endPoint=\"" << num_queue.front()-1 << "\"/>\n";
            }
            else{ 
              file  <<"\" startPoint=\"" << match_prev;
              file <<"\" endPoint=\"";
            }
            match_prev = num_queue.front();
            num_queue.pop();
          }
          //file << match_prev-1 << "\"/>\n" ;
          file << "<MotionLabel name=\"";
          file << filling(label_name, c);
          file  <<"\" startPoint=\"" << match_prev; 
          file <<"\" endPoint=\"";
          file << i << "\"/>\n</MotionLabeling>";
          file.close();
        }else{ 
          file << "<MotionLabel name=\"pointing";
          file <<"\" startPoint=\""<< 1;
          file <<"\" endPoint=\"" << i <<"\"/>\n</MotionLabeling>";

        }
        count_file++;
      }

    }

  }
  return 0;
}

