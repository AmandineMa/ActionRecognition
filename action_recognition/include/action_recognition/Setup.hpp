#ifndef SETUP_HPP
#define SETUP_HPP

#include <ros/ros.h>
#include <boost/filesystem.hpp>
#include <string>

struct Setup
{
  std::string path_root;
  std::string path_input; 
  std::string path_data; 
  std::string path_seg_files; 
  std::string path_output; 
  std::string path_htk_tmp_files;

  int default_state_number;

  Setup(std::string path_r, std::string path_i, int default_state_nb = 5){
    default_state_number = default_state_nb;
    path_root=path_r;
    path_input=path_i;
    path_data=path_input+"data/";
    path_seg_files=path_input+"segmentation/";
    path_htk_tmp_files=path_root+"htk_tmp/";
    path_output=path_root+"output/";

    boost::filesystem::path dir(path_output.c_str());
    try{
      if (boost::filesystem::create_directory(dir))
	std::cout<<"output directory created"<<std::endl;

      dir = path_htk_tmp_files.c_str();
      if (boost::filesystem::create_directory(dir))
	std::cout<<"tmp directory created"<<std::endl;
     
    }catch(const boost::filesystem::filesystem_error& e ){
      std::cerr <<"Error creating directories :"<< e.code().message() << '\n';
      exit(1);
    }

  }

};

#endif
