#ifndef SETUP_HPP
#define SETUP_HPP

#include <ros/ros.h>
#include <boost/filesystem.hpp>
#include <string>

struct Setup
{
  std::string root_path;
  std::string input_path; 
  std::string data_path; 
  std::string seg_files_path; 
  std::string output_path; 
  std::string htk_tmp_files_path;
  std::string labels_list_path;
  std::string grammar_net_path;
  std::string dict_path;
  std::string grammar_path;

  int default_state_number;

  Setup(std::string root_p, std::string input_p, int default_state_nb = 5){
    default_state_number = default_state_nb;
    root_path = root_p;
    input_path = input_p;
    data_path = input_p+"data/";
    seg_files_path = input_p+"segmentation/";
    htk_tmp_files_path = root_p+"htk_tmp/";
    output_path = root_p+"output/";
    labels_list_path = output_path+"labels.list";
    grammar_net_path = output_path+"grammar.net.slf";
    dict_path = root_path+"labels.dict";
    grammar_path = root_path+"labels.grammar";

    boost::filesystem::path dir(output_path.c_str());
    try{
      if (boost::filesystem::create_directory(dir))
	std::cout<<"output directory created"<<std::endl;

      dir = htk_tmp_files_path.c_str();
      if (boost::filesystem::create_directory(dir))
	std::cout<<"tmp directory created"<<std::endl;
     
    }catch(const boost::filesystem::filesystem_error& e ){
      std::cerr <<"Error creating directories :"<< e.code().message() << '\n';
      exit(1);
    }

  }

};

#endif
