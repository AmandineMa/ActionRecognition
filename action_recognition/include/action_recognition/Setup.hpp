#ifndef SETUP_HPP
#define SETUP_HPP

#include <ros/ros.h>
#include <boost/filesystem.hpp>
#include <string>

struct Setup
{
  std::string root_path;
  std::string data_path; 
  std::string seg_files_path; 
  std::string output_path; 
  std::string output_hmm; 
  std::string htk_tmp_files_path;
  std::string labels_list_path;
  std::string grammar_net_path;
  std::string dict_path;
  std::string grammar_path;
  std::string  mlf_path;
  std::string data_list_path;
  std::string hmmsdef_path;

  Setup(std::string root_p, std::string data_p, std::string seg_p){
    root_path = root_p;
    data_path = data_p;
    seg_files_path = seg_p;
    htk_tmp_files_path = root_p+"htk_tmp/";
    output_path = root_p+"output/";
    output_hmm = output_path+"hmms/";
    labels_list_path = output_path+"labels.list";
    grammar_net_path = output_path+"grammar.net.slf";
    dict_path = root_path+"labels.dict";
    grammar_path = root_path+"labels.grammar";

    create_directories();
  }

  void create_directories(void){
    boost::filesystem::path dir(output_path.c_str());
    try{
      if (boost::filesystem::create_directory(dir))
	std::cout<<"output directory created"<<std::endl;

      dir = htk_tmp_files_path.c_str();
      if (boost::filesystem::create_directory(dir))
	std::cout<<"tmp directory created"<<std::endl;

      dir = output_hmm.c_str();
      if (boost::filesystem::create_directory(dir))
	std::cout<<"output hmms directory created"<<std::endl;
     
    }catch(const boost::filesystem::filesystem_error& e ){
      std::cerr <<"Error creating directories :"<< e.code().message() << '\n';
      exit(1);
    }
  }

};

#endif
