#include <ros/ros.h>
#include <ros/console.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>
#include <string>
#include <stdio.h>
#include <cstdlib>

#include "action_recognition/DataHandler.hpp"
#include "action_recognition/TrainHMM.hpp"
#include "action_recognition/Setup.hpp"
#include "action_recognition/HTKHeader.hpp"

void generate_MMF_from_HMM_files(std::string files_directory, std::string hmmsdef_path, Labels labels);


int main(int argc, char** argv){
  ros::init(argc, argv, "action_reco");

  ros::NodeHandle node;
  
  ros::Rate rate(10.0); 

  std::string path_root = "/home/amayima/catkin_ws/src/ActionRecognition/test_data_handler/";
  std::string path_input = "/home/amayima/catkin_ws/src/ActionRecognition/test_data_handler/";

  Setup setup(path_root,path_input);
  std::string hmmsdef_path = setup.htk_tmp_files_path+"hmmsdef";

  std::string HTK_conf_file_name = path_root+"HTK_conf_file";
  std::ofstream HTK_conf_file(HTK_conf_file_name);
  HTK_conf_file << "NATURALREADORDER = TRUE" << "\n" << "NATURALWRITEORDER = TRUE" << "\n";
  HTK_conf_file.close();
  setenv("HCONFIG",const_cast<char*>(HTK_conf_file_name.c_str()),true);

  DataHandler datah(setup);

  bool enable_recognition; 
  bool enable_training; 
  node.getParam("setup/enable_recognition", enable_recognition);
  node.getParam("setup/enable_training", enable_training);
  
  if(enable_training){
 
    datah.raw_data_from_file_to_feature_matrices(setup.seg_files_path, setup.data_path);
    datah.normalize(NormalizationTypes::no);

    std::pair<std::map<std::string, std::vector<FeatureMatrix> >::iterator,
              std::map<std::string, std::vector<FeatureMatrix> >::iterator > it = datah.get_map_iterator();
    for(; it.first != it.second ; it.first++)
      TrainHMM::train_HMM(true, EmissionTypes::Gaussian, it.first->second, StatesNumDefs::median, 100, setup);

    datah.get_labels().write_to_file(LabelFileFormats::txt);
    datah.get_labels().write_to_file(LabelFileFormats::grammar);
    datah.get_labels().write_to_file(LabelFileFormats::dict);

    // ROS_INFO(datah.get_labels().compile_grammar().c_str());
    // ROS_INFO(datah.get_labels().test_grammar().c_str());
  
    generate_MMF_from_HMM_files(setup.htk_tmp_files_path, hmmsdef_path, datah.get_labels());
  
  }

  if(enable_recognition){
    std::string data_file_name = setup.data_path+"act1/test_seg.txt";
    FeatureMatrix fm = datah.raw_data_from_file_to_feature_matrix(data_file_name);
    fm.normalize(NormalizationTypes::no);
    // Open the new data file
    std::ofstream data_file("/home/amayima/catkin_ws/src/ActionRecognition/test_data_handler/htk_tmp/test_seg.dat");
    // Define the HTK header
    HTKHeader header;
    header.BytesPerSample = fm.get_feature_vector_size()*4; 
    header.nSamples = fm.get_samples_number();
    // Write the header to the data file
    header.write_to_file(data_file);
    fm.write_to_file(data_file);
    
    //std::string output = tools::execute_command("HList -h "+data_file_name+".dat");
    std::string command = "HVite -A -T 1 -H "+hmmsdef_path
      +" -i "+setup.output_path+"reco.mlf"
      +" -w "+setup.grammar_net_path
      +" "+setup.dict_path
      +" "+setup.labels_list_path
      +" -S "+setup.htk_tmp_files_path+"SIL.scp"; //TODO : change path
    std::string output = tools::execute_command(command);
    std::cout << output << std::endl;

  }

  return 0;
};



void generate_MMF_from_HMM_files(std::string files_directory, std::string hmmsdef_path, Labels labels){
  std::pair<std::set<std::string>::iterator, std::set<std::string>::iterator> it = labels.get_iterator();
  std::ofstream MMF_file(hmmsdef_path);
  for(; it.first != it.second ; it.first++){
    std::ifstream hmm((files_directory+(*it.first)+".hmm").c_str());
    MMF_file << hmm.rdbuf();
  }
  MMF_file.close();

  std::ifstream MMF_file_2(hmmsdef_path);
  std::ofstream temp(files_directory+"temp");
  std::string line;
  int count = 0;
  while(std::getline(MMF_file_2, line)){
    if((line.find("<VecSize>") == std::string::npos && line.find("~o") == std::string::npos) || count < 2){
      temp << line << std::endl;
    }
    count++;
  }
  MMF_file_2.close();
  temp.close();
  remove((hmmsdef_path).c_str());
  rename((files_directory+"temp").c_str(), (hmmsdef_path).c_str());
}


