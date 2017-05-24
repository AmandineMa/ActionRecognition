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

namespace bf = boost::filesystem;

void generate_MMF_from_HMM_files(std::string files_directory, std::string hmmsdef_path, Labels labels);


int main(int argc, char** argv){
  ros::init(argc, argv, "action_reco");

  ros::NodeHandle node;
  
  ros::Rate rate(10.0); 

  std::string path_root;
  std::string path_input;
  int state_num_def;
  int iterations_nb;
  int emission_type;

  node.getParam("setup/path_root", path_root);
  node.getParam("setup/path_input", path_input);
  node.getParam("setup/state_num_def", state_num_def);
  node.getParam("setup/iteration_nb", iterations_nb);
  node.getParam("setup/emission_type", emission_type);

  Setup setup(path_root,path_input);
  std::string hmmsdef_path = setup.output_path+"hmmsdef";

  std::string HTK_conf_file_name = path_root+"HTK_conf_file";
  std::ofstream HTK_conf_file(HTK_conf_file_name);
  HTK_conf_file << "NATURALREADORDER = TRUE" << "\n" << "NATURALWRITEORDER = TRUE" << "\n"
                << "FORCEOUT = TRUE" << "\n";
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
      TrainHMM::train_HMM(true, static_cast<EmissionType>(emission_type), it.first->second, static_cast<StatesNumDef>(state_num_def), iterations_nb, setup);

    datah.get_labels().write_to_file(LabelFileFormats::txt);
    datah.get_labels().write_to_file(LabelFileFormats::grammar);
    datah.get_labels().write_to_file(LabelFileFormats::dict);

    ROS_INFO("%s", datah.get_labels().compile_grammar().c_str());
    ROS_INFO("%s", datah.get_labels().test_grammar().c_str());
  
    generate_MMF_from_HMM_files(setup.output_path, hmmsdef_path, datah.get_labels());
  
  }

  if(enable_recognition){    
    std::string path_dir;
    node.getParam("setup/path_data_to_reco", path_dir);
    bf::directory_iterator end_it;
    std::ofstream file_list(path_dir+"dat/file_list.scp"); 
    std::string dir_data = path_dir+"dat/";
    for(bf::directory_iterator file_it(path_dir); file_it != end_it; file_it++){  

      bf::path file_path = file_it->path();
      if(bf::is_regular_file(file_path) && !tools::is_hidden(file_path)){ 
    
        FeatureMatrix fm = datah.raw_data_from_file_to_feature_matrix(file_path.c_str());
        fm.normalize(NormalizationTypes::no);
        // Open the new data file
        std::string file_name = dir_data+tools::get_file_name(file_path)+".dat";
        std::ofstream data_file(file_name);
        // Define the HTK header
        HTKHeader header;
        header.BytesPerSample = fm.get_feature_vector_size()*4; 
        header.nSamples = fm.get_samples_number();
        // Write the header to the data file
        header.write_to_file(data_file);
        fm.write_to_file(data_file);
        data_file.close();
        file_list << file_name << "\n";         
      }
    }
    file_list.close();
    std::string command = "HVite -A -T 1 -C "+HTK_conf_file_name +
      " -H "+hmmsdef_path
      +" -i "+setup.output_path+"reco.mlf"
      +" -w "+setup.grammar_net_path
      +" "+setup.dict_path
      +" "+setup.labels_list_path
      +" -S "+dir_data+"file_list.scp";
    std::string output = tools::execute_command(command);
    ROS_INFO("%s", output.c_str());
  }

  return 0;
};



void generate_MMF_from_HMM_files(std::string files_directory, std::string hmmsdef_path, Labels labels){
  std::pair<std::set<std::string>::iterator, std::set<std::string>::iterator> it = labels.get_iterator();
  std::ofstream MMF_file(hmmsdef_path);
  for(; it.first != it.second ; it.first++){
    std::ifstream hmm((files_directory+(*it.first)).c_str());
    MMF_file << hmm.rdbuf();
  }
  MMF_file.close();

  std::ifstream MMF_file_2(hmmsdef_path);
  std::ofstream temp(files_directory+"temp");
  std::string line;
  int count = 0;
  while(std::getline(MMF_file_2, line)){
    if( ( (line.find("<VECSIZE>") == std::string::npos && line.find("~o") == std::string::npos 
           && line.find("<STREAMINFO>") == std::string::npos) && count >= 3) ||
        ( (line.find("<VECSIZE>") != std::string::npos || line.find("~o") != std::string::npos) && count < 3) ){
      temp << line << std::endl;
    }
    count++;
  }
  MMF_file_2.close();
  temp.close();
  remove((hmmsdef_path).c_str());
  rename((files_directory+"temp").c_str(), (hmmsdef_path).c_str());
}


