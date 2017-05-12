#include <ros/ros.h>
#include <iostream>

#include "action_recognition/DataHandler.hpp"
#include "action_recognition/TrainHMM.hpp"
#include "action_recognition/Setup.hpp"

int main(int argc, char** argv){
  ros::init(argc, argv, "action_reco");

  ros::NodeHandle node;

  ros::Rate rate(10.0); 

  std::string path_root = "/home/amayima/catkin_ws/src/ActionRecognition/test_data_handler/";
  std::string path_input = "/home/amayima/catkin_ws/src/ActionRecognition/test_data_handler/";

  Setup setup(path_root,path_input);

  DataHandler datah(setup);

  bool enable_recognition; 
  bool enable_training; 
  node.getParam("action_recognition/enable_recognition", enable_recognition);
  node.getParam("action_recognition/enable_training", enable_training);


  if(enable_training){
 
  datah.raw_data_from_file_to_feature_matrices(setup.seg_files_path, setup.data_path, NormalizationTypes::no);

  std::pair<std::map<std::string, std::vector<FeatureMatrix> >::iterator,
          std::map<std::string, std::vector<FeatureMatrix> >::iterator > it = datah.get_map_iterator();
  for(; it.first != it.second ; it.first++)
    TrainHMM::train_HMM(EmissionTypes::Gaussian, it.first->second, StatesNumDefs::median, 100, setup);

  datah.get_labels().write_to_file(LabelFileFormats::txt);
  datah.get_labels().write_to_file(LabelFileFormats::grammar);
  datah.get_labels().write_to_file(LabelFileFormats::dict);

  std::cout <<  datah.get_labels().compile_grammar() << std::endl;
  std::cout <<  datah.get_labels().test_grammar() << std::endl;

  }

  if(enable_recognition){

    std::cout << "not yet here hehe"<< std::endl;


  }

  /** for a listener 

  while (node.ok()){
    rate.sleep();
  }

  **/

  /** for a broadcaster 

  ros::spin();

  **/

  return 0;
};
