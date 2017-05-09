#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <stdint.h>
//#include <sys/stat.h>
//#include "action_recognition/config.h"
//#include "action_recognition/load_features.h"
//#include <boost/filesystem.hpp>

#include "action_recognition/Vector3D.hpp"
//#include <eigen3/Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.h>
#include "action_recognition/SensorFeatureVector.hpp"
#include "action_recognition/SensorFeatureVectorExtended.hpp"
#include "action_recognition/FeatureVector.hpp"
#include "action_recognition/FeatureMatrix.hpp"
#include "action_recognition/common.hpp"
#include "action_recognition/HTKHeader.hpp"
#include "action_recognition/Labels.hpp"
#include "action_recognition/HMM.hpp"
#include "action_recognition/DataHandler.hpp"

int main(int argc, char** argv){
  ros::init(argc, argv, "action_reco");

  ros::NodeHandle node;

  ros::Rate rate(10.0); 

  /* ------ Tests of FeatureMatrix class ------ */


  // FeatureMatrix fm("take");
  // fm.new_feature_vector();
  // std::vector<float> vec2;
  // vec2.push_back(1);
  // vec2.push_back(2);
  // vec2.push_back(3);
  // fm.add_sensor_feature_vector(vec2);
  // std::vector<float> vec;
  // vec.push_back(1);
  // vec.push_back(2);
  // vec.push_back(3);
  // vec.push_back(4);
  // vec.push_back(5);
  // vec.push_back(6);
  // vec.push_back(7);
  // fm.add_sensor_feature_vector(vec);
  // fm.add_flag(1);
  // fm.add_flag(0);

  // fm.print();

  // FeatureMatrix fm2("pour");
  // fm2.new_feature_vector(std::vector<float>(4,1));
  // fm2.add_sensor_feature_vector(std::vector<float>(3,5));
  // fm2.new_feature_vector();
  // fm2.add_sensor_feature_vector(std::vector<float>(3,6)); 
  // fm2.add_flag(1);
  // fm2.add_flag(0);
  // fm2.add_flag(1);
  // fm2.add_flag(0);

  // fm2.print();

  // fm2.normalize();

  // std::ofstream ofile("/home/amayima/test.dat");
  // HTKHeader header;
  // header.BytesPerSample = fm.get_feature_vector_size()*4; 
  // header.nSamples = fm.get_samples_number();
  // header.Period = 100000;
  // header.FeatureType = HTK_USER;
  // header.write_to_file(ofile);
  // fm.write_to_file(ofile); 
  // ofile.close();  // verification of correct writing in binary file with HList -h command

  /* ----- Test Labels ----- */
  // std::vector<std::string> lab_vec;
  // lab_vec.push_back("stir");
  // lab_vec.push_back("put");
  // Labels labels(lab_vec,"/home/amayima/test.list","/home/amayima/test.net.slf","/home/amayima/test.dict",  "/home/amayima/test.grammar");
  // labels.add_label("take");
  // labels.add_label("pour");
  // // labels.set_labels_list_path("/home/amayima/test.list");
  // // labels.set_grammar_net_path("/home/amayima/test.net.slf");
  // // labels.set_dict_path("/home/amayima/test.dict"); 
  // // labels.set_grammar_path("/home/amayima/test.grammar");
  // labels.write_to_file(LabelFileFormats::txt);
  // labels.write_to_file(LabelFileFormats::grammar);
  // labels.write_to_file(LabelFileFormats::dict);
  // try{
  //   std::cout << labels.compile_grammar() << std::endl;
  //   std::cout << labels.test_grammar() << std::endl;
  // }catch(std::string const& error){
  //   std::cerr << error << std::endl;
  // }


 /* ----- Test HMMs ----- */
  // HMM hmm("take", 18, EmissionTypes::Gaussian);
  // hmm.write_to_file(std::string("/home/amayima/test.hmm"));

  /* ----- Test DataHandler ----- */
  DataHandler datah;
  // datah.raw_data_from_file_to_feature_matrices("/home/amayima/hmm_tools/HTK_actionRecognition/demo_breakfast/segmentation(copy)","/home/amayima/hmm_tools/HTK_actionRecognition/demo_breakfast/breakfast_data(copy)/s1", NormalizationTypes::no);
  datah.raw_data_from_file_to_feature_matrices("/home/amayima/catkin_ws/src/ActionRecognition/test_data_handler/segmentation","/home/amayima/catkin_ws/src/ActionRecognition/test_data_handler/data", NormalizationTypes::no);
  datah.print_map();
  datah.normalize();
  datah.print_map();
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
