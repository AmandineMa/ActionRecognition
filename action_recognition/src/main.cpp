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
  /*
  Config config("/home/amayima/tests/","/home/amayima/catkin_ws/src/turtlesim_cleaner/src/input/");
  FeaturesLoader f_loader;
  f_loader.load_features(config.path_data, config.path_seg_files);
  */

  /* ------ Tests of Vector3D class ------ */
  //  Vector3D vector3D_1;
  //  vector3D_1.set_x(1.0);
  // vector3D_1.set_y(1.0);
  // vector3D_1.set_z(1.0);
  // std::cout << vector3D_1.get_x() << "  "<<vector3D_1.get_y() << "  "<<vector3D_1.get_z() << "  "<<std::endl;
  
  //Vector3D vector3D_2(1.2,1.0,1.0);
  // std::cout <<"vector2 "<< vector3D_2.get_x() << std::endl;
  //  vector3D_2.set_new_values(0,0,0);
  // std::cout <<"vector2 "<< vector3D_2.get_x() << std::endl;

  // std::vector<float> vec(3,1);
  // Vector3D vector3D_3(vec);

  // std::vector<float> vec2(2,1);
  // Vector3D vector3D_4(vec2);
 

  /* ------ Tests of SensorFeatureVector class ------ */
  // float w = 3.12121212;
  // std::cout.precision(6);
  // tf2::Quaternion quat(4,5,6,7); 
  // Vector3D vector3D(1.2,1.0,1.0);
  // SensorFeatureVectorExtended sfv(vector3D, quat);
  // std::cout << quat.getY() << std::endl;
  // sfv.print_vector();
  // SensorFeatureVectorExtended sfv2(1,2,3, quat);
  // sfv2.print_vector();
  // SensorFeatureVectorExtended sfv3(1,2,3,4,5,6,7);
  // sfv3.print_vector();
  // SensorFeatureVectorExtended sfv4(std::vector<float>(7,100));
  // sfv4.print_vector();
  // SensorFeatureVectorExtended sfv_n = sfv.normalize();
  // sfv_n.print_vector();

  // SensorFeatureVector sfv5(vector3D);
  // sfv5.print_vector();
  // SensorFeatureVector sfv6(1,2,3);
  // sfv6.print_vector();
  // SensorFeatureVector sfv7(std::vector<float>(3,100));
  // sfv7.print_vector();
  // SensorFeatureVector sfv_n2 = sfv5.normalize();
  // sfv_n2.print_vector();
  // std::ofstream ofile("/home/amayima/test.dat");

  // HTKHeader header;
  // header.BytesPerSample =  sfv_n2.get_size()*4; 
  // header.nSamples = 1;
  // header.Period = 100000;
  // header.FeatureType = HTK_USER;
  // header.write_to_file(ofile);
  // sfv_n2.write_to_file(ofile); 
  // ofile.close(); // verification of correct writing in binary file with HList -h command

  /* ------ Tests of FeatureVector class ------ */
  SensorFeatureVector sfv(1,2,3);
  SensorFeatureVectorExtended sfv2(4,5,6,7,8,9,10);
  std::vector<SensorFeatureVector*> feature_vector;
  feature_vector.push_back(new SensorFeatureVector(1,2,3));
  feature_vector.push_back(new SensorFeatureVectorExtended(4,5,6,7,8,9,10));
  FeatureVector fv(feature_vector,std::vector<float>(2,2));
  fv.print_vector();
 
  fv.add_sensor_feature_vector(new SensorFeatureVector(11,12,13));
  fv.add_sensor_feature_vector(new SensorFeatureVectorExtended(14,15,16,17,18,19,20));
  fv.add_flag(1);
  fv.print_vector();
  fv.normalize();
  fv.print_vector();

  std::ofstream ofile("/home/amayima/test.dat");

  HTKHeader header;
  header.BytesPerSample = 12*4; 
  header.nSamples = 1;
  header.Period = 100000;
  header.FeatureType = HTK_USER;
  header.write_to_file(ofile);
  fv.write_to_file(ofile); 
  ofile.close(); // verification of correct writing in binary file with HList -h command 

  /* ------ Tests of FeatureMatrix class ------ */
  // SensorFeatureVector sfv3(1,2,3,4,5,6,7);
  // SensorFeatureVector sfv(1,2,3,4,5,6,8);
  // std::vector<SensorFeatureVector> feature_vector;
  // feature_vector.push_back(sfv3);
  // feature_vector.push_back(sfv);
  // FeatureVector fv2(feature_vector);

  // FeatureVector fv4;
  // fv4.add_sensor_feature_vector(sfv);
  // fv4.add_sensor_feature_vector(sfv3);
  // std::vector<FeatureVector> feature_matrix;
  // feature_matrix.push_back(fv2);
  // feature_matrix.push_back(fv4);
  // FeatureMatrix fm("take", feature_matrix);

  // FeatureMatrix fm2("take");
  // fm2.add_feature_vector(fv2);
  // fm2.add_feature_vector(fv4);

  // std::ofstream ofile("/home/amayima/test.dat");
  // HTKHeader header;
  // header.BytesPerSample = fm.get_feature_vector_size()*4; 
  // header.nSamples = fm.get_samples_number();
  // header.Period = 100000;
  // header.FeatureType = HTK_USER;
  // header.write_to_file(ofile);
  // fm2.write_to_file(ofile); 
  // ofile.close(); // verification of correct writing in binary file with HList -h command

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
  // DataHandler datah;
  // datah.raw_data_from_file_to_feature_matrices("/home/amayima/hmm_tools/HTK_actionRecognition/demo_breakfast/segmentation(copy)","/home/amayima/hmm_tools/HTK_actionRecognition/demo_breakfast/breakfast_data(copy)/s1", NormalizationTypes::no);


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
