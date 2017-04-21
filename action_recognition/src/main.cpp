#include <ros/ros.h>
#include <iostream>

//#include <sys/stat.h>
//#include "action_recognition/config.h"
//#include "action_recognition/load_features.h"
//#include <boost/filesystem.hpp>

#include "action_recognition/Vector3D.hpp"

//#include <eigen3/Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.h>
#include "action_recognition/SensorFeatureVector.hpp"
#include "action_recognition/FeatureVector.hpp"

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
  
  // Vector3D vector3D_2(1.2,1.0,1.0);
  // std::cout <<"vector2 "<< vector3D_2.get_x() << std::endl;
  //  vector3D_2.set_new_values(0,0,0);
  // std::cout <<"vector2 "<< vector3D_2.get_x() << std::endl;

  // std::vector<double> vec(3,1);
  // Vector3D vector3D_3(vec);

  // std::vector<double> vec2(2,1);
  // Vector3D vector3D_4(vec2);

  /* ------ Tests of SensorFeatureVector class ------ */
 // double w = 3.1212121211211211111111111111111111111111;
 //  Eigen::Quaternion<double> quat(w,1,1,1);
 //  std::cout.precision(17);
 //  std::cout<<std::fixed<<quat.w()<<std::endl;

  // double w = 3.1212121211211211111111111111111111111111;
  // std::cout.precision(17);
  // tf2::Quaternion quat(30,1,1,w); 
  // Vector3D vector3D(1.2,1.0,1.0);
  // //std::cout<<quat.getW()<<std::endl;
  // SensorFeatureVector sfv(vector3D, quat);

  // std::pair<std::vector<double>::iterator,
  //         std::vector<double>::iterator> it_pair = sfv.get_values_pair_iterator();
  // for(; it_pair.first != it_pair.second ; it_pair.first++)
  //   std::cout<<*it_pair.first<<std::endl;


  // SensorFeatureVector sfv2(1,2,3, quat);
  // SensorFeatureVector sfv3(1,2,3,4,5,6,7);
  // std::cout<<sfv.get_quaternion().getW()<<std::endl;
  // std::cout<<sfv.get_vector3D().get_x()<<std::endl;
  // std::cout<<sfv2.get_quaternion().getW()<<std::endl;
  // std::cout<<sfv2.get_vector3D().get_x()<<std::endl;
  // std::cout<<sfv3.get_quaternion().getW()<<std::endl;
  // std::cout<<sfv3.get_vector3D().get_x()<<std::endl;
  // SensorFeatureVector sfv_n = sfv.normalize();
  // std::cout<<sfv_n.get_quaternion().getW()<<std::endl;
  // std::cout<<sfv_n.get_vector3D().get_x()<<std::endl;
  // std::cout<<sfv_n.get_quaternion().getAxis().getX()<<std::endl;

  // it_pair = sfv_n.get_values_pair_iterator();
  // std::cout<<"normalize"<<std::endl;
  // for(; it_pair.first != it_pair.second ; it_pair.first++)
  //   std::cout<<*it_pair.first<<std::endl;


  /* ------ Tests of FeatureVector class ------ */
  // FeatureVector fv(3);
  // SensorFeatureVector sfv3(1,2,3,4,5,6,7);
  // std::vector<SensorFeatureVector> feature_vector(2);
  // feature_vector[0] = sfv3;
  // std::cout<<feature_vector.size()<<std::endl;
  // feature_vector.push_back(sfv3);
  // std::cout<<feature_vector.size()<<std::endl;
  // FeatureVector fv1(2,feature_vector);

  // FeatureVector fv;
  // SensorFeatureVector sfv3(1,2,3,4,5,6,7);
  // //std::cout<<fv.get_element_number()<<std::endl;
  // fv.add_sensor_feature_vector(sfv3);
  // //std::cout<<fv.get_element_number()<<std::endl;

  // SensorFeatureVector sfv(1,2,3,4,5,6,8);
  // std::vector<SensorFeatureVector> feature_vector;
  // feature_vector.push_back(sfv3);
  // feature_vector.push_back(sfv);
  // FeatureVector fv2(feature_vector);
  // std::pair<std::vector<SensorFeatureVector>::iterator,
  //           std::vector<SensorFeatureVector>::iterator>it_pair = fv2.get_pair_iterator();

  // std::pair<std::vector<double>::iterator,
  //         std::vector<double>::iterator>it_pair2;
  // int i = 0;
  // for(; it_pair.first != it_pair.second ; it_pair.first++){
  //   it_pair2 =it_pair.first->get_values_pair_iterator();
  //   std::cout<<"tour "<<i++<<std::endl;
  //   for(; it_pair2.first != it_pair2.second ; it_pair2.first++)
  //      std::cout<<*it_pair2.first<<std::endl;
  // }
  
  // FeatureVector fv3 = fv2.normalize();
  // it_pair = fv3.get_pair_iterator();
  // for(; it_pair.first != it_pair.second ; it_pair.first++){
  //   it_pair2 =it_pair.first->get_values_pair_iterator();
  //   std::cout<<"tour "<<i++<<std::endl;
  //   for(; it_pair2.first != it_pair2.second ; it_pair2.first++)
  //      std::cout<<*it_pair2.first<<std::endl;
  // }

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
