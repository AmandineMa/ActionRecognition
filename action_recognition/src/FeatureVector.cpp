#include <vector>
#include <iostream>

#include "action_recognition/SensorFeatureVector.hpp"
#include "action_recognition/FeatureVector.hpp"
#include "action_recognition/common.hpp"

FeatureVector::FeatureVector(){}

FeatureVector::~FeatureVector(){
  for(std::vector<SensorFeatureVector*>::iterator it = feature_vector_.begin(); it != feature_vector_.end() ; it++)
    delete (*it);
  feature_vector_.clear();
}

FeatureVector::FeatureVector(std::vector<SensorFeatureVector *> feature_vector, std::vector<float> flag_vector):
  feature_vector_(feature_vector), flag_vector_(flag_vector){}

FeatureVector::FeatureVector(std::vector<float> flag_vector):flag_vector_(flag_vector){}

void FeatureVector::add_sensor_feature_vector(SensorFeatureVector * sensor_feature_vector){
  feature_vector_.push_back(sensor_feature_vector); 
}

void FeatureVector::add_flag(float flag){
  flag_vector_.push_back(flag); 
}

void FeatureVector::normalize(){
  std::vector<SensorFeatureVector*>::iterator it = feature_vector_.begin();
  int i = 0;
  for( ; it != feature_vector_.end() ; it++)
    (*it)->normalize();
}

void FeatureVector::write_to_file(std::ofstream &os){
  std::vector<SensorFeatureVector*>::iterator it = feature_vector_.begin();
  for(; it != feature_vector_.end() ; it++)
    (*it)->write_to_file(os);
  tools::swap_endian(flag_vector_.begin(), flag_vector_.end());
  os.write((char *)&flag_vector_[0], flag_vector_.size()*sizeof(float));
}

void FeatureVector::print_vector(void){
 std::vector<SensorFeatureVector*>::iterator it_feature = feature_vector_.begin(); 
 std::vector<float>::iterator it_flag = flag_vector_.begin();
 for(; it_feature != feature_vector_.end() ; it_feature++){
   (*it_feature)->print_vector();
 }
  for(; it_flag != flag_vector_.end() ; it_flag++)
    std::cout << *it_flag << " ";
  std::cout << "\n";

}
