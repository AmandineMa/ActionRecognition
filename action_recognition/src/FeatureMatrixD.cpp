#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <mutex>
#include <ros/console.h>

#include "action_recognition/SensorFeatureVector.hpp"
#include "action_recognition/SensorFeatureVectorExtended.hpp"
#include "action_recognition/FeatureVector.hpp"
#include "action_recognition/FeatureMatrixD.hpp"
#include "action_recognition/common.hpp"

FeatureMatrixD::FeatureMatrixD():feature_vector_array_(0){}



int FeatureMatrixD::get_samples_number(void) const{
  return feature_vector_array_.size();
}

void FeatureMatrixD::write_to_file(std::ofstream &os) const{
  std::lock_guard<std::mutex> lock(mutex_);
  //Write HTK Header so that the file is comprehensible by HTK Toolkit
  tools::write_HTK_header_to_file(os, get_feature_vector_size(), 
                                  get_samples_number());
  std::deque<FeatureVector>::const_iterator it = feature_vector_array_.begin();
  //Write each FeatureVector to the file
  for(; it != feature_vector_array_.end() ; it++)
    it->write_to_file(os, FeatureFileFormat::dat); 
  os.close(); 
}

void FeatureMatrixD::new_feature_vector(void){
  std::lock_guard<std::mutex> lock(mutex_);
  feature_vector_array_.emplace_back(); //C++11
}

void FeatureMatrixD::new_feature_vector(const std::vector<float> &flag_vector){
  std::lock_guard<std::mutex> lock(mutex_);
  feature_vector_array_.emplace_back(flag_vector); //C++11
}

void FeatureMatrixD::pop_feature_vector(void){
  std::lock_guard<std::mutex> lock(mutex_);
  feature_vector_array_.pop_front();
}

void FeatureMatrixD::pop_feature_vectors(int n){
  std::lock_guard<std::mutex> lock(mutex_);
  int original_size = feature_vector_array_.size();
  while(feature_vector_array_.size() > original_size-n)
    feature_vector_array_.pop_front();
}

void FeatureMatrixD::add_sensor_feature_vector(const std::vector<float> &values_vector){
  std::lock_guard<std::mutex> lock(mutex_);
  feature_vector_array_.back().add_sensor_feature_vector(values_vector);
}

void FeatureMatrixD::add_flag(float flag){
  std::lock_guard<std::mutex> lock(mutex_);
  feature_vector_array_.back().add_flag(flag);
}

void FeatureMatrixD::set_flags(const std::vector<float> &flags){
  std::lock_guard<std::mutex> lock(mutex_);
  feature_vector_array_.back().set_flags(flags);
}

void FeatureMatrixD::normalize(NormalizationType normalization_type){
  std::lock_guard<std::mutex> lock(mutex_);
  std::deque<FeatureVector>::iterator it = feature_vector_array_.begin();
  for( ; it != feature_vector_array_.end() ; it++)
    it->normalize(normalization_type);
}

void FeatureMatrixD::print(void){
  std::lock_guard<std::mutex> lock(mutex_);
 std::deque<FeatureVector>::iterator it = feature_vector_array_.begin();
  for( ; it != feature_vector_array_.end() ; it++)
    it->print_vector();
}

/***** Private function ******/

int FeatureMatrixD::get_feature_vector_size(void) const{
  if(feature_vector_array_.empty())
    return 0;
  else
    return feature_vector_array_.front().get_size();
}
