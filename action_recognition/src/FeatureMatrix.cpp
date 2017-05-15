#include <vector>
#include <string>
#include <fstream>
#include <iostream>

#include "action_recognition/SensorFeatureVector.hpp"
#include "action_recognition/SensorFeatureVectorExtended.hpp"
#include "action_recognition/FeatureVector.hpp"
#include "action_recognition/FeatureMatrix.hpp"
#include "action_recognition/common.hpp"

FeatureMatrix::FeatureMatrix():feature_vector_array_(0){}

FeatureMatrix::FeatureMatrix(std::string label):label_(label),feature_vector_array_(0){}

void FeatureMatrix::set_label(std::string label){label_=label;}
std::string FeatureMatrix::get_label(void) const{return label_;}

int FeatureMatrix::get_feature_vector_size(void) const{
  if(feature_vector_array_.empty())
    return 0;
  else
    return feature_vector_array_[0].get_size();
}

int FeatureMatrix::get_samples_number(void) const{return feature_vector_array_.size();}

void FeatureMatrix::write_to_file(std::ofstream &os) const{
 std::vector<FeatureVector>::const_iterator it = feature_vector_array_.begin();
  for(; it != feature_vector_array_.end() ; it++)
    it->write_to_file(os);
}

void FeatureMatrix::new_feature_vector(void){
  feature_vector_array_.emplace_back(); //C++11
}

void FeatureMatrix::new_feature_vector(std::vector<float> flag_vector){
  feature_vector_array_.emplace_back(flag_vector); //C++11
}

void FeatureMatrix::add_sensor_feature_vector(std::vector<float> values_vector){
  feature_vector_array_.back().add_sensor_feature_vector(values_vector);
}

void FeatureMatrix::add_flag(float flag){
  feature_vector_array_.back().add_flag(flag);
}

void FeatureMatrix::set_flags(std::vector<float> flags){
  feature_vector_array_.back().set_flags(flags);
}

void FeatureMatrix::normalize(NormalizationType normalization_type){
  std::vector<FeatureVector>::iterator it = feature_vector_array_.begin();
  for( ; it != feature_vector_array_.end() ; it++)
    it->normalize(normalization_type);
}

void FeatureMatrix::print(void){
 std::vector<FeatureVector>::iterator it = feature_vector_array_.begin();
 std::cout << label_ << std::endl;
  for( ; it != feature_vector_array_.end() ; it++)
    it->print_vector();
}
