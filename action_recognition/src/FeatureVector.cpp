#include <vector>
#include <iostream>
#include <typeinfo>
#include <memory>

#include "action_recognition/SensorFeatureVector.hpp"
#include "action_recognition/SensorFeatureVectorExtended.hpp"
#include "action_recognition/FeatureVector.hpp"
#include "action_recognition/common.hpp"

FeatureVector::FeatureVector():feature_vector_size_(0){}

FeatureVector::FeatureVector(std::vector<float> flag_vector):
  flag_vector_(flag_vector), feature_vector_size_(flag_vector.size()){}


void FeatureVector::add_sensor_feature_vector(std::vector<float> values_vector){
  feature_vector_size_ += values_vector.size();
  switch(values_vector.size()){
    case SENSOR_FEATURE_VECTOR_SIZE:
      sensor_feature_vectors_.push_back(std::unique_ptr<SensorFeatureVector>
                                        (new SensorFeatureVector(values_vector))); //C++11
      break;
    case SENSOR_FEATURE_VECTOR_EXTENDED_SIZE:
      sensor_feature_vectors_.push_back(std::unique_ptr<SensorFeatureVector>
                                        (new SensorFeatureVectorExtended(values_vector))); //C++11
      break;
    default:
      break;
  }
}

void FeatureVector::add_flag(float flag){
  flag_vector_.push_back(flag); 
  feature_vector_size_ += 1;
}

void FeatureVector::set_flags(std::vector<float> flags){
  feature_vector_size_ = feature_vector_size_ - flag_vector_.size() + flags.size();
  flag_vector_ = flags;
}

void FeatureVector::normalize(NormalizationType normalization_type){
  std::vector<std::unique_ptr<SensorFeatureVector> >::iterator it = sensor_feature_vectors_.begin();
  for( ; it != sensor_feature_vectors_.end() ; it++)
    (*it)->normalize(normalization_type);
}

void FeatureVector::write_to_file(std::ofstream &os, FeatureFileFormat file_format) const{
  switch(file_format){
    case FeatureFileFormat::dat:{
      std::vector<std::unique_ptr<SensorFeatureVector> >::const_iterator it = sensor_feature_vectors_.begin();
      for(; it != sensor_feature_vectors_.end() ; it++)
        (*it)->write_to_file(os, file_format);
      os.write((char *)&flag_vector_[0], flag_vector_.size()*sizeof(float));
      break;
    }
    case FeatureFileFormat::lab:{
      std::vector<std::unique_ptr<SensorFeatureVector> >::const_iterator it = sensor_feature_vectors_.begin();
      for(; it != sensor_feature_vectors_.end() ; it++)
        (*it)->write_to_file(os, file_format); 
      os << "<Flags>";
      std::ostream_iterator<float> output_iterator(os, " ");
      std::copy(flag_vector_.begin(), flag_vector_.end(), output_iterator); 
      os << "</Flags>";
      break;
    }
    default:
      break;
  }
}

void FeatureVector::print_vector(void){
  // Write the vector with the format [ SensorFeatureVector_0, SensorFeatureVector_1, ..., SensorFeatureVector_k, Flag_0, ..., Flag_i ]
  std::vector<std::unique_ptr<SensorFeatureVector> >::iterator it_feature = sensor_feature_vectors_.begin(); 
  std::vector<float>::iterator it_flag = flag_vector_.begin();
  for(; it_feature != sensor_feature_vectors_.end() ; it_feature++){
    (*it_feature)->print_vector();
  }
  for(; it_flag != flag_vector_.end() ; it_flag++)
    std::cout << *it_flag << " ";
  std::cout << "\n";
}

int FeatureVector::get_size(void) const{return feature_vector_size_;}
