#include <vector>

#include "action_recognition/SensorFeatureVector.hpp"
#include "action_recognition/FeatureVector.hpp"

FeatureVector::FeatureVector(){}

FeatureVector::FeatureVector(std::vector<SensorFeatureVector> feature_vector):feature_vector_(feature_vector){}

// FeatureVector::FeatureVector(int feature_vector_size):feature_vector_size_(feature_vector_size),feature_vector_(feature_vector_size){}

// FeatureVector::FeatureVector(int feature_vector_size, std::vector<SensorFeatureVector> feature_vector):feature_vector_size_(feature_vector_size){
//  try{
//     if(feature_vector.size() != feature_vector_size_)
//       throw std::string("The feature vector has to be initialized with a vector of size "+feature_vector_size_);
//     else
//       std::vector<SensorFeatureVector> feature_vector_(feature_vector);
//   }catch(std::string const& error){
//     std::cerr<<error<<std::endl;
//   }
// }

void FeatureVector::add_sensor_feature_vector(SensorFeatureVector sensor_feature_vector){
  feature_vector_.push_back(sensor_feature_vector);
}

FeatureVector FeatureVector::normalize(){
  std::vector<SensorFeatureVector> fv(feature_vector_.size());
  std::vector<SensorFeatureVector>::iterator it = feature_vector_.begin();
  int i = 0;
  for( ; it != feature_vector_.end() ; it++)
    fv[i++]=it->normalize();
  return fv;
}

int FeatureVector::get_element_number(void){return feature_vector_.size();}

std::pair<std::vector<SensorFeatureVector>::iterator,
          std::vector<SensorFeatureVector>::iterator> FeatureVector::get_pair_iterator(void){
  return std::make_pair(feature_vector_.begin(),feature_vector_.end());
}
