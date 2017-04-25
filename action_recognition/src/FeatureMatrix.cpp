#include <vector>
#include <string>
#include <fstream>

#include "action_recognition/FeatureVector.hpp"
#include "action_recognition/FeatureMatrix.hpp"
#include "action_recognition/common.hpp"

FeatureMatrix::FeatureMatrix():feature_vector_array_(0){}

FeatureMatrix::FeatureMatrix(std::string label):label_(label),feature_vector_array_(0){}

FeatureMatrix::FeatureMatrix(std::string label, std::vector<FeatureVector> feature_vector_array):label_(label),feature_vector_array_(feature_vector_array){}

void FeatureMatrix::add_feature_vector(FeatureVector feature_vector){
  feature_vector_array_.push_back(feature_vector);
}

void FeatureMatrix::set_label(std::string label){label_=label;}
std::string FeatureMatrix::get_label(void){return label_;}

int FeatureMatrix::get_feature_vector_size(void){
  if(feature_vector_array_.empty())
    return 0;
  else
    return feature_vector_array_[0].get_values_number();
}

int FeatureMatrix::get_samples_number(void){return feature_vector_array_.size();}

void FeatureMatrix::write_to_file(std::ofstream &os){
 std::vector<FeatureVector>::iterator it = feature_vector_array_.begin();
  for(; it != feature_vector_array_.end() ; it++)
    it->write_to_file(os);
}
