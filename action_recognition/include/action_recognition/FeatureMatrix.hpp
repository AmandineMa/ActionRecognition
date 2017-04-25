#ifndef FEATUREMATRIX_HPP
#define	FEATUREMATRIX_HPP

#include <vector>
#include <string>

#include "action_recognition/FeatureVector.hpp"

class FeatureMatrix{
private:
  std::string label_;
  std::vector<FeatureVector> feature_vector_array_;
    
public:
  FeatureMatrix();
  FeatureMatrix(std::string label);
  FeatureMatrix(std::string label, std::vector<FeatureVector> feature_vector_array);

  void add_feature_vector(FeatureVector feature_vector);

  void set_label(std::string label);
  std::string get_label(void);

  int get_feature_vector_size(void);

  int get_samples_number(void);

  void write_to_file(std::ofstream &os);

 
};

#endif
