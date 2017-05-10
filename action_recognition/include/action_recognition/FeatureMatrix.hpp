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

  void new_feature_vector(void);

  void new_feature_vector(std::vector<float> flag_vector);

  void add_sensor_feature_vector(std::vector<float> values_vector);

  void add_flag(float flag);

  void add_flags(std::vector<float> flags);

  void set_label(std::string label);

  std::string get_label(void) const;

  int get_feature_vector_size(void) const;

  int get_samples_number(void) const;

  void write_to_file(std::ofstream &os) const;

  void normalize(void);

  void print(void);

 
};

#endif
