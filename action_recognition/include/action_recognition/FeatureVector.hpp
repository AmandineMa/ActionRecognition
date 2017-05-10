#ifndef FEATUREVECTOR_HPP
#define	FEATUREVECTOR_HPP

#include <vector>
#include <memory>
#include <fstream>
#include "action_recognition/SensorFeatureVector.hpp"

//#define FEATURE_VECTOR_SIZE 2

class FeatureVector{

private:
  std::vector<std::unique_ptr<SensorFeatureVector>> sensor_feature_vectors_;
  std::vector<float> flag_vector_; //float for compatibility with HTK
  int feature_vector_size_;

  int element_number_sensor_feature_vector(SensorFeatureVector sfv);

public:
  FeatureVector();
  FeatureVector(std::vector<float> flag_vector);

  void add_sensor_feature_vector(std::vector<float> values_vector);
  void add_flag(float flag);
  void add_flags(std::vector<float> flags);

  int get_size(void) const;

  void normalize(void);

  void write_to_file(std::ofstream &os) const;

  void print_vector(void);

};

#endif
