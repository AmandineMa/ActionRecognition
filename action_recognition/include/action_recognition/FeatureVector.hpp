#ifndef FEATUREVECTOR_HPP
#define	FEATUREVECTOR_HPP

#include <vector>
#include <fstream>
#include "action_recognition/SensorFeatureVector.hpp"

//#define FEATURE_VECTOR_SIZE 2

class FeatureVector{

private:
  std::vector<SensorFeatureVector *> feature_vector_;
  std::vector<float> flag_vector_; //float for compatibility with HTK


public:
  FeatureVector();
  FeatureVector(std::vector<SensorFeatureVector *> feature_vector, std::vector<float> flag_vector);
  FeatureVector(std::vector<float> flag_vector);
  ~FeatureVector();

  void add_sensor_feature_vector(SensorFeatureVector * sensor_feature_vector);
  void add_flag(float flag);

  void normalize(void);

  void write_to_file(std::ofstream &os);

  void print_vector(void);

};

#endif
