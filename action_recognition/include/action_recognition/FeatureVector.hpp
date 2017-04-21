#ifndef FEATUREVECTOR_HPP
#define	FEATUREVECTOR_HPP

#include <vector>
#include "action_recognition/SensorFeatureVector.hpp"

//#define FEATURE_VECTOR_SIZE 2

class FeatureVector{

private:
  std::vector<SensorFeatureVector> feature_vector_;
  // int feature_vector_size_;

public:
  FeatureVector();
  FeatureVector(std::vector<SensorFeatureVector> feature_vector);
  // FeatureVector(int feature_vector_size);
  // FeatureVector(int feature_vector_size, std::vector<SensorFeatureVector> feature_vector);

  void add_sensor_feature_vector(SensorFeatureVector sensor_feature_vector);
  FeatureVector normalize(void);

  int get_element_number(void);

  std::pair<std::vector<SensorFeatureVector>::iterator,
            std::vector<SensorFeatureVector>::iterator> get_pair_iterator(void);

};

#endif
