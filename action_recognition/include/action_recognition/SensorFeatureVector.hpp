#ifndef SENSORFEATUREVECTOR_HPP
#define	SENSORFEATUREVECTOR_HPP
#include <vector>
#include <fstream>
#include "action_recognition/Vector3D.hpp"
#include "action_recognition/common.hpp"

#define SENSOR_FEATURE_VECTOR_SIZE 3

class SensorFeatureVector{

protected:
  Vector3D vector3D_;

public:
  SensorFeatureVector();
  SensorFeatureVector(Vector3D vector3D);
  SensorFeatureVector(std::vector<float> values_vector_);
  SensorFeatureVector(float x, float y, float z);
  
  Vector3D get_vector3D(void);

  SensorFeatureVector normalize(void);

  void write_to_file(std::ofstream &os);

  void print_vector(void);

  int get_size(void);

};
#endif
