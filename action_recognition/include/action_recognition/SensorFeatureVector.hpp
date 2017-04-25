#ifndef SENSORFEATUREVECTOR_HPP
#define	SENSORFEATUREVECTOR_HPP
#include <vector>
#include <fstream>
#include <tf2/LinearMath/Quaternion.h>
#include "action_recognition/Vector3D.hpp"
#include "action_recognition/common.hpp"

#define SENSOR_FEATURE_VECTOR_SIZE 7

class SensorFeatureVector{

private:
  Vector3D translation_;
  tf2::Quaternion quaternion_;
  std::vector<float> values_vector_;

  void set_new_vector_values(void);
  void set_new_vector_values(float x, float y, float z, float x_q, float y_q, float z_q, float w);

public:
  SensorFeatureVector();
  SensorFeatureVector(Vector3D vector3D, tf2::Quaternion quaternion);
  SensorFeatureVector(float x, float y, float z, tf2::Quaternion quaternion);
  SensorFeatureVector(float x, float y, float z, float x_q, float y_q, float z_q, float w);

  Vector3D get_vector3D(void);
  tf2::Quaternion get_quaternion(void);
  std::pair<std::vector<float>::iterator,
            std::vector<float>::iterator> get_values_pair_iterator(void);

  int get_values_vector_size(void);

  SensorFeatureVector normalize(void);

  void write_to_file(std::ofstream &os);

};
#endif
