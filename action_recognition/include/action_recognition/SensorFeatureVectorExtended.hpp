#ifndef SENSORFEATUREVECTOREXTENDED_HPP
#define	SENSORFEATUREVECTOREXTENDED_HPP
#include <vector>
#include <fstream>
#include <tf2/LinearMath/Quaternion.h>
#include "action_recognition/Vector3D.hpp"
#include "action_recognition/common.hpp"
#include "action_recognition/SensorFeatureVector.hpp"

#define SENSOR_FEATURE_VECTOR_EXTENDED_SIZE 7

class SensorFeatureVectorExtended : public SensorFeatureVector {

private:
  tf2::Quaternion quaternion_;

public:
  SensorFeatureVectorExtended(Vector3D vector3D, tf2::Quaternion quaternion);   
  SensorFeatureVectorExtended(float x, float y, float z, tf2::Quaternion quaternion); 
  SensorFeatureVectorExtended(float x, float y, float z, float x_q, float y_q, float z_q, float w);
  SensorFeatureVectorExtended(std::vector<float> values_vector_);

  tf2::Quaternion get_quaternion(void);

  SensorFeatureVectorExtended normalize(void);

  void write_to_file(std::ofstream &os);

  void print_vector(void);

  int get_size(void);

};
#endif
