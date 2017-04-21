#ifndef SENSORFEATUREVECTOR_HPP
#define	SENSORFEATUREVECTOR_HPP

#include <vector>
#include <tf2/LinearMath/Quaternion.h>

#include "action_recognition/Vector3D.hpp"

#define SENSOR_FEATURE_VECTOR_SIZE 7

class SensorFeatureVector{

private:
  Vector3D translation_;
  tf2::Quaternion quaternion_;
  std::vector<double> values_vector_;

  void set_new_vector_values(void);
  void set_new_vector_values(double x, double y, double z, double x_q, double y_q, double z_q, double w);

public:
  SensorFeatureVector();
  SensorFeatureVector(Vector3D vector3D, tf2::Quaternion quaternion);
  SensorFeatureVector(double x, double y, double z, tf2::Quaternion quaternion);
  SensorFeatureVector(double x, double y, double z, double x_q, double y_q, double z_q, double w);

  Vector3D get_vector3D(void);
  tf2::Quaternion get_quaternion(void);
  std::pair<std::vector<double>::iterator,
            std::vector<double>::iterator> get_values_pair_iterator(void);

  SensorFeatureVector normalize(void);

};

#endif
