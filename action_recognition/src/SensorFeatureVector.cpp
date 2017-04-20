#include <tf2/LinearMath/Quaternion.h>

#include "action_recognition/SensorFeatureVector.hpp"
#include "action_recognition/common.hpp"

SensorFeatureVector::SensorFeatureVector():translation_(),quaternion_(),values_vector_(SENSOR_FEATURE_VECTOR_SIZE){}

SensorFeatureVector::SensorFeatureVector(Vector3D vector3D, tf2::Quaternion quaternion):values_vector_(SENSOR_FEATURE_VECTOR_SIZE){
  translation_ = vector3D;
  quaternion_ = quaternion;
  set_new_vector_values();
}

SensorFeatureVector::SensorFeatureVector(double x, double y, double z, tf2::Quaternion quaternion):translation_(x,y,z),values_vector_(SENSOR_FEATURE_VECTOR_SIZE){
  quaternion_ = quaternion;
  set_new_vector_values();
} 

SensorFeatureVector::SensorFeatureVector(double x, double y, double z, double x_q, double y_q, double z_q, double w):translation_(x,y,z),quaternion_(x_q, y_q, z_q, w),values_vector_(SENSOR_FEATURE_VECTOR_SIZE){
  set_new_vector_values();
}

void SensorFeatureVector::set_new_vector_values(void){
  values_vector_[VectorElements::X]=translation_.get_x();
  values_vector_[VectorElements::Y]=translation_.get_y();
  values_vector_[VectorElements::Z]=translation_.get_z();
  values_vector_[VectorElements::X_Q]=quaternion_.getX();
  values_vector_[VectorElements::Y_Q]=quaternion_.getY();
  values_vector_[VectorElements::Z_Q]=quaternion_.getZ();
  values_vector_[VectorElements::W]=quaternion_.getW();
}

void SensorFeatureVector::set_new_vector_values(double x, double y, double z, double x_q, double y_q, double z_q, double w){
  values_vector_[VectorElements::X]=x;
  values_vector_[VectorElements::Y]=y;
  values_vector_[VectorElements::Z]=z;
  values_vector_[VectorElements::X_Q]=x_q;
  values_vector_[VectorElements::Y_Q]=y_q;
  values_vector_[VectorElements::Z_Q]=z_q;
  values_vector_[VectorElements::W]=w;
}

Vector3D SensorFeatureVector::get_vector3D(void){return translation_;}
tf2::Quaternion SensorFeatureVector::get_quaternion(void){return quaternion_;}
std::pair<std::vector<double>::iterator,
          std::vector<double>::iterator> SensorFeatureVector::get_values_pair_iterator(void){
  return std::make_pair(values_vector_.begin(),values_vector_.end());
}

SensorFeatureVector SensorFeatureVector::normalize(void){
  SensorFeatureVector sfv(translation_.normalize(), quaternion_.normalize());
  return sfv;
}

//void set_vector3D(Vector3D vector3D){}
