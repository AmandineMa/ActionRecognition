#include <tf2/LinearMath/Quaternion.h>
#include <fstream>
#include <iostream> 
#include "action_recognition/SensorFeatureVector.hpp"
#include "action_recognition/common.hpp"

SensorFeatureVector::SensorFeatureVector(){}

SensorFeatureVector::SensorFeatureVector(Vector3D vector3D, tf2::Quaternion quaternion):values_vector_(VEC3D_QUAT_SIZE){
  translation_ = vector3D;
  quaternion_ = quaternion;
  set_new_vector_values();
}

SensorFeatureVector::SensorFeatureVector(Vector3D vector3D):values_vector_(VEC3D_SIZE){
  translation_ = vector3D;
  set_new_vector_values();
}

SensorFeatureVector::SensorFeatureVector(std::vector<float> values_vector):values_vector_(values_vector){
  int i = 0;
  translation_ = Vector3D(values_vector[i++],values_vector[i++],values_vector[i++]);
     
  if(values_vector.size() == VEC3D_QUAT_SIZE)
    quaternion_ = tf2::Quaternion(values_vector[i++],values_vector[i++],values_vector[i++], values_vector[i++]);
   
}

SensorFeatureVector::SensorFeatureVector(float x, float y, float z, tf2::Quaternion quaternion):
  translation_(x,y,z),values_vector_(VEC3D_QUAT_SIZE){
  quaternion_ = quaternion;
  set_new_vector_values();
} 

SensorFeatureVector::SensorFeatureVector(float x, float y, float z):
  translation_(x,y,z),values_vector_(VEC3D_SIZE){
  set_new_vector_values();
} 

SensorFeatureVector::SensorFeatureVector(float x, float y, float z, float x_q, float y_q, float z_q, float w):
  translation_(x,y,z),quaternion_(x_q, y_q, z_q, w),values_vector_(VEC3D_QUAT_SIZE){
  set_new_vector_values();
}

void SensorFeatureVector::set_new_vector_values(void){

  values_vector_[VectorElements::X]=translation_.get_x();
  values_vector_[VectorElements::Y]=translation_.get_y();
  values_vector_[VectorElements::Z]=translation_.get_z();
  if(values_vector_.size() == VEC3D_QUAT_SIZE){
    values_vector_[VectorElements::X_Q]=quaternion_.getX();
    values_vector_[VectorElements::Y_Q]=quaternion_.getY();
    values_vector_[VectorElements::Z_Q]=quaternion_.getZ();
    values_vector_[VectorElements::W]=quaternion_.getW();
  }
}

void SensorFeatureVector::set_new_vector_values(float x, float y, float z, float x_q, float y_q, float z_q, float w){
  values_vector_[VectorElements::X]=x;
  values_vector_[VectorElements::Y]=y;
  values_vector_[VectorElements::Z]=z;
  values_vector_[VectorElements::X_Q]=x_q;
  values_vector_[VectorElements::Y_Q]=y_q;
  values_vector_[VectorElements::Z_Q]=z_q;
  values_vector_[VectorElements::W]=w;
}

void SensorFeatureVector::set_new_vector_values(float x, float y, float z){
  values_vector_[VectorElements::X]=x;
  values_vector_[VectorElements::Y]=y;
  values_vector_[VectorElements::Z]=z;
}

Vector3D SensorFeatureVector::get_vector3D(void){return translation_;}

tf2::Quaternion SensorFeatureVector::get_quaternion(void){
  if(quaternion_.length()==0) 
    std::cerr << "There is no quaternion initialized" << std::cout;
   return quaternion_;
}
std::pair<std::vector<float>::iterator,
          std::vector<float>::iterator> SensorFeatureVector::get_values_pair_iterator(void){
  return std::make_pair(values_vector_.begin(),values_vector_.end());
}

int SensorFeatureVector::get_values_vector_size(void){return values_vector_.size();}

SensorFeatureVector SensorFeatureVector::normalize(void){
  SensorFeatureVector sfv;
  if(values_vector_.size() == VEC3D_SIZE)
    sfv = SensorFeatureVector(translation_.normalize());
  else if(values_vector_.size() == VEC3D_QUAT_SIZE)
    sfv = SensorFeatureVector(translation_.normalize(), quaternion_.normalize());
  return sfv;
}

void SensorFeatureVector::write_to_file(std::ofstream &os){
  tools::swap_endian(values_vector_.begin(),values_vector_.end());
  os.write((char *)&values_vector_[0], values_vector_.size()*sizeof(float));
}
