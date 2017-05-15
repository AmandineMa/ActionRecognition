#include <vector>
#include <string>
#include <iostream>

#include "action_recognition/Vector3D.hpp"
#include "action_recognition/common.hpp"

Vector3D::Vector3D():vector3D_(VECTOR3D_SIZE){}

Vector3D::Vector3D(float x, float y, float z): vector3D_(VECTOR3D_SIZE){
  set_new_values(x,y,z);
}

Vector3D::Vector3D(std::vector<float> vector3D):vector3D_(vector3D){
  if(vector3D.size() != VECTOR3D_SIZE)
    std::cerr <<"A 3D vector has to be initialized with a vector of size "+VECTOR3D_SIZE << std::endl;
}

float Vector3D::get_x(void){return vector3D_[VectorElements::X];}

float Vector3D::get_y(void){return vector3D_[VectorElements::Y];}

float Vector3D::get_z(void){return vector3D_[VectorElements::Z];}
 
void Vector3D::set_new_values(float x, float y, float z){
  vector3D_[VectorElements::X]=x;
  vector3D_[VectorElements::Y]=y;
  vector3D_[VectorElements::Z]=z;
}

void Vector3D::set_x(float x){vector3D_[VectorElements::X]=x;}

void Vector3D::set_y(float y){vector3D_[VectorElements::Y]=y;}

void Vector3D::set_z(float z){vector3D_[VectorElements::Z]=z;}

Vector3D Vector3D::normalize(NormalizationType normalization_type){
  return *this;
}

