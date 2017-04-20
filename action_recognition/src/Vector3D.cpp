#include <vector>
#include <string>
#include <iostream>

#include "action_recognition/Vector3D.hpp"
#include "action_recognition/common.hpp"

Vector3D::Vector3D():vector3D_(VECTOR3D_SIZE){}

Vector3D::Vector3D(double x, double y, double z): vector3D_(VECTOR3D_SIZE){
  set_new_values(x,y,z);
}

Vector3D::Vector3D(std::vector<double> vector3D){
  try{
    if(vector3D.size() != VECTOR3D_SIZE)
      throw std::string("A 3D vector has to be initialized with a vector of size "+VECTOR3D_SIZE);
    else
      std::vector<double> vector3D_(vector3D);
  }catch(std::string const& error){
    std::cerr<<error<<std::endl;
  }
}

double Vector3D::get_x(void){return vector3D_[VectorElements::X];}

double Vector3D::get_y(void){return vector3D_[VectorElements::Y];}

double Vector3D::get_z(void){return vector3D_[VectorElements::Z];}
 
void Vector3D::set_new_values(double x, double y, double z){
  vector3D_[VectorElements::X]=x;
  vector3D_[VectorElements::Y]=y;
  vector3D_[VectorElements::Z]=z;
}

void Vector3D::set_x(double x){vector3D_[VectorElements::X]=x;}

void Vector3D::set_y(double y){vector3D_[VectorElements::Y]=y;}

void Vector3D::set_z(double z){vector3D_[VectorElements::Z]=z;}

Vector3D Vector3D::normalize(void){
  return *this;
}

