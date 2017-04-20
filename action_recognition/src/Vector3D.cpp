#include <vector>
#include <string>
#include <iostream>

#include "action_recognition/Vector3D.hpp"

Vector3D::Vector3D():vector3D_(3){}

Vector3D::Vector3D(double x, double y, double z): vector3D_(3){
  set_new_values(x,y,z);
}

Vector3D::Vector3D(std::vector<double> vector3D){
  try{
    if(vector3D.size() != 3)
      throw std::string("A 3D vector has to be initialized with a vector of size "+3);
    else
      std::vector<double> vector3D_(vector3D);
  }catch(std::string const& error){
    std::cerr<<error<<std::endl;
  }
}

double Vector3D::get_x(void){return vector3D_[0];}

double Vector3D::get_y(void){return vector3D_[1];}

double Vector3D::get_z(void){return vector3D_[2];}
 
void Vector3D::set_new_values(double x, double y, double z){
  vector3D_[0]=x;
  vector3D_[1]=y;
  vector3D_[2]=z;
}

void Vector3D::set_x(double x){vector3D_[0]=x;}

void Vector3D::set_y(double y){vector3D_[1]=y;}

void Vector3D::set_z(double z){vector3D_[2]=z;}

Vector3D Vector3D::normalize(void){
  return *this;
}

