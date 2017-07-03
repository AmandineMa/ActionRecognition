#include <tf2/LinearMath/Quaternion.h>
#include <fstream>
#include <iostream> 

#include "action_recognition/SensorFeatureVector.hpp"
#include "action_recognition/common.hpp"

SensorFeatureVector::SensorFeatureVector(){}

SensorFeatureVector::SensorFeatureVector(const Vector3D &vector3D):vector3D_(vector3D){}

SensorFeatureVector::SensorFeatureVector(const std::vector<float> &values_vector):vector3D_(values_vector){}

SensorFeatureVector::SensorFeatureVector(float x, float y, float z):vector3D_(x,y,z){} 

Vector3D SensorFeatureVector::get_vector3D(void){return vector3D_;}

void SensorFeatureVector::normalize(NormalizationType normalization_type){
  vector3D_.normalize(normalization_type);
}

void SensorFeatureVector::write_to_file(std::ofstream &os, FeatureFileFormat file_format){  
  std::vector<float> values_vector(SENSOR_FEATURE_VECTOR_SIZE);
  values_vector[VectorElements::X]=vector3D_.get_x();
  values_vector[VectorElements::Y]=vector3D_.get_y();
  values_vector[VectorElements::Z]=vector3D_.get_z();

  switch(file_format){
    case FeatureFileFormat::dat:{
      // Write the vector values in  the file given in parameter
      os.write((char *)&values_vector[0], values_vector.size()*sizeof(float));
      break;
    } case FeatureFileFormat::lab:{
        os << "<SensorFeatureVector>";
        std::ostream_iterator<float> output_iterator(os, " ");
        std::copy(values_vector.begin(), values_vector.end(), output_iterator); 
        os << "</SensorFeatureVector>";
        break;
      }
    default:
      break;
  }
}

void SensorFeatureVector::print_vector(void){
  // Write the vector with the format [ x, y, z ]
  std::cout << "[ "<< vector3D_.get_x() << ", " << vector3D_.get_y() << ", " << vector3D_.get_z() << " ]"; 
}

int SensorFeatureVector::get_size(void){return SENSOR_FEATURE_VECTOR_SIZE;}
