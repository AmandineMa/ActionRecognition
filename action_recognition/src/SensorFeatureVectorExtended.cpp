#include <tf2/LinearMath/Quaternion.h>
#include <fstream>
#include <iostream> 
#include "action_recognition/SensorFeatureVectorExtended.hpp"
#include "action_recognition/SensorFeatureVector.hpp"
#include "action_recognition/common.hpp"

SensorFeatureVectorExtended::SensorFeatureVectorExtended(const Vector3D &vector3D, const tf2::Quaternion &quaternion):SensorFeatureVector(vector3D), quaternion_(quaternion){}

SensorFeatureVectorExtended::SensorFeatureVectorExtended(float x, float y, float z, const tf2::Quaternion &quaternion):
  SensorFeatureVector(x,y,z), quaternion_(quaternion){}

SensorFeatureVectorExtended::SensorFeatureVectorExtended(const std::vector<float> &values_vector){
  if(values_vector.size() != SENSOR_FEATURE_VECTOR_EXTENDED_SIZE)
    std::cerr << "The vector size for a Sensorfeaturevectorextended initialization should be " << SENSOR_FEATURE_VECTOR_EXTENDED_SIZE << std::endl;
  else{
    vector3D_ = Vector3D(values_vector[VectorElements::X],values_vector[VectorElements::Y],values_vector[VectorElements::Z]);  
    quaternion_ = tf2::Quaternion(values_vector[VectorElements::X_Q],values_vector[VectorElements::Y_Q],
                                  values_vector[VectorElements::Z_Q], values_vector[VectorElements::W]);
  }
}

SensorFeatureVectorExtended::SensorFeatureVectorExtended(float x, float y, float z, float x_q, float y_q, float z_q, float w):
  SensorFeatureVector(x,y,z),quaternion_(x_q, y_q, z_q, w){}

tf2::Quaternion SensorFeatureVectorExtended::get_quaternion(void){return quaternion_;}

void SensorFeatureVectorExtended::normalize(NormalizationType normalization_type){
  SensorFeatureVector::normalize(normalization_type);
}


void SensorFeatureVectorExtended::write_to_file(std::ofstream &os, FeatureFileFormat file_format){  
  std::vector<float> values_vector(SENSOR_FEATURE_VECTOR_EXTENDED_SIZE);
  values_vector[VectorElements::X]=vector3D_.get_x();
  values_vector[VectorElements::Y]=vector3D_.get_y();
  values_vector[VectorElements::Z]=vector3D_.get_z();
  values_vector[VectorElements::X_Q]=quaternion_.getX();
  values_vector[VectorElements::Y_Q]=quaternion_.getY();
  values_vector[VectorElements::Z_Q]=quaternion_.getZ();
  values_vector[VectorElements::W]=quaternion_.getW(); 

  switch(file_format){
    case FeatureFileFormat::dat:{
      // Write the vector values in the file given in parameter - Binary format
      os.write((char *)&values_vector[0], values_vector.size()*sizeof(float));
      break;
    } case FeatureFileFormat::lab:{
        os << "<SensorFeatureVectorExtended>";
        std::ostream_iterator<float> output_iterator(os, " ");
        // Copy the value from the vector to the file, one by one
        // separated by the delimiter passed in the std::ostream_iterator
        std::copy(values_vector.begin(), values_vector.end(), output_iterator); 
        os << "</SensorFeatureVectorExtended>";
        break;
      }
    default:
      break;
  }
}


void SensorFeatureVectorExtended::print_vector(void){ 
  // Write the vector with the format [ x, y, z, x_q, y_q, z_q, w ]
  std::cout << "[ "<< vector3D_.get_x() << ", " << vector3D_.get_y() << ", " << vector3D_.get_z() << ", " << quaternion_.getX() << ", " << quaternion_.getY() << ", " << quaternion_.getZ() << ", " <<quaternion_.getW()<< " ]"; 

}

int SensorFeatureVectorExtended::get_size(void){return SENSOR_FEATURE_VECTOR_EXTENDED_SIZE;}
