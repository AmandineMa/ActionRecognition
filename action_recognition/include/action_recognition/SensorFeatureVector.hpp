#ifndef SENSORFEATUREVECTOR_HPP
#define	SENSORFEATUREVECTOR_HPP

/**
 * \file SensorFeatureVector.hpp
 * \brief SensorFeatureVector class
 * \author Amandine M.
 */

#include <vector>
#include <fstream>
#include "action_recognition/Vector3D.hpp"
#include "action_recognition/common.hpp"

/** Size of a simple sensor feature vector */
#define SENSOR_FEATURE_VECTOR_SIZE 3

/** 
 * \brief A (base) class that contains the coordinates (features) of a sensor at time t
 */
class SensorFeatureVector{

protected:
  Vector3D vector3D_;

public: 
  /** 
   * \brief Constructor for an emtpy sensor feature vector
   */
  SensorFeatureVector();
  /** 
   * \brief Constructor for a sensor feature vector
   * \param #Vector3D containing the values
   */
  SensorFeatureVector(const Vector3D &vector3D);
  /** 
   * \brief Constructor for a sensor feature vector
   * \param std::vector containing the values
   */
  SensorFeatureVector(const std::vector<float> &values_vector_);
  /** 
   * \brief Constructor for a sensor feature vector
   * \param x value
   * \param y value
   * \param z value
   */
  SensorFeatureVector(float x, float y, float z);
  
  Vector3D get_vector3D(void);

  /** 
   * \brief Normalize the values of the vector with the method given in parameter
   * \param #NormalizationType
   */
  virtual void normalize(NormalizationType normalization_type);
  /** 
   * \brief Write the vector values to the file given in parameter
   * \param Reference to an opened std::ofstream file
   */
  virtual void write_to_file(std::ofstream &os, FeatureFileFormat file_format);
  /** 
   * \brief Write the vector values to the standard output
   */
  virtual void print_vector(void);
  /** 
   * \brief Return the size of the sensor feature vector (number of values)
   * \retval Number of values
   */
  virtual int get_size(void);

};
#endif
