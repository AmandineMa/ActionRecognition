#ifndef SENSORFEATUREVECTOREXTENDED_HPP
#define	SENSORFEATUREVECTOREXTENDED_HPP

/**
 * \file SensorFeatureVectorExtended.hpp
 * \brief SensorFeatureVectorExtended class
 * \author Amandine M.
 */

#include <vector>
#include <fstream>
#include <tf2/LinearMath/Quaternion.h>

#include "action_recognition/Vector3D.hpp"
#include "action_recognition/common.hpp"
#include "action_recognition/SensorFeatureVector.hpp"

/** Size of an extended sensor feature vector */
#define SENSOR_FEATURE_VECTOR_EXTENDED_SIZE 7

/** 
 * \brief A (derived) class that contains the features (3D vector + quaternion) of a sensor at time t
 */
class SensorFeatureVectorExtended : public SensorFeatureVector {

private:
  tf2::Quaternion quaternion_;

public:
  /** 
   * \brief Constructor for an extended sensor feature vector
   * \param #Vector3D containing the values
   * \param quaternion
   */
  SensorFeatureVectorExtended(Vector3D vector3D, tf2::Quaternion quaternion);   
  /** 
   * \brief Constructor for an extended sensor feature vector
   * \param x value of the 3D vector
   * \param y value of the 3D vector
   * \param z value of the 3D vector
   * \param quaternion
   */
  SensorFeatureVectorExtended(float x, float y, float z, tf2::Quaternion quaternion); 
  /** 
   * \brief Constructor for an extended sensor feature vector
   * \param x value of the 3D vector
   * \param y value of the 3D vector
   * \param z value of the 3D vector
   * \param x value of the quaternion associated to the i element
   * \param y value of the quaternion associated to the j element
   * \param z value of the quaternion associated to the k element
   *\ param w value of the quaternion
   */
  SensorFeatureVectorExtended(float x, float y, float z, float x_q, float y_q, float z_q, float w);
  /** 
   * \brief Constructor for an extended sensor feature vector
   * \param std::vector containing the values
   */
  SensorFeatureVectorExtended(std::vector<float> values_vector_);

  tf2::Quaternion get_quaternion(void);

  /** 
   * \brief Normalize the values of the 3D vector with the method given in parameter 
   * and the quaternion such that x^2 + y^2 + z^2 +w^2 = 1
   * \param #NormalizationType
   */
  virtual void normalize(NormalizationType normalization_type);
  /** 
   * \brief Write the vector values to the standard output
   */
  virtual void write_to_file(std::ofstream &os);
  /** 
   * \brief Write the vector values to the standard output
   */
  virtual void print_vector(void);
  /** 
   * \brief Return the size of the extended sensor feature vector 
   * (3 + 4)
   * \retval Number of values
   */
  virtual int get_size(void);

};
#endif
