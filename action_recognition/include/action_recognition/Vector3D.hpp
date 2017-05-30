#ifndef VECTOR3D_HPP
#define	VECTOR3D_HPP

/**
 * \file Vector3D.hpp
 * \brief Vector3D class
 * \author Amandine M.
 */

#include <vector>
#include "action_recognition/common.hpp"

/** Size of a 3D vector */
#define VECTOR3D_SIZE 3

/** 
 * \brief A (x, y, z) vector
 */
class Vector3D{

private:
  std::vector<float> vector3D_; /** Values vector */

public:
  /** 
   * \brief Constructor for an emtpy 3D vector
   */
  Vector3D();
  /** 
   * \brief Constructor for a 3D vector
   * \param x value
   * \param y value
   * \param z value
   */
  Vector3D(float x, float y, float z);
  /** 
   * \brief Constructor for a 3D vector
   * \param std::vector containing the values
   */
  Vector3D(std::vector<float> vector3D);

  /**
   * \brief Set new values for the vector
   * \param x value
   * \param y value
   * \param z value
   */
  void set_new_values(float x, float y, float z);

  float get_x(void);
  float get_y(void);
  float get_z(void);

  void set_x(float x);
  void set_y(float y);
  void set_z(float z);

  /** 
   * \brief Normalize the vector with the method given in parameter. 
   The values of the object remain unchanged.
   * \param #NormalizationType
   */
  void normalize(NormalizationType normalization_type);
};


#endif
