#ifndef FEATUREVECTOR_HPP
#define	FEATUREVECTOR_HPP

/**
 * \file FeatureVector.hpp
 * \brief FeatureVector class
 * \author Amandine M.
 */

#include <vector>
#include <memory>
#include <fstream>

#include "action_recognition/SensorFeatureVector.hpp"

/** 
 * \brief A class that represents a feature vector
 * [ SensorFeatureVector_0, SensorFeatureVector_1, ..., SensorFeatureVector_k, Flag_0, ..., Flag_i ]
 */
class FeatureVector{

private:
  std::vector<std::unique_ptr<SensorFeatureVector>> sensor_feature_vectors_; /** Vector of unique_ptr (to use polymorphism) on #SensorFeatureVector */
  std::vector<float> flag_vector_; /** Vector of flags (values like 1, 0, -1). 
                                       Float for compatibility with HTK */
  int feature_vector_size_; /** Total number of data values in the feature vector */

public:
  /** 
   * \brief Constructor for an emtpy feature vector
   */
  FeatureVector();
  /** 
   * \brief Constructor for a feature vector
   * \param std::vector of flags
   */
  FeatureVector(std::vector<float> flag_vector);

  /** 
   * \brief Build a pointer on a new sensor feature vector object 
   * (simple or extended according to the size of the vector given in parameter)
   * and add it to the vector of unique_ptr
   * \param std::vector of values corresponding to a sensor feature vector
   */
  void add_sensor_feature_vector(std::vector<float> values_vector);
  /** 
   * \brief Add a flag value to the vector of flags
   * \param Flag value
   */
  void add_flag(float flag);
  /** 
   * \brief Set the vector of flags
   * \param Flags vector
   */
  void set_flags(std::vector<float> flags);
  /** 
   * \brief Return the size of the feature vector (number of values)
   * \retval Number of values
   */
  int get_size(void) const;
  /** 
   * \brief Normalize the values of all the sensor feature vectors with the method given in parameter
   * \param #NormalizationType
   */
  void normalize(NormalizationType normalization_type);
  /** 
   * \brief Write the vector values to the file given in parameter
   * \param Reference to an opened std::ofstream file
   */
  void write_to_file(std::ofstream &os) const;
  /** 
   * \brief Write the vector values to the standard output
   */
  void print_vector(void);

};

#endif
