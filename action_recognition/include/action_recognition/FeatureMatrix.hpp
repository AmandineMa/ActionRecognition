#ifndef FEATUREMATRIX_HPP
#define	FEATUREMATRIX_HPP

/**
 * \file FeatureMatrix.hpp
 * \brief FeatureMatrix class
 * \author Amandine M.
 */

#include <vector>
#include <string>

#include "action_recognition/FeatureVector.hpp"

/** 
 * \brief A class that represents feature vectors from time 0 to T
 * [ FeatureVector_t0 
 *   FeatureVector_t1
 *   ...
 *   FeatureVector_T ]
 */
class FeatureMatrix{
private:
  std::string label_; /** Label (name) of the action associated to the matrix */
  std::vector<FeatureVector> feature_vector_array_; /** Serie of feature vectors */
    
public:
  /** 
   * \brief Constructor for an emtpy feature matrix
   */
  FeatureMatrix();
  /** 
   * \brief Constructor for a feature matrix
   * \param Name of the associated action
   */
  FeatureMatrix(std::string label);  
  /** 
   * \brief Add a new empty feature vector 
   */
  void new_feature_vector(void);
  /** 
   * \brief Add a new feature vector with flags
   * \param Vector of flags
   */
  void new_feature_vector(const std::vector<float> &flag_vector);
  /** 
   * \brief Add a sensor feature vector to the last added feature vector
   * \param Vector of values of the new sensor feature vector
   */
  void add_sensor_feature_vector(const std::vector<float> &values_vector);

  /** 
   * \brief Add a flag to the last added feature vector
   * \param Flag value
   */
  void add_flag(float flag);
  /** 
   * \brief Set a vector of flags for the last added feature vector
   * \param Vector of flags
   */
  void set_flags(const std::vector<float> &flags);

  /** 
   * \brief Set the name of the feature matrix
   */
  void set_label(std::string label);

  /** 
   * \brief Return the name of the feature matrix
   * \retval Name of the feature matrix
   */
  std::string get_label(void) const;
  /** 
   * \brief Return the size of feature vector (matrix columns number)
   *\retval Size
   */
  int get_feature_vector_size(void) const;
  /** 
   * \brief Return the samples number of the matrix  
   * (matrix lines number <=> number of feature vectors)
   * \retval Samples number
   */
  int get_samples_number(void) const;
  /** 
   * \brief Write the matrix values to the file given in parameter
   * \param Reference to an opened std::ofstream file
   * \param Output format of the file (HTK, text, ...)
   */
  void write_to_file(std::ofstream &os, FeatureFileFormat file_format) const;
  /** 
   * \brief Normalize the values of all the #FeatureVector with the method given in parameter
   * \param #NormalizationType
   */
  void normalize(NormalizationType normalization_type);
  /** 
   * \brief Write the matrix values to the standard output
   */
  void print(void);

 
};

#endif
