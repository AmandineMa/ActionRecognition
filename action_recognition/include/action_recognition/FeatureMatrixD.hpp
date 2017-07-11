#ifndef FEATUREMATRIXD_HPP
#define	FEATUREMATRIXD_HPP

/**
 * \file FeatureMatrixD.hpp
 * \brief FeatureMatrixD class to be used for real-time recognition
 * \author Amandine M.
 */

#include <vector>
#include <string>
#include <mutex>

#include "action_recognition/FeatureVector.hpp"

/** 
 * \brief A class that represents feature vectors from time 0 to T, 
 * to be used for real-time recognition
 *
 * Thead safe
 * [ FeatureVector_t0 
 *   FeatureVector_t1
 *   ...
 *   FeatureVector_T ]
 */
class FeatureMatrixD{
private:
 
  std::deque<FeatureVector> feature_vector_array_; /** Serie of feature vectors */
  mutable std::mutex mutex_;

  /** 
   * \brief Return the size of feature vector (matrix columns number)
   * 
   * Not thread safe
   *\retval Size
   */
  int get_feature_vector_size(void) const;
public:
  /** 
   * \brief Constructor for an emtpy feature matrix
   */
  FeatureMatrixD();
 
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
   * \brief Remove the first feature vector of the queue
   */
  void pop_feature_vector(void);

  /** 
   * \brief Remove the n first feature vectors of the queue
   */
  void pop_feature_vectors(int n);

  /** 
   * \brief Return the samples number of the matrix  
   *
   * Not thread safe, to be used with a mutex
   * (matrix lines number <=> number of feature vectors)
   * \retval Samples number
   */
  int get_samples_number(void) const;
  /** 
   * \brief Write the matrix values to the file given in parameter
   * \param Reference to an opened std::ofstream file
   */
  void write_to_file(std::ofstream &os) const;
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
