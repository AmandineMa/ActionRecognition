#ifndef HMM_HPP
#define HMM_HPP

/**
 * \file HMM.hpp
 * \brief HMM class and Gaussian classes
 * \author Amandine M.
 */

#include <vector>
#include <string>

#include "action_recognition/common.hpp"

/**
 * \brief Gaussian (base) class that represents a Gaussian distribution
 */
class Gaussian{
 friend class HMM;
protected:
  std::vector<std::vector<float> > means_; /** Matrix of means */
  std::vector<std::vector<float> > covars_; /** Matrix of covariances */
};

/**
 * \brief Gaussian Mixture Model (derived) class that represents a mixture distribution
 */
class GMM : public Gaussian{ 
  friend class HMM;
protected:
  std::vector<float> priors_; /** Matrix of priors */
  int nb_mixtures_; /** Number of mixtures */
};

/**
 * \brief HMM class that reprents a Hidden Markov Model 
 */
class HMM{
 
private:
  int nb_states_; /** HMM states number */
  std::vector<float> component_weights_; /** Vector of the component weights */
  EmissionType emission_type_; /** Emission distribution type */
  std::vector<float> start_proba_; /** Vector of the start probabilities */
  std::vector<float> end_proba_; /** Vector of the end probabilities */
  std::vector<Gaussian*> observations_; /** Vector of Gaussians */
  std::vector<std::vector<float> > transmat_; /** Transition matrix */

public: 
  std::string name_;

  /**
   * \brief Constructor for an HMM
   * \param Name
   * \param States number
   * \param Dimension
   * \param Emission distribution type
   * \param Number of mixtures, default value = 1
   */
  HMM(std::string name, int nb_states, int dim, EmissionType emission_type, int nb_mixtures = 1);
  /**
   * \brief Destructor for an HMM. Delete the Gaussian pointers.
   */
  ~HMM();

  /**
   * \brief Write the HMM to HTK HMM file format
   */
  void write_to_file(std::string file_path);
  /**
   * \brief Get the HMM from HTK HMM file format
   */
  void get_from_HMM_file(std::string file_path);

};

#endif
