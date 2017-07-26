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
#include "action_recognition/Labels.hpp"

/**
 * \brief Gaussian (base) class that represents a Gaussian distribution
 */
class Gaussian{
 friend class HMM;
protected:
  /** \brief Matrix of means */
  std::vector<std::vector<float> > means_; 
  /** \brief Matrix of covariances */
  std::vector<std::vector<float> > covars_; 
};

/**
 * \brief Gaussian Mixture Model (derived) class that represents a mixture distribution
 */
class GMM : public Gaussian{ 
  friend class HMM;
protected:
  /** \brief Matrix of priors */
  std::vector<float> mixture_weight_; 
  /** \brief Number of mixtures */
  int nb_mixtures_; 
};

/**
 * \brief HMM class that reprents a Hidden Markov Model 
 */
class HMM{
 
private:
  /** \brief HMM states number */
  int nb_states_; 
  /** \brief Emission distribution type */
  EmissionType emission_type_; 
  /** \brief Topology type (Bakis, ergodic) */
  TopologyType topology_type_;
  /** 
   * \brief Vector of the start probabilities of the transition matrix 
   * (Probabilities of the start non-emitting state) 
   */
  std::vector<float> start_proba_; 
  /** 
   * \brief Vector of the end probabilities of the transition matrix
   * (Probabilities of the end non-emitting state) 
   */
  std::vector<float> end_proba_; 
  /** \brief Vector of Gaussians */
  std::vector<Gaussian*> observations_; 
  /** \brief Transition matrix */
  std::vector<std::vector<float> > transmat_; 

public:
  /** \brief Name (label) of the HMM */
  std::string name_;

  /**
   * \brief Constructor for a prototype HMM
   *
   * It builds a prototype of an HMM by defining initial probabilities 
   * according to the topology type. 
   * All values of the transition matrix set to 0 will remain 0 after every algorithm execution.
   * \param Name
   * \param States number
   * \param Dimension
   * \param Emission distribution type
   * \param Number of mixtures, default value = 1
   */
  HMM(std::string name, int nb_states, int dim, EmissionType emission_type, TopologyType topology_type, int nb_mixtures = 1);

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
