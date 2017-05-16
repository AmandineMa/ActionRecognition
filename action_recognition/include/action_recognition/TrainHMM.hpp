#ifndef TRAINHMM_HPP
#define TRAINHMM_HPP

/**
 * \file TrainHMM.hpp
 * \brief Namespace containing function to train an HMM
 * \author Amandine M.
 */

#include <vector>

#include "action_recognition/common.hpp"
#include "action_recognition/FeatureMatrix.hpp"
#include "action_recognition/Setup.hpp"

namespace TrainHMM{

/**
 * \brief Return the median value of the elements of the std::vector
 * \param Median value of the elements of the std::vector
 */
  float median(std::vector<int> samples_number);
/**
 * \brief Train a HMM for all the feature matrices corresponding to the same label
 * \param Distribution type of the HMM
 * \param Vector of #FeatureMatrix passed by const reference (read-only)
 * \param HMM states number
 * \param Number of iterations for the training
 * \param #Setup for the files paths
 * \param Number of mixtures for the HMM, default value = 1
 */
void train_HMM(bool print_output, EmissionType emission_type, const std::vector<FeatureMatrix> &feature_matrix_array, 
               StatesNumDef num_states_def, int iterations_nb, Setup setup, int mixtures_nb = 1);


};

#endif
