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

void train_HMM(std::string label, bool print_output, bool isolated_unit_init, bool isolated_unit_flat_init, bool isolated_unit_training, EmissionType emission_type, TopologyType topology_type, int dim, int median_samp_nb, StatesNumDef num_states_def, int iterations_nb, Setup setup, int mixtures_nb=1);

void train_HMMs(bool print_output, Setup setup);


void embedded_unit_flat_init(bool print_output, Setup setup, std::string hmm_name, int state_number, int dim, EmissionType emission_type, TopologyType topology_type, int mixtures_nb=1);

};
#endif
