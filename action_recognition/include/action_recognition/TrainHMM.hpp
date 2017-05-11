#ifndef TRAINHMM_HPP
#define TRAINHMM_HPP

#include <vector>

#include "action_recognition/HMM.hpp"
#include "action_recognition/common.hpp"
#include "action_recognition/FeatureMatrix.hpp"
#include "action_recognition/Setup.hpp"

namespace TrainHMM{

  float median(std::vector<int> samples_number);

  void train_HMM(EmissionType emission_type, const std::vector<FeatureMatrix> &feature_matrix_array, StatesNumDef num_states_def, int iterations_nb, Setup setup, int mixtures_nb = 1);


};

#endif
