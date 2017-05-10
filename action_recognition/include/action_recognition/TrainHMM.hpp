#ifndef TRAINHMM_HPP
#define TRAINHMM_HPP

#include <vector>

#include "action_recognition/HMM.hpp"
#include "action_recognition/common.hpp"
#include "action_recognition/FeatureMatrix.hpp"

class TrainHMM{
private:
  HMM hmm_;
  std::vector<std::unique_ptr<FeatureMatrix> > feature_vector_array_; //C++11

  float median(std::vector<int> samples_number);

public:
  TrainHMM();
  void train_HMM(EmissionType emission_type, const std::vector<FeatureMatrix> &feature_matrix_array, StatesNumDef num_states_def, int iterations_nb, int mixtures_nb, int state_nb);
  void init_hmm(void);

};

#endif
