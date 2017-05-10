#ifndef HMM_HPP
#define HMM_HPP

#include <vector>
#include <string>

#include "action_recognition/common.hpp"

class Gaussian{
 friend class HMM;
protected:
  std::vector<std::vector<float> > means_;
  std::vector<std::vector<float> > covars_;
};

class GMM : public Gaussian{ 
  friend class HMM;
protected:
  std::vector<float> priors_;
  int nb_mixtures_;
};

class HMM{
 
private:
  // std::string name_;
  int nb_states_;
  std::vector<float> component_weitghts_;
  EmissionType emission_type_;
  std::vector<float> start_proba_;
  std::vector<float> end_proba_;
  std::vector<Gaussian*> observations_;
  std::vector<std::vector<float> > transmat_;

public: 
  std::string name_;
  HMM();
  HMM(std::string name, int nb_states, int dim, EmissionType emission_type, int nb_mixtures);
  ~HMM();

  void write_to_file(std::string file_path);
  void get_from_HMM_file(std::string file_path);

};

#endif
