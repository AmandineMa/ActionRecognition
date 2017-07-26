#include <vector>
#include <string>
#include <fstream>
#include <iterator>
#include <iomanip>
#include <memory>
#include <iostream>


#include "action_recognition/HMM.hpp"
#include "action_recognition/common.hpp"
#include "action_recognition/Labels.hpp"

HMM::HMM(std::string name, int nb_states, int dim, EmissionType emission_type, TopologyType topology_type, int nb_mixtures):
  name_(name), nb_states_(nb_states), 
  emission_type_(emission_type), topology_type_(topology_type), 
  start_proba_(nb_states,0),end_proba_(nb_states,0),
  transmat_(nb_states, std::vector<float>(nb_states,0)){
  
  // The HMM will always starts on the first state
  start_proba_[0] = 1;

  switch(topology_type){
    case TopologyType::L_to_R:{
     
      // Definition of the transition matrix 
      for(int i = 0; i < (nb_states - 1) ; i++){
        // The probability for each state to self-transited is 0.6
        transmat_[i][i] = 0.6;
        // The probability for each state to go to the next one is 0.4
        transmat_[i][i+1] = 0.4;
      }
      // Set the probability of the last emitting state to loop  on himself
      transmat_[nb_states-1][nb_states-1] = 0.7;
      // Set the probability of the last emitting state 
      // to go to the end non-emitting state (leave the HMM)
      end_proba_[nb_states-1] = 0.3;
      break;
    }
    case TopologyType::Bakis:{
      
      for(int i = 0; i < (nb_states - 2) ; i++){
        // The probability for each state to self-transited is 0.4
        transmat_[i][i] = 0.4;
        // The probability for each state to go to the next one is 0.3
        transmat_[i][i+1] = 0.3;
        // The probability for each state to go to 2 states further is 0.3
        transmat_[i][i+2] = 0.3;
      }

      // Set the probability of the penultimate state to loop  on himself
      transmat_[nb_states-2][nb_states-2] = 0.4;
      // Set the probability of the penultimate emitting state 
      // to go to the last state
      transmat_[nb_states-2][nb_states-1] = 0.3;
      // Set the probability of the penultimate emitting state 
      // to go to the end non-emitting state (leave the HMM)
      end_proba_[nb_states-2] = 0.3;

      // Set the probability of the last state to loop  on himself
      transmat_[nb_states-1][nb_states-1] = 0.7;
      // Set the probability of the last emitting state 
      // to go to the end non-emitting state (leave the HMM)
      end_proba_[nb_states-1] = 0.3;

      break;
    }
    case TopologyType::Ergodic:{
      // All the probabilities of the transition matrix are equally distributed
      float prob = 1.0/(nb_states+1);
      for(int i = 0; i < nb_states; i++){
        end_proba_[i]=prob;
        for(int j = 0; j < nb_states; j++)
        transmat_[i][j] = prob;
      }
      break;
    }
  }

  switch(emission_type_){
    case EmissionTypes::Gaussian:{
      // There is one Gaussian object for the HMM
      /*    Means :
            State 1 [ 0 0 0 0 0 ...]
            State 2 [ 0 0 0 0 0 ...]
            State 3 [ 0 0 0 0 0 ...]
            ...
            State N [ 0 0 0 0 0 ...]

            Covars :
            State 1 [ 1 1 1 1 1 ...]
            State 2 [ 1 1 1 1 1 ...]
            State 3 [ 1 1 1 1 1 ...]
            ...
            State N [ 1 1 1 1 1 ...]
      */
      observations_.emplace_back(new Gaussian());
      // All the means, for each state, are initialized to 0
      std::vector<std::vector<float> > temp0(nb_states, std::vector<float>(dim,0));  
      observations_.back()->means_ = temp0;
      // All the variances, for each state, are initialized to 1
      std::vector<std::vector<float> > temp1(nb_states, std::vector<float>(dim,1));
      observations_.back()->covars_ = temp1;
      break;
    }
    case EmissionTypes::GMM:{
      std::vector<float> mixture_weight(nb_mixtures, 1.0/nb_mixtures);
      // For each state, there are a certain number of mixtures
      // There is a Gaussian object (GMM) for each state
      /*    
            State 1 - GMM 1
            Means :
            Mixture 1 [ 0 0 0 0 0 ...]
            Mixture 2 [ 0 0 0 0 0 ...]
            Mixture 3 [ 0 0 0 0 0 ...]
            ...
            Mixture M [ 0 0 0 0 0 ...]

            Covars :
            Mixture 1 [ 1 1 1 1 1 ...]
            Mixture 2 [ 1 1 1 1 1 ...]
            Mixture 3 [ 1 1 1 1 1 ...]
            ...
            Mixture M [ 1 1 1 1 1 ...] 
            
            State 2 - GMM 2
            Means :
            Mixture 1 [ 0 0 0 0 0 ...]
            Mixture 2 [ 0 0 0 0 0 ...]
            Mixture 3 [ 0 0 0 0 0 ...]
            ...
            Mixture M [ 0 0 0 0 0 ...]

            Covars :
            Mixture 1 [ 1 1 1 1 1 ...]
            Mixture 2 [ 1 1 1 1 1 ...]
            Mixture 3 [ 1 1 1 1 1 ...]
            ...
            Mixture M [ 1 1 1 1 1 ...]

            ...

            State N - GMM N
            Means :
            Mixture 1 [ 0 0 0 0 0 ...]
            Mixture 2 [ 0 0 0 0 0 ...]
            Mixture 3 [ 0 0 0 0 0 ...]
            ...
            Mixture M [ 0 0 0 0 0 ...]

            Covars :
            Mixture 1 [ 1 1 1 1 1 ...]
            Mixture 2 [ 1 1 1 1 1 ...]
            Mixture 3 [ 1 1 1 1 1 ...]
            ...
            Mixture M [ 1 1 1 1 1 ...]
      */
      for(int i = 0; i < nb_states; i++){
        observations_.emplace_back(new GMM());
        static_cast<GMM*>(observations_.back())->nb_mixtures_ = nb_mixtures;
        static_cast<GMM*>(observations_.back())->mixture_weight_ = mixture_weight;
        std::vector<std::vector<float> > temp0(nb_mixtures, std::vector<float>(dim,0));
        std::vector<std::vector<float> > temp1(nb_mixtures, std::vector<float>(dim,1));
        observations_.back()->means_ = temp0;
        observations_.back()->covars_ = temp1;
      }
      break;
    }
    default:
      break;
   }
}

HMM::~HMM(){
  for(std::vector<Gaussian*>::iterator it = observations_.begin(); it != observations_.end() ; it++)
    delete (*it);
  observations_.clear();
}

void HMM::write_to_file(std::string file_path){
  std::ofstream ofile(file_path.c_str());
  ofile.setf(std::ios::fixed);
  ofile.precision(6);
  int ndim = observations_.back()->means_[0].size(); 
  std::ostream_iterator<float> it_file(ofile, " ");
  ofile << "~o\n";
  ofile << "<VecSize> " << ndim << " <USER>\n";
  ofile << "~h " <<"'"<< name_<< "'" << "\n";
  ofile << "<BeginHMM>\n";
  ofile << "   <NumStates> " << nb_states_+2 << "\n";
  int i;
  for(i = 0; i < nb_states_; i++){
    switch(emission_type_){
      case EmissionTypes::Gaussian:
        ofile << "   <State> " << i+2 << "\n";
        ofile << "     <Mean> " << ndim << "\n";
        std::copy(observations_[0]->means_[i].begin(), observations_[0]->means_[i].end(), it_file);
        ofile << "\n";
        ofile << "     <Variance> " << ndim << "\n";
        std::copy(observations_[0]->covars_[i].begin(), observations_[0]->covars_[i].end(), it_file);
        ofile << "\n";
        break;
      case EmissionTypes::GMM:{ 
        int nb_mixtures =  static_cast<GMM*>(observations_[0])->nb_mixtures_; 
        ofile << "   <State> " << i+2 << "\n";
        ofile << "     <NumMixes> " << nb_mixtures << "\n";
        int j;
        for(j = 0; j < nb_mixtures; j++){
          ofile << "        <MIXTURE> " << j+1 << " " << static_cast<GMM*>(observations_[i])->mixture_weight_[j];
          ofile << "\n";
          ofile << "          <MEAN> " << ndim << "\n";
          std::copy(observations_[i]->means_[j].begin(), observations_[i]->means_[j].end(), it_file);
          ofile << "\n";
          ofile << "          <VARIANCE> " << ndim << "\n";
          std::copy(observations_[i]->means_[j].begin(), observations_[i]->means_[j].end(), it_file);
          ofile << "\n";
        }
        break;
      }
      default:
        break;
    }
  } 
  ofile << "   <TransP> " << nb_states_+2 << "\n";
  ofile << float(0) << " "; std::copy(start_proba_.begin(), start_proba_.end(), it_file); ofile << float(0) << "\n";
  std::vector<float>::iterator it_vector = it_vector = end_proba_.begin();
  std::vector<std::vector<float> >::iterator it_tr_ = transmat_.begin();
  for(; it_tr_ != transmat_.end() ; it_tr_++){
    ofile << float(0) << " "; std::copy(it_tr_->begin(), it_tr_->end(), it_file); ofile << *it_vector <<"\n";
    it_vector++;
  }
  std::vector<float> zeros(transmat_.size()+2,0);
  std::copy(zeros.begin(), zeros.end(), it_file);
  ofile << "\n";
  ofile << "<EndHMM>\n";
  ofile.close();
}
