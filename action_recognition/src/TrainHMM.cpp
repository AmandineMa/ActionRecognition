#include <ros/console.h>
#include <vector>
#include <math.h>
#include <boost/filesystem.hpp>

#include "action_recognition/TrainHMM.hpp"
#include "action_recognition/HTKHeader.hpp"
#include "action_recognition/HMM.hpp"
#include "action_recognition/common.hpp"
#include "action_recognition/FeatureMatrix.hpp"

// Anonymous namespace for private functions of the TrainHMM namespace 
namespace{
  int compute_state_number(int num_states_def, int median_samp_nb){
    // Definition of the number of states for the HMM
    int state_number;
    switch(num_states_def){
      case StatesNumDefs::median:
        state_number = ceil(std::sqrt(median_samp_nb)); //C++11
        break;
      case StatesNumDefs::linear_scaling:
        state_number = ceil(median_samp_nb/10);
        break;
      default:
        state_number = num_states_def;
        break;
    }

    if(state_number < 3) 
      state_number = 3;
  }
}

void TrainHMM::train_HMM(std::string label, bool print_output, 
                         bool isolated_unit_init, bool isolated_unit_flat_init, 
                         bool isolated_unit_training, EmissionType emission_type, 
                         TopologyType topology_type,
                         int dim, int median_samp_nb, 
                         StatesNumDef num_states_def, int iterations_nb, 
                         Setup setup, int mixtures_nb){

  // Define the path of the file of the list of data files : ?/tmp_files/Label_x.scp
  std::string data_files_list_path = setup.htk_tmp_files_path+label+".scp";

  int state_number = compute_state_number(num_states_def, median_samp_nb);

  //int dim = feature_matrix_array[0].get_feature_vector_size();

  // Instantiate a new HMM object   
  HMM hmm(label, state_number, dim, emission_type, topology_type, mixtures_nb);

  std::string hmm_path = setup.output_hmm+hmm.name_;
  // Write the HMM to an HTK format file
  hmm.write_to_file(hmm_path);

  if(isolated_unit_init){
    // Inialize the HMM with the HInit command, the HMM file and the data files
    // Write the initialized HMM in tmp/
    std::string command = "HInit -i "+std::to_string(iterations_nb)+" -A -T 1 -M "+setup.output_hmm+" "+hmm_path+" -S "+data_files_list_path; //C++11
    std::string output = tools::execute_command(command);

    if(print_output)
      ROS_INFO("%s", output.c_str());
  }

  if(isolated_unit_flat_init){
    // Inialize the HMM with the HCompv command, the HMM file and the data files (not usual use)
    // Write or modify the initialized HMM in tmp/
    std::string command = "HCompV -A -T 1 -M "+setup.output_hmm+" "+hmm_path+" -S "+data_files_list_path;
    std::string output = tools::execute_command(command);

    if(print_output)
      ROS_INFO("%s", output.c_str());
  }  
  
  if(isolated_unit_training){
    // Train the HMM with the HRest command, the initialized HMM model and the data files
    std::string command = "HRest -i "+std::to_string(iterations_nb)+" -A -T 1 -M "+setup.output_hmm+" "+setup.output_hmm+label+" -S "+data_files_list_path;//C++11
    std::string output = tools::execute_command(command);
    if(print_output)
      ROS_INFO("%s", output.c_str());
  }
}

void TrainHMM::embedded_unit_flat_init(bool print_output, Setup setup, std::string hmm_name, StatesNumDef num_states_def, int dim, int median_samp_nb, EmissionType emission_type, TopologyType topology_type, int mixtures_nb){ 

  int state_number = compute_state_number(num_states_def, median_samp_nb);

  // Instantiate a new HMM object (that is why 
  // it cannot be used with isolated_unit_init and isolated_unit_flat_init)
  HMM hmm(hmm_name, state_number, dim, emission_type, topology_type, mixtures_nb);
  // Write the HMM to an HTK format file
  std::string hmm_path = setup.output_hmm +hmm_name;
  hmm.write_to_file(hmm_path);

  // Inialize the HMM with the HCompv command, the HMM file and all the data files (all actions)
  // (usual use)
  // Write the initialized HMM in tmp/
  std::string command = "HCompV -A -T 1 -M "+setup.output_hmm+" "+hmm_path+" -S "+setup.data_list_path;
  std::string output = tools::execute_command(command);

  if(print_output)
    ROS_INFO("%s", output.c_str());
}

void TrainHMM::train_HMMs(bool print_output, Setup setup){ 
  // Train the all the HMM with the HERest command,
  // the initialized HMM models wrote in an MMF file, all the data files and the sgmentation
  std::string command = "HERest -t 120.0 60.0 250.0 -A -T 1 -S "+setup.data_list_path+" -H "+setup.hmmsdef_path+" -I "+setup.mlf_path+" "+setup.labels_list_path;
  std::string output = tools::execute_command(command);
  if(print_output)
    ROS_INFO("%s", output.c_str());

}


