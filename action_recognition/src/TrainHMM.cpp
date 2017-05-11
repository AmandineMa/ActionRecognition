#include <vector>
#include <math.h>

#include "action_recognition/TrainHMM.hpp"
#include "action_recognition/HTKHeader.hpp"
#include "action_recognition/Setup.hpp"


void TrainHMM::train_HMM(EmissionType emission_type, const std::vector<FeatureMatrix> &feature_matrix_array, StatesNumDef num_states_def, int iterations_nb, Setup setup, int mixtures_nb){
  std::vector<int> sample_numbers;
  int i = 0;
  std::string data_files_list_path = setup.path_htk_tmp_files+feature_matrix_array[0].get_label()+".scp";
  std::ofstream data_files_list(data_files_list_path.c_str());
  for(std::vector<FeatureMatrix>::const_iterator it = feature_matrix_array.begin(); it != feature_matrix_array.end(); it++){
    sample_numbers.push_back(it->get_samples_number());
    std::string data_file_name = setup.path_htk_tmp_files+feature_matrix_array[0].get_label()+"_"+std::to_string(i)+".dat"; //C++11
    std::ofstream ofile(data_file_name.c_str());
    HTKHeader header;
    header.BytesPerSample = it->get_feature_vector_size()*4; 
    header.nSamples = it->get_samples_number();
    header.Period = 100000;
    header.FeatureType = HTK_USER;
    header.write_to_file(ofile);
    it->write_to_file(ofile); 
    ofile.close();

    data_files_list << data_file_name.c_str() << "\n";
    i++;
  }
  data_files_list.close();
  
  int state_number;
  switch(num_states_def){
    case StatesNumDefs::median:
      state_number = ceil(std::sqrt(median(sample_numbers))); //C++11
      break;
    case StatesNumDefs::linear_scaling:
      state_number = ceil(median(sample_numbers)/10);
      break;
    default:
      state_number = setup.default_state_number;
      break;
  }

  if(state_number < 3) 
    state_number = 3;

  int dim = feature_matrix_array[0].get_feature_vector_size();
    
  HMM hmm(feature_matrix_array[0].get_label(), state_number, dim, emission_type, mixtures_nb);

  std::string hmm_path = setup.path_htk_tmp_files +hmm.name_+".hmm";
  hmm.write_to_file(hmm_path);
  std::string command = "HInit -A -T 1 -M "+setup.path_output+" "+hmm_path+" -S "+data_files_list_path;

  std::string HInit_output = tools::execute_command(command);
  std::cout << HInit_output << std::endl;
}

float TrainHMM::median(std::vector<int> samples_number){
  float median;
  int size = samples_number.size();

  std::sort(samples_number.begin(), samples_number.end());

  if (size  % 2 == 0)
    median = (samples_number[size / 2 - 1] + samples_number[size / 2]) / 2;
  else 
    median = samples_number[size / 2];

  return median;
}
