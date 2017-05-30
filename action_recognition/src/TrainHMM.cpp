#include <ros/console.h>
#include <vector>
#include <math.h>
#include <boost/filesystem.hpp>

#include "action_recognition/TrainHMM.hpp"
#include "action_recognition/HTKHeader.hpp"
#include "action_recognition/HMM.hpp"
#include "action_recognition/common.hpp"
#include "action_recognition/FeatureMatrix.hpp"

void TrainHMM::train_HMM(bool print_output, EmissionType emission_type, 
                         const std::vector<FeatureMatrix> &feature_matrix_array, 
                         StatesNumDef num_states_def, int iterations_nb, Setup setup, int mixtures_nb){
  std::string label = feature_matrix_array[0].get_label();
  // Vector that contains the number of samples of each matrix
  std::vector<int> sample_numbers;
  // To give a different name to each data file
  int i = 0;
  // Define the path of the file of the list of data files : ?/tmp_files/Label_x.scp
  std::string data_files_list_path = setup.htk_tmp_files_path+label+".scp";
  // Open the new scp file
  std::ofstream data_files_list(data_files_list_path.c_str());

  // Iteration on the vector of feature matrices to write each of them in a HTK format file
  for(std::vector<FeatureMatrix>::const_iterator it = feature_matrix_array.begin(); 
      it != feature_matrix_array.end(); it++){

    // Add the number of samples of the feature matrix
    sample_numbers.push_back(it->get_samples_number());

    // Define the path of the data file corresponding to the (*it) matrix: ?/tmp_files/Label_x_i.dat
    std::string data_file_name = setup.htk_tmp_files_path+label
      +"_"+std::to_string(i)+".dat"; //C++11

    // Open the new data file
    std::ofstream data_file(data_file_name.c_str());

    // Define the HTK header
    HTKHeader header;
    header.BytesPerSample = it->get_feature_vector_size()*4; 
    header.nSamples = it->get_samples_number();
    // Write the header to the data file
    header.write_to_file(data_file);
    // Write the feature matrix to the data file
    it->write_to_file(data_file); 
    // Close the data file
    data_file.close();

    // Write the path of the data file to the scp file
    data_files_list << data_file_name.c_str() << "\n";

    i++;
  }
  data_files_list.close();
  
  // Definition of the number of states for the HMM
  int state_number;
  switch(num_states_def){
    case StatesNumDefs::median:
      state_number = ceil(std::sqrt(median(sample_numbers))); //C++11
      break;
    case StatesNumDefs::linear_scaling:
      state_number = ceil(median(sample_numbers)/10);
      break;
    default:
      state_number = num_states_def;
      break;
  }

  if(state_number < 3) 
    state_number = 3;

  int dim = feature_matrix_array[0].get_feature_vector_size();

  // Instantiate a new HMM object   
  HMM hmm(feature_matrix_array[0].get_label(), state_number, dim, emission_type, mixtures_nb);

  std::string hmm_path = setup.htk_tmp_files_path +hmm.name_+".hmm";
  // Write the HMM to an HTK format file
  hmm.write_to_file(hmm_path);

  // Inialize the HMM with the HInit command, the HMM file and the data files
  // Write the initialized HMM in tmp/
  std::string command = "HInit -A -T 1 -M "+setup.htk_tmp_files_path+" "+hmm_path+" -S "+data_files_list_path;
  std::string output = tools::execute_command(command);
  if(print_output)
    ROS_INFO("%s", output.c_str());

  // Train the HMM with the HRest command, the initialized HMM model and the data files
  // command = "HRest -A -T 1 -M "+setup.output_path+" "+hmm_path+" -S "+data_files_list_path;
  command = "HRest -i 30 -A -T 1 -M "+setup.output_path+" "+setup.htk_tmp_files_path+label+" -S "+data_files_list_path;
  output = tools::execute_command(command);
  if(print_output)
    ROS_INFO("%s", output.c_str());
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
