#include <ros/ros.h>
#include <ros/console.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>
#include <string>
#include <stdio.h>
#include <cstdlib>
#include <boost/regex.hpp>

#include "action_recognition/DataHandler.hpp"
#include "action_recognition/TrainHMM.hpp"
#include "action_recognition/Setup.hpp"
#include "action_recognition/HTKHeader.hpp"

namespace bf = boost::filesystem;

void generate_MMF_from_HMM_files(Setup setup, Labels labels);
void build_threshold_model(Setup setup);

int main(int argc, char** argv){
  ros::init(argc, argv, "action_reco");

  ros::NodeHandle node;
  
  ros::Rate rate(10.0); 

  std::string path_root;
  std::string path_segmentation;
  std::string path_data;
  bool print_output;

  bool enable_recognition; 
  bool enable_training;   
  bool isolated_unit_init;
  bool isolated_unit_flat_init;    
  bool embedded_unit_flat_init;
  bool isolated_unit_training;
  bool embedded_unit_training;  
  int state_num_def;
  int iterations_nb;
  int emission_type;
  int mixtures_nb;
  int topology_type;
  int normalization_type;
  bool threshold;

  std::map<std::string, bool> map_features;

  node.getParam("file_setup/path_root", path_root);
  node.getParam("file_setup/path_data", path_data);
  node.getParam("file_setup/path_segmentation", path_segmentation);
  node.getParam("setup/print_output", print_output);

  node.getParam("setup/enable_recognition", enable_recognition);
  node.getParam("setup/enable_training", enable_training);  
  node.getParam("training_options/isolated_unit_init", isolated_unit_init);  
  node.getParam("training_options/isolated_unit_flat_init", isolated_unit_flat_init);
  node.getParam("training_options/isolated_unit_training", isolated_unit_training);   
  node.getParam("training_options/embedded_unit_flat_init", embedded_unit_flat_init); 
  node.getParam("training_options/embedded_unit_training", embedded_unit_training);
  node.getParam("training_options/normalization", normalization_type);
  node.getParam("training_options/state_num_def", state_num_def);
  node.getParam("training_options/iterations_nb", iterations_nb);
  node.getParam("training_options/emission_type", emission_type);
  node.getParam("training_options/mixtures_nb", mixtures_nb);
  node.getParam("training_options/topology_type", topology_type);
  node.getParam("training_options/threshold", threshold);

  Setup setup(path_root,path_data, path_segmentation);
  setup.hmmsdef_path = setup.output_path+"hmmsdef";

  std::string HTK_conf_file_name = path_root+"HTK_conf_file";
  std::ofstream HTK_conf_file(HTK_conf_file_name);
  HTK_conf_file << "NATURALREADORDER = TRUE" << "\n" << "NATURALWRITEORDER = TRUE" << "\n"
                << "FORCEOUT = TRUE" << "\n";
  HTK_conf_file.close();
  setenv("HCONFIG",const_cast<char*>(HTK_conf_file_name.c_str()),true);

  DataHandler datah(setup);

  if(enable_training){
    bf::path path(setup.output_hmm);
    for (bf::directory_iterator end_dir_it, it(path); it!=end_dir_it; ++it)
      bf::remove(it->path());

    std::string data_list_path;
    if(embedded_unit_flat_init || embedded_unit_training){
      datah.raw_data_from_files_to_data_files
        (setup, static_cast<NormalizationType>(normalization_type));
    }

    if(isolated_unit_init || isolated_unit_training || isolated_unit_flat_init || embedded_unit_flat_init){
      datah.raw_data_from_file_to_feature_matrices(setup);
      datah.normalize(static_cast<NormalizationType>(normalization_type));

      std::pair<std::map<std::string, std::vector<FeatureMatrix> >::iterator,
                std::map<std::string, std::vector<FeatureMatrix> >::iterator > it = datah.get_map_iterator();
      int dim = it.first->second[0].get_feature_vector_size();
      for(; it.first != it.second ; it.first++){ 

        if(embedded_unit_flat_init)
          TrainHMM::embedded_unit_flat_init(print_output, setup, it.first->first, 
                                            state_num_def, dim,
                                            static_cast<EmissionType>(emission_type), 
                                            static_cast<TopologyType>(topology_type), 
                                            mixtures_nb);

        if(isolated_unit_init || isolated_unit_training || isolated_unit_flat_init){
          int median_samp_nb = datah.feature_matrices_to_file(setup, it.first->second);

          TrainHMM::train_HMM(it.first->second[0].get_label(), print_output, 
                              isolated_unit_init, isolated_unit_flat_init, 
                              isolated_unit_training, static_cast<EmissionType>(emission_type), 
                              static_cast<TopologyType>(topology_type), 
                              it.first->second[0].get_feature_vector_size(), 
                              median_samp_nb, static_cast<StatesNumDef>(state_num_def), 
                              iterations_nb, setup, mixtures_nb);
        }
      }
    }
    
    datah.get_labels().write_to_file(LabelFileFormats::txt);
    generate_MMF_from_HMM_files(setup, datah.get_labels());
    

    if(embedded_unit_training){
      datah.seg_files_to_MLF(setup);
      TrainHMM::train_HMMs(print_output, setup);
     }
    
    if(threshold){
      Labels new_labels = datah.get_labels();
      new_labels.add_label("threshold");
      build_threshold_model(setup);
      generate_MMF_from_HMM_files(setup, new_labels);
      bool gen_gram_file=false;
      new_labels.write_to_file(LabelFileFormats::txt);
      node.getParam("setup/generate_grammar_file", gen_gram_file);
      if(gen_gram_file)
        new_labels.write_to_file(LabelFileFormats::grammar);
      new_labels.write_to_file(LabelFileFormats::dict);

      ROS_INFO("%s",new_labels.compile_grammar().c_str());
      ROS_INFO("%s", new_labels.test_grammar().c_str());
    }else{

      bool gen_gram_file=false;
      node.getParam("setup/generate_grammar_file", gen_gram_file);
      if(gen_gram_file)
        datah.get_labels().write_to_file(LabelFileFormats::grammar);
      datah.get_labels().write_to_file(LabelFileFormats::dict);

      ROS_INFO("%s", datah.get_labels().compile_grammar().c_str());
      ROS_INFO("%s", datah.get_labels().test_grammar().c_str());
    }
    
  }

  if(enable_recognition){    
    std::string path_dir;
    node.getParam("file_setup/path_data_to_reco", path_dir);
    bf::directory_iterator end_it;    
    std::string dir_data = path_dir+"dat/";
    bf::create_directory(dir_data);
    std::ofstream file_list(path_dir+"dat/file_list.scp"); 

    for(bf::directory_iterator file_it(path_dir); file_it != end_it; file_it++){  

      bf::path file_path = file_it->path();
      if(bf::is_regular_file(file_path) && !tools::is_hidden(file_path)){ 
    
        FeatureMatrix fm = datah.raw_data_from_file_to_feature_matrix(file_path.c_str());
        fm.normalize(static_cast<NormalizationType>(normalization_type));
        // Open the new data file
        std::string file_name = dir_data+tools::get_file_name(file_path)+".dat";
        std::ofstream data_file(file_name);
        fm.write_to_file(data_file, FeatureFileFormat::dat);
        file_list << file_name << "\n";         
      }
    }
    file_list.close();

    std::string command = "HVite -A -T 1 -C "+HTK_conf_file_name +
      " -H "+setup.hmmsdef_path
      +" -i "+setup.output_path+"reco.mlf"
      +" -w "+setup.grammar_net_path
      +" "+setup.dict_path
      +" "+setup.labels_list_path
      +" -S "+dir_data+"file_list.scp";
    std::string output = tools::execute_command(command);
    ROS_INFO("%s", output.c_str());

    command = "HResults -A -T 1 -C "+HTK_conf_file_name +
      " -p -I "+path_root+"test_ref.mlf"
      +" "+setup.labels_list_path
      +" "+setup.output_path+"reco.mlf";
    output = tools::execute_command(command);
    ROS_INFO("%s", output.c_str());
  }

  return 0;
};


void generate_MMF_from_HMM_files(Setup setup, Labels labels){
  std::pair<std::set<std::string>::iterator, std::set<std::string>::iterator> it = labels.get_iterator();
  std::ofstream MMF_file(setup.hmmsdef_path);
  for(; it.first != it.second ; it.first++){
    std::ifstream hmm((setup.output_hmm+(*it.first)).c_str());
    MMF_file << hmm.rdbuf();
  }
  MMF_file.close();

  std::ifstream MMF_file_2(setup.hmmsdef_path);
  std::ofstream temp(setup.output_hmm+"temp");
  std::string line;
  int count = 0;
  while(std::getline(MMF_file_2, line)){
    if( ( (line.find("<VECSIZE>") == std::string::npos && line.find("~o") == std::string::npos 
            &&  line.find("<STREAMINFO>") == std::string::npos) && count >= 3) ||
        ( (line.find("<VECSIZE>") != std::string::npos || line.find("~o") != std::string::npos
           || line.find("<STREAMINFO>") != std::string::npos) && count < 3) ){
      temp << line << std::endl;
    }
    count++;
  }
  MMF_file_2.close();
  temp.close();
  remove((setup.hmmsdef_path).c_str());
  rename((setup.output_hmm+"temp").c_str(), (setup.hmmsdef_path).c_str());
}


void build_threshold_model(Setup setup){

  std::ifstream mmf_file(setup.hmmsdef_path);
  std::ofstream threshold_file(setup.output_hmm+"threshold");
  std::string line;
  int nb_states=0;
  boost::smatch match;
  boost::regex reg_exp("\\d+");
  while(std::getline(mmf_file, line)){
    if(line.find("<NUMSTATES>") != std::string::npos ){
      if(boost::regex_search(line, match, reg_exp))
        nb_states = nb_states+std::stoi(match[0])-2;
    }
  }
  mmf_file.clear();
  mmf_file.seekg(0, mmf_file.beg);
  std::vector<std::vector<float> > trans_mat;
  int count_line = 0; 
  int count_state = 2;
  bool get_trans_mat = false;
  int index_filling = 0;
  int count_transmat_row = 0;
  int count_transmat_line = 0;
  bool first_line_transmat = true;
  int num_state_unit_model = 0;
  bool skip = true;
  std::vector<float> end_proba_;
  while(std::getline(mmf_file, line)){
    if((line.find("<VECSIZE>") == std::string::npos 
        && line.find("~o") == std::string::npos 
        && line.find("<STREAMINFO>") == std::string::npos 
        && line.find("~h") == std::string::npos
        && line.find("<BEGINHMM>") == std::string::npos
        && line.find("<ENDHMM>") == std::string::npos
        && count_line >= 6 ) 
       || count_line < 6){
      if(line.find("<STATE>") != std::string::npos){
        threshold_file << "<STATE> "<< count_state << std::endl;
        count_state++;
      }else if(line.find("~h") != std::string::npos)
        threshold_file << "~h \"threshold\"" << std::endl;
      else if(line.find("<NUMSTATES>") != std::string::npos){
        if(count_line < 6)
          threshold_file << "<NUMSTATES> "<< std::to_string(nb_states+2) << std::endl;
        boost::regex_search(line, match, reg_exp);
        num_state_unit_model += std::stoi(match[0])-2;  
        index_filling = count_transmat_row;
        get_trans_mat = false;
        skip = false;
      }else if(line.find("<TRANSP>") != std::string::npos){
        get_trans_mat = true;
        first_line_transmat = true;
      }else if(get_trans_mat){
        float n;
        if(!first_line_transmat && !skip){
          trans_mat.emplace_back(std::vector<float>(nb_states,0));
          std::stringstream string_stream(line);
          count_transmat_row = 0;
          while(string_stream >> n){
            int transmat_index = count_transmat_row-1+index_filling;
            if(count_transmat_row != 0 && transmat_index < num_state_unit_model)
                trans_mat.back().at(transmat_index)=n;
            count_transmat_row++;
          }
          count_transmat_row = count_transmat_row - 2 + index_filling;

          float new_a = (1.0-trans_mat.back().at(count_transmat_line))/float(nb_states);
          end_proba_.push_back(new_a);
          for(int i = 0; i < nb_states; i++){
            if(i != count_transmat_line)
              trans_mat.back().at(i)=new_a;
          }
          count_transmat_line++;
          if(count_transmat_line == num_state_unit_model)
            skip = true;
        }
        first_line_transmat = false; 
      }else
        threshold_file << line << std::endl;

    }

    count_line++;
  } 
  threshold_file.setf(std::ios::fixed);
  threshold_file.precision(6);
  threshold_file << "   <TransP> " << nb_states+2 << "\n";
  std::vector<float> start_proba_(nb_states,1.0/nb_states);
  std::ostream_iterator<float> it_file(threshold_file, " ");
  threshold_file << float(0) << " "; std::copy(start_proba_.begin(), start_proba_.end(), it_file); threshold_file << float(0) << "\n";
  std::vector<float>::iterator it_vector = it_vector = end_proba_.begin();
  std::vector<std::vector<float> >::iterator it_tr_ = trans_mat.begin();
  for(; it_tr_ != trans_mat.end() ; it_tr_++){
    threshold_file << float(0) << " "; std::copy(it_tr_->begin(), it_tr_->end(), it_file); threshold_file << *it_vector <<"\n";
    it_vector++;
  }
  std::vector<float> zeros(trans_mat.size()+2,0);
  std::copy(zeros.begin(), zeros.end(), it_file);
  threshold_file << "\n";

  threshold_file << "<ENDHMM>";
  threshold_file.close();
  threshold_file.close();
  //remove((setup.).c_str());
  // rename((dir_path+"temp").c_str(), (dir_path+"threshold").c_str());

}
