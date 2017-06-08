#include <map>
#include <vector>
#include <queue>
#include <string>
#include <boost/filesystem.hpp>
#include <fstream>
#include <utility>
#include <math.h>

#include "action_recognition/DataHandler.hpp"
#include "action_recognition/common.hpp"
#include "action_recognition/FeatureMatrix.hpp"
#include "rapidxml/rapidxml.hpp"
#include "action_recognition/HTKHeader.hpp"

namespace bf = boost::filesystem;

//TODO:split class into 2 classes : base class DataHandler, child class SegDataHandler with map in child class

DataHandler::DataHandler(Setup setup/*, std::map<std::string, bool> map_features*/):labels_
                                                             (setup.labels_list_path,
                                                              setup.grammar_net_path, 
                                                              setup.dict_path,
                                                              setup.grammar_path)
                                                              /*map_features_(map_features)*/{}

void DataHandler::raw_data_from_file_to_feature_matrices(Setup setup){

  // To iterate the contents of a directoryg
  bf::directory_iterator end_it;
  
  std::string data_path = setup.data_path;

  // If the iterator is on a directory and that directory is not empty
  if( bf::is_directory(data_path) && !tools::is_hidden(data_path)){
      
    // Iterate files of the directory (1 file = 1 data example of the general activity)
    for(bf::directory_iterator file_it(data_path); file_it != end_it; file_it++){
      bf::path file_path = file_it->path();
      if(bf::is_regular_file(file_path) && !tools::is_hidden(file_path)){ 
    
        // Get the segmentation file corresponding to the data file
        /*
          queue of pairs
          pairs of {"action", {start_point, end_point} }
          Example :
          take_object {0, 324}
          put_object {325, 600}
        */
         
        std::queue<std::pair<std::string,std::pair<int, int> > > seg_queue 
          = parse_seg_file(setup.seg_files_path+tools::get_file_name(file_path)+".xml");
        // To parse the data file
        rapidxml::xml_document<> doc;
        rapidxml::xml_node<> * root_node;
        std::ifstream data_file(file_path.c_str());
        std::vector<char> buffer((std::istreambuf_iterator<char>(data_file)), std::istreambuf_iterator<char>()); 
        buffer.push_back('\0'); 
        doc.parse<0>(&buffer[0]);
        root_node = doc.first_node("Data");
         
        // To count the number of FeatureVector to add to a FeatureMatrix
        int count = 0;
        std::pair<std::string,std::pair<int, int> > seg_element;      
        FeatureMatrix fm;         
        int begin;        
        int end;

        // Iterate the feature vectors in the data file (do - while)
        rapidxml::xml_node<> * feature_vector_node = root_node->first_node("FeatVect"); 
        do{

          // If there is still some data from the file to read
          if(!seg_queue.empty() && count == 0){
            // Get the first pair from the segmentation queue
            seg_element = seg_queue.front(); 
            seg_queue.pop();
            // Create a new FeatureMatrix for the action
            fm = FeatureMatrix(seg_element.first);
            // Add the label to the Labels object if not already existing
            labels_.add_label(seg_element.first);
            // Get the line of the first vector of the action
            begin = seg_element.second.first;
            // Get the line of the last vector of the action
            end = seg_element.second.second;      
          }      

          // Add a new feature vector
          fm.new_feature_vector();
          count++; 

          // Iterate the sensor feature vectors and the flags in the data file
          rapidxml::xml_node<> * node = feature_vector_node->first_node();
          //std::map<std::string, bool>::iterator map_features_it = map_features_.begin();
          do{
            //if(map_features_it->second){
              std::vector<float> vector;
              std::stringstream string_stream(node->value());
              float n;
              // While there is an integer, it is push backed to the vector
              while(string_stream >> n)
                vector.push_back(n);
              
              // When the vector is filled, it is added to the feature vector 
              // as a flags vector or a sensor feature vector
              if((std::string(node->name())).compare("Flags") == 0)
                fm.set_flags(vector);
              else
                fm.add_sensor_feature_vector(vector);
              //}
              //map_features_it++;
          }while(node = node->next_sibling());   
        
          // If the FeatureMatrix is filled
          if(count > end - begin){  
            // Add the FeatureMatrix to the map, to the corresponding label
            label_features_map_[seg_element.first].push_back(std::move(fm)); //C++11

            // Reset the count
            count = 0;
          }
        }while (feature_vector_node = feature_vector_node->next_sibling());
                 
      }
    }
  }
}

//TODO: check file extension
FeatureMatrix DataHandler::raw_data_from_file_to_feature_matrix(std::string raw_data_file_path){
  FeatureMatrix fm;
  // To parse the data file
  rapidxml::xml_document<> doc;
  rapidxml::xml_node<> * root_node;
  std::ifstream data_file(raw_data_file_path.c_str());
  std::vector<char> buffer((std::istreambuf_iterator<char>(data_file)), std::istreambuf_iterator<char>());
  //std::cout << raw_data_file_path << std::endl;
  buffer.push_back('\0');
  doc.parse<0>(&buffer[0]);
  root_node = doc.first_node("Data");
  // Iterate the feature vectors in the data file (do - while)
  int count = 0;
  rapidxml::xml_node<> * feature_vector_node = root_node->first_node("FeatVect"); 
  do{
    // Add a new feature vector
    fm.new_feature_vector();
    count++;
    // Iterate the sensor feature vectors and the flags in the data file
    rapidxml::xml_node<> * node = feature_vector_node->first_node();
    //std::map<std::string, bool>::iterator map_features_it = map_features_.begin();
    do{
      //if(map_features_it->second){
        std::vector<float> vector;
        std::stringstream string_stream(node->value());
        float n;
        // While there is an integer, it is push backed to the vector
        while(string_stream >> n)
          vector.push_back(n);
             
        // When the vector is filled, it is added to the feature vector 
        // as a flags vector or a sensor feature vector
        if((std::string(node->name())).compare("Flags") == 0)
          fm.set_flags(vector);
        else
          fm.add_sensor_feature_vector(vector);
        //}
        // map_features_it++;
    }while(node = node->next_sibling());   

  }while (feature_vector_node = feature_vector_node->next_sibling());
  return fm;
}

void DataHandler::raw_data_from_files_to_data_files(Setup &setup, NormalizationType normalization_type){
  setup.data_list_path = setup.htk_tmp_files_path+"data_list.scp";
  std::ofstream file_list(setup.data_list_path);
  bf::directory_iterator end_it;
  std::string output_data_path = setup.htk_tmp_files_path+"dat/";
  boost::filesystem::create_directory(output_data_path);
  for(bf::directory_iterator file_it(setup.data_path); file_it != end_it; file_it++){  

    bf::path file_path = file_it->path();
    if(bf::is_regular_file(file_path) && !tools::is_hidden(file_path)){ 
    
      FeatureMatrix fm = raw_data_from_file_to_feature_matrix(file_path.c_str());
      fm.normalize(static_cast<NormalizationType>(normalization_type));
      // Open the new data file
      std::string file_name = output_data_path+tools::get_file_name(file_path)+".dat";
      std::ofstream data_file(file_name);
      // write_HTK_header_to_file(data_file, fm.get_feature_vector_size(), fm.get_samples_number());
      fm.write_to_file(data_file, FeatureFileFormat::dat);
      //data_file.close();
      file_list << file_name << "\n";         
    }
  }
  file_list.close();
}

int DataHandler::feature_matrices_to_file(Setup setup, const std::vector<FeatureMatrix> &feature_matrix_array){
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

    //write_HTK_header_to_file(data_file, it->get_feature_vector_size(), it->get_samples_number());
    // Write the feature matrix to the data file
    it->write_to_file(data_file, FeatureFileFormat::dat); 
    // Close the data file
    //data_file.close();

    // Write the path of the data file to the scp file
    data_files_list << data_file_name.c_str() << "\n";

    i++;
  }
  data_files_list.close();

  return tools::median(sample_numbers); 

}

void DataHandler::seg_files_to_MLF(Setup &setup){
  setup.mlf_path = setup.htk_tmp_files_path+"segmentation.mlf";
  std::ofstream mlf_file(setup.mlf_path);
  mlf_file << "#!MLF!#\n";
  bf::directory_iterator end_it;
  for(bf::directory_iterator file_it(setup.seg_files_path); file_it != end_it; file_it++){
 
    bf::path file_path = file_it->path();
    if(bf::is_regular_file(file_path) && !tools::is_hidden(file_path)){

      // Get the segmentation file corresponding to the data file
      /*
        queue of pairs
        pairs of {"action", {start_point, end_point} }
        Example :
        take_object {0, 324}
        put_object {325, 600}
      */
      mlf_file << "\"*/"+tools::get_file_name(file_path)+".lab\"\n";
      std::queue<std::pair<std::string,std::pair<int, int> > > seg_queue 
        = parse_seg_file(setup.seg_files_path+tools::get_file_name(file_path)+".xml");
      while(!seg_queue.empty()){
        mlf_file << seg_queue.front().second.first*1e5 << " " 
                 << seg_queue.front().second.second*1e5 << " "
                 << seg_queue.front().first << "\n";
        seg_queue.pop();
      }
      mlf_file << ".\n";
    }
  }
  mlf_file.close();
}

void DataHandler::print_map(void){
  std::map<std::string, std::vector<FeatureMatrix> >::iterator it_map = label_features_map_.begin();
  for(; it_map != label_features_map_.end(); it_map++){
    for(std::vector<FeatureMatrix>::iterator it_matrix = it_map->second.begin(); it_matrix != it_map->second.end(); it_matrix++)
      it_matrix->print();
  }
}

void DataHandler::normalize(NormalizationType normalization_type){
  std::map<std::string, std::vector<FeatureMatrix> >::iterator it_map = label_features_map_.begin();
  for(; it_map != label_features_map_.end(); it_map++){
    for(std::vector<FeatureMatrix>::iterator it_matrix = it_map->second.begin(); it_matrix != it_map->second.end(); it_matrix++)
      it_matrix->normalize(normalization_type);
  }
}

std::pair<std::map<std::string, std::vector<FeatureMatrix> >::iterator,
          std::map<std::string, std::vector<FeatureMatrix> >::iterator > DataHandler::get_map_iterator(){
  return std::make_pair(label_features_map_.begin(),label_features_map_.end());
}

Labels DataHandler::get_labels(void){return labels_;}

/****************** Private functions********************/

std::queue<std::pair<std::string,std::pair<int, int> > > DataHandler::parse_seg_file(std::string file_path){
  std::queue<std::pair<std::string,std::pair<int, int> > > segmentation_queue;
  rapidxml::xml_document<> doc;
  rapidxml::xml_node<> * root_node;
  std::ifstream seg_file(file_path.c_str());
  std::vector<char> buffer((std::istreambuf_iterator<char>(seg_file)), std::istreambuf_iterator<char>());
  buffer.push_back('\0');
  doc.parse<0>(&buffer[0]);
  root_node = doc.first_node("MotionLabeling");
  for (rapidxml::xml_node<> * motion_label_node = root_node->first_node("MotionLabel"); motion_label_node; 
       motion_label_node = motion_label_node->next_sibling())
    segmentation_queue.push(std::pair<std::string,std::pair<int,int> >
                            (motion_label_node->first_attribute("name")->value(), 
                             std::pair<int,int>(atoi(motion_label_node->first_attribute("startPoint")->value()), 
                                                atoi(motion_label_node->first_attribute("endPoint")->value())))); 

  return segmentation_queue;
}



