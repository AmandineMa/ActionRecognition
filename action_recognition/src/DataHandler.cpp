#include <map>
#include <vector>
#include <queue>
#include <string>
#include <boost/filesystem.hpp>
#include <fstream>
#include <utility>

#include "action_recognition/DataHandler.hpp"
#include "action_recognition/common.hpp"
#include "action_recognition/FeatureMatrix.hpp"
#include "rapidxml/rapidxml.hpp"

namespace bf = boost::filesystem;

DataHandler::DataHandler(Setup setup):labels_(setup.labels_list_path, 
                                              setup.grammar_net_path, 
                                              setup.dict_path, 
                                              setup.grammar_path){}

void DataHandler::raw_data_from_file_to_feature_matrices
(std::string seg_dir, std::string raw_data_dir){

  // To iterate the contents of a directory
  bf::directory_iterator end_it;
  bf::directory_iterator action_it(raw_data_dir);

  // Iterate directories (1 directory = 1 general activity)
  for(action_it; action_it != end_it; action_it++){
  
    bf::path path_action_dir = action_it->path();

    // If the iterator is on a directory and that directory is not empty
    if( bf::is_directory(path_action_dir) && !tools::is_hidden(path_action_dir)){
      
      // Iterate files of the directory (1 file = 1 data example of the general activity)
      for(bf::directory_iterator file_it(path_action_dir); file_it != end_it; file_it++){
 
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
            = parse_seg_file(seg_dir+"/"+tools::get_last_dir_name(file_path)+"/"+tools::get_file_name(file_path)+".xml");
          // To parse the data file
          rapidxml::xml_document<> doc;
          rapidxml::xml_node<> * root_node;
          std::ifstream data_file(file_path.c_str());
          std::vector<char> buffer((std::istreambuf_iterator<char>(data_file)), std::istreambuf_iterator<char>()); 
          buffer.push_back('\0'); 
          std::cout << file_path << std::endl;
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
            do{
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
}

//TODO: check file extension
FeatureMatrix DataHandler::raw_data_from_file_to_feature_matrix(std::string raw_data_file_path){
  FeatureMatrix fm;
  // To parse the data file
  rapidxml::xml_document<> doc;
  rapidxml::xml_node<> * root_node;
  std::ifstream data_file(raw_data_file_path.c_str());
  std::vector<char> buffer((std::istreambuf_iterator<char>(data_file)), std::istreambuf_iterator<char>());
  std::cout << raw_data_file_path << std::endl;
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
    do{
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

    }while(node = node->next_sibling());   

  }while (feature_vector_node = feature_vector_node->next_sibling());
  return fm;
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




