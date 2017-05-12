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
#include "action_recognition/SensorFeatureVector.hpp"
#include "action_recognition/SensorFeatureVectorExtended.hpp"
#include "rapidxml/rapidxml.hpp"

namespace bf = boost::filesystem;

DataHandler::DataHandler(Setup setup):labels_(setup.labels_list_path, setup.grammar_net_path, setup.dict_path, setup.grammar_path){}

void DataHandler::raw_data_from_file_to_feature_matrices(std::string seg_dir, std::string raw_data_dir,
                                                         NormalizationType normalization_type){

  bf::directory_iterator end_it;
  bf::directory_iterator action_it(raw_data_dir);
  for(action_it; action_it != end_it; action_it++){ //iteration on directories (1 directory = 1 action)
  
    bf::path path_action_dir = action_it->path();

    if( bf::is_directory(path_action_dir) && !is_hidden(path_action_dir)){
      
      for(bf::directory_iterator file_it(path_action_dir); file_it != end_it; file_it++){
 
        bf::path file_path = file_it->path();
        if(bf::is_regular_file(file_path) && !is_hidden(file_path)){ // iteration on files 
    
          std::queue<std::pair<std::string,std::pair<int, int> > > seg_queue = parse_seg_file(seg_dir+"/"+get_last_dir_name(file_path)+"/"+get_file_name(file_path)+".xml");
        
          rapidxml::xml_document<> doc;
          rapidxml::xml_node<> * root_node;
          std::ifstream data_file(file_path.c_str());
          std::vector<char> buffer((std::istreambuf_iterator<char>(data_file)), std::istreambuf_iterator<char>());
          buffer.push_back('\0');
          doc.parse<0>(&buffer[0]);
  
          root_node = doc.first_node("Data");
          int count = 0;
          std::pair<std::string,std::pair<int, int> > seg_element = seg_queue.front();
          seg_queue.pop();
          int begin = seg_element.second.first;
          int end = seg_element.second.second;
          FeatureMatrix fm(seg_element.first);
          labels_.add_label(seg_element.first);

          for (rapidxml::xml_node<> * feature_vector_node = root_node->first_node("FeatVect"); feature_vector_node; 
               feature_vector_node = feature_vector_node->next_sibling()){

            if(count > end - begin){
              label_features_map_[seg_element.first].push_back(std::move(fm)); //C++11

              count = 0;
              if(!seg_queue.empty()){
                seg_element = seg_queue.front(); 
                seg_queue.pop();
                fm = FeatureMatrix(seg_element.first);
                labels_.add_label(seg_element.first);
                begin = seg_element.second.first;
                end = seg_element.second.second;       
              }  
            }

            fm.new_feature_vector();
            count++; 

            for (rapidxml::xml_node<> * node = feature_vector_node->first_node(); node; node = node->next_sibling()){
              std::vector<float> vector;
              std::stringstream string_stream(node->value());
              int n;
              while(string_stream >> n)
                vector.push_back(n);
             
              if(vector.size() != SENSOR_FEATURE_VECTOR_SIZE && vector.size() != SENSOR_FEATURE_VECTOR_EXTENDED_SIZE)
                fm.add_flags(vector);
              else
                fm.add_sensor_feature_vector(vector);
            }       
          }
        }
      }
    }
  }
}

void DataHandler::print_map(void){
  std::map<std::string, std::vector<FeatureMatrix> >::iterator it_map = label_features_map_.begin();
  for(; it_map != label_features_map_.end(); it_map++){
    for(std::vector<FeatureMatrix>::iterator it_matrix = it_map->second.begin(); it_matrix != it_map->second.end(); it_matrix++)
      it_matrix->print();
  }
}

void DataHandler::normalize(void){
std::map<std::string, std::vector<FeatureMatrix> >::iterator it_map = label_features_map_.begin();
  for(; it_map != label_features_map_.end(); it_map++){
    for(std::vector<FeatureMatrix>::iterator it_matrix = it_map->second.begin(); it_matrix != it_map->second.end(); it_matrix++)
      it_matrix->normalize();
  }
}

std::pair<std::map<std::string, std::vector<FeatureMatrix> >::iterator,
          std::map<std::string, std::vector<FeatureMatrix> >::iterator > DataHandler::get_map_iterator(){
  return std::make_pair(label_features_map_.begin(),label_features_map_.end());
}

Labels DataHandler::get_labels(void){return labels_;}

/****************** Private functions********************/

std::string DataHandler::get_file_name(bf::path path){return path.stem().c_str();}

std::string DataHandler::get_last_dir_name(bf::path path){
  std::string path_str = path.c_str();
  std::string dir_and_file = path_str.substr(path_str.find_last_of("/", path_str.find_last_of("/")-1)+1);
  return dir_and_file.substr(0, dir_and_file.find_last_of("/"));

}

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
    segmentation_queue.push(std::pair<std::string,std::pair<int,int> >(motion_label_node->first_attribute("name")->value(), 
                                                                       std::pair<int,int>(atoi(motion_label_node->first_attribute("startPoint")->value()), 
                                                                                          atoi(motion_label_node->first_attribute("endPoint")->value())))); 

  return segmentation_queue;
}

bool DataHandler::is_hidden(bf::path p)
{
  std::string name = p.filename().string();
  if((name != ".." && name != "."  && name[0] == '.') || name.find("~")!=std::string::npos)    
    return true;

  return false;
}


