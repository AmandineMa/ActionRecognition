/**
 * \file data_file_processing.cpp
 * \brief This ROS node aims to custom the feature vectors contained in files of recorded data.
 *
 * Thanks to the ROS params, it is possible to choose for the flags vector and for each SensorFeatureVector the desired size.
 * If the original SensorFeatureVector is of size 7, it can be down to 3 or 0.
 * If the original SensorFeatureVector is of size 3, it can be down to 0.
 * For now, the flags vector can only be customized by disappearing, so from a size N, it can be down to 0.
 * The modified data are written in a new file.
 * \author Amandine M.
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <boost/filesystem.hpp>
#include <fstream>
#include <algorithm>  
#include <vector>

#include "action_recognition/common.hpp"
#include "rapidxml/rapidxml.hpp"

namespace bf = boost::filesystem;


int main(int argc, char** argv){

  // ROS node initialization
  ros::init(argc, argv, "data_file_processing");
  ros::NodeHandle node;

  std::string data_path;
  std::string data_path2;
  std::vector<int> vector_features_size; 

  node.getParam("data_file_processing/data_path", data_path);
  node.getParam("data_file_processing/data_path2", data_path2);
  node.getParam("feature_vector_size", vector_features_size);
  
  bf::directory_iterator end_it;

  if (bf::create_directory(data_path2))
    std::cout<<"data_path2 created"<<std::endl;

  // If the iterator is on a directory and that directory is not empty
  if( bf::is_directory(data_path) && !tools::is_hidden(data_path)){

    // Iterate files of the directory (1 file = 1 data example of the general activity)
    for(bf::directory_iterator file_it(data_path); file_it != end_it; file_it++){
 
      bf::path file_path = file_it->path();

      if(bf::is_regular_file(file_path) && !tools::is_hidden(file_path)){  
        rapidxml::xml_document<> doc;
        rapidxml::xml_node<> * root_node; 
        std::ifstream f(file_path.c_str());
        std::ofstream f2(data_path2+file_path.filename().c_str());
        std::vector<char> buffer((std::istreambuf_iterator<char>(f)), 
                                 std::istreambuf_iterator<char>()); 
        
        buffer.push_back('\0'); 
        doc.parse<0>(&buffer[0]);
        root_node = doc.first_node("Data"); 
        f2 << "<Data>" << std::endl;
        rapidxml::xml_node<> * feature_vector_node = root_node->first_node("FeatVect");   
       
        // Iteration of each FeatureVector of the data file
        do{
          f2 << "<FeatVect>" << std::endl; 
          rapidxml::xml_node<> * node = feature_vector_node->first_node();  
          // Iterator for the vector containing the size of each sub-vector
          std::vector<int>::iterator vector_it =  vector_features_size.begin(); 
          // Iteration on each element (SensorFeatureVectors, Flags) of the data file
          do{
            int count = 0; 
            float n;
            std::stringstream string_stream(node->value());

            // Check if the element is a vector of flags
            if((std::string(node->name())).compare("Flags") == 0){
              // If the expected size of the vector of flags is different of 0,
              // the whole vector is re-written in the new file.
              // Otherwise, none of the elements of the vector is re-written
              if(*vector_it != 0){
                f2 << "<Flags>";
                while(string_stream >> n){
                  f2 << n << " ";
                }
                f2 << "</Flags>";
              }                
            }
            // If else, the element is a SensorFeatureVector
            else{
              // If the new size of the new size of the SensorFeatureVector is 0,
              // nothing is written in the new file.

              // If the new size of the element is a simple SensorFeatureVector,
              // only the 3 first elements of the vector will be written in the new file
              if(*vector_it == SensorFeatureVectorType::SensorFeatureVector){
                f2<<"<SensFeat>";
                while(string_stream >> n 
                      && count < SensorFeatureVectorType::SensorFeatureVector){
                  f2 << n << " ";
                  count++;
                }
                f2<<"</SensFeat>";

                // If the new size of the element is a SensorFeatureVectorExtended,
                // all the elements of the vector will be written in the new file
                // TODO : check the initial size of the SensorFeatureVector, 
                //        if it is 3, it cannot work
              }else if(*vector_it == SensorFeatureVectorType::SensorFeatureVectorExtended){
                f2<<"<SensFeatExt>";
                while(string_stream >> n){
                  f2 << n << " ";
                }
                f2<<"</SensFeatExt>";
              }
            }
            vector_it++;
          }while (node = node->next_sibling());
          f2 << "</FeatVect>" << std::endl;
        }while (feature_vector_node = feature_vector_node->next_sibling());

        f2 << "</Data>" << std::endl;
        f2.close();
      }
    }
  }

  return 0;
}
