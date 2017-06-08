#ifndef DATAHANDLER_HPP
#define DATAHANDLER_HPP

/**
 * \file DataHandler.hpp
 * \brief DataHandler class
 * \author Amandine M.
 */

#include <map>
#include <vector>
#include <string>
#include <queue>
#include <boost/filesystem.hpp>
#include <fstream>

#include "action_recognition/common.hpp"
#include "action_recognition/FeatureMatrix.hpp"
#include "action_recognition/Labels.hpp"
#include "action_recognition/Setup.hpp"

#define HTK_SAMPLE_SIZE 4

/**
 * \brief Class that handles data from sensors and format them to #FeatureMatrix
 */
class DataHandler{
private:
  std::map<std::string, std::vector<FeatureMatrix> > label_features_map_; /** Map of {label, vector of FeatureMatrix}
                                                                              Association of FeatureMatrix with the same label */
  Labels labels_; /** Labels object */
  //std::map<std::string, bool> map_features_;

  /** 
   * \brief Parse an XML segmentation file to a queue of pairs of {"action", {start_point, end_point} }
   * Example :
   *    take_object {0, 324}
   *    put_object {325, 600}
   * \param Path of the XML file
   */
  std::queue<std::pair<std::string,std::pair<int, int> > > parse_seg_file(std::string file_path);
 

void write_HTK_header_to_file(std::ofstream& data_file, int bytes_per_sample, int samp_nb);

public:
  /** 
   * \brief Constructor of a data handler
   * \param #Setup for the input and output files
   */
  DataHandler(Setup setup);

  /**
   * \brief Get raw data from training files to format them to #FeatureMatrix
   * \param Location of the segmentation files
   * \param Location of the data files
   */
  void raw_data_from_file_to_feature_matrices(Setup setup);

  int feature_matrices_to_file(Setup setup, const std::vector<FeatureMatrix> &feature_matrix_array);

  /**
   * \brief Get raw data from file to format it to #FeatureMatrix
   * \param Location of the data file
   * \retval Generated #FeatureMatrix from the data file
   */
  FeatureMatrix raw_data_from_file_to_feature_matrix(std::string raw_data_file);

 void raw_data_from_files_to_data_files(Setup &setup, NormalizationType normalization_type);

  void seg_files_to_MLF(Setup &setup);

  /**
   * \brief Check if the segmentation is valid
   */
  void check_segmentation(void);
  /** 
   * \brief Normalize the values of all the #FeatureMatrix with the method given in parameter
   * \param #NormalizationType
   */
  void normalize(NormalizationType normalization_type);
  /** 
   * \brief Get a pair of iterators of the map
   * \retval Pair of iterators {map_begin_iterator, map_end_iterator}
   */
  std::pair<std::map<std::string, std::vector<FeatureMatrix> >::iterator,
            std::map<std::string, std::vector<FeatureMatrix> >::iterator > get_map_iterator(void);

  Labels get_labels(void);

  void print_map(void);

};


#endif
