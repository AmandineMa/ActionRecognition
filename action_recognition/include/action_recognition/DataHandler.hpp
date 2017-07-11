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


/**
 * \brief Class that handles data from sensors and format them to #FeatureMatrix
 */
class DataHandler{
private:

  /** 
   * \brief Map of {label, vector of FeatureMatrix}
   *
   * Association of FeatureMatrix with the same label
   */
  std::map<std::string, std::vector<FeatureMatrix> > label_features_map_;

  //Labels object
  Labels labels_;
 

  /** 
   * \brief Parse an XML segmentation file to a queue of pairs of {"action", {start_point, end_point} }
   * Example :
   *    take_object {0, 324}
   *    put_object {325, 600}
   * \param Path of the XML file
   */
  std::queue<std::pair<std::string,std::pair<int, int> > > parse_seg_file(std::string file_path);
 
public:
  /** 
   * \brief Constructor of a data handler
   * \param #Setup for the input and output files
   */
  DataHandler(Setup setup);

  /**
   * \brief Get raw data from training files to format them to #FeatureMatrix
   *
   * For each data file, parse it according to the different actions. 
   * Each part of the file corresponding to an action is saved in a #FeatureMatrix.
   * This #FeatureMatrix is saved in the #label_features_map_, with its label as a key.
   * \param setup Contain the paths of segmentation and data files 
   */
  void raw_data_from_files_to_feature_matrices(Setup setup);

  /**
   * \brief Write in individual HTK files each #FeatureMatrix belonging to the vector given in parameter 
   * (all examples of the same action)
   * \param setup Contain the folder path where to write the data files
   * \retval Median number of samples for this action, computed from all the data files
   */
  int feature_matrices_to_data_files(Setup setup, const std::vector<FeatureMatrix> &feature_matrix_array);

  /**
   * \brief Get raw data from the file to format it to one #FeatureMatrix
   * \param raw_data_file Location of the data file
   * \retval Generated #FeatureMatrix from the data file
   */
  FeatureMatrix raw_data_from_file_to_feature_matrix(std::string raw_data_file);

 /**
   * \brief Format each raw data file to a HTK file (without parsing/segmentation)
   * \param setup Contain the folder path where to write the data files
   * \param normalization_type #NormalizationType
   * \param seg_to_get_samp_nb Boolean to signal if there are existing segmentation files, 
   * so the median sample numbers can be computed for each action
   * \retval A map {label, median_sample_number}
   */
  std::map<std::string, int> raw_data_from_files_to_data_files(Setup &setup, NormalizationType normalization_type, bool seg_to_get_samp_nb);

  /**
   * \brief Write XML segmentation files to MLF files (HTK)
   * \param setup Contain the folder path of the segmentation files
   */
  void seg_files_to_MLF(Setup &setup);

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

  /** 
   * \brief Get the #Labels object corresponding to the labels contained in the #label_features_map
   * \retval #Labels
   */
  Labels get_labels(void);

  void print_map(void);

};


#endif
