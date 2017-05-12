#ifndef DATAHANDLER_HPP
#define DATAHANDLER_HPP

#include <map>
#include <vector>
#include <string>
#include <queue>
#include <boost/filesystem.hpp>

#include "action_recognition/common.hpp"
#include "action_recognition/FeatureMatrix.hpp"
#include "action_recognition/Labels.hpp"
#include "action_recognition/Setup.hpp"

class DataHandler{
private:
  std::map<std::string, std::vector<FeatureMatrix> > label_features_map_;
  Labels labels_;

  std::string get_file_name(boost::filesystem::path path);
  std::string get_last_dir_name(boost::filesystem::path path);
  std::queue<std::pair<std::string,std::pair<int, int> > > parse_seg_file(std::string file_path);
  bool is_hidden(boost::filesystem::path p);

public:
  DataHandler(Setup setup);

  void raw_data_from_file_to_feature_matrices(std::string seg_dir, std::string raw_data_dir,
                                              NormalizationType normalization_type);
  void check_segmentation(void);
  void normalize(void);
 std::pair<std::map<std::string, std::vector<FeatureMatrix> >::iterator,
          std::map<std::string, std::vector<FeatureMatrix> >::iterator > get_map_iterator();

  Labels get_labels(void);

  void print_map(void);

};


#endif
