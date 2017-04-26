#ifndef LABELS_HPP
#define LABELS_HPP

#include <vector>
#include <string>

#include "action_recognition/common.hpp"

class Labels{

private:
  std::vector<std::string> labels_;
  std::string labels_list_path_;
  std::string grammar_net_path_;
  std::string dict_path_;
  std::string grammar_path_;

public:
  Labels();
  Labels(std::string labels_list_path_, std::string grammar_net_path, std::string dict_path, std::string grammar_path_);
  Labels(std::vector<std::string> labels, std::string labels_list_path_, std::string grammar_net_path, std::string dict_path, std::string grammar_path_);

  void add_label(std::string label);
  void set_labels_list_path(std::string labels_list_path);
  void set_grammar_net_path(std::string grammar_net_path);
  void set_dict_path(std::string dict_path);
  void set_grammar_path(std::string grammar_path_);

  void write_to_file(LabelFileFormats::LabelFileFormat file_format);
  std::string compile_grammar(void);
  std::string test_grammar(void);

};

#endif
