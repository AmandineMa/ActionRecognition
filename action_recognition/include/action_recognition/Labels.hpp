#ifndef LABELS_HPP
#define LABELS_HPP

/**
 * \file Labels.hpp
 * \brief Labels class
 * \author Amandine M.
 */

#include <set>
#include <string>

#include "action_recognition/common.hpp"
/**
 * \brief Labels class which contains a set of all the labels (non-duplicated elements)
 */
class Labels{

private:
  std::set<std::string> labels_; /** set of all the labels (non-duplicated elements) */
  std::string labels_list_path_; /** Path of the labels list file */
  std::string grammar_net_path_; /** Path of the grammar net file */
  std::string dict_path_;        /** Path of the dictionnary file */
  std::string grammar_path_;     /** Path of the grammar file */

public:
  /** 
   * \brief Constructor for an emtpy Labels object
   */
  Labels();
  /** 
   * \brief Constructor for a Labels object
   * \param Path of the labels list file
   * \param Path of the grammar net fil
   * \param Path of the dictionnary file
   * \param Path of the grammar file
   */
  Labels(std::string labels_list_path_, std::string grammar_net_path, 
         std::string dict_path, std::string grammar_path_);  
  /** 
   * \brief Constructor for a Labels object
   * \param Set of labels
   * \param Path of the labels list file
   * \param Path of the grammar net fil
   * \param Path of the dictionnary file
   * \param Path of the grammar file
   */
  Labels(std::set<std::string> labels, std::string labels_list_path_, 
         std::string grammar_net_path, std::string dict_path, std::string grammar_path_);

  /** 
   * \brief Insert a label in the set if the label is not already in it
   */
  void add_label(std::string label);

  void set_labels_list_path(std::string labels_list_path);
  void set_grammar_net_path(std::string grammar_net_path);
  void set_dict_path(std::string dict_path);
  void set_grammar_path(std::string grammar_path_);

  std::pair<std::set<std::string>::iterator, std::set<std::string>::iterator> get_iterator(void);

  /** 
   * \brief Write the labels to a dictionnary file, a grammar file or a text file (list)
   * in the location given at the #Labels initialization
   * \param #LabelFileFormats::LabelFileFormat (dict, grammar, text)
   */
  void write_to_file(LabelFileFormats::LabelFileFormat file_format);
  /** 
   * \brief Compile a grammar to output a grammar net file
   */
  std::string compile_grammar(void);
  /** 
   * \brief Test the grammar net file from the grammar compilation output
   */
  std::string test_grammar(void);

};

#endif
