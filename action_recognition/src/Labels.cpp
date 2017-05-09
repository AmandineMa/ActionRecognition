#include <vector>
#include <fstream>
#include <iostream>

#include "action_recognition/Labels.hpp"
#include "action_recognition/common.hpp"

Labels::Labels(){}

Labels::Labels(std::string labels_list_path, std::string grammar_net_path, std::string dict_path, std::string grammar_path):
  labels_(0), labels_list_path_(labels_list_path), grammar_net_path_(grammar_net_path), dict_path_(dict_path), grammar_path_(grammar_path){}

Labels::Labels(std::vector<std::string> labels, std::string labels_list_path, std::string grammar_net_path, std::string dict_path, std::string grammar_path):
labels_(labels), labels_list_path_(labels_list_path), grammar_net_path_(grammar_net_path),dict_path_(dict_path), grammar_path_(grammar_path){}

void Labels::add_label(std::string label){labels_.push_back(label);} 

void Labels::set_labels_list_path(std::string labels_list_path){labels_list_path_ = labels_list_path;}

void Labels::set_grammar_net_path(std::string grammar_net_path){grammar_net_path_ = grammar_net_path;}

void Labels::set_dict_path(std::string dict_path){dict_path_ = dict_path;}

void Labels::set_grammar_path(std::string grammar_path){grammar_path_ = grammar_path;}

void Labels::write_to_file(LabelFileFormats::LabelFileFormat file_format){
  std::ofstream ofile;
  std::vector<std::string>::iterator it = labels_.begin();

  switch(file_format){
    case LabelFileFormats::txt:
      ofile.open(labels_list_path_.c_str());
      if(ofile.is_open()){
        for(; it != labels_.end() ; it++)
          ofile << *it << "\n";
      }else
        std::cout << "Unable to open "+labels_list_path_ << std::endl;
      break;

    case LabelFileFormats::dict:
        ofile.open(dict_path_.c_str());
        if(ofile.is_open()){
        for(; it != labels_.end() ; it++)
          ofile << *it << " [" << *it << "] " << *it << "\n";
      }else
          std::cout << "Unable to open "+dict_path_ << std::endl;
      break;

    case LabelFileFormats::grammar:
      ofile.open(grammar_path_.c_str());
      if(ofile.is_open()){
        ofile << "$ACT = ";
        ofile << "<" << *it++ << ">" ;
        for(; it != labels_.end() ; it++)
          ofile << " | <" << *it << ">" ;
       
        ofile << ";\n" << "([$ACT])";
      }else
        std::cout << "Unable to open "+grammar_path_ << std::endl;
      break;

    default:
      break;

      ofile.close();
  }
}

std::string Labels::compile_grammar(void){
  std::string HParse_output = tools::execute_command("HParse -A -D -T 1 "+grammar_path_+" "+grammar_net_path_);
  return HParse_output;
}

std::string Labels::test_grammar(void){
std::string HSGen_output = tools::execute_command("HSGen -l -n 20 -s "+grammar_net_path_+" "+dict_path_);
  return HSGen_output;
}


