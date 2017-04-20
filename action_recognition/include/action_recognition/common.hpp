#ifndef COMMON_H
#define	COMMON_H

#include <vector>

namespace EmissionTypes
{
enum EmissionType
  {
    Gaussian,
    GMM
  };
}
typedef EmissionTypes::EmissionType EmissionType;

namespace FeatureFileFormats
{
enum FeatureFileFormat
  {
    dat,
    lab
  };
}
typedef FeatureFileFormats::FeatureFileFormat FeatureFileFormat;

namespace LabelFileFormats
{
enum LabelFileFormat
  {
    txt,
    dict,
    grammar
  };
}
typedef LabelFileFormats::LabelFileFormat LabelFileFormat;

namespace NormalizationTypes
{
enum NormalizationType
  {
    no,
    standard,
    log
  };
}
typedef NormalizationTypes::NormalizationType NormalizationType;

typedef struct
{
  std::string hmms;
  std::string labels_list;
  std::string dict;
  std::string grammar;
  std::string grammar_net;
  std::string recognition;
}FilesPaths;


namespace tools
{
void generate_script_file(std::vector<std::string> file_name_array);
}

#endif
