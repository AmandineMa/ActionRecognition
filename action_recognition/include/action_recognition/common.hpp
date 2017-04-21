#ifndef COMMON_HPP
#define	COMMON_HPP

#include <vector>
#include <string>

namespace VectorElements{
 enum VectorElement
    {
      X = 0,
      Y = 1,
      Z = 2,
      X_Q = 3,
      Y_Q = 4,
      Z_Q = 5,
      W = 6
    };
}
typedef VectorElements::VectorElement VectorElement;

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
