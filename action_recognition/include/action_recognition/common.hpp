#ifndef COMMON_HPP
#define	COMMON_HPP

#include <vector>
#include <string>
#include <stdint.h>
#include <iterator>
#include <algorithm>
#include <boost/type_traits.hpp>

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

uint16_t swap_endian(uint16_t n);

uint32_t swap_endian(uint32_t n);

template <size_t Size> void swap_endian(void* p);

template <> void swap_endian<4>(void* p);

// template <typename Type> Type swap_endian(Type v);

template <typename InputIterator> 
void swap_endian(InputIterator first, InputIterator last){
  typedef typename boost::remove_cv<
    typename std::iterator_traits<InputIterator>::value_type>::type
    input_type;

  for (; first != last; ++first) {
    swap_endian<sizeof(input_type)>((void*)&(*first));
  }
}
}

#endif
