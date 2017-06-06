#ifndef COMMON_HPP
#define	COMMON_HPP

/**
 * \file common.hpp
 * \brief Namespaces, enums, structs, functions that can be used by every classes
 * \author Amandine M.
 */

#include <vector>
#include <string>
#include <stdint.h>
#include <iterator>
#include <algorithm>
#include <boost/type_traits.hpp>
#include <boost/filesystem.hpp>

namespace SensorFeatureVectorTypes{
enum SensorFeatureVectorType
  {
  SensorFeatureVector = 3,
  SensorFeatureVectorExtended = 7
  };
}
typedef SensorFeatureVectorTypes::SensorFeatureVectorType SensorFeatureVectorType;

namespace VectorElements{
/**
 * \addtogroup VectorElements
 * Indexes of the elements in #SensorFeatureVector and #SensorFeatureVectorExtended
 */
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
/**
 * \addtogroup EmissionTypes
 * To model the distribution with Gaussian Mixture model (GMM) or single Gaussian (Gaussian)
 */
enum EmissionType
  {
    Gaussian = 0,
    GMM = 1
  };
}
typedef EmissionTypes::EmissionType EmissionType;

namespace StatesNumDefs
{
/**
 * \addtogroup StatesNumDefs
 * To define the way to compute the states number of a HMM
 */
enum StatesNumDef
  {
    /** ceil(sqrt(median(sample_numbers))) **/
    median = 0,
    /** ceil(median(sample_numbers)/10) **/
    linear_scaling = 1,
    /** Number of states defined by user **/
    fix_number = 2
  };
}
typedef StatesNumDefs::StatesNumDef StatesNumDef;

namespace FeatureFileFormats
{
/**
 * \addtogroup FeatureFileFormats
 * To define the output format of a#FeatureMatrix
 */
enum FeatureFileFormat
  {
    /** Output data to binary file **/
    dat,
    /** Output data to txt file **/
    lab
  };
}
typedef FeatureFileFormats::FeatureFileFormat FeatureFileFormat;

namespace LabelFileFormats
{
/**
 * \addtogroup LabelFileFormats
 * To define the output format of a #Labels object
 */
enum LabelFileFormat
  {
    /** Output labels as a list (txt file) **/
    txt,
    /** Output a dictionnary (txt file) **/
    dict,
    /** Output a grammar (txt file) **/
    grammar
  };
}
typedef LabelFileFormats::LabelFileFormat LabelFileFormat;

namespace NormalizationTypes
{
/**
 * \addtogroup NormalizationTypes
 * To define the normalization method to be used
 */
enum NormalizationType
  {
    /** No normalization **/
    no = 0,
    /** unit vector **/
    unit_vec = 1,
    /** Log normalization **/
    log = 2,
    /** Standard score **/
    score = 3,
    /** Feature scaling **/
    scaling = 4
  };
}
typedef NormalizationTypes::NormalizationType NormalizationType;

/** 
 * Namespace gathering different useful functions
 */
namespace tools
{

/** 
 * \brief Swap endian
 * Convert the endianness of a 16 bits unsigned integer
 *
 * \param 16 bits unsigned integer
 * \retval Swapped 16 bits unsigned integer
 */
uint16_t swap_endian(uint16_t n);

/** 
 * \brief Swap endian
 * Convert the endianness of a 32 bits unsigned integer
 *
 * \param 32 bits unsigned integer
 * \retval Swapped 32 bits unsigned integer
 */
uint32_t swap_endian(uint32_t n);


/** 
 * \brief Swap endian
 * Convert the endianness of an type
 */
template <size_t Size> void swap_endian(void* p);

/** \brief Swap endian
 * Convert the endianness of a 4 bytes element
 *
 * \param 4 bytes element
 */
template <> void swap_endian<4>(void* p);

// template <typename Type> Type swap_endian(Type v);

/** 
 * \brief Swap endian
 * Convert the endianness of the element of a container
 *
 * \param Pointer to the first element in the container
 * \param Pointer to the past-the-end elemenet in the container
 */
template <typename InputIterator> 
void swap_endian(InputIterator first, InputIterator last){
  typedef typename boost::remove_cv<
    typename std::iterator_traits<InputIterator>::value_type>::type
    input_type;

  for (; first != last; ++first) {
    swap_endian<sizeof(input_type)>((void*)&(*first));
  }
}

/** 
 * \brief Execute a system command
 * Execute a system command with the popen() so to be used in a linux environment
 * \param system command to execute
 * \retval Output of the command
 */
std::string execute_command(std::string command);

/** 
 * \brief Get a file name from an absolute path
 * \param Path
 */
std::string get_file_name(boost::filesystem::path path);
/** 
 * \brief Get the last directory from an absolute path 
 * Example : /home/user/LAST_DIRECTORY_NAME/example.txt
 * \param Path
 */
std::string get_last_dir_name(boost::filesystem::path path);

/** 
 * \brief To determine if a file or directory is hidden or not
 * \retval True if is hidden, False if not
 */
bool is_hidden(boost::filesystem::path p);
/**
 * \brief Return the median value of the elements of the std::vector
 * \param Median value of the elements of the std::vector
 */
float median(std::vector<int> samples_number);
}

#endif
