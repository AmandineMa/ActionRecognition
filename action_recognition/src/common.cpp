#include <stdint.h>
#include <iterator>
#include <algorithm>
#include <boost/type_traits.hpp>

#include "action_recognition/common.hpp"

uint16_t tools::swap_endian(uint16_t n) {
  return ((uint16_t)((n & 0x00ff) << 8) |
		  (uint16_t)((n & 0xff00) >> 8));
}

uint32_t tools::swap_endian(uint32_t n) {
  return (((n & 0x000000ffUL) << 24) | ((n & 0x0000ff00UL) << 8) |
		  ((n & 0x00ff0000UL) >> 8) | ((n & 0xff000000UL) >> 24));
}


template <>
void tools::swap_endian<4>(void* p) {
  char* pb = (char*)p;
  std::swap(pb[0], pb[3]);
  std::swap(pb[1], pb[2]);
}

// template <typename Type>
// Type tools::swap_endian(Type v) {
//   Type result = v;
//   swap_endian<sizeof(v)>((void*)&result);
//   return result;
// }

// template <typename InputIterator>
// void tools::swap_endian(InputIterator first, InputIterator last) {
//   typedef typename boost::remove_cv<
//     typename std::iterator_traits<InputIterator>::value_type>::type
//     input_type;

//   for (; first != last; ++first) {
//     swap_endian<sizeof(input_type)>((void*)&(*first));
//   }
//}
