#include <stdint.h>
#include <stdio.h>

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

std::string tools::execute_command(std::string command) {

  std::string data;
  FILE * stream;
  const int max_buffer = 256;
  char buffer[max_buffer];
  command.append(" 2>&1");

  stream = popen(command.c_str(), "r");
  if (stream) {
    while (!feof(stream))
      if (fgets(buffer, max_buffer, stream) != NULL) data.append(buffer);
    int stat = pclose(stream); 
    // if(stat != 0){
    //   throw std::string(data);
    // }
  }
  return data;
}
