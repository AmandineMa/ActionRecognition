#include <stdint.h>
#include <stdio.h>
#include <boost/filesystem.hpp>

#include "action_recognition/common.hpp"
#include "action_recognition/HTKHeader.hpp"

namespace bf = boost::filesystem;

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


std::string tools::get_file_name(bf::path path){return path.stem().c_str();}

std::string tools::get_last_dir_name(bf::path path){
  std::string path_str = path.c_str();
  std::string dir_and_file = path_str.substr(path_str.find_last_of("/", path_str.find_last_of("/")-1)+1);
  return dir_and_file.substr(0, dir_and_file.find_last_of("/"));

}

bool tools::is_hidden(bf::path p)
{
  std::string name = p.filename().string();
  if((name != ".." && name != "."  && name[0] == '.') || name.find("~")!=std::string::npos)    
    return true;

  return false;
}

float tools::median(std::vector<int> samples_number){
  float median;
  int size = samples_number.size();

  std::sort(samples_number.begin(), samples_number.end());

  if (size  % 2 == 0)
    median = (samples_number[size / 2 - 1] + samples_number[size / 2]) / 2;
  else 
    median = samples_number[size / 2];

  return median;
}


void tools::write_HTK_header_to_file(std::ofstream& data_file, int vector_size, int samp_nb){
  // Define the HTK header
  HTKHeader header;
  header.BytesPerSample = vector_size*HTK_SAMPLE_SIZE;
  header.nSamples = samp_nb;
  // Write the header to the data file
  header.write_to_file(data_file);
}
