#ifndef HTKHEADER_HPP
#define HTKHEADER_HPP

#include <iostream>
#include <fstream>

using namespace std;

#include "action_recognition/common.hpp"

const unsigned HTK_USER = 9;
const unsigned HTK_MFCC_E_D = 326;
const unsigned HTK_MFCC_E_D_Z = 326 + 2048;

struct HTKHeader {
  /** \brief Number of samples */
  unsigned nSamples;            
  // TODO: have to match with period of ROS node
  /** \brief Period of data acquisition */
  unsigned Period = 330000;               
  /** \brief Number of bytes per sample */
  unsigned short BytesPerSample;
  /** \brief HTK feature type */
  unsigned short FeatureType = HTK_USER;

public:

    void write_to_file(ostream& out) const
    {
        // HTKHeader h;
        // h.nSamples = tools::swap_endian(nSamples);
        // h.Period = tools::swap_endian(Period);
        // h.BytesPerSample = tools::swap_endian(BytesPerSample);
        // h.FeatureType = tools::swap_endian(FeatureType);      

        out.write((char*)&nSamples, sizeof(nSamples));
        out.write((char*)&Period, sizeof(Period));
        out.write((char*)&BytesPerSample, sizeof(BytesPerSample));
        out.write((char*)&FeatureType, sizeof(FeatureType));
    }
};

#endif
