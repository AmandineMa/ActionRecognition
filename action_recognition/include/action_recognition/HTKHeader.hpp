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
    unsigned nSamples;              // number of samples
    unsigned Period;                // period [100ns]
    unsigned short BytesPerSample;
    unsigned short FeatureType;     // if you use a user-defined type, set to HTK_USER.

public:
    // void read(istream& in)
    // {
    //  	in.read((char*)&nSamples, sizeof(nSamples));
    //     in.read((char*)&Period, sizeof(Period));
    //     in.read((char*)&BytesPerSample, sizeof(BytesPerSample));
    //     in.read((char*)&FeatureType, sizeof(FeatureType));
        
    //     nSamples = tools::swap_endian(nSamples);
    //     Period = tools::swap_endian(Period);
    //     BytesPerSample = tools::swap_endian(BytesPerSample);
    //     FeatureType = tools::swap_endian(FeatureType);   
    // }

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
