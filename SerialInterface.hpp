#ifndef _SERIALINTERFACE_HPP_
#define _SERIALINTERFACE_HPP_

#include "Arduino.h"
#include "configurations.hpp"
#include "ADS1261.hpp"
#include "MUX36S16.hpp"

bool setExcitationFrequency(float newFreqStep);
bool setAdcSamplingRate(uint8_t newSampCode);
bool setAdcPgaGain(uint8_t newGain);
bool setFilter(uint8_t newFilter);
void serialCommandTask(void* pvParam);

// Command‐handler signature
using CmdHandler = bool(*)(const char* valueStr);

// Table of supported parameters
static const struct 
{
    const char*      name;
    CmdHandler       apply;
    const char*      help;     // for usage text
} parameters[] = 
{
    {
        "curr", [](const char* v)
        {
            return setExcitationFrequency(atof(v));
        },
        "sets excitation frequency step (float)" 
    },

    { 
        "samp", [](const char* v)
        {
            return setAdcSamplingRate((uint8_t)atoi(v));
        },
        "sets sampling-rate code (integer)"
    },

    { 
        "pga",  [](const char* v)
        {
            return setAdcPgaGain((uint8_t)atoi(v)); 
        },
        "sets PGA gain (1,2,4,8,...)" 
    },
    { 
        "filter",  [](const char* v)
        {
            return setFilter((uint8_t)atoi(v)); 
        },
        "sets sinc 1-4 or FIR fitler" 
    },
};

static const size_t numParams = sizeof(parameters)/sizeof(parameters[0]);

#endif /*_SERIALINTERFACE_HPP_*/