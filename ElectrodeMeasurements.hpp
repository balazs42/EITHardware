#ifndef _ELECTRODEMEASUREMENTS_HPP_
#define _ELECTRODEMEASUREMENTS_HPP_

#include "Arduino.h"
#include <SPI.h>
#include "MUX36S16.hpp"
#include "ADS1261.hpp"
#include <math.h> // For NAN

// --- Task Function Prototypes ---
void measurementTask(void *pvParam);
void printMeasurements(float measurements[16][16]);
void sendMeasurements(float measurements[16][16]);

// Helper structure to return ADC mapping info
struct AdcMappingInfo 
{
    ADS1261_ADC* adc; // Pointer to the ADC instance (adc1 or adc2)
    uint8_t ain_p;    // AIN pin for positive input (meas_ch1)
    uint8_t ain_n;    // AIN pin for negative input (meas_ch2)
    bool valid;       // True if this pair can be measured by a single ADC
};

// Function to determine which ADC and AIN pins to use for a measurement pair
AdcMappingInfo getAdcForMeasurementPair(uint8_t meas_ch1, uint8_t meas_ch2, ADS1261_ADC& adc1_ref, ADS1261_ADC& adc2_ref);

#endif /* _ELECTRODEMEASUREMENTS_HPP_ */