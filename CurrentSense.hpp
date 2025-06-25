#ifndef _CURRENTSENSE_HPP_
#define _CURRENTSENSE_HPP_

#include "Arduino.h"
#include <SPI.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "configurations.hpp"

// ------------------------------
// System Constants and Settings
// ------------------------------

// Precision resistor and voltage divider parameters
const float RES_PRECISION      = 10.0f;    	// 10 Ω resistor
const float VOLT_DIVIDER_RATIO = 3.0f;     	// Each ADC input is scaled by 1/3
const float I_OUT_TARGET       = 0.0005f;  	// 0.5 mA target current (in A)
const float V_OUT              = 9.0f;    	// Voltage used in outgoing current equation

#define ADC_REF 5.0
const double ADC_VREF       = (double)ADC_REF;
const double ADC_DIVISOR    = 8388608.0; 	// 2^23, used in voltage conversion formula

#define ADC_EFFECTIVE_BITS 22  				// Effective resolution of the MCP3550 (22 bits)

const long ADC_MAX_CODE = (1L << (ADC_EFFECTIVE_BITS - 1)) - 1;  // Maximum positive code = 2^(21)-1 = 1048575

// Digital potentiometer (AD5292) parameters
// Assumption: AD5292 range is 0–20 kΩ, mapped linearly to an 8-bit value (0–255)
const float MAX_DIGIPOT_RES     = 20000.0f; 	// 20 kΩ maximum resistance
const uint16_t MAX_DIGIPOT_CODE = 255;     		// 8-bit maximum

// ------------------------------
// Function Prototypes
// ------------------------------
double readADCVoltage(unsigned long timeout_ms = 100); // Add timeout parameter
uint16_t calculateDigipotSetting(double resistance_target);
void writeDigipot(uint16_t value);
void setPotentiometer(double desiredDigipotResistance);

#endif /*_CURRENTSENSE_HPP_*/