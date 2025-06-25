#ifndef _CONTROLSIGNALGENERATION_HPP_
#define _CONTROLSIGNALGENERATION_HPP_

#include <Arduino.h>
#include "driver/i2s.h"
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "soc/rtc_io_reg.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc.h"

#include "driver/dac.h"

#include "configurations.hpp"

// signal frequency is determined by frequency_step like this:
// frequency_step = desiredFrequencyHz x 65536 / 8500000
// -or-
// frequency = 8500000 x frequency_step / 65536
// 8:       1.04kHz
// 16:      2.08kHz
// 50:      6.5kHz
// 77.1:    10 kHz
// 100:     12.97kHz
// 140:     20 kHz
// 200:     25.94kHz
// 500:     64.85kHz
static float frequency_step = FREQENCY_STEP;

static int clk_8m_div = 0;      // RTC 8M clock divider (0=8MHz)
static int scale = 0;           // 1/2 scale
static int offset = 128;        // no offset
static int invert = 2;          // invert MSB (most significant bit) for sine wave

static float frequency = RTC_FAST_CLK_FREQ_APPROX / (1 + clk_8m_div) * (float) frequency_step / 65536;

// all manipulations are direct writes to a particular register
// DAC is configured using register SENS_SAR_DAC_CTRL1_REG and SENS_SAR_DAC_CTRL2_REG
// SENS_SAR_DAC_CTRL1_REG enables the cosine generator
// SENS_SAR_DAC_CTRL2_REG connects it to a DAC channel

// Register bits can be changed using SET_PERI_REG_MASK();

// enables the cosine generator for a DAC channel:
void dac_cosine_enable(dac_channel_t channel);

// frequency (for both channels)is determined by two parameters:
// clk_8m_div (RTC 8M clock divider): 0=8Mhz clock, range is 0-273
// frequency_step: range is 1-65535
void dac_frequency_set(int clk_8m_div, int frequency_step);

// scaling, range is 0-3:
// 0: no scale
// 1: scale to 1/2
// 2: scale to 1/4
// 3: scale to 1/8
void dac_scale_set(dac_channel_t channel, int scale);

// Offset output for a particular channel: range is 0-255
void dac_offset_set(dac_channel_t channel, int offset = 128);

// Invert output for a particular channel, range is 0-3:
// 0: no inversion
// 1: completely inverted
// 2: invert MSB (most significant bit)
// 3: invert all but MSB
void dac_invert_set(dac_channel_t channel, int invert);

// task that handles the DAC
void dactask(void* arg);

#endif /*_CONTROLSIGNALGENERATION_HPP_*/