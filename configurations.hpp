#ifndef _CONFIGURATIONS_HPP_
#define _CONFIGURATIONS_HPP_

#define _TEST_MODE_
//#define _RELEASE_MODE_

// --- ADS1261 Pin Definitions --- 
#define ADC_START_PIN 26            // START (common to both)
#define ADC_RESET_AND_PWDN_PIN 27   // /PWDN and RESET (common to both)
#define ADC1_DRDY_PIN 32            // DRDY1 - Data ready (active low, common)
#define ADC2_DRDY_PIN 33            // DRDY2

// --- Multiplexer Pin Definitions ---
#define MUX_ENABLE_PIN 13           // Enable pin for the mux
#define MUX_S0 16                   // Select line A0
#define MUX_S1 17                   // Select line A1
#define MUX_S2 21                   // Select line A2
#define MUX_S3 22                   // Select line A3

// --- SPI Definitions ---
#define ADC1_CS_PIN 14              // Chip select for ADS1261 #1 (E1–E10)
#define ADC2_CS_PIN 15              // Chip select for ADS1261 #2 (E10–E16 & E1)
#define DIGIPOT_CS_PIN 4            // Chip Select pin for the digital potentiometer
#define MCP_CS_PIN 5                // Chip Select pin for MCP3550 ADC (adjust as needed)

#define SCLK 18                     // SCLK signal pin for SPI
#define MISO 19                     // MISO signal pin for SPI
#define MOSI 23                     // MOSI signal pin for SPI

// --- Core Definitions for Tasks ---
#define CTRL_VOL_CORE 0
#define SENSE_CORE 0
#define SERIAL_CORE 0
#define MEAS_CORE 1

// --- Priority Definitions for Tasks ---
#define CTRL_VOL_GEN_PRIO 1
#define SENSE_PRIO 2
#define SERIAL_PRIO 3
#define MEAS_PRIO 1

// --- Stack Size Definitions for Tasks ---
#define CTRL_VOL_GEN_STCK_SZ 1024*3
#define SENSE_STCK_SZ 1024*4
#define SERIAL_STCK_SZ 1024*4
#define MEAS_STCK_SZ 1024*4

// --- Task handles, so we can run/halt tasks ---
static TaskHandle_t CTRL_VOL_GEN_TSK_HNDL = new TaskHandle_t();
static TaskHandle_t SENSE_TSK_HNDL = new TaskHandle_t();
static TaskHandle_t SERIAL_TSK_HNDL = new TaskHandle_t();
static TaskHandle_t MEAS_TSK_HNDL = new TaskHandle_t();

static SemaphoreHandle_t ADC_SEMAPHORE = xSemaphoreCreateMutex();

// --- Baude Rate ---
static uint32_t BAUDE_RATE = 115200;

// --- ADC Sampling Rate ---
// 0  -> 00000:   2.5 SPS
// 1  -> 00001:   5 SPS
// 2  -> 00010:   10 SPS
// 3  -> 00011:   16.6 SPS
// 4  -> 00100:   20 SPS (default)
// 5  -> 00101:   50 SPS
// 6  -> 00110:   60 SPS
// 7  -> 00111:   100 SPS
// 8  -> 01000:   400 SPS
// 9  -> 01001:   1200 SPS
// 10 -> 01010:   2400 SPS
// 11 -> 01011:   4800 SPS
// 12 -> 01100:   7200 SPS
// 13 -> 01101:   14400 SPS
// 14 -> 01110:   19200 SPS
// 15 -> 01111:   25600 SPS
// 16 -> 10000- 11111: 40000 SPS 
static uint8_t SAMPLING_RATE = 16;

// --- ADC Filter Type ---
// 0: SINC1
// 1: SINC2
// 2: SINC3
// 3: SINC4
// 4: FIR (default)
static uint8_t FILTER_TYPE = 0;

// --- ADC PGA Gain ---
// 000:     1 (default)
// 001:     2
// 010:     4
// 011:     8
// 100:     16
// 101:     32
// 110:     64
// 111:     128
static uint8_t PGA_GAIN = 1;

// --- Voltage Generator Frequency ---
// This determines the frequency of the excitation current
//
// 8.0f:       1.04kHz
// 16.0f:      2.08kHz
// 50.0f:      6.5kHz
// 77.0f:      10 kHz
// 100.0f:     12.97kHz
// 140.0f:     20 kHz
// 200.0f:     25.94kHz
// 500.0f:     64.85kHz
static float FREQENCY_STEP = 8.0f;

#endif /*_CONFIGURATIONS_HPP_*/