#include <Arduino.h>
#include "driver/dac.h"
#include "ControlSignalGeneration.hpp"
#include "CurrentSense.hpp"
#include "ElectrodeMeasurements.hpp"
#include "SerialInterface.hpp"


// --- Global object definitions ---
ADS1261_ADC adc1 = *(new ADS1261_ADC(ADC1_CS_PIN, ADC1_DRDY_PIN, ADC_RESET_AND_PWDN_PIN, ADC_RESET_AND_PWDN_PIN, ADC_START_PIN));
ADS1261_ADC adc2 = *(new ADS1261_ADC(ADC2_CS_PIN, ADC2_DRDY_PIN, ADC_RESET_AND_PWDN_PIN, ADC_RESET_AND_PWDN_PIN, ADC_START_PIN));
MUX36S16Pair multiplexers(MUX_S0, MUX_S1, MUX_S2, MUX_S3, MUX_ENABLE_PIN);

// --- Test functions ---
extern void simpleMeasurements(void);

void createTasks(void)
{
    xTaskCreatePinnedToCore(dactask,
                            "CTR_VOL_TSK",
                            CTRL_VOL_GEN_STCK_SZ,
                            NULL,
                            CTRL_VOL_GEN_PRIO,
                            &CTRL_VOL_GEN_TSK_HNDL,
                            CTRL_VOL_CORE);

    Serial.println("Waveform generation tasks started on DAC1 (GPIO25).");

    xTaskCreatePinnedToCore(serialCommandTask,
                            "SERIA_TSK",
                            SERIAL_STCK_SZ,
                            nullptr,
                            SERIAL_PRIO,
                            &SERIAL_TSK_HNDL,
                            SERIAL_CORE);

    Serial.println("Serial message handling task created!");

    //xTaskCreatePinnedToCore(currentSenseTask, 
    //                        "SENSE",
    //                        SENSE_STCK_SZ,
    //                        NULL,
    //                        SENSE_PRIO,
    //                        &SENSE_TSK_HNDL,
    //                        SENSE_CORE);
//
    //Serial.println("Current Sensing Task Started!");

    //xTaskCreatePinnedToCore(measurementTask,
    //                        "MEAS_TSK",
    //                        CTRL_VOL_GEN_STCK_SZ,
    //                        NULL,
    //                        CTRL_VOL_GEN_PRIO,
    //                        &MEAS_TSK_HNDL,
    //                        MEAS_CORE);
//
    //Serial.println("Measurement Task Started!");
}

void setup(void) 
{
    // --- Initilaize Serial Communication ---
    Serial.begin(BAUDE_RATE);
    
    pinMode(SCLK, OUTPUT);
    pinMode(MOSI, OUTPUT);
    pinMode(MISO, INPUT);

    SPI.begin(SCLK, MISO, MOSI);           

    // Initialize sine wave generation on DAC1 channel
    dac_cosine_enable(DAC_CHANNEL_1);
    dac_output_enable(DAC_CHANNEL_1);

    // Create tasks that will be running on the ESP
    createTasks();
}

void loop(void) 
{
    #ifdef _TEST_MODE_
    simpleMeasurements();
    #endif
    // Delete the default Loop task
    vTaskDelete(nullptr);
}