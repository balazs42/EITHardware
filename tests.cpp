#include "ADS1261.hpp"
#include "MUX36S16.hpp"
#include "ControlSignalGeneration.hpp"

extern ADS1261_ADC adc1;
extern MUX36S16Pair multiplexers;

// Performs a single measurement. Creates the neccessary instances of hw objects, and sets
// CH0 as ground CH1 as Excitation. After these we perform measurements between CH2-CH3 
// periodically every 2 seconds.
void simpleMeasurements(void)
{
    // Increase buffer size to capture a full waveform
    const int buffer_size = 600;
    double samples[buffer_size];

    // 1 -> CH1 GND CH2 EXCITATION
    multiplexers.setMuxInjectionPair(1);
    adc1.initilaize();
    
    adc1.setInputChannels(3, 4);     
    double max = 0.0;

    while(1)
    {
        // --- Stage 1: High-Speed Data Acquisition ---
        // This loop should run as fast as possible, without any Serial prints.
        // It reads from the ADC and stores data in the 'samples' buffer.
        
        if (xSemaphoreTake(ADC_SEMAPHORE, (TickType_t) 1000) == pdTRUE)
        {
            for(int i = 0; i < buffer_size; i++)
            {
                // To make this even faster, temporarily comment out the
                // Serial.printf line inside adc1.readConversion()
                samples[i] = adc1.readConversion();
            }
            xSemaphoreGive(ADC_SEMAPHORE);
        }

        double mean = 0.0;
        for(int i = 0; i < buffer_size; i++)
            mean += samples[i];
        mean /= buffer_size;

        // --- Stage 2: Slower Data Printing ---
        // After the buffer is full, print all the collected data at once.
        // This prevents the serial port from bottlenecking the ADC reading.
        for(int i = 0; i < buffer_size; i++)
            if(abs(samples[i]) < mean * 30 )
                Serial.printf(">meas:%f\n", samples[i]);
    }
}

void measurementBetweenSeveralChannels(void)
{
    MUX36S16Pair multiplexers(MUX_S0, MUX_S1, MUX_S2, MUX_S3, MUX_ENABLE_PIN);
    ADS1261_ADC adc1(ADC1_CS_PIN, ADC1_DRDY_PIN, ADC_RESET_AND_PWDN_PIN, ADC_RESET_AND_PWDN_PIN, ADC_START_PIN);

    while(1)
    {

        for(int i = 1; i < 2; i++)
        {
            multiplexers.setMuxInjectionPair(i + 1);
    
            adc1.setInputChannels(i + 2, i + 3);
            
            double result = adc1.readConversion();

            Serial.printf("Iteration %d's result: %f\n", i, result);
        }
        vTaskDelay(1 / portTICK_RATE_MS);
    }
}