#include "SerialInterface.hpp"

extern ADS1261_ADC adc1;
extern ADS1261_ADC adc2;
const TickType_t semaphoreTimeout = 1000;

bool setExcitationFrequency(float newFreqStep)
{
    if (newFreqStep <= 0.0f || newFreqStep > 500.0f) 
        return false;

    // Reassign global variable that corresponds to excitation freq
    FREQENCY_STEP = newFreqStep;

    return true;
}

bool setAdcSamplingRate(uint8_t newSampCode) 
{
    if(newSampCode == 0 || newSampCode > 16)
        return false;

    Serial.println("Setting new sampling code: " + String(newSampCode));

    SAMPLING_RATE = newSampCode;
    if(xSemaphoreTake(ADC_SEMAPHORE, semaphoreTimeout) == pdTRUE)
    {   
        adc1.stopConversions();
        adc1.initilaize();
        xSemaphoreGive(ADC_SEMAPHORE);
    }   

    return true;
}

bool setAdcPgaGain(uint8_t newGain) 
{
    if(newGain % 2 != 0 && newGain != 1)
        return false;

    Serial.println("Setting new gain: " + String(newGain));

    // Reassign gain value, error checking implemented in the function
    PGA_GAIN = newGain;

    if(xSemaphoreTake(ADC_SEMAPHORE, semaphoreTimeout) == pdTRUE)
    {   
        adc1.stopConversions();
        adc1.initilaize();
        xSemaphoreGive(ADC_SEMAPHORE);
    }
    return true;
}

bool setFilter(uint8_t newFilter)
{
    if(newFilter > 4)
        return false;

    // Reassign filter type
    FILTER_TYPE = newFilter;

    if(xSemaphoreTake(ADC_SEMAPHORE, semaphoreTimeout) == pdTRUE)
    {   
        adc1.stopConversions();
        adc1.initilaize();
        xSemaphoreGive(ADC_SEMAPHORE);
    }
    return true;
}

void serialCommandTask(void* pvParam)
{
    char line[64];

    while(1) 
    {
        if (Serial.available()) 
        {
            size_t len = Serial.readBytesUntil('\n', line, sizeof(line)-1);
            line[len] = '\0';
            // strip trailing CR
            if (len && line[len-1]=='\r') line[--len] = '\0';

            // find the '='
            char* eq = strchr(line, '=');
            if (!eq)
            {
                Serial.println("ERR: invalid format, use <param>=<value>");
            } 
            else 
            {
                *eq = '\0';
                const char* param = line;
                const char* val   = eq + 1;
                bool matched = false;

                for (size_t i = 0; i < numParams; ++i) 
                {
                    if (strcmp(param, parameters[i].name) == 0) 
                    {
                        matched = true;
                        if (parameters[i].apply(val)) 
                            Serial.printf("OK: %s = %s\n", param, val);
                        else
                            Serial.printf("ERR: bad value for %s (expected %s)\n", param, parameters[i].help);
                        break;
                    }
                }

                if (!matched)
                {
                    Serial.printf("ERR: unknown param '%s'\n", param);
                    Serial.println("Supported parameters:");
                    for (size_t i = 0; i < numParams; ++i) 
                        Serial.printf("  %s — %s\n",
                                    parameters[i].name, parameters[i].help);
                }
            }
        }
        vTaskDelay(500 / portTICK_RATE_MS);
    }
}