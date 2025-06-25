#include "ElectrodeMeasurements.hpp"

extern ADS1261_ADC adc1;
extern ADS1261_ADC adc2;
extern MUX36S16Pair multiplexers;

// --- Global measurement matrix ---
double measurements[16][16];

// --- Measurement related functions ---
AdcMappingInfo getAdcForMeasurementPair(uint8_t meas_ch1, uint8_t meas_ch2, ADS1261_ADC& adc1_ref, ADS1261_ADC& adc2_ref) 
{
    AdcMappingInfo info = {nullptr, 0, 0, false};

    // Ensure channels are within 1-16 range
    if (meas_ch1 < 1 || meas_ch1 > 16 || meas_ch2 < 1 || meas_ch2 > 16 || meas_ch1 == meas_ch2) 
        return info; // Invalid electrode numbers or same electrode
    
    // --- Rules based on wiring ---
    // ADC1: AIN0-AIN9 -> Electrodes 1-10
    // ADC2: AIN0-AIN6 -> Electrodes 10-16, AIN7 -> Electrode 1

    bool ch1_on_adc1 = (meas_ch1 >= 1 && meas_ch1 <= 10);
    bool ch2_on_adc1 = (meas_ch2 >= 1 && meas_ch2 <= 10);
    bool ch1_on_adc2 = (meas_ch1 >= 10 && meas_ch1 <= 16) || (meas_ch1 == 1);
    bool ch2_on_adc2 = (meas_ch2 >= 10 && meas_ch2 <= 16) || (meas_ch2 == 1);

    // Try ADC1 first
    if (ch1_on_adc1 && ch2_on_adc1) 
    {
        info.adc = &adc1_ref;
        info.ain_p = meas_ch1 - 1; // E1->AIN0 ... E10->AIN9
        info.ain_n = meas_ch2 - 1; // E1->AIN0 ... E10->AIN9
        info.valid = true;
    }
    // Try ADC2 if ADC1 didn't work or if it's clearly an ADC2 pair
    else if (ch1_on_adc2 && ch2_on_adc2) 
    {
         // Handle the special case E1 on ADC2
        uint8_t adc2_ain_p = (meas_ch1 == 1) ? 7 : meas_ch1 - 10; // E1->AIN7, E10->AIN0,... E16->AIN6
        uint8_t adc2_ain_n = (meas_ch2 == 1) ? 7 : meas_ch2 - 10; // E1->AIN7, E10->AIN0,... E16->AIN6

        // Check if mapping is valid for ADC2's pins (0-7)
        if (adc2_ain_p <= 7 && adc2_ain_n <= 7) 
        {
            info.adc = &adc2_ref;
            info.ain_p = adc2_ain_p;
            info.ain_n = adc2_ain_n;
            info.valid = true;
        }
    }
    // Any other combination implies cross-ADC or invalid mapping
    else 
        info.valid = false;

    // Final check: ensure pins are different if valid
    if (info.valid && info.ain_p == info.ain_n) 
        info.valid = false;
    
    // Ensure resulting AIN pins are within the valid range for the selected ADC
    if (info.valid && info.adc == &adc1_ref && (info.ain_p > 9 || info.ain_n > 9)) 
         info.valid = false; // ADC1 only has AIN0-AIN9
    
    if (info.valid && info.adc == &adc2_ref && (info.ain_p > 7 || info.ain_n > 7)) 
         info.valid = false; // ADC2 only has AIN0-AIN7 connected as described

    return info;
}

void measurementTask(void *pvParam)
{
    // --- Create ADC and MUX Objects ---
    Serial.println("Starting measurement loop...");

    // --- Initialize Global Measurement Array ---
    for (int i = 0; i < 16; i++) 
        for (int j = 0; j < 16; j++) 
            measurements[i][j] = NAN; // Initialize with Not-a-Number

    while(true)
    {
        Serial.println("\n--- Starting Full Measurement Cycle ---");
    
        // Outer loop: Iterate through injection pairs (1-2, 2-3, ..., 16-1)
        for(uint8_t inj_ch1 = 0; inj_ch1 < 16; inj_ch1++)
        {
            uint8_t inj_ch2 = (inj_ch1 % 16) + 1; // Calculate second injection electrode
    
            Serial.printf("Injecting: CH%d - CH%d\n", inj_ch1, inj_ch2);
    
            // 1. Set Injection Pair using MUX
            multiplexers.setMuxInjectionPair(inj_ch1);
            delayMicroseconds(1);
    
            // Inner loop: Iterate through measurement pairs (1-2, 2-3, ..., 16-1)
            for (uint8_t meas_ch1 = 1; meas_ch1 <= 16; meas_ch1++)
            {
                uint8_t meas_ch2 = (meas_ch1 % 16) + 1; // Calculate second measurement electrode
    
                // 2. Skip if measurement pair overlaps injection pair
                bool overlap = (meas_ch1 == inj_ch1 || meas_ch1 == inj_ch2 ||
                                meas_ch2 == inj_ch1 || meas_ch2 == inj_ch2);
    
                if (overlap) 
                {
                    measurements[inj_ch1 - 1][meas_ch1 - 1] = NAN; // Store NAN for overlap
                    continue;
                }
    
                // 3. Determine which ADC and AIN pins to use
                AdcMappingInfo adc_info = getAdcForMeasurementPair(meas_ch1, meas_ch2, adc1, adc2);
    
                // 4. Skip if measurement pair requires cross-ADC reading or is invalid
                if (!adc_info.valid) 
                {
                    measurements[inj_ch1 - 1][meas_ch1 - 1] = NAN; // Store NAN for invalid/cross-ADC
                    Serial.printf("  Skipping measurement %d-%d (Invalid/Cross-ADC)\n", meas_ch1, meas_ch2);
                    continue;
                }
    
                // 5. Set Input Channels on the determined ADC
                adc_info.adc->setInputChannels(adc_info.ain_p, adc_info.ain_n);
    
                // 6. Read Conversion from the ADC
                double voltage = adc_info.adc->readConversion();
    
                // 7. Store Result
                measurements[inj_ch1 - 1][meas_ch1 - 1] = (float)voltage;
            }
        } 
    
        Serial.println("\n--- Measurement Cycle Complete ---");
    
        // Print the full matrix for verification
        Serial.println("Measurement Matrix [Injection Pair Start][Measurement Pair Start]:");
        for(int i = 0; i < 16; i++) 
        {
            for (int j = 0; j < 16; j++) 
            {
                if (isnan(measurements[i][j])) 
                    Serial.print("  NAN   ");
                else 
                    Serial.printf("%+.3f ", measurements[i][j] * 1000.0); // Print in mV
            }
            Serial.println();
        }
    
        // Delay between full cycles
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void printMeasurements(float measurements[16][16])
{
    Serial.println("The measured values:");

    for (int i = 0; i < 16; i++)
    {
        for (int j = 0; j < 16; j++)
            Serial.print(String(measurements[i][j], 6) + ", ");
        Serial.println();
    }
    Serial.println();
}

void sendMeasurements(float measurements[16][16])
{
    if(measurements == nullptr)
        return;

    Serial.println("---RESULTS---");
    for (int i = 0; i < 16; i++)
    {
        for (int j = 0; j < 16; j++) 
        {
            if (isnan(measurements[i][j])) 
                Serial.print("  NAN   ");
            else 
                Serial.printf("%+.3f ", measurements[i][j] * 1000.0); // Print in mV
        }
        Serial.println();
    }
    Serial.println("---END---");
}