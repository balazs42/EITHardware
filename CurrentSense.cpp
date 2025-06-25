#include "CurrentSense.hpp"

static SPISettings adcSPISettings(2000000, MSBFIRST, SPI_MODE0); // 2 MHz, Mode 0 for MCP3550
static SPISettings digipotSPISettings(5000000, MSBFIRST, SPI_MODE1); // 5 MHz, Mode 1 for AD5292

void currentSenseTask(void* pvParam)
{
    // Initialize MCP3550 and AD5292 pins
	pinMode(MCP_CS_PIN, OUTPUT);

	digitalWrite(MCP_CS_PIN, LOW);
	digitalWrite(MCP_CS_PIN, HIGH);
	
	//pinMode(DIGIPOT_CS_PIN, OUTPUT);
	//digitalWrite(DIGIPOT_CS_PIN, HIGH);

	double averageVoltage = 0.0;
	int measureCntr = 0;
	const int avgCnt = 200;

	while(true)
	{
		// 1. Read the ADC voltage from the MCP3550 (this value is post divider).
		double adcVoltage = readADCVoltage();
		measureCntr++;

		if (isnan(adcVoltage) || adcVoltage > 5) // Check for invalid reading
        {
            //Serial.println("Failed to read ADC voltage.");
        }
        else
        {
			averageVoltage += adcVoltage;

        }

		if(measureCntr % avgCnt == 0)
		{
			double voltage = averageVoltage / avgCnt;

			averageVoltage = 0.0;

			Serial.println("---Sense---");
            Serial.printf("ADC Raw Voltage: %.6f V\n", voltage);

            // 2. Scale the ADC voltage up (compensate for voltage divider).
            double actualVoltageDrop = voltage * VOLT_DIVIDER_RATIO;
            Serial.printf("Actual Vdrop: %.3f mV\n", actualVoltageDrop*1000);

            // 3. Compute the current using Ohm’s law.
            double measuredCurrent = actualVoltageDrop / RES_PRECISION;

            Serial.printf("Measured Current: %.3f mA\n\n", measuredCurrent*1000);

			Serial.println("---END Sense---");
            // --- Potentiometer Control (Example) ---
            // double desiredDigipotResistance = V_OUT / I_OUT_TARGET;
            // setPotentiometer(desiredDigipotResistance);
            // ---
		}

        // Delay before the next measurement (e.g., 1 second).
        vTaskDelay(1000 / avgCnt / portTICK_PERIOD_MS); // Use portTICK_PERIOD_MS for portability
    }
}

void setPotentiometer(double desiredDigipotResistance)
{
	// 4. Calculate the required digital potentiometer resistance for the target I_out.
	// Equation: I_out = V_OUT / R_digpot  =>  R_digpot = V_OUT / I_OUT_TARGET.
	 
	// 5. Convert the desired resistance into the corresponding 8-bit code.
	uint16_t digipotValue = calculateDigipotSetting(desiredDigipotResistance);
	 
	// 6. Write the digital code to the AD5292 via SPI.
	writeDigipot(digipotValue);

	Serial.print("Setting digital potentiometer to value: ");
	Serial.println(digipotValue);
	 
}

// ------------------------------
// Function Definitions
// ------------------------------

/**
 * @brief Reads the ADC voltage from the MCP3550, including wait for data ready.
 * Handles sign extension, overflow check, and voltage conversion.
 * @param timeout_ms Timeout duration in milliseconds to wait for data ready signal.
 * @return The measured voltage as a double, or NAN if timeout/overflow occurs.
 */
double readADCVoltage(unsigned long timeout_ms)
{
    uint32_t value = 0;
  	uint32_t timeout = 200000UL;  // Timeout counter
	//IO_SET(SPI2_PORT, SPI2_SCK);		//set sck -> mode 11
	digitalWrite(MCP_CS_PIN, LOW);		//cs brought low
	
	while (timeout-- && (digitalRead(MISO) != LOW)) continue;	//wait for SDO/READY to go low
	//while ((IO_GET(MCP3550_PORT_IN, MCP3550_MISO))) continue;	//wait for SDO/READY to go low
	//now data is ready
	SPI.beginTransaction(adcSPISettings);

	// Read 4 bytes (MSB first)
	value = SPI.transfer(0);              	// Read most significant byte
	value = (value << 8) | SPI.transfer(0); // Read second byte
	value = (value << 8) | SPI.transfer(0); // Read third byte
	value = (value << 8) | SPI.transfer(0); // Read least significant byte
	
	SPI.endTransaction();

	//IO_SET(SPI2_PORT, SPI2_SCK);		//set sck -> mode 11
	//IO_CLR(SPI2_PORT, SPI2_SCK);		//set sck -> mode 11
	//IO_SET(SPI2_PORT, SPI2_SCK);		//set sck -> mode 11
	digitalWrite(MCP_CS_PIN, HIGH);
	
    // Convert the raw 24-bit signed ADC code to a voltage using datasheet formula:
    // V_IN = DataOut * (VREF / 2^23)TODO:2^22
    // 'DataOut' is the 24-bit signed value we have in raw_24bit.
    double voltage = (double)value / ADC_DIVISOR * ADC_VREF;

    return voltage;
}
/*
 * Maps a desired resistance (in ohms) to the corresponding 8-bit digital code
 * for the AD5292. The mapping assumes a linear range from 0 to MAX_DIGIPOT_RES.
*/
uint16_t calculateDigipotSetting(double resistance_target) 
{
	if (resistance_target > MAX_DIGIPOT_RES) 
		resistance_target = MAX_DIGIPOT_RES; // Cap at the maximum available resistance.
	
	uint16_t code = (uint16_t)((resistance_target / MAX_DIGIPOT_RES) * MAX_DIGIPOT_CODE);
	return code;
}
  
/*
 * Sends the digital potentiometer value to the AD5292 over SPI.
*/
void writeDigipot(uint16_t value) 
{
	SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
	
	// Activate the chip select for the digital potentiometer.
	digitalWrite(DIGIPOT_CS_PIN, LOW);
	
	// Send the 8-bit value. (If your AD5292 requires additional command bits or multiple bytes,
	// adjust this SPI.transfer() accordingly.)
	SPI.transfer(value);
	
	// Deactivate the chip select.
	digitalWrite(DIGIPOT_CS_PIN, HIGH);
	
	SPI.endTransaction();
}
