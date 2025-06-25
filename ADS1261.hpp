#ifndef _ADS1261_HPP_
#define _ADS1261_HPP_

#include "Arduino.h"
#include "SPI.h"

#include "configurations.hpp"

/*----------------------------------------------------------------------------*/
/* SPI command opcodes */
/*----------------------------------------------------------------------------*/

/** "NOP" SPI command */
#define OPCODE_NOP								((uint8_t) 0x00)

/** "RESET" SPI command */
#define OPCODE_RESET							((uint8_t) 0x06)

/** "START" SPI command */
#define OPCODE_START							((uint8_t) 0x08)

/** "STOP" SPI command */
#define OPCODE_STOP								((uint8_t) 0x0A)

/** "RDATA" SPI command */
#define OPCODE_RDATA							((uint8_t) 0x12)

/** "SYOCAL" SPI command */
#define OPCODE_SYOCAL							((uint8_t) 0x16)

/** "SYGCAL" SPI command */
#define OPCODE_SYGCAL							((uint8_t) 0x17)

/** "SFOCAL" SPI command */
#define OPCODE_SFOCAL							((uint8_t) 0x19)

/** "RREG" SPI command */
#define OPCODE_RREG								((uint8_t) 0x20)

/** "WREG" SPI command */
#define OPCODE_WREG								((uint8_t) 0x40)

/** "LOCK" SPI command */
#define OPCODE_LOCK								((uint8_t) 0xF2)

/** "UNLOCK" SPI command */
#define OPCODE_UNLOCK							((uint8_t) 0xF5)

/** ID register address */
#define REG_ADDR_ID								((uint8_t) 0x00)

/** INPMUX register address */
#define REG_ADDR_INPMUX							((uint8_t) 0x11)

/** REF register address */
#define REG_ADDR_REF							((uint8_t) 0x06)

/** PGA register address */
#define REG_ADDR_PGA							((uint8_t) 0x10)

/** MODE0 register address */
#define REG_ADDR_MODE0							((uint8_t) 0x02)

/** MODE1 register address */
#define REG_ADDR_MODE1							((uint8_t) 0x03)

/** MODE3 register address */
#define REG_ADDR_MODE3							((uint8_t) 0x05)

#define STATUS_DRDY_MASK						((uint8_t) 0x04)

/** Internal reference enable bit mask */
#define REF_REFENB_MASK							((uint8_t) 0x10)

/* Define the positive reference inputs */
#define REF_RMUXP_INT_P							((uint8_t) 0x00)

/* Define the negative reference inputs */
#define REF_RMUXN_INT_N							((uint8_t) 0x00)

/** PGA default (reset) value */
#define	PGA_DEFAULT								((uint8_t) 0x00)

/** Define the filter modes */
#define MODE0_SINC1								((uint8_t) 0x00)
#define MODE0_SINC2								((uint8_t) 0x01)
#define MODE0_SINC3								((uint8_t) 0x02)
#define MODE0_SINC4								((uint8_t) 0x03)
#define MODE0_FIR								((uint8_t) 0x04)


// ADS1261_ADC Class using functions from ads1261.hpp/cpp
class ADS1261_ADC
{
private:
    int ChipSelectPin;
    int DRDYPin;        // Specific DRDY pin for this ADC
    int nRESETPin;      // Common RESET pin
    int nPWDNPin;       // Common PWDN pin
    int STARTPin;       // Common START pin

    SPISettings spiSettings;
	uint8_t gain;
	double max;
public:
    ADS1261_ADC(int csPin, int drdyPinToUse, int nRstPin, int nPWDNPin, int strtPin)
        : ChipSelectPin(csPin), DRDYPin(drdyPinToUse), nRESETPin(nRstPin), nPWDNPin(nPWDNPin), STARTPin(strtPin),
        spiSettings(2000000, MSBFIRST, SPI_MODE1)
    {
        Serial.println("Initializing shared pins and SPI...");

        initilaize();
		max = 0.0;
    }
public:
	// Initializes the ADC according to the datasheet's 79 page sequence
	// https://www.ti.com/lit/ds/symlink/ads1261.pdf?ts=1750435565603&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FADS1261%253FkeyMatch%253DADS1261%2526tisearch%253Duniversal_search%2526usecase%253DGPN
    void initilaize(void)
    {   
        pinMode(ChipSelectPin, OUTPUT);

		digitalWrite(ChipSelectPin, HIGH);
        pinMode(DRDYPin, INPUT); // Use pullup for DRDY
        delayMicroseconds(5);

		pinMode(ADC_RESET_AND_PWDN_PIN, OUTPUT);
		pinMode(ADC_START_PIN, OUTPUT);
	
		// Set /RESET and /PWDN high
		Serial.println("Pulling ADC /RESET and /PWDN pin HIGH!");
		digitalWrite(ADC_RESET_AND_PWDN_PIN, HIGH);
	
		delayMicroseconds(5);
	
		// Check if DRDY pin is high
		waitForDRDY();

		// Write LOW for START pin to set adc to idle, so DRDY pin will be HIGH
		digitalWrite(ADC_START_PIN, LOW);

		unlock();

		// Send STOP command to stop any ongoing conversions during initialization
		stopConversions();

		delayMicroseconds(5);

		// Configure for internal 2.5V reference
		bool refConfig = checkAndConfigureREFRegister();

		if(!refConfig)
		{
			Serial.println("REF register configuration failed, returning...");
		}
		Serial.println("REF register configured!");

		delayMicroseconds(5);

		// Configure for the defined PGA_GAIN
		bool pgaConfig = checkAndConfigurePGARegister(PGA_GAIN);

		if(!pgaConfig)
		{
			Serial.println("PGA register configuration failed, returning...");
		}
		Serial.println("PGA register configured!");

		delayMicroseconds(5);

        // Configure input MUX for differential AIN1-AIN2
        uint8_t muxConfig = 0x20        			// 0x20: positive input = AIN2
                          | 0x03;       			// 0x03: negative input = AIN3
        writeRegister(REG_ADDR_INPMUX, muxConfig); 	// Write INPMUX register (address 0x11) with value 0x23
        // Now the ADC will measure the voltage difference V(AIN2) – V(AIN3) on each conversion

		delayMicroseconds(5);

		// Set the sampling rate defined in SAMPLE_RATE
		bool samplingRateConfig = checkAndConfigureSamplingRate(SAMPLING_RATE);

		if(!samplingRateConfig)
			Serial.println("MODE0 register configuration failed, returning...");

		delayMicroseconds(5);

		// Disable status 
		// [PWDN STATENB CRCENB SPITIM GPIO_DAT3:0]
		uint8_t mode3Config = 0b01000000;
		writeRegister(REG_ADDR_MODE3, mode3Config);
		
		delay(25);

		startConversion();

        // Now ADS1261 is running in continuous conversion mode (CONVRT bit = 0)
        // DRDY pin will pulse low every 25 µs (40kHz) to indicate new data ready hopefully.
    }
	// Waits until we get a pulse on the DRDY pin specified in the constructor
    bool waitForDRDY(int waitMs = 1000)
    {
        unsigned long start = millis();
        while(digitalRead(DRDYPin) == LOW)
		{
            if(millis() - start > 1000)
			{
                Serial.println("DRDY timeout...");
                Serial.println("DRDY state:" + String(digitalRead(DRDYPin)));
                start = millis();
            }
        }
        return true;
    }
private:
	
	void printByteFormats(const char* name, uint8_t v) 
	{
		// build an 8-char binary string
		char bin[9];
		for (int i = 0; i < 8; i++) 
			bin[i] = (v & (1 << (7 - i))) ? '1' : '0';
		bin[8] = '\0';
		// and print: label [binary 0xHH DDD]
		Serial.printf("%-6s [ %s 0x%02X %3u ]\n", name, bin, v, v);
	}

	void unlock(void)
	{
		digitalWrite(ChipSelectPin, LOW);

		SPI.beginTransaction(spiSettings);

		uint8_t ffh = SPI.transfer(OPCODE_UNLOCK);
		uint8_t echoByte = SPI.transfer(0x00);

		SPI.endTransaction();
		
		digitalWrite(ChipSelectPin, HIGH);

	}

	// Writes the specified reg register, with the specified value
    void writeRegister(uint8_t reg, uint8_t value)
    {
		digitalWrite(ChipSelectPin, LOW);

		SPI.beginTransaction(spiSettings);

        uint8_t ffh = SPI.transfer(OPCODE_WREG + (reg & 0x1F));
        uint8_t echobyte1 = SPI.transfer(value);

		SPI.endTransaction();
		
		digitalWrite(ChipSelectPin, HIGH);

		if(ffh != 0xFF)
			Serial.println("Something went wrong during WREG command!");
    }

	// Reads the specified register's value
	uint8_t readSingleRegister(uint8_t reg)
	{
		digitalWrite(ChipSelectPin, LOW);

		SPI.beginTransaction(spiSettings);

		uint8_t ffh = SPI.transfer(OPCODE_RREG + (reg & 0x1F));	// 0x20h + rrh (5 bit register address)
		uint8_t echobyte = 	SPI.transfer(0x00);		// Arbitrary
		uint8_t value = SPI.transfer(0x00);		// Register data associated to address

		SPI.endTransaction();

        digitalWrite(ChipSelectPin, HIGH);
		
		return value;
	}

	// Configures the REF register so we use the internal 2.5V reference
	bool checkAndConfigureREFRegister()
	{
		// 2. Enable internal 2.5V reference and select it as reference source
		// First check REF register for default value
		Serial.println("Check REF default value: ");
		uint8_t refValue = readSingleRegister(REG_ADDR_REF);	// Should return 0x05

		if(refValue != 0x05)
			Serial.println("REF default value should be 0x05, but it differs!");

		Serial.println("Setting REF register to internal 2.5V mode!");

		uint8_t refConfig = (REF_REFENB_MASK | REF_RMUXP_INT_P | REF_RMUXN_INT_N);;

		writeRegister(REG_ADDR_REF, refConfig); // Write REF register (address 0x06) with value 0x10
		delay(20);  							// Wait 20+ ms for internal reference to stabilize (required for accuracy)
		
		refValue = readSingleRegister(REG_ADDR_REF); 

		bool config = refValue == refConfig;
		Serial.println("Refernce configuration success: " + String(config));

		printByteFormats("REF config: ", refConfig);
		printByteFormats("REF value: ", refValue);

		return config;
	}

public:
	// Configures the PGA register to x64 mode
	bool checkAndConfigurePGARegister(uint8_t pgaGain = PGA_GAIN)
	{
		Serial.println("Check PGA default value: ");
		uint8_t pgaValue = readSingleRegister(REG_ADDR_PGA);	// Should return 0x00

		if(pgaValue != 0x00)
			Serial.println("PGA default value should be 0x00, but it differs : " + String(pgaValue));

		uint8_t pgaConfig = PGA_DEFAULT;       // 0x06: PGA gain = 64 (binary 110)
		uint8_t gain = PGA_GAIN;

		switch(gain)
		{
			case 1:
				pgaConfig = 0x00;
				break;
			case 2:
				pgaConfig = 0x01;
				break;
			case 4:
				pgaConfig = 0x02;
				break;
			case 8:
				pgaConfig = 0x03;
				break;
			case 16:
				pgaConfig = 0x04;
				break;
			case 32:
				pgaConfig = 0x05;
				break;
			case 64:
				pgaConfig = 0x06;
				break;
			case 128:
				pgaConfig = 0x07;
				break;
			default: break;
		}

        // Set PGA gain to 64
        writeRegister(REG_ADDR_PGA, pgaConfig); // Write PGA register (address 0x10)
		delay(20);  							
		
		pgaValue = readSingleRegister(REG_ADDR_PGA);

		bool config = pgaValue == pgaConfig;
		Serial.println("PGA configuration success: " + String(config) + "PGA value: " + String(pgaValue));
		
		printByteFormats("PGA config: ", pgaConfig);
		printByteFormats("PGA value: ", pgaValue);

		return config;
	}
	
	// Configure MODE0 register with the defined SAMPLING_RATE
	bool checkAndConfigureSamplingRate(uint8_t samplingCode = SAMPLING_RATE)
	{
		uint8_t mode0Config = 0x00;
		uint8_t samlpingRate = (uint8_t)samplingCode;

		// Sight left by 3 bits
		mode0Config = (samlpingRate << 3) | FILTER_TYPE;

        writeRegister(REG_ADDR_MODE0, mode0Config); // Write MODE0 register (address 0x02)

		// Set MODE1 register to Normal mode, Continous mode and 0 latency before conversion
		uint8_t mode1Config = 0x00;
		writeRegister(REG_ADDR_MODE1, mode1Config);

		delayMicroseconds(5);		

		uint8_t mode0Value = readSingleRegister(REG_ADDR_MODE0);
		uint8_t mode1Value = readSingleRegister(REG_ADDR_MODE1);

		bool config = (mode1Config == mode1Value) && (mode0Config == mode0Value);
		Serial.println("MODE0 and MODE1 registers configured, configuration success: " + String(config));
		printByteFormats("MODE0 config: ", mode0Config);
		printByteFormats("MODE0 value: ", mode0Value);
		printByteFormats("MODE1 config: ", mode1Config);
		printByteFormats("MOD1 value: ", mode1Value);

		return config;
	}
	// Reads the result of the last conversion and converts it to double
    double readConversion()
    {
		const double vref = 2.5;

        uint8_t data[3];
        digitalWrite(ChipSelectPin, LOW);
		SPI.beginTransaction(spiSettings);

        uint8_t ffh = SPI.transfer(OPCODE_RDATA);   // 0x12 command to read data

		uint8_t echoByte = SPI.transfer(0x00);
		uint8_t status = SPI.transfer(0x00);

		// 1st: MSB, 2nd: MID, 3rd: LSB
		data[0] = SPI.transfer(0x00);
        data[1] = SPI.transfer(0x00);
        data[2] = SPI.transfer(0x00);

		SPI.endTransaction();

        digitalWrite(ChipSelectPin, HIGH);
        
		if(!(status & STATUS_DRDY_MASK))
			return -99.0;

		// Combine 24-bit data (ADS1261 outputs in binary two's complement)
        uint32_t rawValue = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | (uint32_t)data[2];
	
		// sign-extend if negative (bit23 set)
		if (rawValue & 0x00800000) 
			rawValue |= 0xFF000000;

		int32_t signedRaw = (int32_t)rawValue;

		double voltage = ((double)signedRaw) * vref / 8388608.0;  

		return voltage;
    }
	
	// Set the INPMUX register, to the specified channels
	void setInputChannels(uint8_t ch1, uint8_t ch2)
	{
		stopConversions();
		
		// Calculate configuration channel
		// 7:4 bits are positive channel, 3:0 bits are negative channel selection
		uint8_t muxConfig = (ch1 << 4) | (ch2 & 0b00001111);

		// Write to the INPMUX register
		writeRegister(REG_ADDR_INPMUX, muxConfig); // Write INPMUX register (address 0x11) with value 0x23

		uint8_t inpmux = readSingleRegister(REG_ADDR_INPMUX);
		Serial.printf("INPMUX register value: %d\n", inpmux);

		// Start the new conversion
		startConversion();
	}

	// Starts a new conversion in single shot mode, or starts a continous conversion
	void startConversion()
	{
		// Start the new conversion
		digitalWrite(ChipSelectPin, LOW);

		SPI.beginTransaction(spiSettings);

		// Send start command and arbitrary
		uint8_t ffh = SPI.transfer(OPCODE_START);
		SPI.transfer(0x00);
		
		SPI.endTransaction();	
		
		digitalWrite(ChipSelectPin, HIGH);
		
		if(ffh != 0xFF)
			Serial.print("Start command answere didn't result in 0xFF, something went wrong!");
	}

	// Stops any ongoing conversion
	void stopConversions()
	{
		digitalWrite(ChipSelectPin, LOW);
     
        SPI.beginTransaction(spiSettings);

        uint8_t ffh = SPI.transfer(OPCODE_STOP);
		SPI.transfer(0x00);

        SPI.endTransaction();
		
		digitalWrite(ChipSelectPin, HIGH);

		if(ffh != 0xFF)
			Serial.print("Stop command answere didn't result in 0xFF, something went wrong!");
	}

	void restart()
	{
 		digitalWrite(ChipSelectPin, LOW);
		SPI.beginTransaction(spiSettings);

        uint8_t o6h = SPI.transfer(OPCODE_RESET);   // 0x12 command to read data
		uint8_t echoByte = SPI.transfer(0x00);

		SPI.endTransaction();

        digitalWrite(ChipSelectPin, HIGH);
	}
};

/* Define DEV_ID (device) */
#define ID_DEV_ADS1261						((uint8_t) 0x80)
#define ID_DEV_ADS1260						((uint8_t) 0xA0)
#define ID_DEV_MASK							((uint8_t) 0xF0)

/* Define REV_ID (revision) */
/* Note: Revision ID can change without notification */
#define ID_REV_A							((uint8_t) 0x00)
#define ID_REV_B                            ((uint8_t) 0x01)
#define ID_REV_MASK							((uint8_t) 0x0F)


/* Register 0x01 (STATUS) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |  LOCK   | CRC_ERR |PGAL_ALM |PGAH_ALM |REFL_ALM |  DRDY   |  CLOCK  |  RESET  |
 * ---------------------------------------------------------------------------------
 */

	/** STATUS register address */
	#define REG_ADDR_STATUS						((uint8_t) 0x01)

	/** STATUS default (reset) value */
	#define STATUS_DEFAULT						((uint8_t) 0x01)

	/* Define the STATUS byte bit masks */
	#define STATUS_LOCK_MASK					((uint8_t) 0x80)
	#define STATUS_CRC_ERR_MASK					((uint8_t) 0x40)
	#define STATUS_PGAL_ALM_MASK				((uint8_t) 0x20)
	#define STATUS_PGAH_ALM_MASK				((uint8_t) 0x10)
	#define STATUS_REFL_ALM_MASK				((uint8_t) 0x08)
	#define STATUS_DRDY_MASK					((uint8_t) 0x04)
	#define STATUS_CLOCK_MASK					((uint8_t) 0x02)
	#define STATUS_RESET_MASK					((uint8_t) 0x01)

	/** Write this value to clear the STATUS register */
	#define STATUS_CLEAR						((uint8_t) 0x00)


/* Register 0x02 (MODE0) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |         			 DR[4:0]   				 	 |       	FILTER[2:0]		   |
 * ---------------------------------------------------------------------------------
 */

	/** MODE0 register address */
	#define REG_ADDR_MODE0						((uint8_t) 0x02)

	/** MODE0 default (reset) value */
	#define MODE0_DEFAULT						((uint8_t) 0x24)

	/* Define the data rates */
	#define MODE0_DR_2_5_SPS					((uint8_t) 0x00)
	#define MODE0_DR_5_SPS						((uint8_t) 0x08)
	#define MODE0_DR_10_SPS						((uint8_t) 0x10)
	#define MODE0_DR_16_6_SPS					((uint8_t) 0x18)
	#define MODE0_DR_20_SPS						((uint8_t) 0x20)
	#define MODE0_DR_50_SPS						((uint8_t) 0x28)
	#define MODE0_DR_60_SPS						((uint8_t) 0x30)
	#define MODE0_DR_100_SPS					((uint8_t) 0x38)
	#define MODE0_DR_400_SPS					((uint8_t) 0x40)
	#define MODE0_DR_1200_SPS					((uint8_t) 0x48)
	#define MODE0_DR_2400_SPS					((uint8_t) 0x50)
	#define MODE0_DR_4800_SPS					((uint8_t) 0x58)
	#define MODE0_DR_7200_SPS					((uint8_t) 0x60)
	#define MODE0_DR_14400_SPS					((uint8_t) 0x68)
	#define MODE0_DR_19200_SPS					((uint8_t) 0x70)
	#define MODE0_DR_25600_SPS					((uint8_t) 0x78)
	#define MODE0_DR_40000_SPS					((uint8_t) 0x80)
	#define MODE0_DR_MASK						((uint8_t) 0xF8)

	/** Define the filter modes */
	#define MODE0_SINC1							((uint8_t) 0x00)
	#define MODE0_SINC2							((uint8_t) 0x01)
	#define MODE0_SINC3							((uint8_t) 0x02)
	#define MODE0_SINC4							((uint8_t) 0x03)
	#define MODE0_FIR							((uint8_t) 0x04)


/* Register 0x03 (MODE1) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |    0    |     CHOP[1:0]**	 | CONVRT  |               DELAY[3:0]     	  	   |
 * ---------------------------------------------------------------------------------
 * ** ADS1260: ALWAYS write "0" to Bit 6 of MODE1 register!
 */

	/** MODE1 register address */
	#define REG_ADDR_MODE1						((uint8_t) 0x03)

	/** MODE1 default (reset) value */
	#define MODE1_DEFAULT						((uint8_t) 0x01)

	/* Define chop and AC-excitation modes */
	#define MODE1_CHOP_OFF    					((uint8_t) 0x00)
	#define MODE1_CHOP_ON						((uint8_t) 0x20)
#ifdef ADS1261_ONLY_FEATURES
	#define MODE1_CHOP_2WIRE_ACX				((uint8_t) 0x40)
	#define MODE1_CHOP_4WIRE_ACX				((uint8_t) 0x60)
#endif

	/* Define ADC conversion modes */
	#define MODE1_CONVRT_CONTINUOUS				((uint8_t) 0x00) // default
	#define MODE1_CONVRT_PULSE 					((uint8_t) 0x10)

	/* Define conversion start delays */
	#define MODE1_DELAY_0_uS					((uint8_t) 0x00)
	#define MODE1_DELAY_50_uS					((uint8_t) 0x01)
	#define MODE1_DELAY_59_uS					((uint8_t) 0x02)
	#define MODE1_DELAY_67_uS					((uint8_t) 0x03)
	#define MODE1_DELAY_85_uS					((uint8_t) 0x04)
	#define MODE1_DELAY_119_uS					((uint8_t) 0x05)
	#define MODE1_DELAY_189_uS					((uint8_t) 0x06)
	#define MODE1_DELAY_328_uS					((uint8_t) 0x07)
	#define MODE1_DELAY_605_uS					((uint8_t) 0x08)
	#define MODE1_DELAY_1160_uS					((uint8_t) 0x09)
	#define MODE1_DELAY_2270_uS					((uint8_t) 0x0A)
	#define MODE1_DELAY_4490_uS					((uint8_t) 0x0B)
	#define MODE1_DELAY_8930_uS					((uint8_t) 0x0C)
	#define MODE1_DELAY_17800_uS				((uint8_t) 0x0D)


/* Register 0x04 (MODE2) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |   			GPIO_CON[3:0]**			   |			  GPIO_DIR[3:0]**		   |
 * ---------------------------------------------------------------------------------
 * ** ADS1260: RESERVED, always write "0"!
 */

	/** MODE2 register address */
	#define REG_ADDR_MODE2						((uint8_t) 0x04)

	/** MODE2 default (reset) value */
	#define MODE2_DEFAULT						((uint8_t) 0x00)

#ifdef ADS1261_ONLY_FEATURES
	/* Define the GPIO pin connection masks (0-Analog Input; 1-GPIO) */
	#define MODE2_GPIOCON_AIN5_ENABLE_MASK		((uint8_t) 0x80)
	#define MODE2_GPIOCON_AIN4_ENABLE_MASK		((uint8_t) 0x40)
	#define MODE2_GPIOCON_AIN3_ENABLE_MASK		((uint8_t) 0x20)
	#define MODE2_GPIOCON_AIN2_ENABLE_MASK		((uint8_t) 0x10)

	/* Define the GPIO pin direction masks (0-Output; 1-Input) */
	#define MODE2_GPIODIR_AIN5_INPUT_MASK		((uint8_t) 0x08)
	#define MODE2_GPIODIR_AIN4_INPUT_MASK		((uint8_t) 0x04)
	#define MODE2_GPIODIR_AIN3_INPUT_MASK		((uint8_t) 0x02)
	#define MODE2_GPIODIR_AIN2_INPUT_MASK		((uint8_t) 0x01)
#endif


/* Register 0x05 (MODE3) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |  PWDN	 | STATENB | CRCENB  | SPITIM  |		     GPIO_DAT[3:0]**		   |
 * ---------------------------------------------------------------------------------
 * ** ADS1260: Always returns "0000".
 */

	/** MODE3 register address */
	#define REG_ADDR_MODE3						((uint8_t) 0x05)

	/** MODE3 default (reset) value */
	#define MODE3_DEFAULT						((uint8_t) 0x00)

	/* Define the MODE3 upper nibble bit masks */
	#define MODE3_PWDN_MASK						((uint8_t) 0x80)
	#define MODE3_STATEN_MASK					((uint8_t) 0x40)
	#define MODE3_CRCEN_MASK					((uint8_t) 0x20)
	#define MODE3_SPITIM_MASK					((uint8_t) 0x10)

#ifdef ADS1261_ONLY_FEATURES
	/* Define the GPIO data setting masks (0-Low; 1-High)*/
	#define MODE3_GPIODAT_AIN5_DATA_MASK		((uint8_t) 0x08)
	#define MODE3_GPIODAT_AIN4_DATA_MASK		((uint8_t) 0x04)
	#define MODE3_GPIODAT_AIN3_DATA_MASK		((uint8_t) 0x02)
	#define MODE3_GPIODAT_AIN2_DATA_MASK		((uint8_t) 0x01)
#endif


/* Register 0x06 (REF) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |    0    |    0    |    0    |  REFENB | 	RMUXP[1:0]**   |	 RMUXN[1:0]**  |
 * ---------------------------------------------------------------------------------
 * ** ADS1260: AIN2/AIN3 pins are not available for reference input!
 */

	/** REF register address */
	#define REG_ADDR_REF						((uint8_t) 0x06)

	/** REF default (reset) value */
	#define REF_DEFAULT							((uint8_t) 0x05)

	/** Internal reference enable bit mask */
	#define REF_REFENB_MASK						((uint8_t) 0x10)

	/* Define the positive reference inputs */
	#define REF_RMUXP_INT_P						((uint8_t) 0x00)
	#define REF_RMUXP_AVDD						((uint8_t) 0x04)
	#define REF_RMUXP_AIN0						((uint8_t) 0x08)
#ifdef ADS1261_ONLY_FEATURES
	#define REF_RMUXP_AIN2					    ((uint8_t) 0x0C)
#endif

	/* Define the negative reference inputs */
	#define REF_RMUXN_INT_N						((uint8_t) 0x00)
	#define REF_RMUXN_AVSS						((uint8_t) 0x01)
	#define REF_RMUXN_AIN1						((uint8_t) 0x02)
#ifdef ADS1261_ONLY_FEATURES
    #define REF_RMUXN_AIN3                      ((uint8_t) 0x03)
#endif


/* Register 0x07 (OFCAL0) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |    								OFC[7:0]								   |
 * ---------------------------------------------------------------------------------
 */

	/** OFCAL0 register address */
	#define REG_ADDR_OFCAL0						((uint8_t) 0x07)

	/** OFCAL0 default (reset) value */
	#define OFCAL0_DEFAULT						((uint8_t) 0x00)


/* Register 0x08 (OFCAL1) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |    								OFC[15:8]								   |
 * ---------------------------------------------------------------------------------
 */
	/** OFCAL1 register address */
	#define REG_ADDR_OFCAL1						((uint8_t) 0x08)

	/** OFCAL1 default (reset) value */
	#define OFCAL1_DEFAULT						((uint8_t) 0x00)


/* Register 0x09 (OFCAL2) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |    							   OFC[23:16]								   |
 * ---------------------------------------------------------------------------------
 */
	/** OFCAL2 register address */
	#define REG_ADDR_OFCAL2						((uint8_t) 0x09)

	/** OFCAL2 default (reset) value */
	#define OFCAL2_DEFAULT						((uint8_t) 0x00)


/* Register 0x0A (FSCAL0) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |    								FSC[7:0]								   |
 * ---------------------------------------------------------------------------------
 */

	/** FSCAL0 register address */
	#define REG_ADDR_FSCAL0						((uint8_t) 0x0A)

	/** FSCAL0 default (reset) value */
	#define FSCAL0_DEFAULT						((uint8_t) 0x00)


/* Register 0x0B (FSCAL1) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |    							   FSC[15:8]								   |
 * ---------------------------------------------------------------------------------
 */

	/** FSCAL1 register address */
	#define REG_ADDR_FSCAL1						((uint8_t) 0x0B)

	/** FSCAL1 default (reset) value */
	#define FSCAL1_DEFAULT						((uint8_t) 0x00)


/* Register 0x0C (FSCAL2) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |    							   FSC[23:16]								   |
 * ---------------------------------------------------------------------------------
 */

	/** FSCAL2 register address */
	#define REG_ADDR_FSCAL2						((uint8_t) 0x0C)

	/** FSCAL2 default (reset) value */
	#define FSCAL2_DEFAULT						((uint8_t) 0x40)


/* Register 0x0D (IMUX) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |               IMUX2[3:0]**    		   | 			   IMUX1[3:0]**			   |
 * ---------------------------------------------------------------------------------
 * ** ADS1260: Pins AIN5 though AIN9 are not available!
 */

	/** IMUX register address */
	#define REG_ADDR_IMUX						((uint8_t) 0x0D)

	/** IMUX default (reset) value */
	#define IMUX_DEFAULT						((uint8_t) 0xFF)

	/* Define the IDAC2 pin connection */
	#define IMUX_IMUX2_AIN0						((uint8_t) 0x00)
	#define IMUX_IMUX2_AIN1						((uint8_t) 0x10)
	#define IMUX_IMUX2_AIN2						((uint8_t) 0x20)
	#define IMUX_IMUX2_AIN3						((uint8_t) 0x30)
	#define IMUX_IMUX2_AIN4						((uint8_t) 0x40)
#ifdef ADS1261_ONLY_FEATURES
	#define IMUX_IMUX2_AIN5						((uint8_t) 0x50)
	#define IMUX_IMUX2_AIN6						((uint8_t) 0x60)
	#define IMUX_IMUX2_AIN7						((uint8_t) 0x70)
	#define IMUX_IMUX2_AIN8						((uint8_t) 0x80)
	#define IMUX_IMUX2_AIN9						((uint8_t) 0x90)
#endif
	#define IMUX_IMUX2_AINCOM					((uint8_t) 0xA0)
	#define IMUX_IMUX2_NOCONNECT				((uint8_t) 0xF0)

	/* Define the IDAC1 pin connection */
	#define IMUX_IMUX1_AIN0						((uint8_t) 0x00)
	#define IMUX_IMUX1_AIN1						((uint8_t) 0x01)
	#define IMUX_IMUX1_AIN2						((uint8_t) 0x02)
	#define IMUX_IMUX1_AIN3						((uint8_t) 0x03)
	#define IMUX_IMUX1_AIN4						((uint8_t) 0x04)
#ifdef ADS1261_ONLY_FEATURES
	#define IMUX_IMUX1_AIN5						((uint8_t) 0x05)
	#define IMUX_IMUX1_AIN6						((uint8_t) 0x06)
	#define IMUX_IMUX1_AIN7						((uint8_t) 0x07)
	#define IMUX_IMUX1_AIN8						((uint8_t) 0x08)
	#define IMUX_IMUX1_AIN9						((uint8_t) 0x09)
#endif
	#define IMUX_IMUX1_AINCOM					((uint8_t) 0x0A)
	#define IMUX_IMUX1_NOCONNECT				((uint8_t) 0x0F)


/* Register 0x0E (IMAG) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |               IMAG2[3:0]    		   | 			   IMAG1[3:0]			   |
 * ---------------------------------------------------------------------------------
 */

	/** IMAG register address */
	#define REG_ADDR_IMAG						((uint8_t) 0x0E)

	/** IMAG default (reset) value */
	#define IMAG_DEFAULT						((uint8_t) 0x00)

	/* Define the IDAC2 current magnitude */
	#define IMAG_IMAG2_OFF						((uint8_t) 0x00)
	#define IMAG_IMAG2_50_uA					((uint8_t) 0x10)
	#define IMAG_IMAG2_100_uA					((uint8_t) 0x20)
	#define IMAG_IMAG2_250_uA					((uint8_t) 0x30)
	#define IMAG_IMAG2_500_uA					((uint8_t) 0x40)
	#define IMAG_IMAG2_750_uA					((uint8_t) 0x50)
	#define IMAG_IMAG2_1000_uA					((uint8_t) 0x60)
	#define IMAG_IMAG2_1500_uA					((uint8_t) 0x70)
	#define IMAG_IMAG2_2000_uA					((uint8_t) 0x80)
	#define IMAG_IMAG2_2500_uA					((uint8_t) 0x90)
	#define IMAG_IMAG2_3000_uA					((uint8_t) 0xA0)

	/* Define the IDAC1 current magnitude */
	#define IMAG_IMAG1_OFF						((uint8_t) 0x00)
	#define IMAG_IMAG1_50_uA					((uint8_t) 0x01)
	#define IMAG_IMAG1_100_uA					((uint8_t) 0x02)
	#define IMAG_IMAG1_250_uA					((uint8_t) 0x03)
	#define IMAG_IMAG1_500_uA					((uint8_t) 0x04)
	#define IMAG_IMAG1_750_uA					((uint8_t) 0x05)
	#define IMAG_IMAG1_1000_uA					((uint8_t) 0x06)
	#define IMAG_IMAG1_1500_uA					((uint8_t) 0x07)
	#define IMAG_IMAG1_2000_uA					((uint8_t) 0x08)
	#define IMAG_IMAG1_2500_uA					((uint8_t) 0x09)
	#define IMAG_IMAG1_3000_uA					((uint8_t) 0x0A)


/* Register 0x0F (RESERVED) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |    0    |    0    |    0    |    0    |    0    |    0    |    0    |    0    |
 * ---------------------------------------------------------------------------------
 */

	/** RESERVED register address */
	#define REG_ADDR_RESERVED					((uint8_t) 0x0F)

	/** RESERVED default (reset) value */
	#define RESERVED_DEFAULT					((uint8_t) 0x00)


/* Register 0x10 (PGA) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |  BYPASS |    0    |    0    |    0    |    0    |    		GAIN[2:0]	       |
 * ---------------------------------------------------------------------------------
 */

	/** PGA register address */
	#define REG_ADDR_PGA						((uint8_t) 0x10)

	/** PGA default (reset) value */
	#define	PGA_DEFAULT							((uint8_t) 0x00)

	/** Define the PGA Bypass mask (0=PGA normal mode; 1=PGA bypass mode) */
	#define PGA_BYPASS_MASK						((uint8_t) 0x80)

	/* Define the PGA gain definitions */
	#define PGA_GAIN_1							((uint8_t) 0x00)
	#define PGA_GAIN_2							((uint8_t) 0x01)
	#define PGA_GAIN_4							((uint8_t) 0x02)
	#define PGA_GAIN_8							((uint8_t) 0x03)
	#define PGA_GAIN_16							((uint8_t) 0x04)
	#define PGA_GAIN_32							((uint8_t) 0x05)
	#define PGA_GAIN_64							((uint8_t) 0x06)
	#define PGA_GAIN_128						((uint8_t) 0x07)
	#define PGA_GAIN_MASK						((uint8_t) 0x07)


/* Register 0x11 (INPMUX) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |   			   MUXP[3:0]**			   |     			MUXN[3:0]**		       |
 * ---------------------------------------------------------------------------------
 * ** ADS1260: Pins AIN5 though AIN9 are not available!
 */

	/** INPMUX register address */
	#define REG_ADDR_INPMUX						((uint8_t) 0x11)

	/** INPMUX default (reset) value */
	#define	INPMUX_DEFAULT						((uint8_t) 0xFF)

	/* Define the positive input MUX connections */
	#define INPMUX_MUXP_AINCOM					((uint8_t) 0x00)
	#define INPMUX_MUXP_AIN0					((uint8_t) 0x10)
	#define INPMUX_MUXP_AIN1					((uint8_t) 0x20)
	#define INPMUX_MUXP_AIN2					((uint8_t) 0x30)
	#define INPMUX_MUXP_AIN3					((uint8_t) 0x40)
	#define INPMUX_MUXP_AIN4					((uint8_t) 0x50)
#ifdef ADS1261_ONLY_FEATURES
	#define INPMUX_MUXP_AIN5					((uint8_t) 0x60)
	#define INPMUX_MUXP_AIN6					((uint8_t) 0x70)
	#define INPMUX_MUXP_AIN7					((uint8_t) 0x80)
	#define INPMUX_MUXP_AIN8					((uint8_t) 0x90)
	#define INPMUX_MUXP_AIN9					((uint8_t) 0xA0)
#endif
	#define INPMUX_MUXP_TEMP_P					((uint8_t) 0xB0)
	#define INPMUX_MUXP_ASUPPLY_P				((uint8_t) 0xC0)
	#define INPMUX_MUXP_DSUPPLY_P				((uint8_t) 0xD0)
	#define INPMUX_MUXP_OPEN					((uint8_t) 0xE0)
	#define INPMUX_MUXP_VCOM					((uint8_t) 0xF0)

	/* Define the negative input MUX connections */
	#define INPMUX_MUXN_AINCOM					((uint8_t) 0x00)
	#define INPMUX_MUXN_AIN0					((uint8_t) 0x01)
	#define INPMUX_MUXN_AIN1					((uint8_t) 0x02)
	#define INPMUX_MUXN_AIN2					((uint8_t) 0x03)
	#define INPMUX_MUXN_AIN3					((uint8_t) 0x04)
	#define INPMUX_MUXN_AIN4					((uint8_t) 0x05)
#ifdef ADS1261_ONLY_FEATURES
	#define INPMUX_MUXN_AIN5					((uint8_t) 0x06)
	#define INPMUX_MUXN_AIN6					((uint8_t) 0x07)
	#define INPMUX_MUXN_AIN7					((uint8_t) 0x08)
	#define INPMUX_MUXN_AIN8					((uint8_t) 0x09)
	#define INPMUX_MUXN_AIN9					((uint8_t) 0x0A)
#endif
	#define INPMUX_MUXN_TEMP_N					((uint8_t) 0x0B)
	#define INPMUX_MUXN_ASUPPLY_N				((uint8_t) 0x0C)
	#define INPMUX_MUXN_DSUPPLY_N				((uint8_t) 0x0D)
	#define INPMUX_MUXN_OPEN					((uint8_t) 0x0E)
	#define INPMUX_MUXN_VCOM					((uint8_t) 0x0F)


/* Register 0x12 (INPBIAS) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |    0    |    0    |    0    |  VBIAS  |  BCSPOL |     	   BCSMAG[2:0]		   |
 * ---------------------------------------------------------------------------------
 */

	/** INPBIAS register address */
	#define REG_ADDR_INPBIAS					((uint8_t) 0x12)

	/** INPBIAS default (reset) value */
	#define	INPBIAS_DEFAULT						((uint8_t) 0x00)

	/** Define the PGA Bypass mask (0=VBIAS disabled; 1=VBIAS enabled) */
	#define INPBIAS_VBIAS_MASK					((uint8_t) 0x10)

	/* Define the PGA Bypass mask (0=PGA normal mode; 1=PGA bypass mode) */
	#define INPBIAS_BCSPOL_PULLUP				((uint8_t) 0x00)
	#define INPBIAS_BCSPOL_PULLDOWN				((uint8_t) 0x08)

	/* Define the PGA gain definitions */
	#define INPBIAS_BCSMAG_OFF					((uint8_t) 0x00)
	#define INPBIAS_BCSMAG_50_nA				((uint8_t) 0x01)
	#define INPBIAS_BCSMAG_200_nA				((uint8_t) 0x02)
	#define INPBIAS_BCSMAG_1000_nA				((uint8_t) 0x03)
	#define INPBIAS_BCSMAG_10000_nA				((uint8_t) 0x04)


/*----------------------------------------------------------------------------*/
/* END OF REGISTER DEFINITIONS */
/*----------------------------------------------------------------------------*/

#endif /*_ADS1261_HPP_*/