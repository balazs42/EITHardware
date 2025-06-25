#include "ControlSignalGeneration.hpp"

void dac_cosine_enable(dac_channel_t channel)
{
    // Step 1: ENABLE the cosine generator:
    SET_PERI_REG_MASK(SENS_SAR_DAC_CTRL1_REG, SENS_SW_TONE_EN);
    
    // Step 2: CONNECT the cosine generator to a DAC channel:
    switch(channel) 
	{
        // cosine generator is enabled per channel using SENS_DAC_CW_EN1_M and SENS_DAC_CW_EN2_M
        // MSB must be inverted by SENS_DAC_INV1 and SENS_DAC_INV2 
        case DAC_CHANNEL_1:
            SET_PERI_REG_MASK(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_CW_EN1_M);
            SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_INV1, 2, SENS_DAC_INV1_S);
            break;
        case DAC_CHANNEL_2:
            SET_PERI_REG_MASK(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_CW_EN2_M);
            SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_INV2, 2, SENS_DAC_INV2_S);
            break;
    }
}


// frequency (for both channels)is determined by two parameters:
// clk_8m_div (RTC 8M clock divider): 0=8Mhz clock, range is 0-273
// frequency_step: range is 1-65535
void dac_frequency_set(int clk_8m_div, int frequency_step)
{
    REG_SET_FIELD(RTC_CNTL_CLK_CONF_REG, RTC_CNTL_CK8M_DIV_SEL, clk_8m_div);
    SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL1_REG, SENS_SW_FSTEP, frequency_step, SENS_SW_FSTEP_S);
}

// scaling, range is 0-3:
// 0: no scale
// 1: scale to 1/2
// 2: scale to 1/4
// 3: scale to 1/8
void dac_scale_set(dac_channel_t channel, int scale)
{
    switch(channel) 
	{
        case DAC_CHANNEL_1:
            SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_SCALE1, scale, SENS_DAC_SCALE1_S);
            break;
        case DAC_CHANNEL_2:
            SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_SCALE2, scale, SENS_DAC_SCALE2_S);
            break;
    }
}

// Offset output for a particular channel: range is 0-255
void dac_offset_set(dac_channel_t channel, int offset)
{
    switch(channel) 
	{
        case DAC_CHANNEL_1:
            SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_DC1, offset, SENS_DAC_DC1_S);
            break;
        case DAC_CHANNEL_2:
            SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_DC2, offset, SENS_DAC_DC2_S);
            break;
    }
}

// Invert output for a particular channel, range is 0-3:
// 0: no inversion
// 1: completely inverted
// 2: invert MSB (most significant bit)
// 3: invert all but MSB
void dac_invert_set(dac_channel_t channel, int invert)
{
    switch(channel) 
	{
        case DAC_CHANNEL_1:
            SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_INV1, invert, SENS_DAC_INV1_S);
            break;
        case DAC_CHANNEL_2:
            SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_INV2, invert, SENS_DAC_INV2_S);
            break;
    }
}

// task that handles the DAC
void dactask(void* arg) 
{
	Serial.println("Starting signal generation task on DAC1.");

    while(1)
	{
        // Re assign frequency_step, if serial changed FREQ_STEP
        frequency_step = FREQENCY_STEP;

        // set frequency
        dac_frequency_set(clk_8m_div, frequency_step);

        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}
