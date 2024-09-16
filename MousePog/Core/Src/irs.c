/*
 * irs.c
 */

#include "main.h"
#include "irs.h"
#include "delay.h"

// Buffer that will get filled up with all IR measurements
uint16_t adc_buf[NUM_SAMPLES];
// Tell when the ADC has finished filling up the buffer
uint8_t complete = 0;

/*
 Handles reading from a specific IR (Left, FrontLeft, Right, FrontRight)
 */
uint16_t readIR(IR ir)
{

	uint16_t result;

	switch(ir){

		case IR_LEFT:
			HAL_GPIO_WritePin(LeftEmitter_GPIO_Port, LeftEmitter_Pin, GPIO_PIN_SET);
			delayMicroseconds(20);
			result = analogRead(ir);
			HAL_GPIO_WritePin(LeftEmitter_GPIO_Port, LeftEmitter_Pin, GPIO_PIN_RESET);
			break;

		case IR_FRONT_LEFT:
			HAL_GPIO_WritePin(FrontLeftEmitter_GPIO_Port, FrontLeftEmitter_Pin, GPIO_PIN_SET);
			delayMicroseconds(20);
			result = analogRead(ir);
			HAL_GPIO_WritePin(FrontLeftEmitter_GPIO_Port, FrontLeftEmitter_Pin, GPIO_PIN_RESET);
			break;

		case IR_FRONT_RIGHT:
			HAL_GPIO_WritePin(FrontRightEmitter_GPIO_Port, FrontRightEmitter_Pin, GPIO_PIN_SET);
			delayMicroseconds(20);
			result = analogRead(ir);
			HAL_GPIO_WritePin(FrontRightEmitter_GPIO_Port, FrontRightEmitter_Pin, GPIO_PIN_RESET);
			break;

		case IR_RIGHT:
			HAL_GPIO_WritePin(RightEmitter_GPIO_Port, RightEmitter_Pin, GPIO_PIN_SET);
			delayMicroseconds(20);
			result = analogRead(ir);
			HAL_GPIO_WritePin(RightEmitter_GPIO_Port, RightEmitter_Pin, GPIO_PIN_RESET);
			break;

		default:
			result = 0;

	}

	return result;
}


uint16_t readLeftIR(void)
{
	return readIR(IR_LEFT);
}

uint16_t readFrontLeftIR(void)
{
	return readIR(IR_FRONT_LEFT);
}

uint16_t readFrontRightIR(void)
{
	return readIR(IR_FRONT_RIGHT);
}


uint16_t readRightIR(void)
{
	return readIR(IR_RIGHT);
}

/*
 Reads the specific channel of the ADC corresponding to the correct IR
 */
uint16_t analogRead(IR ir)
{
    ADC_ChannelConfTypeDef sConfig = {0}; // Initializes the IR ADC [Analog to Digital Converter]
    ADC_HandleTypeDef *hadc1_ptr = Get_HADC1_Ptr(); // pointer to the hal_adc
    // Pointer will also be used to read the analog value, val = HAL_ADC_GetValue(hadc1_ptr);

    // Selects the IR direction to choose the right ADC.
    switch(ir)
    {
        case IR_LEFT:
            sConfig.Channel = ADC_CHANNEL_9;
            break;
        case IR_FRONT_LEFT:
            sConfig.Channel = ADC_CHANNEL_15;
            break;
        case IR_FRONT_RIGHT:
            sConfig.Channel = ADC_CHANNEL_14;
            break;
        case IR_RIGHT:
            sConfig.Channel = ADC_CHANNEL_6;
            break;
        default:
            return 0;
    }

    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;

    // makes sure everything was set up correctly
    if (HAL_ADC_ConfigChannel(hadc1_ptr, &sConfig) != HAL_OK)
    {
        return 0;
    }
    
    complete = 0;

    // Start filling up the ADC buffer
    HAL_ADC_Start_DMA(hadc1_ptr, (uint32_t*)adc_buf, NUM_SAMPLES);

    // Wait for the buffer to become full
    while (complete == 0)
    {
        continue;
    }

    uint32_t sum = 0;
    // Calculate the sum of the measurements in order to calculate the average
    uint16_t measurement = 0;
    while(measurement < NUM_SAMPLES) // Takes multiple measurements
    {
        sum += adc_buf[measurement];
        ++measurement;
    }

    return sum/NUM_SAMPLES;
}

/*
 Function is called when the ADC buffer is filled
 Stops the ADC and changes "complete" variable to be "true"
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    // stop the ADC
    HAL_ADC_Stop_DMA(hadc);
    complete = 1;
}
