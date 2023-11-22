#include "perception.h"

struct envData envRead(uint32_t *adc_in)
{
	struct envData environment;
	
	environment.adc = adcRead(adc_in);
	environment.input = inputRead();
	
	return environment;
}

struct adcData adcRead(uint32_t *adc_in)
{
	struct adcData adcInfo;
	
	adcInfo.batteryVoltage = adc_in[2];
	adcInfo.leftBallastPosition = adc_in[0];
	adcInfo.rightBallastPosition = adc_in[1];

	return adcInfo;
	
}

struct inputData inputRead()
{
	struct inputData inputInfo;
	
	inputInfo.nFaultRight = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11);
	inputInfo.nFaultProp = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_12);
	inputInfo.nFaultLeft = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13);
	
	return inputInfo;
}
