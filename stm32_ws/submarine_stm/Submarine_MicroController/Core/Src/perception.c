#include "perception.h"

struct envData envRead()
{
	struct envData environment;
	
	environment.adc = adcRead();
	environment.input = inputRead();
	
	return environment;
}

struct adcData adcRead()
{
	struct adcData adcInfo;
	
	adcInfo.batteryVoltage = 16.9;
	adcInfo.leftBallastPosition = 0.5;
	adcInfo.rightBallastPosition = 0.5;

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
