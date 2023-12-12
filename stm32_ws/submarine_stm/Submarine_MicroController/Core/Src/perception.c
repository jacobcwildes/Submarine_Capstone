#include "perception.h"
/*
Dyllon Dunton
12/11/2023

This file serves as the method of perceiving the world around it by aquiring sensor data.
*/



//Gather sensor data
struct envData envRead(uint32_t *adc_in)
{
	struct envData environment;
	
	environment.adc = adcRead(adc_in);
	environment.input = inputRead();
	
	return environment;
}

//Get adc Data
struct adcData adcRead(uint32_t *adc_in)
{
	struct adcData adcInfo;
	
	adcInfo.batteryVoltage = (adc_in[2])*0.07632773; //This is the value that relates the buck convertor voltage to battery voltage
	adcInfo.leftBallastPosition = adc_in[0];
	adcInfo.rightBallastPosition = adc_in[1];

	return adcInfo;
	
}

//Get inputs
struct inputData inputRead()
{
	struct inputData inputInfo;
	
	inputInfo.nFaultRight = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11);
	inputInfo.nFaultProp = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_12);
	inputInfo.nFaultLeft = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13);
	
	return inputInfo;
}
