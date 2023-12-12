#ifndef __PERCEPTION_H
#define __PERCEPTION_H

#include <sys/types.h>
#include "stm32l4xx_hal.h"

struct envData envRead(uint32_t *adc_in);
struct adcData adcRead(uint32_t *adc_in);
struct inputData inputRead();

//Store ADC Data
struct __attribute__((__packed__)) adcData 
{
	float batteryVoltage;
	float leftBallastPosition;
	float rightBallastPosition;
};

//Store fault data from motor drivers
struct __attribute__((__packed__)) inputData 
{
	uint8_t nFaultLeft;
	uint8_t nFaultRight;
	uint8_t nFaultProp;
};

//make one struct with all sensor data
struct __attribute__((__packed__)) envData {
  //GLOBAL IMU DOF 
	struct adcData adc;
	struct inputData input;
};

#endif
