#ifndef __PERCEPTION_H
#define __PERCEPTION_H

#include <sys/types.h>
#include "stm32l4xx_hal.h"

struct envData envRead(I2C_HandleTypeDef com);
struct adcData adcRead();
struct inputData inputRead();
struct imuData imuRead(I2C_HandleTypeDef com);
uint16_t twosComptoDec(uint8_t low_reg, uint8_t high_reg);


struct __attribute__((__packed__)) envData {
  //GLOBAL IMU DOF 
	struct imuData imu;
	struct adcData adc;
	struct inputData input;
};

struct __attribute__((__packed__)) imuData 
{
	uint16_t x_ang;
	uint16_t y_ang;
	uint16_t z_ang;
	uint16_t x_lin;
	uint16_t y_lin;
	uint16_t z_lin;
	uint16_t x_mag;
	uint16_t y_mag;
	uint16_t z_mag;
};

struct __attribute__((__packed__)) adcData 
{
	float batteryVoltage;
	float leftBallastPosition;
	float rightBallastPosition;
};

struct __attribute__((__packed__)) inputData 
{

	uint8_t nFaultLeft;
	uint8_t nFaultRight;
	uint8_t nFaultProp;
};

#endif
