#include "perception.h"

//GLOBAL I2C Addresses
#define ACCEL_GYRO_ADR 0x6a
#define MAG_ADR 0x1c

//gyro
#define OUTX_L_G (0x22)
#define OUTX_H_G (0x23)
#define OUTY_L_G (0x24)
#define OUTY_H_G (0x25)
#define OUTZ_L_G (0x26)
#define OUTZ_H_G (0x27)
//accel
#define OUTX_L_XL (0x28)
#define OUTX_H_XL (0x29)
#define OUTY_L_XL (0x2A)
#define OUTY_H_XL (0x2B)
#define OUTZ_L_XL (0x2C)
#define OUTZ_H_XL (0x2D)
//magentometer
#define OUT_X_L (0x28)
#define OUT_X_H (0x29)
#define OUT_Y_L (0x2A)
#define OUT_Y_H (0x2B)
#define OUT_Z_L (0x2C)
#define OUT_Z_H (0x2D)



struct envData envRead(I2C_HandleTypeDef com)
{
	struct envData environment;
	
	environment.imu = imuRead(com);
	environment.adc = adcRead();
	environment.input = inputRead();
	
	return environment;
}


struct adcData adcRead()
{
	struct adcData adcInfo;
	/*
	float batteryVoltage;
	uint8_t nFaultLeft;
	uint8_t nFaultRight;
	uint8_t nFaultProp;
	float leftBallastPosition;
	float rightBallastPosition;
	*/
	
	adcInfo.batteryVoltage = 16.9;
	adcInfo.leftBallastPosition = 0.5;
	adcInfo.rightBallastPosition = 0.5;

	return adcInfo;
	
}


struct inputData inputRead()
{
	struct inputData inputInfo;
	
	inputInfo.nFaultLeft = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11);
	inputInfo.nFaultRight = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_12);
	inputInfo.nFaultProp = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13);
	
	return inputInfo;
}

struct imuData imuRead(I2C_HandleTypeDef com)
{
	/*
		HAL_StatusTypeDef HAL_I2C_Mem_Read	(	I2C_HandleTypeDef * 	hi2c,
																					uint16_t 	DevAddress,
																					uint16_t 	MemAddress,
																					uint16_t 	MemAddSize,
																					uint8_t * 	pData,
																					uint16_t 	Size,
																					uint32_t 	Timeout )		
	*/
	
	//GLOBAL I2C pData registers
	uint8_t x_ang_L;
	uint8_t x_ang_H;
	uint8_t y_ang_L;
	uint8_t y_ang_H;
	uint8_t z_ang_L;
	uint8_t z_ang_H;

	uint8_t x_lin_L;
	uint8_t x_lin_H;
	uint8_t y_lin_L;
	uint8_t y_lin_H;
	uint8_t z_lin_L;
	uint8_t z_lin_H;

	uint8_t x_mag_L;
	uint8_t x_mag_H;
	uint8_t y_mag_L;
	uint8_t y_mag_H;
	uint8_t z_mag_L;
	uint8_t z_mag_H;
	
	struct imuData imuVectors;
	
	HAL_StatusTypeDef retVal = HAL_OK;
	
	//Gyro
	if (HAL_I2C_Mem_Read(&com, ACCEL_GYRO_ADR << 1, OUTX_L_G << 1, 1, &x_ang_L, sizeof(x_ang_L), 10) != HAL_OK) retVal = HAL_ERROR;
	if (HAL_I2C_Mem_Read(&com, ACCEL_GYRO_ADR << 1, OUTX_H_G << 1, 1, &x_ang_H, sizeof(x_ang_H), 10) != HAL_OK) retVal = HAL_ERROR;
	if (HAL_I2C_Mem_Read(&com, ACCEL_GYRO_ADR << 1, OUTY_L_G << 1, 1, &y_ang_L, sizeof(y_ang_L), 10) != HAL_OK) retVal = HAL_ERROR;
	if (HAL_I2C_Mem_Read(&com, ACCEL_GYRO_ADR << 1, OUTY_H_G << 1, 1, &y_ang_H, sizeof(y_ang_H), 10) != HAL_OK) retVal = HAL_ERROR;
	if (HAL_I2C_Mem_Read(&com, ACCEL_GYRO_ADR << 1, OUTZ_L_G << 1, 1, &z_ang_L, sizeof(z_ang_L), 10) != HAL_OK) retVal = HAL_ERROR;
	if (HAL_I2C_Mem_Read(&com, ACCEL_GYRO_ADR << 1, OUTZ_H_G << 1, 1, &z_ang_H, sizeof(z_ang_H), 10) != HAL_OK) retVal = HAL_ERROR;
	//Accel
	if (HAL_I2C_Mem_Read(&com, ACCEL_GYRO_ADR << 1, OUTX_L_XL << 1, 1, &x_lin_L, sizeof(x_lin_L), 10) != HAL_OK) retVal = HAL_ERROR;
	if (HAL_I2C_Mem_Read(&com, ACCEL_GYRO_ADR << 1, OUTX_H_XL << 1, 1, &x_lin_H, sizeof(x_lin_H), 10) != HAL_OK) retVal = HAL_ERROR;
	if (HAL_I2C_Mem_Read(&com, ACCEL_GYRO_ADR << 1, OUTY_L_XL << 1, 1, &y_lin_L, sizeof(y_lin_L), 10) != HAL_OK) retVal = HAL_ERROR;
	if (HAL_I2C_Mem_Read(&com, ACCEL_GYRO_ADR << 1, OUTY_H_XL << 1, 1, &y_lin_H, sizeof(y_lin_H), 10) != HAL_OK) retVal = HAL_ERROR;
	if (HAL_I2C_Mem_Read(&com, ACCEL_GYRO_ADR << 1, OUTZ_L_XL << 1, 1, &z_lin_L, sizeof(z_lin_L), 10) != HAL_OK) retVal = HAL_ERROR;
	if (HAL_I2C_Mem_Read(&com, ACCEL_GYRO_ADR << 1, OUTZ_H_XL << 1, 1, &z_lin_H, sizeof(z_lin_H), 10) != HAL_OK) retVal = HAL_ERROR;
	//Magnet
	if (HAL_I2C_Mem_Read(&com, MAG_ADR << 1, OUT_X_L << 1, 1, &x_mag_L, sizeof(x_mag_L), 10) != HAL_OK) retVal = HAL_ERROR;
	if (HAL_I2C_Mem_Read(&com, MAG_ADR << 1, OUT_X_H << 1, 1, &x_mag_H, sizeof(x_mag_H), 10) != HAL_OK) retVal = HAL_ERROR;
	if (HAL_I2C_Mem_Read(&com, MAG_ADR << 1, OUT_Y_L << 1, 1, &y_mag_L, sizeof(y_mag_L), 10) != HAL_OK) retVal = HAL_ERROR;
	if (HAL_I2C_Mem_Read(&com, MAG_ADR << 1, OUT_Y_H << 1, 1, &y_mag_H, sizeof(y_mag_H), 10) != HAL_OK) retVal = HAL_ERROR;
	if (HAL_I2C_Mem_Read(&com, MAG_ADR << 1, OUT_Z_L << 1, 1, &z_mag_L, sizeof(z_mag_L), 10) != HAL_OK) retVal = HAL_ERROR;
	if (HAL_I2C_Mem_Read(&com, MAG_ADR << 1, OUT_Z_H << 1, 1, &z_mag_H, sizeof(z_mag_H), 10) != HAL_OK) retVal = HAL_ERROR;
	
	if (retVal == HAL_OK) //If reading without error
	{
		//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); //toggle LED for dev	
		imuVectors.x_ang = twosComptoDec(x_ang_L, x_ang_H);//get xyz values for all 9DOF
		imuVectors.y_ang = twosComptoDec(y_ang_L, y_ang_H);//will need to do further calc for position, rotation, compass
		imuVectors.z_ang = twosComptoDec(z_ang_L, z_ang_H);

		imuVectors.x_lin = twosComptoDec(x_lin_L, x_lin_H);
		imuVectors.y_lin = twosComptoDec(y_lin_L, y_lin_H);
		imuVectors.z_lin = twosComptoDec(z_lin_L, z_lin_H);

		imuVectors.x_mag = twosComptoDec(x_mag_L, x_mag_H);
		imuVectors.y_mag = twosComptoDec(y_mag_L, y_mag_H);
		imuVectors.z_mag = twosComptoDec(z_mag_L, z_mag_H);
	}
	else
	{
		imuVectors.x_ang = -1;//get xyz values for all 9DOF
		imuVectors.y_ang = -1;//will need to do further calc for position, rotation, compass
		imuVectors.z_ang = -1;

		imuVectors.x_lin = -1;
		imuVectors.y_lin = -1;
		imuVectors.z_lin = -1;

		imuVectors.x_mag = -1;
		imuVectors.y_mag = -1;
		imuVectors.z_mag = -1;
	}
	
	return imuVectors;
}

uint16_t twosComptoDec(uint8_t low_reg, uint8_t high_reg)
{
	uint16_t combined = ((uint16_t)high_reg << 8) | low_reg;
	if ((combined & 0x8000) == 0) return combined; //negative if top bit is 1
	else return -(~combined + 1);
}
