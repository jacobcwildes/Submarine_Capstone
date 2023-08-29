#include "perception.h"

HAL_StatusTypeDef imuRead(void)
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
	HAL_StatusTypeDef retVal = HAL_OK;
	
	//Gyro
	if (HAL_I2C_Mem_Read(&hi2c1, ACCEL_GYRO_ADR, OUTX_L_G, 1, &x_ang_L, sizeof(x_ang_L), 10) != HAL_OK) retVal = HAL_ERROR;
	if (HAL_I2C_Mem_Read(&hi2c1, ACCEL_GYRO_ADR, OUTX_H_G, 1, &x_ang_H, sizeof(x_ang_H), 10) != HAL_OK) retVal = HAL_ERROR;
	if (HAL_I2C_Mem_Read(&hi2c1, ACCEL_GYRO_ADR, OUTY_L_G, 1, &y_ang_L, sizeof(y_ang_L), 10) != HAL_OK) retVal = HAL_ERROR;
	if (HAL_I2C_Mem_Read(&hi2c1, ACCEL_GYRO_ADR, OUTY_H_G, 1, &y_ang_H, sizeof(y_ang_H), 10) != HAL_OK) retVal = HAL_ERROR;
	if (HAL_I2C_Mem_Read(&hi2c1, ACCEL_GYRO_ADR, OUTZ_L_G, 1, &z_ang_L, sizeof(z_ang_L), 10) != HAL_OK) retVal = HAL_ERROR;
	if (HAL_I2C_Mem_Read(&hi2c1, ACCEL_GYRO_ADR, OUTZ_H_G, 1, &z_ang_H, sizeof(z_ang_H), 10) != HAL_OK) retVal = HAL_ERROR;
	//Accel
	if (HAL_I2C_Mem_Read(&hi2c1, ACCEL_GYRO_ADR, OUTX_L_XL, 1, &x_lin_L, sizeof(x_lin_L), 10) != HAL_OK) retVal = HAL_ERROR;
	if (HAL_I2C_Mem_Read(&hi2c1, ACCEL_GYRO_ADR, OUTX_H_XL, 1, &x_lin_H, sizeof(x_lin_H), 10) != HAL_OK) retVal = HAL_ERROR;
	if (HAL_I2C_Mem_Read(&hi2c1, ACCEL_GYRO_ADR, OUTY_L_XL, 1, &y_lin_L, sizeof(y_lin_L), 10) != HAL_OK) retVal = HAL_ERROR;
	if (HAL_I2C_Mem_Read(&hi2c1, ACCEL_GYRO_ADR, OUTY_H_XL, 1, &y_lin_H, sizeof(y_lin_H), 10) != HAL_OK) retVal = HAL_ERROR;
	if (HAL_I2C_Mem_Read(&hi2c1, ACCEL_GYRO_ADR, OUTZ_L_XL, 1, &z_lin_L, sizeof(z_lin_L), 10) != HAL_OK) retVal = HAL_ERROR;
	if (HAL_I2C_Mem_Read(&hi2c1, ACCEL_GYRO_ADR, OUTZ_H_XL, 1, &z_lin_H, sizeof(z_lin_H), 10) != HAL_OK) retVal = HAL_ERROR;
	//Magnet
	if (HAL_I2C_Mem_Read(&hi2c1, MAG_ADR, OUT_X_L, 1, &x_mag_L, sizeof(x_mag_L), 10) != HAL_OK) retVal = HAL_ERROR;
	if (HAL_I2C_Mem_Read(&hi2c1, MAG_ADR, OUT_X_H, 1, &x_mag_H, sizeof(x_mag_H), 10) != HAL_OK) retVal = HAL_ERROR;
	if (HAL_I2C_Mem_Read(&hi2c1, MAG_ADR, OUT_Y_L, 1, &y_mag_L, sizeof(y_mag_L), 10) != HAL_OK) retVal = HAL_ERROR;
	if (HAL_I2C_Mem_Read(&hi2c1, MAG_ADR, OUT_Y_H, 1, &y_mag_H, sizeof(y_mag_H), 10) != HAL_OK) retVal = HAL_ERROR;
	if (HAL_I2C_Mem_Read(&hi2c1, MAG_ADR, OUT_Z_L, 1, &z_mag_L, sizeof(z_mag_L), 10) != HAL_OK) retVal = HAL_ERROR;
	if (HAL_I2C_Mem_Read(&hi2c1, MAG_ADR, OUT_Z_H, 1, &z_mag_H, sizeof(z_mag_H), 10) != HAL_OK) retVal = HAL_ERROR;
	
	
	return retVal;
}

void imuPullData(void)
{
  //Begin reading IMU data
	if (imuRead() == HAL_OK) //If reading without error
	{
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); //toggle LED for dev	
		x_ang = twosComptoDec(x_ang_L, x_ang_H);//get xyz values for all 9DOF
		y_ang = twosComptoDec(y_ang_L, y_ang_H);//will need to do further calc for position, rotation, compass
		z_ang = twosComptoDec(z_ang_L, z_ang_H);

		x_lin = twosComptoDec(x_lin_L, x_lin_H);
		y_lin = twosComptoDec(y_lin_L, y_lin_H);
		z_lin = twosComptoDec(z_lin_L, z_lin_H);

		x_mag = twosComptoDec(x_mag_L, x_mag_H);
		y_mag = twosComptoDec(y_mag_L, y_mag_H);
		z_mag = twosComptoDec(z_mag_L, z_mag_H);
	}
}

uint16_t twosComptoDec(uint8_t low_reg, uint8_t high_reg)
{
	uint16_t combined = ((uint16_t)high_reg << 8) | low_reg;
	if ((combined & 0x8000) == 0) return combined; //negative if top bit is 1
	else return -(~combined + 1);
}
