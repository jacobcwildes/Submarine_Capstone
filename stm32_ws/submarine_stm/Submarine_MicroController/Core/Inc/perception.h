#ifndef __PERCEPTION_H
#define __PERCEPTION_H

HAL_StatusTypeDef imuRead(void);
void imuPullData(void);
uint16_t twosComptoDec(uint8_t low_reg, uint8_t high_reg);

#endif
