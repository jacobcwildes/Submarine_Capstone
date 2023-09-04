#ifndef __ACTUATOR_H
#define __ACTUATOR_H


#include <sys/types.h>
#include <stdio.h>
#include "stm32l4xx_hal.h"


void MTR_DRV_INIT(uint8_t currentValue, uint8_t decay, uint8_t reset, uint8_t sleep);

void updateActuators(UART_HandleTypeDef uart_com, struct actuator_command actuate);
void updateProps(struct actuator_command actuate);
void updateServos(struct actuator_command actuate);
void updateSteppers(struct actuator_command actuate);
void transmitData(UART_HandleTypeDef uart_com, struct actuator_command actuate);

#endif
