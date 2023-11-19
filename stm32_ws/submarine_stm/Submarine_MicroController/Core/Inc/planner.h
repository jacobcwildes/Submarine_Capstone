#ifndef __PLANNER_H
#define __PLANNER_H

#include <sys/types.h>
#include <math.h>
#include "stm32l4xx_hal.h"

uint8_t binaryToDecimal(int start_index, int bitCount, uint8_t *data);
struct goalCommand parseComs(uint8_t *data, volatile uint8_t received);



struct __attribute__((__packed__)) goalCommand {
	uint8_t depthUp;
	uint8_t depthDown;
	uint8_t captureImage;
	uint8_t forwardThrust;
	uint8_t turnThrust;
	uint8_t camUpDown;
	uint8_t camLeftRight;
};
#endif
