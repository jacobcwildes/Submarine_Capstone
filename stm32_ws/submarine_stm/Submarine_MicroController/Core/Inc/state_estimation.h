#ifndef __STATE_ESTIMATION_H
#define __STATE_ESTIMATION_H

#include <sys/types.h>
#include <math.h>
#include "perception.h"

struct state stateEstimation(struct envData *environment);

void initialize_states();
void STATE_INIT(struct state *s);
void double_integrate();

struct __attribute__((__packed__)) state {
	uint16_t roll;
	uint16_t pitch;
	uint16_t yaw;
	uint16_t x_lin;
	uint16_t y_lin;
	
	uint16_t w_roll;
	uint16_t w_pitch;
	uint16_t w_yaw;
	uint16_t w_z;
	uint16_t w_for;
	
	uint16_t v_roll;
	uint16_t v_pitch;
	uint16_t v_yaw;
	uint16_t v_z;
	uint16_t v_for;
	
	uint16_t speedScalar;
	uint16_t degreesNorth;
	uint16_t depthApprox;
	
	struct envData env;
};


#endif
