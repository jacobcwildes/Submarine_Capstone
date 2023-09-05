#ifndef __STATE_ESTIMATION_H
#define __STATE_ESTIMATION_H

#include <sys/types.h>
#include <math.h>
#include "perception.h"

struct state stateEstimation(struct envData *environment);

void initialize_states();
void STATE_INIT(struct state *s);
void getRoll(struct state *now, struct envData *env);
void getPitch(struct state *now, struct envData *env);
void getYaw(struct state *now, struct envData *env);
void getX(struct state *now, struct envData *env);
void getY(struct state *now, struct envData *env);
void getZ(struct state *now, struct envData *env);
void getSpeed(struct state *now, struct envData *env);
void getDirection(struct state *now, struct envData *env);
void getDepth(struct state *now, struct envData *env);

struct __attribute__((__packed__)) state {
	uint16_t roll;
	uint16_t pitch;
	uint16_t yaw;
	uint16_t x_lin;
	uint16_t y_lin;
	uint16_t z_lin;
	
	uint16_t roll_Speed;
	uint16_t pitch_Speed;
	uint16_t yaw_Speed;
	uint16_t x_lin_Speed;
	uint16_t y_lin_Speed;
	uint16_t z_lin_Speed;
	
	uint16_t speedScalar;
	uint16_t degreesNorth;
	uint16_t depthApprox;
	
	struct envData env;
};


#endif
