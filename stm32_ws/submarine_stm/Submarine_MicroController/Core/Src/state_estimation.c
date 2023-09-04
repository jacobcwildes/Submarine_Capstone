#include "state_estimation.h"

struct state lastState;

struct state stateEstimation(struct envData *environment)
{
	struct state *now;
	
	now->env = *environment;
	
	getRoll(now, lastState, environment);
	getPitch(now, lastState, environment);
	getYaw(now, lastState, environment);
	getX(now, lastState, environment);
	getY(now, lastState, environment);
	getZ(now, lastState, environment);
	getSpeed(now, lastState, environment);
	getDirection(now, lastState, environment);
	
	lastState = *now;
	
	return *now;
	
}

void getRoll(struct state *now, struct state last, struct envData *env)
{
	now->roll = 0;
	now->roll_Speed = 0;
}
void getPitch(struct state *now, struct state last, struct envData *env)
{
	now->pitch = 0;
	now->pitch_Speed = 0;
}
void getYaw(struct state *now, struct state last, struct envData *env)
{
	now->yaw = 0;
	now->yaw_Speed = 0;
}

void getX(struct state *now, struct state last, struct envData *env)
{
	now->x_lin = 0;
	now->x_lin_Speed = 0;
}

void getY(struct state *now, struct state last, struct envData *env)
{
	now->y_lin = 0;
	now->y_lin_Speed = 0;
}

void getZ(struct state *now, struct state last, struct envData *env)
{
	now->z_lin = 0;
	now->z_lin_Speed = 0;
}

void getSpeed(struct state *now, struct state last, struct envData *env)
{
	now->speedScalar = now->x_lin_Speed;
}

void getDirection(struct state *now, struct state last, struct envData *env)
{
	now->degreesNorth = (float)(tan((float)env->x_mag/(float)env->y_mag))

}

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
	
	struct envData env;
};
