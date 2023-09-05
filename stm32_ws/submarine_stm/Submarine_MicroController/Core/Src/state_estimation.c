#include "state_estimation.h"

struct state delayOne;
struct state delayTwo;




struct state stateEstimation(struct envData *environment)
{
	struct state now;
	
	now.env = *environment;
	
	getRoll(&now, environment);
	getPitch(&now, environment);
	getYaw(&now, environment);
	getX(&now, environment);
	getY(&now, environment);
	getZ(&now, environment);
	getSpeed(&now, environment);
	getDirection(&now, environment);
	getDepth(&now, environment);
	
	delayTwo = delayOne;
	delayOne = now;
	
	return now;
	
}

void initialize_states()
{
	STATE_INIT(&delayOne);
	STATE_INIT(&delayTwo);
}

void STATE_INIT(struct state *s)
{
	struct imuData imu;
	struct adcData adc;
	struct inputData in;
	struct envData env;
	
	imu.x_ang = 0;
	imu.y_ang = 0;
	imu.z_ang = 0;
	imu.x_lin = 0;
	imu.y_lin = 0;
	imu.z_lin = 0;
	imu.x_mag = 0;
	imu.y_mag = 0;
	imu.z_mag = 0;
	env.imu = imu;
	
	adc.batteryVoltage = 16.8;
	adc.leftBallastPosition = 0;
	adc.rightBallastPosition = 0;
	env.adc = adc;
	
	in.nFaultLeft = 0;
	in.nFaultRight = 0;
	in.nFaultProp = 0;
	env.input = in;
	
	s->env = env;
	s->roll = 0;
	s->pitch = 0;
	s->yaw = 0;
	s->x_lin = 0;
	s->y_lin = 0;
	s->z_lin = 0;
	s->roll_Speed = 0;
	s->pitch_Speed = 0;
	s->yaw_Speed = 0;
	s->x_lin_Speed = 0;
	s->y_lin_Speed = 0;
	s->z_lin_Speed = 0;
	s->speedScalar = 0;
	s->degreesNorth = 0;
	s->depthApprox = 0;
	
}
void getRoll(struct state *now, struct envData *env)
{
	now->roll = 0;
	now->roll_Speed = 0;
}
void getPitch(struct state *now, struct envData *env)
{
	now->pitch = 0;
	now->pitch_Speed = 0;
}
void getYaw(struct state *now, struct envData *env)
{
	now->yaw = 0;
	now->yaw_Speed = 0;
}

void getX(struct state *now, struct envData *env)
{
	now->x_lin = 0;
	now->x_lin_Speed = 0;
}

void getY(struct state *now, struct envData *env)
{
	now->y_lin = 0;
	now->y_lin_Speed = 0;
}

void getZ(struct state *now, struct envData *env)
{
	now->z_lin = 0;
	now->z_lin_Speed = 0;
}

void getSpeed(struct state *now, struct envData *env)
{
	now->speedScalar = now->x_lin_Speed;
}

void getDirection(struct state *now, struct envData *env)
{
	now->degreesNorth = (float)(tan((float)env->imu.x_mag/(float)env->imu.y_mag));
}

void getDepth(struct state *now, struct envData *env)
{
	now->depthApprox = 0;
}

