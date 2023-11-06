#include "state_estimation.h"
#include <math.h>

struct state delayOne;


struct state stateEstimation(struct envData *environment)
{
	struct state now;
	
	now.env = *environment;
	
	//double integrate everything needed
	now.w_roll = now.env.imu.x_ang + delayOne.w_roll;
	now.w_pitch = now.env.imu.y_ang + delayOne.w_pitch;
	now.w_yaw = now.env.imu.z_ang + delayOne.w_yaw;
	now.w_z = now.env.imu.z_lin + delayOne.w_z;
	now.w_for = now.env.imu.x_lin + delayOne.w_for;
	
	now.rollSpeed = (now.w_roll + delayOne.w_roll)/2;
	uint16_t s_pitch = (now.w_pitch + delayOne.w_pitch)/2;
	uint16_t s_yaw = (now.w_yaw + delayOne.w_yaw)/2;
	now.upwardSpeed = (now.w_z + delayOne.w_z)/2;
	now.speedScalar = (now.w_roll + delayOne.w_roll)/2; //SPEED SCALAR
	
	now.v_roll = now.rollSpeed + delayOne.w_roll;
	now.v_pitch = s_pitch + delayOne.w_pitch;
	now.v_yaw = s_yaw + delayOne.w_yaw;
	now.v_z = now.upwardSpeed + delayOne.w_z;
	now.v_for = now.speedScalar + delayOne.w_for;
	
	now.roll = (now.v_roll + delayOne.v_roll)/2;
	now.pitch = (now.v_pitch + delayOne.v_pitch)/2;
	now.yaw = (now.v_yaw + delayOne.v_yaw)/2;
	now.depthApprox = (now.v_z + delayOne.v_z)/2;
	uint16_t forward = (now.v_for + delayOne.v_for)/2;
	
	now.degreesNorth = atan(now.env.imu.y_mag/now.env.imu.x_mag);
	
	now.x_lin = forward*cos(now.degreesNorth) + delayOne.x_lin;
	now.y_lin = forward*sin(now.degreesNorth) + delayOne.y_lin;
	
	delayOne = now;
	
	return now;
	
}

void initialize_states()
{
	STATE_INIT(&delayOne);
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
	s->w_roll = 0;
	s->w_pitch = 0;
	s->w_yaw = 0;
	s->w_z = 0;
	s->w_for = 0;
	s->v_roll = 0;
	s->v_pitch = 0;
	s->v_yaw = 0;
	s->v_z = 0;
	s->v_for = 0;
	
	s->speedScalar = 0;
	s->degreesNorth = 0;
	s->depthApprox = 0;
	s->upwardSpeed = 0;
	s->rollSpeed = 0;
	
}

