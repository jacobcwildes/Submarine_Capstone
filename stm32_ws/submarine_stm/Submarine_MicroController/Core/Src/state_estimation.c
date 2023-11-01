#include "state_estimation.h"

struct state delayOne;


struct state stateEstimation(struct envData *environment)
{
	struct state now;
	
	now.env = *environment;
	
	//double integrate everything needed
	now.w_roll = now.env->x_ang + delayOne.w_roll;
	now.w_pitch = now.env->y_ang + delayOne.w_pitch;
	now.w_yaw = now.env->z_ang + delayOne.w_yaw;
	now.w_z = now.env->z_lin + delayOne.w_z;
	now.w_for = now.env->x_lin + delayOne.w_for;
	
	now.rollSpeed = (now.w_roll + delayOne.w_roll)/2;
	uint16_t s_pitch = (now.w_pitch + delayOne.w_pitch)/2;
	uint16_t s_yaw = (now.w_yaw + delayOne.w_yaw)/2;
	now.upwardSpeed = (now.w_z + delayOne.w_z)/2;
	now.speedScalar = (now.w_roll + delayOne.w_roll)/2; //SPEED SCALAR
	
	now.v_roll = rollSpeed + delayOne.w_roll;
	now.v_pitch = s_pitch + delayOne.w_pitch;
	now.v_yaw = s_yaw + delayOne.w_yaw;
	now.v_z = upwardSpeed + delayOne.w_z;
	now.v_for = now.speedScalar + delayOne.w_for;
	
	now.roll = (now.v_roll + delayOne.v_roll)/2;
	now.pitch = (now.v_pitch + delayOne.v_pitch)/2;
	now.yaw = (now.v_yaw + delayOne.v_yaw)/2;
	now.depthApprox = (now.v_z + delayOne.v_z)/2;
	uint16_t forward = (now.v_for + delayOne.v_for)/2;
	
	now.degreesNorth = math.atan(now.env->y_mag/now.env->x_mag);
	
	now.x_lin = forward*math.cos(now.degreesNorth) + delayOne.x_lin;
	now.y_lin = forward*math.sin(now.degreesNorth) + delayOne.y_lin;
	
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

