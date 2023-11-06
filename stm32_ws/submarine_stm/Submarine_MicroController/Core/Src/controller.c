#include "controller.h"

#define adc_max_l 0
#define adc_min_l 0
#define adc_max_r 0
#define adc_min_r 0



uint8_t steps[4] = {0b1010, 0b0110, 0b0101, 0b1001};
float depthTarget = 0;
uint8_t step_timer = 0;
uint8_t leftStep = 0;
uint8_t rightStep = 0;



struct actuator_command controller(struct goalCommand com_data, struct state s)
{
	//DO PID controller
	struct actuator_command actuate;
	
	actuate.s = s;
	actuate.c = com_data;
	propellor_control(&actuate, com_data);
	servo_control(&actuate, com_data);
	stepper_control(&actuate, com_data);
	
	return actuate;
}

void propellor_control(struct actuator_command *act, struct goalCommand com)
{
	float leftThrust;
	float rightThrust;
	float forThrust;
	float backThrust;
	
	//Convert coms thrust vals in percentages for f,b,l,r
  if (com.turnThrust < 128)
  {
  	leftThrust = ((float)(-com.turnThrust + 128))/256.0; // 0% <-> 50% 
  	rightThrust = 0;
  }
  else
  {
  	leftThrust = 0;
  	rightThrust = ((float)(com.turnThrust - 128))/256.0; // 0% <-> 50% 
  }
  
  if (com.forwardThrust < 128)
  {
  	forThrust = 0;
		backThrust = ((float)(-com.forwardThrust + 128))/256.0; // 0% <-> 50%
  }
  else
  {
  	forThrust = ((float)(com.forwardThrust - 128))/256.0; // 0% <-> 50%
		backThrust = 0;
  }
  
  //mix individual thrust values into 
  act->rightPropThrust = leftThrust - rightThrust + forThrust - backThrust; // -100% <-> 100%
  act->leftPropThrust = rightThrust - leftThrust + forThrust - backThrust; // -100% <-> 100%
}

void servo_control(struct actuator_command *act, struct goalCommand com)
{
	//Setup for 0.5ms <-> 2.5ms
  act->camVerticalDuty = (0.000392 * (float)com.camUpDown) + 0.025;
  act->camHorizontalDuty = (0.000392 * (float)com.camLeftRight) + 0.025;
}

void stepper_control(struct actuator_command *act, struct goalCommand com)
{
	//Update stepper timer. This is set so that it cant change tooooo much
	step_timer++;
	
	//Need to get target depth first (ALWAYS KEEP TRACK OF THIS)
	if (com.depthUp) depthTarget += 0.25;
	else if (com.depthDown) depthTarget -=0.25;
		
	if (step_timer >= 10) {
		step_timer = 0;
	
		uint8_t currentDepth = act->s.depthApprox;
		
		if (currentDepth < depthTarget) bouyancyUp(*act);
		else if (currentDepth > depthTarget) bouyancyDown(*act);
		
		//check roll
		uint8_t currentRoll = act->s.roll;
		
		if (currentRoll < 0) rotateCCW(*act);
		else if (currentRoll > 0) rotateCW(*act);
		
		struct stepper_instruction left;
		struct stepper_instruction right;
		
		left.a_one = steps[leftStep] & 0b1000;
		left.a_two = steps[leftStep] & 0b0100;
		left.b_one = steps[leftStep] & 0b0010;
		left.b_two = steps[leftStep] & 0b0001;
		
		right.a_one = steps[rightStep] & 0b1000;
		right.a_two = steps[rightStep] & 0b0100;
		right.b_one = steps[rightStep] & 0b0010;
		right.b_two = steps[rightStep] & 0b0001;
		
		act->left_stepper = left;
		act->right_stepper = right;
	}
	
	
}

void bouyancyUp(struct actuator_command act){
	//This needs to check the upwards speed. If positive, do NOTHING
	//if negative or zero, move CW
	
	if (act.s.env.adc.leftBallastPosition < adc_max_l && act.s.env.adc.rightBallastPosition < adc_max_r && act.s.upwardSpeed <= 0){
		leftStep++;
		rightStep++;
	}
}

void bouyancyDown(struct actuator_command act){
	//This needs to check the upwards speed. If negative, do NOTHING
	//if negative or zero, move CW
	
	if (act.s.env.adc.leftBallastPosition > adc_min_l && act.s.env.adc.rightBallastPosition > adc_min_r && act.s.upwardSpeed >= 0){
		leftStep--;
		rightStep--;
	}
}

void rotateCCW(struct actuator_command act){
	//This needs to check the CCW roll. If positive, do NOTHING
	//if negative or zero, move CCW
	
	if (act.s.env.adc.leftBallastPosition > adc_min_l && act.s.env.adc.rightBallastPosition < adc_max_r && act.s.rollSpeed <= 0){
		leftStep--;
		rightStep++;
	}
}

void rotateCW(struct actuator_command act){
	//This needs to check the CW roll. If positive, do NOTHING
	//if negative or zero, move CW
	if (act.s.env.adc.leftBallastPosition < adc_max_l && act.s.env.adc.rightBallastPosition > adc_min_r && act.s.rollSpeed >= 0){
		leftStep++;
		rightStep--;
	}
}

