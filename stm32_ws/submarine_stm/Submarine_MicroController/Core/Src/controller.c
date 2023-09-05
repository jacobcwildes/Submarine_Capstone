#include "controller.h"

struct actuator_command controller(struct goalCommand com_data, struct state s)
{
	//DO PID controller
	struct actuator_command actuate;
	
	actuate.s = s;
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
	struct stepper_instruction left;
	struct stepper_instruction right;
	
	left.a_one = 0;
	left.a_two = 0;
	left.b_one = 0;
	left.b_two = 0;
	
	right.a_one = 0;
	right.a_two = 0;
	right.b_one = 0;
	right.b_two = 0;
	
	act->left_stepper = left;
	act->right_stepper = right;
}
