#include "controller.h"
#define ADC_RECORD_COUNT 10
#define WAIT_COUNT 2

/*
Dyllon Dunton
12/11/2023

This file uses the sensor data and the command from the controller to set values for
driving the motors
*/

//Step values for motors
uint8_t steps[4] = {0b1010, 0b0110, 0b0101, 0b1001};

//current step that the motors are on
int current_left = 0;
int current_right = 0;

//Timer value for the 
int counter = 0;


struct actuator_command controller(struct goalCommand com_data, struct envData in)
{
	//DO PID controller
	struct actuator_command actuate;
	
	actuate.in = in;
	actuate.com = com_data;
	propellor_control(&actuate);
	servo_control(&actuate);
	stepper_control(&actuate);
	
	return actuate;
}

void propellor_control(struct actuator_command *act)
{
	float leftThrust;
	float rightThrust;
	float forThrust;
	float backThrust;
	
	//Convert coms thrust vals in percentages for f,b,l,r
  if (act->com.turnThrust < 128) /
  {
  	leftThrust = ((float)(-act->com.turnThrust + 128))/256.0; // 0% <. 50% 
  	rightThrust = 0;
  }
  else
  {
  	leftThrust = 0;
  	rightThrust = ((float)(act->com.turnThrust - 128))/256.0; // 0% <. 50% 
  }
  
  if (act->com.forwardThrust < 128)
  {
  	forThrust = 0;
		backThrust = ((float)(-act->com.forwardThrust + 128))/256.0; // 0% <. 50%
  }
  else
  {
  	forThrust = ((float)(act->com.forwardThrust - 128))/256.0; // 0% <. 50%
		backThrust = 0;
  }
  
  //mix individual thrust values into 
  act->rightPropThrust = leftThrust - rightThrust + forThrust - backThrust; // -100% <. 100%
  act->leftPropThrust = rightThrust - leftThrust + forThrust - backThrust; // -100% <. 100%
}

void servo_control(struct actuator_command *act)
{
	//Setup for 0.5ms <. 2.5ms
  act->camVerticalDuty = (0.000392 * (float)act->com.camUpDown) + 0.025;
  act->camHorizontalDuty = (0.000392 * (float)act->com.camLeftRight) + 0.025;
}

void stepper_control(struct actuator_command *act)
{
	counter++;
	if (counter >= WAIT_COUNT)
	{
		counter = 0;
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); //toggle LED for dev
		
		//take in the roll value from the submarine
		int roll = ((int)(act->com.roll)) - 90; // -90 <. 90
		int cw = 0;
		
		//calc clockwise steps to be proportional to the roll with a max of 30  
		if (roll > 0) cw = fmin(roll, 30);
		else cw = fmax(roll, -30);
		
		//Split into left and right
		int left = cw; 
		int right = -cw;
		
		//add on depth
		if (act->com.depthUp) 
		{
			left += 15;
			right += 15;
		}
		else if (act->com.depthDown)
		{
			left -= 15;
			right -= 15;
		}
		
		//turn to adc value
		//convert [-45,45] to [30, 225]
		int left_adc = (196*left)/90 + 128;
		int right_adc = (196*right)/90 + 128;
		
		//turn into step values
		if (left_adc < act->in.adc.leftBallastPosition)
		{	
			//left down
			current_left--;
			if (current_left == -1) current_left = 3;
		}
		
		else if (left_adc > act->in.adc.leftBallastPosition)
		{	
			//left up
			current_left++;
			if (current_left == 4) current_left = 0;
		}
		
		if (right_adc < act->in.adc.rightBallastPosition)
		{	
			//right down
			current_right--;
			if (current_right == -1) current_right = 3;
		}
		else if (right_adc > act->in.adc.rightBallastPosition)
		{	
			//right up
			current_right++;
			if (current_right == 4) current_right = 0;
		}
	}
	
	//create structs to hold current steps to send to the controller
	act->left_current = current_left;
	act->right_current = current_right;
	
	//Create structs to hold step values to send to the drivers
	struct stepper_instruction leftStep;
	struct stepper_instruction rightStep;
	
	//Parse out the steps for each lead of each motor
	leftStep.a_one = steps[current_left] & 0b1000;
	leftStep.a_two = steps[current_left] & 0b0100;
	leftStep.b_one = steps[current_left] & 0b0010;
	leftStep.b_two = steps[current_left] & 0b0001;
	
	rightStep.a_one = steps[current_right] & 0b1000;
	rightStep.a_two = steps[current_right] & 0b0100;
	rightStep.b_one = steps[current_right] & 0b0010;
	rightStep.b_two = steps[current_right] & 0b0001;
	
	act->left_stepper = leftStep;
	act->right_stepper = rightStep;
	
}
