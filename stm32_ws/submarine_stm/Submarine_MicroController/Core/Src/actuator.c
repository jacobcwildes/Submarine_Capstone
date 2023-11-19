#include "actuator.h"

#define ARR_Prop 11999
#define ARR_Servo 2399999


#define full_close_steps 100


void updateActuators(UART_HandleTypeDef uart_com, struct actuator_command actuate)
{
	updateProps(actuate);
	updateServos(actuate);
	updateSteppers(actuate);
	transmitData(uart_com, actuate);
}

void MTR_DRV_INIT(uint8_t currentValue, uint8_t decay, uint8_t reset, uint8_t sleep)
{
  //Current Set
  /*
    xI1 xI0 current
    0b00 -> 0x0 -> 100%
    0b01 -> 0x1 -> 71%
    0b10 -> 0x2 -> 38%
    0b11 -> 0x3 -> 0%
    
    AI1 -> PF13
    AI0 -> PF14
    BI1 -> PF15
    BI0 -> PG0
    
  */
  //xI1
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, (currentValue>>1) & ~0x01);
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, (currentValue>>1) & ~0x01);
  //xI0
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, (currentValue) & ~0x01);
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, (currentValue) & ~0x01);
  
  //Decay
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, decay);
  
  //Reset
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, reset);
  
  //Sleep
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, sleep);
  
  //Left Prop DC
  TIM3->CCR2 = 0;
  TIM3->CCR3 = 0;
  
  //Right Prop DC
  TIM4->CCR2 = 0;
  TIM4->CCR3 = 0;
  
  //Servos DC
  TIM5->CCR2 = 0.5 * ARR_Servo;
  TIM5->CCR3 = 0.5 * ARR_Servo;
  
  
}


void updateProps(struct actuator_command actuate)
{
  //LEFT PROP SET DC (TIM3_CH2 = forward) (TIM3_CH3 = backward)
  if (actuate.leftPropThrust >= 0) 
  {
    TIM3->CCR2 = actuate.leftPropThrust*ARR_Prop/3;
    TIM3->CCR3 = 0;
  }
  else 
  {
    TIM3->CCR3 = -actuate.leftPropThrust*ARR_Prop/3;
    TIM3->CCR2 = 0;
  }
  
  //RIGHT PROP SET DC (TIM4_CH2 = forward) (TIM4_CH3 = backward)
  if (actuate.rightPropThrust >= 0) 
  {
    TIM4->CCR2 = actuate.rightPropThrust*ARR_Prop/3;
    TIM4->CCR3 = 0;
  }
  else
  {
    TIM4->CCR3 = -actuate.rightPropThrust*ARR_Prop/3;
    TIM4->CCR2 = 0;
  }
  
}

void updateServos(struct actuator_command actuate)
{
  //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); //toggle LED for dev 
  //Convert coms thrust vals in percentages for f,b,l,r
  
  //Setup for 1ms <-> 2ms
  //camVerticalDuty = (0.000196 * (float)camUpDown) + 0.05;
  //camHorizontalDuty = (0.000196 * (float)camLeftRight) + 0.05;
  
  
  
  //Set Duty Cycles
  TIM5->CCR2 = actuate.camVerticalDuty*ARR_Servo;
  TIM5->CCR3 = actuate.camHorizontalDuty*ARR_Servo;
}


void updateSteppers(struct actuator_command actuate)
{
	//DO STEPPERS
	/*
	//LEFT
	AIN1 -> PA4
	AIN2 -> PA3
	BIN1 -> PA5
	BIN2 -> PA6
	
	//RIGHT
	AIN1 -> PC4
	AIN2 -> PA7
	BIN1 -> PC5
	BIN2 -> PB0
	*/
	
	//Left
	if (actuate.left_stepper.a_one) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	if (actuate.left_stepper.a_two) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
	if (actuate.left_stepper.b_one) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	if (actuate.left_stepper.b_two) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
	
	//Right
	if (actuate.left_stepper.a_one) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
	if (actuate.left_stepper.a_two) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
	if (actuate.left_stepper.b_one) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
	if (actuate.left_stepper.b_two) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	
}



void transmitData(UART_HandleTypeDef uart_com, struct actuator_command actuate)
{
	//Data concat
	
	struct state stateData = actuate.s;
	struct envData environmentData = stateData.env;
	struct adcData analogData = environmentData.adc;
	struct inputData in = environmentData.input;
	struct imuData imu = environmentData.imu;
	struct goalCommand command = actuate.c;
	struct stepper_instruction left_s = actuate.left_stepper;
	struct stepper_instruction right_s = actuate.right_stepper;
	
	char tx_buffer[500] = "";
	
	
	//tHIS IS a test print that prints EVERYTHING I WANT TO KNOW
	/*
	sprintf(tx_buffer,
		"%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n\r", 
		(int)(analogData.leftBallastPosition * 10), //adc
		(int)(analogData.rightBallastPosition * 10), 
		(int)(analogData.batteryVoltage * 10), 
		in.nFaultLeft,  //faults
		in.nFaultRight, 
		in.nFaultProp, 
		imu.x_ang,  //imuacceleration
		imu.y_ang, 
		imu.z_ang, 
		imu.x_lin, 
		imu.y_lin, 
		imu.z_lin, 
		imu.x_mag, 
		imu.y_mag, 
		imu.z_mag,
		stateData.roll, //imupos
		stateData.pitch,
		stateData.yaw,
		stateData.x_lin,
		stateData.y_lin,
		stateData.speedScalar, //important imu info
		stateData.degreesNorth,
		stateData.depthApprox,
		command.depthUp, //information received
		command.depthDown,
		command.captureImage,
		command.forwardThrust,
		command.turnThrust,
		command.camUpDown,
		command.camLeftRight,
		left_s.a_one, //stepper L
		left_s.a_two,
		left_s.b_one,
		left_s.b_two,
		right_s.a_one, //stepper R
		right_s.a_two,
		right_s.b_one,
		right_s.b_two,
		actuate.rightPropThrust,
		actuate.leftPropThrust);
	 
	*/
	
	// For actual communication back to RPI (STILL USING TEST DATA)(This sprintf call works)
	sprintf(tx_buffer, "%u,%u,%u,%u,%u,%u,%u\n\r", stateData.degreesNorth, (uint16_t)(10*stateData.speedScalar), stateData.depthApprox, stateData.roll, stateData.pitch, stateData.yaw, (uint16_t)(10*analogData.batteryVoltage));
	
	// For reading the i2c values on the RPI
	//sprintf(tx_buffer, "%u,%u,%u,%u,%u,%u,%u,%u,%u\n\r", imu.x_ang, imu.y_ang, imu.z_ang, imu.x_lin, imu.y_lin, imu.z_lin, imu.x_mag, imu.y_mag, imu.z_mag );

  //sprintf(tx_buffer, "%u,%u,%u,%u,%d,%d\n\r", (uint8_t)(leftThrust*100.0), (uint8_t)(rightThrust*100.0), (uint8_t)(forThrust*100.0), (uint8_t)(backThrust*100.0), (int)(leftPropThrust*100.0), (int)(rightPropThrust*100.0));

	//Read back what was received
	//sprintf(tx_buffer, "%u,%u,%u,%u,%u,%u,%u\n\r", depthUp, depthDown, captureImage, forwardThrust, turnThrust, camUpDown, camLeftRight);
	
	//sprintf(tx_buffer, "TestMessage,,,,,,,,,,,,,\n\r");

  HAL_UART_Transmit(&uart_com, (uint8_t *)tx_buffer, sizeof(tx_buffer), 10);	
}
