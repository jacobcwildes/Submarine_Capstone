#include "actuator.h"

#define ARR_Prop 11999
#define ARR_Servo 2399999

void MTR_DRV_INIT(void)
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
	//Current 100%, change if too much
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_RESET);

	//Decay
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);

	//Reset
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_SET);

	//Sleep
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);

	//Left Prop DC
	TIM3->CCR2 = 0;
	TIM3->CCR3 = 0;

	//Right Prop DC
	TIM4->CCR2 = 0;
	TIM4->CCR3 = 0;

	//Servos DC
	TIM5->CCR2 = 0.5 * ARR_Servo;
	TIM5->CCR3 = 0.5 * ARR_Servo;

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

		//Right
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

	
  
}

void updateActuators(UART_HandleTypeDef uart_com, struct actuator_command actuate)
{
	updateProps(actuate);
	updateServos(actuate);
	updateSteppers(actuate);
	transmitData(uart_com, actuate);
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
    TIM4->CCR3 = actuate.rightPropThrust*ARR_Prop/3;
    TIM4->CCR4 = 0;
  }
  else
  {
    TIM4->CCR4 = -actuate.rightPropThrust*ARR_Prop/3;
    TIM4->CCR3 = 0;
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
	
	struct adcData analog = actuate.in.adc;
	struct inputData inputs = actuate.in.input;
		
	char tx_buffer[100] = "";
	
	// For actual communication back to RPI (STILL USING TEST DATA)(This sprintf call works)
	sprintf(tx_buffer, "%u,%u,%u,%u,%u,%u\n\r", (uint16_t)(10*analog.batteryVoltage),
	 (uint16_t)analog.leftBallastPosition, 
	 (uint16_t)analog.rightBallastPosition,
	 (uint16_t)inputs.nFaultLeft,
	 (uint16_t)inputs.nFaultProp,
	 (uint16_t)inputs.nFaultRight);
	
	

  HAL_UART_Transmit(&uart_com, (uint8_t *)tx_buffer, sizeof(tx_buffer), 10);	
}
