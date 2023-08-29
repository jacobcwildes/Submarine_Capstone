#include "actuator.h"


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


void updateProps(void)
{
  //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); //toggle LED for dev
  
  //Convert coms thrust vals in percentages for f,b,l,r
  if (turnThrust < 128)
  {
  	leftThrust = ((float)(-turnThrust + 128))/256.0; // 0% <-> 50% 
  	rightThrust = 0;
  }
  else
  {
  	leftThrust = 0;
  	rightThrust = ((float)(turnThrust - 128))/256.0; // 0% <-> 50% 
  }
  
  if (forwardThrust < 128)
  {
  	forThrust = 0;
		backThrust = ((float)(-forwardThrust + 128))/256.0; // 0% <-> 50%
  }
  else
  {
  	forThrust = ((float)(forwardThrust - 128))/256.0; // 0% <-> 50%
		backThrust = 0;
  }
  
  
  //mix individual thrust values into 
  rightPropThrust = leftThrust - rightThrust + forThrust - backThrust; // -100% <-> 100%
  leftPropThrust = rightThrust - leftThrust + forThrust - backThrust; // -100% <-> 100%
  
  //LEFT PROP SET DC (TIM3_CH2 = forward) (TIM3_CH3 = backward)
  if (leftPropThrust >= 0) 
  {
    TIM3->CCR2 = leftPropThrust*ARR_Prop;
    TIM3->CCR3 = 0;
  }
  else 
  {
    TIM3->CCR3 = -leftPropThrust*ARR_Prop;
    TIM3->CCR2 = 0;
  }
  
  //RIGHT PROP SET DC (TIM4_CH2 = forward) (TIM4_CH3 = backward)
  if (rightPropThrust >= 0) 
  {
    TIM4->CCR2 = rightPropThrust*ARR_Prop;
    TIM4->CCR3 = 0;
  }
  else
  {
    TIM4->CCR3 = -rightPropThrust*ARR_Prop;
    TIM4->CCR2 = 0;
  }
  
}

void updateServos(void)
{
  //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); //toggle LED for dev 
  //Convert coms thrust vals in percentages for f,b,l,r
  
  //Setup for 1ms <-> 2ms
  //camVerticalDuty = (0.000196 * (float)camUpDown) + 0.05;
  //camHorizontalDuty = (0.000196 * (float)camLeftRight) + 0.05;
  
  //Setup for 0.5ms <-> 2.5ms
  camVerticalDuty = (0.000392 * (float)camUpDown) + 0.025;
  camHorizontalDuty = (0.000392 * (float)camLeftRight) + 0.025;
  
  //Set Duty Cycles
  TIM5->CCR2 = camVerticalDuty*ARR_Servo;
  TIM5->CCR3 = camHorizontalDuty*ARR_Servo;
}

void leftBalastOpenStep(void)
{
	currentStepLeft++;
	if (currentStepLeft == 4) currentStepLeft = 0; //wrap around
	
}

void leftBalastCloseStep(void)
{
	currentStepLeft--;
	if (currentStepLeft == 255) currentStepLeft = 3; //wrap around
}

void rightBalastOpenStep(void)
{
	currentStepRight++;
	if (currentStepRight == 4) currentStepRight = 0; //wrap around
}

void rightBalastCloseStep(void)
{
	currentStepRight--;
	if (currentStepRight == 255) currentStepRight = 3; //wrap around
}

void updateSteppers(void)
{
	if (depthUp && baseStepperPos >= 10)
	{
		//Move toward full open
		leftBalastOpenStep();
		leftStepperPos--;
		rightStepperOpenStep();
		rightStepperPos--;
		baseStepperPos--;
	}
	if (depthUp && rightStepperPos <= full_close_step - 10)
	{
		rightStepperOpenStep();
		rightStepperPos--;
	}
}



void transmitData(void)
{
	//Data concat
	
	// For actual communication back to RPI (STILL USING TEST DATA)(This sprintf call works)
	sprintf(tx_buffer, "%u,%u,%u,%u,%u,%u,%u\n\r", degreesNorth, (uint16_t)(10*speedScalar), depthApprox, roll, pitch, yaw, (uint16_t)(10*voltageBattery));
	
	// For reading the i2c values on the RPI
	//sprintf(tx_buffer, "%u,%u,%u,%u,%u,%u,%u,%u,%u\n\r", x_ang, y_ang, z_ang, x_lin, y_lin, z_lin, x_mag, y_mag, z_mag );

  //sprintf(tx_buffer, "%u,%u,%u,%u,%d,%d\n\r", (uint8_t)(leftThrust*100.0), (uint8_t)(rightThrust*100.0), (uint8_t)(forThrust*100.0), (uint8_t)(backThrust*100.0), (int)(leftPropThrust*100.0), (int)(rightPropThrust*100.0));

	//Read back what was received
	//sprintf(tx_buffer, "%u,%u,%u,%u,%u,%u,%u\n\r", depthUp, depthDown, captureImage, forwardThrust, turnThrust, camUpDown, camLeftRight);
	
	//sprintf(tx_buffer, "TestMessage,,,,,,,,,,,,,\n\r");

  HAL_UART_Transmit(&hlpuart1, (uint8_t *)tx_buffer, sizeof(tx_buffer), 10);	
}
