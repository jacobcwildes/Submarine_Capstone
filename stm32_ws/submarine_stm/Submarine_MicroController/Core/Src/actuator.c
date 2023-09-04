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


void updateProps(struct actuator_command actuate);
{
  //LEFT PROP SET DC (TIM3_CH2 = forward) (TIM3_CH3 = backward)
  if (act.leftPropThrust >= 0) 
  {
    TIM3->CCR2 = act.leftPropThrust*ARR_Prop;
    TIM3->CCR3 = 0;
  }
  else 
  {
    TIM3->CCR3 = -act.leftPropThrust*ARR_Prop;
    TIM3->CCR2 = 0;
  }
  
  //RIGHT PROP SET DC (TIM4_CH2 = forward) (TIM4_CH3 = backward)
  if (act.rightPropThrust >= 0) 
  {
    TIM4->CCR2 = act.rightPropThrust*ARR_Prop;
    TIM4->CCR3 = 0;
  }
  else
  {
    TIM4->CCR3 = -act.rightPropThrust*ARR_Prop;
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
  TIM5->CCR2 = act.camVerticalDuty*ARR_Servo;
  TIM5->CCR3 = act.camHorizontalDuty*ARR_Servo;
}


void updateSteppers(struct actuator_command actuate)
{
	//DO STEPPERS
}



void transmitData(UART_HandleTypeDef uart_com, struct actuate_command actuate)
{
	//Data concat
	struct state stateData = actuate.s;
	struct envData environmentData = stateData.env;
	struct adcData analogData = environmentData.adc;
	struct inputData in = environment.input;
	
	char tx_buffer[100] = "YAYAYAYYAYA\r\n";
	
	
	
	// For actual communication back to RPI (STILL USING TEST DATA)(This sprintf call works)
	sprintf(tx_buffer, "%u,%u,%u,%u,%u,%u,%u\n\r", degreesNorth, (uint16_t)(10*speedScalar), depthApprox, roll, pitch, yaw, (uint16_t)(10*voltageBattery));
	
	// For reading the i2c values on the RPI
	//sprintf(tx_buffer, "%u,%u,%u,%u,%u,%u,%u,%u,%u\n\r", x_ang, y_ang, z_ang, x_lin, y_lin, z_lin, x_mag, y_mag, z_mag );

  //sprintf(tx_buffer, "%u,%u,%u,%u,%d,%d\n\r", (uint8_t)(leftThrust*100.0), (uint8_t)(rightThrust*100.0), (uint8_t)(forThrust*100.0), (uint8_t)(backThrust*100.0), (int)(leftPropThrust*100.0), (int)(rightPropThrust*100.0));

	//Read back what was received
	//sprintf(tx_buffer, "%u,%u,%u,%u,%u,%u,%u\n\r", depthUp, depthDown, captureImage, forwardThrust, turnThrust, camUpDown, camLeftRight);
	
	//sprintf(tx_buffer, "TestMessage,,,,,,,,,,,,,\n\r");

  HAL_UART_Transmit(&uart_com, (uint8_t *)tx_buffer, sizeof(tx_buffer), 10);	
}
