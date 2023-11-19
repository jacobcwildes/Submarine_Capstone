#include "planner.h"

struct goalCommand *current_Command;

uint64_t iteration = 0;
uint64_t timeout = 0;

uint8_t binaryToDecimal(int start_index, int bitCount, uint8_t *data)
{
	//MSB is on the left so we start high and go low on the exp
	uint8_t result = 0;
	for (int i = bitCount - 1; i >= 0; i--)
	{
		if (data[start_index + i] == 49)
		{
			result += pow(2, 7 - i);
		}
	}
	return result;
}


struct goalCommand parseComs(uint8_t *data, volatile uint8_t received)
{
  struct goalCommand com = *current_Command;
  if (received)
  {
  	timeout = 0;
  	iteration = 0;
  	
  	//Coms parsing
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); //toggle LED for dev
		if (data[0] == 48)
		{
			com.depthUp = 0;
		}
		else if (data[0] == 49)
		{
			com.depthUp = 1;
		}
		
		if (data[1] == 48)
		{
			com.depthDown = 0;
		}
		else if (data[1] == 49)
		{
			com.depthDown = 1;
		}
		
		if (data[2] == 48)
		{
			com.captureImage = 0;
		}
		else if (data[2] == 49)
		{
			com.captureImage = 1;
		}
		
		com.forwardThrust = binaryToDecimal(3, 8, data);
		com.turnThrust = binaryToDecimal(11, 8, data);
		com.camUpDown = binaryToDecimal(19, 8, data);
		com.camLeftRight = binaryToDecimal(27, 8, data);
  }
	else
	{
		timeout++;
		
		if (timeout >= 10) //10 Seconds of nothing from the pi, go into auto mode
		{
			timeout--;
			//DEMO
			iteration++;
			
			com.captureImage = 0;
			if (iteration < 10*40)
			{
				HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); //toggle LED for dev
				com.depthUp = 1;
				com.depthDown = 0;
				com.forwardThrust = 255;
				com.turnThrust = 128;
				com.camUpDown = 160;
				com.camLeftRight = 160;
			}
			else if (iteration < 20*40)
			{
				com.depthUp = 0;
				com.depthDown = 1;
				com.forwardThrust = 0;
				com.turnThrust = 128;
				com.camUpDown = 160;
				com.camLeftRight = 96;
			}
			else if (iteration < 30*40)
			{
				com.depthUp = 1;
				com.depthDown = 0;
				com.forwardThrust = 128;
				com.turnThrust = 255;
				com.camUpDown = 96;
				com.camLeftRight = 96;
			}
			else if (iteration < 40*40)
			{
				com.depthUp = 0;
				com.depthDown = 1;
				com.forwardThrust = 128;
				com.turnThrust = 0;
				com.camUpDown = 96;
				com.camLeftRight = 190;
			}
			
			if (!(iteration%10 == 0)) //ONLY DO DEPTH CHANGE 10% of time
			{
				com.depthUp = 0;
				com.depthDown = 0;
			}
			
			if (iteration >= 40*40) iteration = 0;
		}
	}
  
  *current_Command = com;
  return com;
}

