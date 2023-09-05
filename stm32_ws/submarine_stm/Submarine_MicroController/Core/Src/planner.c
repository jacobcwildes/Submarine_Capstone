#include "planner.h"

struct goalCommand *current_Command;



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


struct goalCommand parseComs(uint8_t received, uint8_t *data)
{
  if (received)
  {
  	//Coms parsing
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); //toggle LED for dev
		if (data[0] == 48)
		{
			current_Command->depthUp = 0;
		}
		else if (data[0] == 49)
		{
			current_Command->depthUp = 1;
		}
		
		if (data[1] == 48)
		{
			current_Command->depthDown = 0;
		}
		else if (data[1] == 49)
		{
			current_Command->depthDown = 1;
		}
		
		if (data[2] == 48)
		{
			current_Command->captureImage = 0;
		}
		else if (data[2] == 49)
		{
			current_Command->captureImage = 1;
		}
		
		current_Command->forwardThrust = binaryToDecimal(3, 8, data);
		current_Command->turnThrust = binaryToDecimal(11, 8, data);
		current_Command->camUpDown = binaryToDecimal(19, 8, data);
		current_Command->camLeftRight = binaryToDecimal(27, 8, data);
  }
  
  return *current_Command;
}

