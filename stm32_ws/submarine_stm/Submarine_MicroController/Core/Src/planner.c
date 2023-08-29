#include "planner.h"

uint8_t binaryToDecimal(int start_index, int bitCount)
{
	//MSB is on the left so we start high and go low on the exp
	uint8_t result = 0;
	for (int i = bitCount - 1; i >= 0; i--)
	{
		if (rx_data[start_index + i] == 49)
		{
			result += pow(2, 7 - i);
		}
	}
	return result;
}


void parseComs(void)
{
  //Coms parsing
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); //toggle LED for dev
  if (rx_data[0] == 48)
  {
  	depthUp = 0;
  }
  else if (rx_data[0] == 49)
  {
  	depthUp = 1;
  }
  
  if (rx_data[1] == 48)
  {
  	depthDown = 0;
  }
  else if (rx_data[1] == 49)
  {
  	depthDown = 1;
  }
  
  if (rx_data[2] == 48)
  {
  	captureImage = 0;
  }
  else if (rx_data[2] == 49)
  {
  	captureImage = 1;
  }
	
	forwardThrust = binaryToDecimal(3, 8);
	turnThrust = binaryToDecimal(11, 8);
	camUpDown = binaryToDecimal(19, 8);
	camLeftRight = binaryToDecimal(27, 8);
}
