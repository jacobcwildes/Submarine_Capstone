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


struct goalCommand parseComs(uint8_t *data)
{
	struct goalCommand com = *current_Command;

	//Coms parsing
	if (data[0] == 48) com.depthUp = 0;
	else if (data[0] == 49) com.depthUp = 1;

	if (data[1] == 48) com.depthDown = 0;
	else if (data[1] == 49) com.depthDown = 1;


	com.forwardThrust = binaryToDecimal(2, 8, data);
	com.turnThrust = binaryToDecimal(10, 8, data);
	com.camUpDown = binaryToDecimal(18, 8, data);
	com.camLeftRight = binaryToDecimal(26, 8, data);
	com.roll = binaryToDecimal(34, 8, data);
  
	*current_Command = com;
	return com;
}

