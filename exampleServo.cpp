#include "motorMX430.h"

int main()
{
	//Create a servo motor: servo*(dynamyxel ID, mode, currentlimit, goalcurrent)
	MotorXM430 servo1(1, 3, 800, 90);
	
	//Accessing the motors function
	servo1.PrintOperatingMode();
	
	//Controlling the motor
	servo1.TorqueON();
	servo1.Goto(180);
	usleep(500000);
	printf("Motor %d, current position: %f\n",servo1.GetID(), servo1.ReadAngle());
	
	
	//Adding others motor:
	MotorXM430 servo2(2, 3, 800, 90);
	printf("Motor %d, current position: %f\n",servo2.GetID(), servo2.ReadAngle());
	
	return(0);
}
