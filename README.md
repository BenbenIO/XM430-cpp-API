# XM430-cpp-API
A simple c++ API for controlling dynamixel XM430-* servomotor.
The API was made in order to control a robotic arm, and consist of an unique class. The programme ran on a Raspberry PI 3B+, with the Dynamixel XM430 motors connected with [U2D2](http://www.robotis.us/u2d2/) (Serial-RS485).
The datasheet references can be found [HERE](http://support.robotis.com/en/product/actuator/dynamixel_x/xm_series/xm430-w350.htm#bookmark5). 
Do not hesitate if you have any Issue or Resquest (function to add)

# Install && Dependencies
The programme depend on the dynamixel_sdk library. Installation information can be found on their [github](https://github.com/ROBOTIS-GIT/DynamixelSDK). On the raspberry Pi please uild and intall the SingleBoard Computer version (linux_sbc). 
Once the install is done download this repository and make the MakeFile in the make_run directory.

# Example Program
The api is illustrated in the exampleServo.cpp file.

```c
//Create a servo motor: servo*(dynamyxel ID, mode, currentlimit, goalcurrent)
MotorXM430 servo1(1, 3, 800, 90);
	
//Accessing the motors function:
servo1.PrintOperatingMode();
	
//Controlling the motor:
servo1.TorqueON();
servo1.Goto(180);
usleep(500000);
printf("Motor %d, current position: %f\n",servo1.GetID(), servo1.ReadAngle());
	
//Adding another motor:
MotorXM430 servo2(2, 3, 800, 90);
printf("Motor %d, current position: %f\n",servo2.GetID(), servo2.ReadAngle());
```

Output of the program:

<img src="/images/ExampleRUN.PNG" width="450">

# API Description:
* int GetID():
<br />GetID: return the motors ID. Input: Void. Output : int ID. Example: MotorXM430 motor; printf("ID: %d\n",motor.GetID());

* uint16_t GetModelNumber():
<br />GetModelNumber(): return the model of the motor. (ex: model numer 1020 is the MX430). This function is used to read the current of the motors. As the first function call in the construstor, we used this function to check if the motor is turn ON or without failure. In case of failure, this function will terminate the program.

* uint8_t IsMoving();
<br />IsMoving(): return the moving status of the motor.Used for motor synchronisation when several motors are moving.

* void MovingStatus();
MovingStatus: printf the moving status of the motors. Used for debugging.

* int16_t ReadCurrent();
ReadCurrent(): return the signed current of the motor in mA. 


* float MAP(uint32_t angle, long in_min, long in_max, long out_min, long out_max);
MAP: return the mapped angle value into the other unit. Used for converting angle degree to angle motors.]
Inputs: angle is the value we want to convert. in_min/max: the minimal/maximal angle value in the 1fst unit. out_min/max: the minimal/maximal angle value in the 2nd unit.
Output: the mapped angle.
Example: MAP(m_present_position, MIN_MOTOR_ANGLE, MAX_MOTOR_ANGLE, 0.0, 360.0)
	
* void TorqueON();
* void TorqueOFF();
TorqueON - TorqueOFF: (dis-)activate the torque inside the motor. Without torque the motor will not move.

	//ReadAngle: return the current angle of the motor in degree
	float ReadAngle(); 
	
	//Goto: set the goal position in the motor. The function do not currently check the validity of the input
	//Input: wanted position in degree float
	void Goto(float position);
	
	//SetOperatingMode: set the operating mode in the motor (Availiable mode: 0 (Current), 1 (Velocity), 3 (Position), 4 (Extended position), 5 (current-based position), 16 (PWM))
	//Input: wanted mode uint8_t
	void SetOperatingMode(uint8_t mode);						
	//PrintOperationMode: printf in the console the operationmode. Used for debugging.
	void PrintOperatingMode();
	
	//SetPID: set the different parameters of the internal motor's PID controler.
	//Input the P / I / D value between 0-16383 uint16_t
	void SetPID(uint16_t P_gain, uint16_t I_gain, uint16_t D_gain);
	//PrintPID: printf the gain value of the motor's PID in the consol. Used for debugging.
	void PrintPID();
	
	//SetFFGain: set the different parameters of the internal motor's FeedForward controler.
	//Input the FF1 FF2 value between 0-16383 uint16_t
	void SetFFGain(uint16_t FF1Gain, uint16_t FF2Gain);
	//PrintFFGain: printf the gain value of the motor's FeedForward control in the consol. Used for debugging.
	void PrintFFGain();
	
	//SetProfile: set the acceleration and velocity profile of the motor. Used to tunned the motor behaviour and for motor synchronisation when several motors are moving.
	//Input: wanted velocity (RPM) and acceleration (Rev/min2) uint32_t. The function check if the wanted value are in the motor's limits ( XM430: VELOCITY_LIMIT 500 / ACCELERATION_LIMIT 32767)
	void SetProfile(uint32_t Velocity, uint32_t Acceleration);
	//PrintProfile: printf the Velocity and Acceleration value in the consol. Used for debugging.
	void PrintProfile();
	
	//SetCurrentLimit: set the maximun current (torque) output of the motor. Used for current-based position control (gripper)
	//Input is set in the define section CURRENT_LIMIT 1193
	void SetCurrentLimit();
	//PrintCurrentLimit: printf the value of the current limit (mA) in the console. Used for debugging.
	void PrintCurrentLimit();
	
	//SetGoalCurrent: set the goal current. Used for current-based position control
	//Input: wanted goal current mA uint16_t. The function check if the wanted goalcurrent is not exceeding the currentlimit
	void SetGoalCurrent(uint16_t GoalCurrent);
	//PrintGoalcurrent: printf the value of the goalcurrent (mA) in the console. Used for debugging.
	void PrintGoalCurrent();
