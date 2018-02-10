#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, in1,    gyro,           sensorGyro)
#pragma config(Sensor, I2C_1,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_2,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_3,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Motor,  port2,           leftD,         tmotorVex393HighSpeed_MC29, PIDControl, encoderPort, I2C_1)
#pragma config(Motor,  port3,           rightD,        tmotorVex393HighSpeed_MC29, PIDControl, reversed, encoderPort, I2C_2)
#pragma config(Motor,  port4,           lift,          tmotorVex393_MC29, openLoop, reversed, encoderPort, I2C_3)
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*        Description: Competition template for VEX EDR                      */
/*                                                                           */
/*---------------------------------------------------------------------------*/

// This code is for the VEX cortex platform
#pragma platform(VEX2)

// Select Download method as "competition"
#pragma competitionControl(Competition)

//Main competition background code...do not modify!
#include "Vex_Competition_Includes.c"

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the cortex has been powered on and    */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/


int threshold=15;

void initializeGyro(){
	SensorType[gyro]=sensorNone;
	wait1Msec(1000);
	SensorType[gyro]=sensorGyro;
	wait1Msec(1000);
}




void rotate(int c, int angle){
	float targetAngle=c*angle*-10;
	float rotateError;
	float rotateIntegral;
	float rotateLastError;
	float integralLimit=30;
	float integralActiveZoneRotation=100;
	float gyroKp=0.05;
	float gyroKi=0.07;
	float gyroKd=4;
	float gyroP;
	float gyroI;
	float gyroD;
	float power;
	SensorValue[in1]=0;
	wait1Msec(100);
	clearTimer(T1);
	rotateError=targetAngle-SensorValue[in1];
	while(time1(T1)<2000 && abs(rotateError)>15){
		//PROPORTIONAL
		rotateError=targetAngle-SensorValue[in1];
		gyroP=rotateError*gyroKp;

		//INTEGRAL
		if(abs(rotateError)<integralActiveZoneRotation){
			rotateIntegral+=rotateError;
		}
		else{
			rotateIntegral=0;
		}
		gyroI=rotateIntegral*gyroKi;
		if(rotateIntegral>integralLimit){
			rotateIntegral=integralLimit;
		}

		//DERIVATIVE
		gyroD=(rotateError-rotateLastError)*gyroKd;
		rotateLastError=rotateError;
		if(rotateError==0){
			rotateLastError=0;
		}
		power=gyroP+gyroI+gyroD;
		motor[leftD]=-power;
		motor[rightD]=power;
	}
	motor[leftD]=0;
	motor[rightD]=0;
	wait1Msec(100);
}



// PID controlled movement
// i: controls direction, set to 1 for forward and -1 for negative
// d: distance in # of tiles
// s: speed of motor from -127 to 127
void moveStraight(int i,float d,int s){
	clearTimer(T1);
	nMotorEncoder(leftD) = 0;
	nMotorEncoder(rightD) = 0;
	short dist = -700*d*i;
	//remember to change code based on wiring
	setMotorTarget(leftD,dist,s,false);
	setMotorTarget(rightD,dist,s,false);

	// Runs till target is reached
	while((!getMotorTargetCompleted(leftD) || !getMotorTargetCompleted(rightD)) && time1[T1]<2000)
	{
		sleep(10);
	}
}

void liftX(int ticks){
	clearTimer(T1);
	nMotorEncoder[lift]=0;
	if(ticks < 0)
	{
		while(nMotorEncoder[lift]>ticks && time1[T1] < 2000)
		{
			motor[lift] = 127;
		}
		}else{
		while(nMotorEncoder[lift]<ticks && time1[T1] < 2000)
		{
			motor[lift]=-127;
		}
	}
	motor[lift]=0;
}


void oldThreeGoal(){
	liftX(-530);
	moveStraight(1,1.5,127);
	liftX(530);
	rotate(-1,165);
	moveStraight(1,2.3,127);
	liftX(-530);// score 20pt
	moveStraight(-1,0.8,120);
	liftX(530);
	rotate(-1,90);
	moveStraight(1,1,100);
	rotate(-1,33);
	liftX(-530);
	moveStraight(1,1.7,100);
	liftX(530); // pick up second goal
	moveStraight(-1,1.9,100);
	rotate(-1,138);
	moveStraight(1,1.7,100);
	rotate(-1,90);
	//moveStraight(1,0.2,120);
	liftX(-490);//score 10pt

//GOAL THREE
	moveStraight(-1,0.2,127);//Score (9,1)
	liftX(530);
	wait1Msec(30);
	rotate(1,90);
	moveStraight(1,1,100);
	rotate(1,28);
	liftX(-530);
	moveStraight(1,1.5,127);
	wait1Msec(10);
	liftX(530);// Pick up 3rd

	wait1Msec(10);
	moveStraight(-1,1.7,127);
	wait1Msec(100);
	rotate(-1,155);
	moveStraight(1,2.6,127);

	liftX(-530);//score 10pt

	//GOAL FOUR (3,4) trying to put it into the opposite corner 20 point zone

	moveStraight(-1, 0.3, 127);
	liftX(230);
	moveStraight(-1, 0.7, 127);
	liftX(-230);
	rotate(-1, 90);
	moveStraight(1, 0.2, 127);
	rotate(-1, 90);
	moveStraight(1, 1, 127); // SET TO FOUR (placeholder idk what the val is), no space on mat, should charge on the 4th goal and hopefully snag it into the next zone
	liftX(530);
	moveStraight(1, 1,120);
	rotate(-1, 19.5);
	moveStraight(1,2.2,100);
	liftX(-530);
}

void anasRoute(){
	liftX(-530);
	moveStraight(1,1.5,127);
	liftX(530);// pick up 1
	rotate(-1,165);
	moveStraight(1,2.3,127);
	liftX(-530);// score 20pt

	moveStraight(-1,1,127);
	rotate(-1,90);
	moveStraight(1,0.5,100);
	rotate(-1,90);
	moveStraight(1,3,120);
	wait1Msec(100);
	liftX(530);
	rotate(-1,20);
	moveStraight(1,2,127);
	liftX(-530);
}

void pre_auton()
{
  // Set bStopTasksBetweenModes to false if you want to keep user created tasks
  // running between Autonomous and Driver controlled modes. You will need to
  // manage all user created tasks if set to false.
  bStopTasksBetweenModes = true;
	initializeGyro();
	// Set bDisplayCompetitionStatusOnLcd to false if you don't want the LCD
	// used by the competition include file, for example, you might want
	// to display your team name on the LCD in this function.
	// bDisplayCompetitionStatusOnLcd = false;

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

task autonomous()
{
	//oldThreeGoal();
	anasRoute();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

task usercontrol()
{
  // User control code here, inside the loop

  while (true)
  {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    // Remove this function call once you have "real" code.
    UserControlCodePlaceholderForTesting();
  }
}
