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

void liftPID(float target){
	float kpLift=0.5;
	float kiLift=0;
	float kdLift=2;

	float errorPLift;
	float errorILift;
	float errorDLift;
	float proportionalLift;
	float integralLift;
	float integralLimitLift=50;
	float integralActiveZoneLift=100;
	float derivativeLift;

	float powerLift;
	clearTimer(T2);
	while(time1(T2)<1000){
		errorPLift=target-nMotorEncoder[lift];
		proportionalLift=errorPLift*kpLift;

		//INTEGRAL
		if(abs(errorPLift)<integralActiveZoneLift){
			errorILift+=errorPLift;
		}
		else{
			errorILift=0;
		}
		integralLift=errorILift*kiLift;
		if(integralLift>integralLimitLift){
			integralLift=integralLimitLift;
		}
		//DERIVATIVE
		derivativeLift=(errorPLift-errorDLift)*kdLift;
		errorDLift=errorPLift;

		powerLift=proportionalLift+integralLift+derivativeLift;

		motor[lift]=powerLift;
	}
}

void pre_auton()
{
	// Set bStopTasksBetweenModes to false if you want to keep user created tasks
	// running between Autonomous and Driver controlled modes. You will need to
	// manage all user created tasks if set to false.
	bStopTasksBetweenModes = true;

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
	// ..........................................................................
	// Insert user code here.
	// ..........................................................................

	// Remove this function call once you have "real" code.
	AutonomousCodePlaceholderForTesting();
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
	while(1){
		if(abs(vexRT[Ch3])>threshold){
			motor[leftD]=vexRT[Ch3];
		}
		else{
			motor[leftD]=0;
		}
		if(abs(vexRT[Ch2])>threshold){
			motor[rightD]=vexRT[Ch2];
		}
		else{
			motor[rightD]=0;
		}
		if(vexRT[Btn6U]==1){
			motor[lift]=127;
			//liftPID(-40);
		}
		else if(vexRT[Btn6D]==1){
			motor[lift]=-127;
			//liftPID(630);
		}
		else{
			motor[lift]=0;
		}
	}
}
