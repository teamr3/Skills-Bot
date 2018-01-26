#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, in1,    gyro,           sensorGyro)
#pragma config(Sensor, I2C_1,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_2,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_3,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Motor,  port2,           leftD,         tmotorVex393HighSpeed_MC29, PIDControl, reversed, encoderPort, I2C_1)
#pragma config(Motor,  port3,           rightD,        tmotorVex393HighSpeed_MC29, PIDControl, encoderPort, I2C_2)
#pragma config(Motor,  port4,           lift,          tmotorVex393_MC29, PIDControl, encoderPort, I2C_3)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

int threshold=15;
void initialize(){
	nMotorEncoder[leftD]=0;
	nMotorEncoder[rightD]=0;
	nMotorEncoder[lift]=0;
}
void initializeGyro(){
	SensorType[in1] = sensorNone;
	wait1Msec(100);
	SensorType[in1] = sensorGyro;
	wait1Msec(100);
}
/*
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

		motor[lift]=-powerLift;
	}
}
*/
// PID controlled movement
// i: controls direction, set to 1 for forward and -1 for negative
// d: distance in # of tiles
// s: speed of motor from -127 to 127
void moveStraight(int i,long d,int s){
	clearTimer(T1);
	nMotorEncoder(leftD) = 0;
	nMotorEncoder(rightD) = 0;
	short dist = 700*d*i;
	//remember to change code based on wiring
	setMotorTarget(leftD,dist,s,false);
	setMotorTarget(rightD,dist,s,false);

	// Runs till target is reached
	while((!getMotorTargetCompleted(leftD) || !getMotorTargetCompleted(rightD))&time1[T1]<4000)
	{
		sleep(10);
	}
}
void doLift(int d, int ticks, int power){
	clearTimer(T2);
	nMotorEncoder[lift]=0;
	setMotorTarget(lift, d*ticks, power, false);
		while((!getMotorTargetCompleted(lift))&time1[T2]<3000)
	{
		sleep(10);
	}
}


/*
void rotateBot(int anglex10){ //rotate robot
	float startAngle = SensorValue[in1];
	float kpRotate = 0.9;
	float kdRotate = 2.8;

	float speed = 0;
	int errorGyro2 = 1;
	float errorGyro = anglex10 - (SensorValue[in1] - startAngle);
	float gyro0 = SensorValue[in1];

	while (abs(errorGyro) > 15) {
		errorGyro = anglex10 - (SensorValue[in1] - startAngle);
		float speed = 0.7*(errorGyro*kpRotate) + ((errorGyro - errorGyro2)*kdRotate);

		motor[frontL] = -speed;
		motor[backL] = -speed;
		motor[frontR] = speed;
		motor[backR] = speed;

		errorGyro2 = errorGyro;
		wait1Msec(20);
	}
	motor[frontL] = 0;
	motor[backL] = 0;
	motor[frontR] = 0;
	motor[backR] = 0;
}

void rotateLeft (int angle) {
	rotateBot(angle*10);
}

void rotateRight (int angle) {
	rotateBot(-angle*10);
}
*/
void turn (int degree){//counter clockwise is positiv	SensorValue[in8] = 0;
	SensorType[in1] = sensorNone;
	wait1Msec(400);
	SensorType[in1] = sensorGyro;
	wait1Msec(400);
	time1[T1] = 0;
	if(degree < 0)
	{
		while(SensorValue[in1] >(degree*10) && time1[T1] < 3000)
		{
			motor[leftD] = (-35);
			motor[rightD] = (35);
		}
	}
	else {
		while(SensorValue[in1] <(degree*10 )&& time1[T1] < 3000)
		{
			motor[leftD] = (35);
			motor[rightD] = (-35);
		}
	}
	motor[rightD] = 0;
	motor[leftD] = 0;
}

task main()
{
	doLift(down);
	moveStraight(1,1.5,70);
	doLift(up);
	moveStraight(-1,1,70);
	turn(-180);
	moveStraight(1,.5,70);
	doLift(down);
	moveStraight(-1,.5,70);
	turn(45);
	moveStraight(1,1,70);
	turn(90);
	doLift(down);
	moveStraight(1,1.5,70);
	doLift(up);
	turn(-160);
	moveStraight(1,3,70);
	doLift(down);
	moveStraight(-1,1,70);
	turn(150);
	moveStraight(1,1.25,70);
	doLift(up);
	turn(-150);
	moveStraight(1,1,70);
	doLift(down);
	moveStraight(-1,.5,70);
	turn(180);
	moveStraight(1,4,70);
	doLift(up);
	turn(10);
	moveStraight(1,3,70);
	doLift(down);
	moveStraight(-1,1,70);
	turn(160);
	moveStraight(1,1,70);
	doLift(up);
	turn(-160);
	moveStraight(1,1.5,70);
	doLift(down);
	moveStraight(-1,1,70);
	turn(-45);
	moveStraight(1,3,70);
	turn(-90);
	moveStraight(1,1.5,70);
	doLift(up);
	moveStraight(-1,1,70);
	turn(-180);
	moveStraight(1,.5,70);
	doLift(down);
	moveStraight(-1,2,70);
	doLift(up);
}