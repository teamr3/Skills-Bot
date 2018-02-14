#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, I2C_1,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_2,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_3,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Motor,  port2,           leftEDrive,    tmotorVex393HighSpeed_MC29, openLoop, encoderPort, I2C_2)
#pragma config(Motor,  port3,           leftDrive,     tmotorVex393HighSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           rightEDrive,   tmotorVex393HighSpeed_MC29, openLoop, reversed, encoderPort, I2C_3)
#pragma config(Motor,  port7,           rightDrive,    tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,  port8,           lift,          tmotorVex393HighSpeed_MC29, openLoop, encoderPort, I2C_1)
#pragma config(Motor,  port9,           slaveLift,     tmotorVex393HighSpeed_MC29, openLoop, reversed)

int threshold = 15;

void initialize(){
	slaveMotor(slaveLift,lift);
	slaveMotor(leftDrive,leftEDrive);
	slaveMotor(rightDrive,rightEDrive);
	nMotorEncoder[leftEDrive]=0;
	nMotorEncoder[rightEDrive]=0;
	wait1Msec(1000);
}


task main(){
	initialize();
	while (true) {

				if (abs(vexRT[Ch3]) > threshold) {
					motor[leftEDrive] = vexRT[Ch3];
				}
				else {
					motor[leftEDrive] = 0;
				}

				if (abs(vexRT[Ch2]) > threshold) {
					motor[rightEDrive] = vexRT[Ch2];
				}
				else {
					motor[rightEDrive] = 0;
				}

				if (vexRT[Btn6U] == 1) {
					motor[lift] = 127;
				}
				else if (vexRT[Btn6D] == 1) {
					motor[lift] = -127;
				}
				else {
					motor[lift] = 0;
				}
	}
}
