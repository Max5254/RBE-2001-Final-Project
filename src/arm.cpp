#include "arm.h"
#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include "Servo.h"
#include "helpers.h"

#define ARM_UP 90
#define ARM_DOWN 0
#define INTAKE_IN 65
#define INTAKE_OUT 980

Adafruit_MMA8451 mma = Adafruit_MMA8451();

Arm::Arm() :
armPID(&armInput,&armOutput,&armSetpoint,Kp_arm,Ki_arm,Kd_arm,DIRECT),
intakePID(&intakeInput,&intakeOutput,&intakeSetpoint,Kp_intake,Ki_intake,Kd_intake,DIRECT){
}

void Arm::initialize(int _armPort, int _intakePort, int _intakePotPort){
	armPort = _armPort;
	intakePort = _intakePort;
	intakePotPort = _intakePotPort;
	armSetpoint = ARM_UP;
	intakeSetpoint = INTAKE_OUT;

	armMotor.attach(armPort, 1000, 2000);
	intakeMotor.attach(intakePort, 1000, 2000);
	armMotor.write(90);
	intakeMotor.write(90);

	armPID.SetOutputLimits(-1,1);
	intakePID.SetOutputLimits(-1,1);

	armPID.SetMode(AUTOMATIC);
	intakePID.SetMode(AUTOMATIC);

	if (! mma.begin()) {
    	Serial.println("Couldn't start accelerometer");
  	}
	mma.setRange(MMA8451_RANGE_2_G);
	armInput = getArmAngle();
	intakeInput = analogRead(intakePotPort);
	// armPID = PID(&armInput,&armOutput,&armSetpoint,Kp_arm,Ki_arm,Kd_arm,AUTOMATIC);
	// intakePID = PID(&intakeInput,&intakeOutput,&intakeSetpoint,Kp_intake,Ki_intake,Kd_intake,AUTOMATIC);
}

double Arm::getArmAngle(){
	mma.read();
	sensors_event_t event;
	mma.getEvent(&event);
	float ang = 90 - atan(event.acceleration.x / event.acceleration.y) * 57.3;
	if (ang < -50) ang += 180;
	return ang;
}

void Arm::updateArm(bool enabled){
	//Update lift control loops
	armInput = getArmAngle();
	armPID.Compute();
	if (armActive  && enabled) {
		armMotor.write(frcToServo(armOutput));
	}
	else {
		armMotor.write(90);
	}


	//Update intake contol loops
	intakeInput = analogRead(intakePotPort);
	intakePID.Compute();
	if (enabled) {
		intakeMotor.write(frcToServo(intakeOutput));
	}
	else {
		intakeMotor.write(90);
	}
}

bool Arm::intakeAtSetpoint(){
	return abs(intakeInput - intakeSetpoint) < intakeThreshold;
}

bool Arm::armAtSetpoint(){
	return abs(armInput - armSetpoint) < armThreshold;
}

bool Arm::raiseArm(){
	//Lifts arm
	//Returns true when target reached
	armSetpoint = ARM_UP;
	armActive = !armAtSetpoint();
	return armAtSetpoint();
}

bool Arm::lowerArm(){
	armSetpoint = ARM_UP;
	armActive = !armAtSetpoint();
	return armAtSetpoint();
}

bool Arm::grab(){
	intakeSetpoint = INTAKE_IN;
	return intakeAtSetpoint();
}

bool Arm::release(){
	intakeSetpoint = INTAKE_OUT;
	return intakeAtSetpoint();
}