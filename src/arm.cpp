#include "arm.h"
#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include "Servo.h"
#include "helpers.h"

#define ARM_UP 85
#define ARM_DOWN -3
#define INTAKE_IN 50 //old 60
#define INTAKE_OUT 980

Adafruit_MMA8451 mma = Adafruit_MMA8451(); //accelerometer

Arm::Arm() :
armPID(&armInput,&armOutput,&armSetpoint,Kp_arm,Ki_arm,Kd_arm,REVERSE),
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

	armPID.setIRange(10);

	if (! mma.begin()) {
    	Serial.println("Couldn't start accelerometer");
  	}
	mma.setRange(MMA8451_RANGE_2_G);
	armInput = getArmAngle();
	intakeInput = analogRead(intakePotPort);

	}

//calculates arm angle using acceleration due to gravity (and tries to ignore the arm moving)
double Arm::getArmAngle(){
	mma.read();
	sensors_event_t event;
	mma.getEvent(&event);
	float ang = 90 - atan(event.acceleration.x / event.acceleration.y) * 57.3 + 2;
	if (ang > 120) ang -= 180;
	return ang;
}

//Update lift control loops (called every loop)
void Arm::updateArm(bool enabled){
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

//Lifts arm - Returns true when target reached
bool Arm::raiseArm(){
	armSetpoint = ARM_UP;
	armActive = !armAtSetpoint();
	return armDelay(armAtSetpoint(),1500);
}
//Lowers arm - Returns true when target reached
bool Arm::lowerArm(){
	armSetpoint = ARM_DOWN;
	armActive = !armAtSetpoint();
	return armDelay(armAtSetpoint(), 1500);
}
//Intakes four bar - Returns true when target reached
bool Arm::grab(){
	intakeSetpoint = INTAKE_IN;
	//Serial.println("Grabbing");
	return booleanDelay(intakeAtSetpoint(), 500);
}
//Releases four bar - Returns true when target reached
bool Arm::release(){
	intakeSetpoint = INTAKE_OUT;
	return booleanDelay(intakeAtSetpoint(), 500);
}

//return true only when a bool has been true for "delay" amount of time
bool Arm::armDelay(bool latch, unsigned long delay){
  if(!latch){
    lastArmLatched = millis();
    return false;
  } else {
    return millis() - lastArmLatched > delay;
  }
}
