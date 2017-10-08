#ifndef arm_h
#define arm_h

#include <PID_v1.h>
#include "Servo.h"

// double armInput, armOutput, armSetpoint;
// PID armPID(&armInput,&armOutput,&armSetpoint,Kp_arm,Ki_arm,Kd_arm,false);

class Arm {
public:
	Arm();
	void initialize(int _armPort, int _intakePort, int _intakePotPort);
	void updateArm(bool enabled);
	bool raiseArm();
	bool lowerArm();
	bool armAtSetpoint();
	bool intakeAtSetpoint();
	bool grab();
	bool release();
//private:
	double getArmAngle();
	double Kp_arm = .025, Ki_arm = 0.01, Kd_arm = 0.001, armThreshold = 4;
	double Kp_intake = .013, Ki_intake = 0, Kd_intake = 0.001, intakeThreshold = 20;
	double armInput, armOutput, armSetpoint;
	double intakeInput, intakeOutput, intakeSetpoint;
	int armPort, intakePort, intakePotPort;
	bool armActive;
	PID armPID;
	PID intakePID;
	Servo armMotor;
	Servo intakeMotor;
};

#endif
