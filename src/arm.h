#ifndef arm_h
#define arm_h

#include <PID_v1.h>
#include "Servo.h"

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
	bool armDelay(bool, unsigned int);
//private:
	double getArmAngle();
	double Kp_arm = .024, Ki_arm = 0.01, Kd_arm = 0.001, armThreshold = 4; //old p=0.025, old
	double Kp_intake = .012, Ki_intake = 0, Kd_intake = 0.0005, intakeThreshold = 25; //old p=0.012 d=0.001, old threshold = 20
	double armInput, armOutput, armSetpoint;
	double intakeInput, intakeOutput, intakeSetpoint;
	int armPort, intakePort, intakePotPort;
	bool armActive;
	PID armPID;
	PID intakePID;
	Servo armMotor;
	Servo intakeMotor;

	unsigned int lastArmLatched;

};

#endif
