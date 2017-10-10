#ifndef DRIVE_H
#define DRIVE_H

#include <Arduino.h>
#include "PID_v1.h"
#include <Servo.h>
#include "Odometry.h"
#include <LiquidCrystal.h>



extern LiquidCrystal lcd;

class Drive {
public:

  Drive();
  void initialize();
  void arcadeDrive(double,double);
  bool turnToAngle(double,bool);
  bool driveDistance(double,double,bool);
  bool driveToLine(double, double);
  bool driveToReactorLine(double, double);
  bool turnToLine(double);
  bool driveToButton(double);
  bool driveToBeamBreak(double);
  bool driveToPoint(double,double);
  bool driveToPeg(double,double);
  bool driveToReactor(double,double);
  void odometry();
  void reset(double,double,double);
  double getX();
  double getY();
  double getTheta();




private:

  Odom odom;

  const int limitSwitch = 13;
  const int beamBreak = 12;


  double driveTolerance = 0.25;

  int frcToServo(double);
  bool booleanDelay(bool, unsigned int);
  unsigned int lastLatched;

  const int leftDrivePort = 11;
  const int rightDrivePort = 10;
  const int lineFollowerFrontPort = A1;
  const int lineFollowerBackPort = A0;

  const int lineFollowerTolerance = 500;

  Servo leftDrive;
  Servo rightDrive;

  double driveSlewRate = 0.01;
  double driveNegativeSlewRate = 0.1;
  double upSlew,downSlew;
  long driveStartingPoint;
  bool driveStarting = true;
  double driveInput, driveOutputDesired, driveOutput, driveSetpoint;
  double Kp_drive = 0.12, Ki_drive = 0.003, Kd_drive = 0.01; //.115 .0025
  PID drivePID;

  double straightInput, straightOutputDesired, straightOutput, straightSetpoint;
  double Kp_straight = 0.015, Ki_straight = 0, Kd_straight = 0.0;
  PID straightPID;

  double turnSlewRate = 0.01;
  double turnNegativeSlewRate = 0.5;
  double turnTolerance = 1.5;
  double turnInput, turnOutputDesired, turnOutput, turnSetpoint;
  double Kp_turn = 0.014, Ki_turn = 0.02, Kd_turn = 0.003; //old p=0.015, old d=0.001-->0.002
  PID turnPID;

  double lineGoal;
  double lineFindingRange = 4;
  double reactorlineFindingRange = 10;
  bool lineStarting = true;

  int pegState = 0;
  bool pegStarting = true;

  double pegX, pegY, pegDistance, pegAngle,lineAngle;
  int buttonDirection;

  int reactorState = 0;
  double reactorX, reactorY, reactorDistance, reactorAngle;
  int bbDirection;

  int pointState = 0;
  double pointX, pointY, pointDistance, pointAngle;

};

#endif
