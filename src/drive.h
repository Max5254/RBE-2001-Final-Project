#ifndef DRIVE_H
#define DRIVE_H

#include <Arduino.h>
#include <PID_v1.h>
#include <Servo.h>
#include "Odometry.h"
#include "Encoder.h"



class Drive {
public:

  Drive();
  void initialize();
  void arcadeDrive(double,double);
  bool turnToAngle(double,bool);
  bool driveDistance(double,bool);
  void driveToPoint(double,double,double);
  void driveToLineAtPoint(double,double);
  void driveToButton();

  void odometry();
  void reset(double,double,double);
  double getX();
  double getY();
  double getTheta();


private:

  Odom odom;

  //Encoders
  const int leftEncoderA = 18;
  const int leftEncoderB = 19;
  const int rightEncoderA = 2;
  const int rightencoderB = 3;
  Encoder leftEncoder;
  Encoder rightEncoder;


  double driveTolerance = 0.125;

  int frcToServo(double);
  bool booleanDelay(bool, unsigned int);
  void resetEncoders();
  unsigned int lastLatched;

  const int leftDrivePort = 11;
  const int rightDrivePort = 10;

  Servo leftDrive;
  Servo rightDrive;

  double driveSlewRate = 0.05;
  double driveNegativeSlewRate = 0.2;

  double driveInput, driveOutputDesired, driveOutput, driveSetpoint;
  double Kp_drive = 0.07, Ki_drive = 0.0, Kd_drive = 0.01;
  PID drivePID;

  double straightInput, straightOutputDesired, straightOutput, straightSetpoint;
  double Kp_straight = 0.015, Ki_straight = 0, Kd_straight = 0.0;
  PID straightPID;

  double turnSlewRate = 0.05;
  double turnNegativeSlewRate = 0.2;

  double turnTolerance = 2;
  double turnInput, turnOutputDesired, turnOutput, turnSetpoint;
  double Kp_turn = 0.003, Ki_turn = 0.002, Kd_turn = 0.0;
  PID turnPID;

};

#endif
