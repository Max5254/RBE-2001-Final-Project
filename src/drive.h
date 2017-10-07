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
  bool turnToAngle(double);
  bool driveDistance(double);
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

  double slewRate = 0.05;
  double driveInput, driveOutputDesired, driveOutput, driveSetpoint;
  double Kp_drive = 0.18, Ki_drive = 0, Kd_drive = 0.02;
  PID drivePID;

  double straightInput, straightOutputDesired, straightOutput, straightSetpoint;
  double Kp_straight = 0.1, Ki_straight = 0, Kd_straight = 0.0;
  PID straightPID;

};

#endif
