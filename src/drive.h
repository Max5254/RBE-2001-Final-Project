#ifndef DRIVE_H
#define DRIVE_H

#include <Arduino.h>
#include <PID_v1.h>
#include <Servo.h>



class Drive {
public:

  Drive();
  void initialize();
  void arcadeDrive(double,double);
  bool turnToAngle(double);
  bool driveDistance(double,double);
  void driveToPoint(double,double,double);
  void driveToLineAtPoint(double,double);
  void driveToButton();


private:

  double driveTolerance = 1;

  int frcToServo(double);
  bool booleanDelay(bool, unsigned int);
  unsigned int lastLatched;


  const int leftDrivePort = 11;
  const int rightDrivePort = 10;

  Servo leftDrive;
  Servo rightDrive;


  double driveInput, driveOutput, driveSetpoint;
  double Kp_drive = 0.15, Ki_drive = 0, Kd_drive = 0.01;
  PID drivePID;

};

#endif
