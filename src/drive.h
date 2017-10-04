<<<<<<< HEAD
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
=======
#ifndef drive_h
#define drive_h


double driveInput, driveOutput, driveSetpoint;
double Kp_drive = 1, Ki_drive = 0, Kd_drive = 0;
PID drivePID(&driveInput,&driveOutput,&driveSetpoint,Kp_drive,Ki_drive,Kd_drive,false);

double straightInput, straightOutput, straightSetpoint;
double Kp_straight = 1, Ki_straight = 0, Kd_straight = 0;
PID straightPID(&straightInput,&straightOutput,&straightSetpoint,Kp_straight,Ki_straight,Kd_straight,false);

double turnInput, turnOutput, turnSetpoint;
double Kp_turn = 1, Ki_turn = 0, Kd_turn = 0;
PID turnPID(&turnInput,&turnOutput,&turnSetpoint,Kp_turn,Ki_turn,Kd_turn,false);

void turnToAngle(double);
void driveToPoint(double,double,double);
void driveToLineAtPoint(double,double);
void driveToButton();

#endif
>>>>>>> origin/master
