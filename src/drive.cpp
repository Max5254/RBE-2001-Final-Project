#include "drive.h"

Drive::Drive() :
 drivePID(&driveInput,&driveOutputDesired,&driveSetpoint,Kp_drive,Ki_drive,Kd_drive,DIRECT),
 straightPID(&straightInput,&straightOutputDesired,&straightSetpoint,Kp_straight,Ki_straight,Kd_straight,DIRECT),
 leftEncoder(leftEncoderA,leftEncoderB),
 rightEncoder(rightEncoderA,rightencoderB),
 odom(0,0,0) {

}

void Drive::initialize(){
     drivePID.SetOutputLimits(-0.75,0.75);
     drivePID.SetMode(AUTOMATIC);
     straightPID.SetOutputLimits(-0.75,0.75);
     straightPID.SetMode(AUTOMATIC);

     leftDrive.attach(leftDrivePort, 1000, 2000);
     rightDrive.attach(rightDrivePort, 1000, 2000);

     odom.setScale(0.024, 5.4);

     resetEncoders();
}


bool Drive::driveDistance(double setpoint){

  driveInput = getY();
  driveSetpoint = setpoint;
  drivePID.Compute();

  if (driveOutputDesired > driveOutput){
    driveOutput += slewRate;
  } else {
    driveOutput -= 0.2;
  }
  arcadeDrive(driveOutput , 0);

  return booleanDelay(getY() < setpoint + driveTolerance && getY() > setpoint - driveTolerance,500);;

}

void Drive::arcadeDrive(double throttle, double turn){
  leftDrive.write(frcToServo(throttle + turn));
  rightDrive.write(frcToServo(turn - throttle));
}

int Drive::frcToServo(double input){ //Converts from -1 to 1 scale to a 0 180 sclase
    return 90 + (input * 90.0);
}

//return true only when a bool has been true for "delay" amount of time
bool Drive::booleanDelay(bool latch, unsigned int delay){
  if(!latch){
    lastLatched = millis();
    return false;
  } else {
    return millis() - lastLatched > delay;
  }
}

void Drive::odometry(){
  odom.track(leftEncoder.read(),rightEncoder.read());
}

void Drive::reset(double newX,double newY, double newTheta){
  odom.reset(newX,newY,newTheta);
  resetEncoders();
}

void Drive::resetEncoders(){
  leftEncoder.write(0);
  rightEncoder.write(0);
}

double Drive::getX(){return odom.getX();}
double Drive::getY(){return odom.getY();}
double Drive::getTheta(){return odom.getTheta();}
