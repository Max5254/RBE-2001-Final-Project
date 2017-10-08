#include "drive.h"

Drive::Drive() :
 drivePID(&driveInput,&driveOutputDesired,&driveSetpoint,Kp_drive,Ki_drive,Kd_drive,DIRECT),
 straightPID(&straightInput,&straightOutput,&straightSetpoint,Kp_straight,Ki_straight,Kd_straight,DIRECT),
 turnPID(&turnInput,&turnOutputDesired,&turnSetpoint,Kp_turn,Ki_turn,Kd_turn,DIRECT),
 leftEncoder(leftEncoderA,leftEncoderB),
 rightEncoder(rightEncoderA,rightencoderB),
 odom(0,0,0) {

}

void Drive::initialize(){
     drivePID.SetOutputLimits(-0.75,0.75);
     drivePID.SetMode(AUTOMATIC);
     straightPID.SetOutputLimits(-0.5,0.5);
     straightPID.SetMode(AUTOMATIC);
     turnPID.SetOutputLimits(-0.5,0.5);
     turnPID.SetMode(AUTOMATIC);
     turnPID.setIRange(10);

     leftDrive.attach(leftDrivePort, 1000, 2000);
     rightDrive.attach(rightDrivePort, 1000, 2000);

     odom.setScale(0.024, 5.4);

     resetEncoders();
}


bool Drive::driveDistance(double setpoint, bool enabled){

  driveInput = getY();
  driveSetpoint = setpoint;
  drivePID.Compute();

  straightInput = getTheta();
  straightSetpoint = 0;
  straightPID.Compute();

  if(enabled){
  if (driveOutputDesired > driveOutput){
    driveOutput += driveOutputDesired - driveOutput > driveSlewRate ? driveSlewRate : driveOutputDesired - driveOutput;
  } else {
    driveOutput -= driveOutput - driveOutputDesired > driveNegativeSlewRate ? driveNegativeSlewRate : driveOutput - driveOutputDesired;
  }
} else {
  driveOutput = 0;
}
  arcadeDrive(driveOutput , straightOutput);
  // char buff[20];
  // sprintf(buff,"%.1f %.1f",straightInput, straightOutput);
  // Serial.println(buff);
  Serial.print(straightInput);
  Serial.print(" ");
  Serial.println(straightOutput);

  return booleanDelay(getY() < setpoint + driveTolerance && getY() > setpoint - driveTolerance,500);;

}

bool Drive::turnToAngle(double angle, bool enabled){
  turnInput = getTheta();
  turnSetpoint = angle;
  turnPID.Compute();

  if(enabled){
  if (turnOutputDesired > turnOutput){
    turnOutput += turnOutputDesired - turnOutput > turnSlewRate ? turnSlewRate : turnOutputDesired - turnOutput;
  } else {
    turnOutput -= turnOutput - turnOutputDesired > turnNegativeSlewRate ? turnNegativeSlewRate : turnOutput - turnOutputDesired;
  }
  } else {
  turnOutput = 0;
  }

  arcadeDrive(0, turnOutput);
  Serial.print(turnInput);
  Serial.print(" ");
  Serial.print(turnSetpoint);
  Serial.print(" ");
  Serial.println(turnOutput);

  return booleanDelay(abs(turnPID.getError()) < turnTolerance, 500);
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
