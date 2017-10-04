#include "drive.h"

Drive::Drive() :
 drivePID(&driveInput,&driveOutput,&driveSetpoint,Kp_drive,Ki_drive,Kd_drive,DIRECT) {
}

void Drive::initialize(){
  drivePID.SetOutputLimits(-1,1);
  drivePID.SetMode(AUTOMATIC);

  leftDrive.attach(leftDrivePort, 1000, 2000);
  rightDrive.attach(rightDrivePort, 1000, 2000);
}

bool Drive::driveDistance(double avgEncoder, double setpoint){

  driveInput = avgEncoder;
  driveSetpoint = setpoint;
  drivePID.Compute();

  arcadeDrive(driveOutput , 0);

  return booleanDelay(avgEncoder < setpoint + driveTolerance && avgEncoder > setpoint - driveTolerance,100);;

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
