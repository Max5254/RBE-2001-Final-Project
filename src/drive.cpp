#include "drive.h"

Drive::Drive() :
 drivePID(&driveInput,&driveOutputDesired,&driveSetpoint,Kp_drive,Ki_drive,Kd_drive,DIRECT),
 straightPID(&straightInput,&straightOutput,&straightSetpoint,Kp_straight,Ki_straight,Kd_straight,DIRECT),
 turnPID(&turnInput,&turnOutputDesired,&turnSetpoint,Kp_turn,Ki_turn,Kd_turn,DIRECT),
 odom(0,0,0)
 { }

void Drive::initialize(){
     pinMode(limitSwitch,INPUT_PULLUP);


     drivePID.SetOutputLimits(-0.75,0.75);
     drivePID.SetMode(AUTOMATIC);
     drivePID.setIRange(0.5);
     straightPID.SetOutputLimits(-0.5,0.5);
     straightPID.SetMode(AUTOMATIC);
     turnPID.SetOutputLimits(-0.5,0.5);
     turnPID.SetMode(AUTOMATIC);
     turnPID.setIRange(10);

     leftDrive.attach(leftDrivePort, 1000, 2000);
     rightDrive.attach(rightDrivePort, 1000, 2000);

     odom.setScale(0.024, 5.4);

     odom.resetEncoders();
}


bool Drive::driveDistance(double setpoint, double angle, bool enabled){
  if(driveStarting){
    driveStartingPoint = odom.getAverageEncoder();
    driveOutput = 0;
    drivePID.flush();
    straightPID.flush();
    driveStarting = false;
  }

  driveInput = odom.getAverageEncoder() - driveStartingPoint;
  driveSetpoint = setpoint;
  drivePID.Compute();

  straightInput = getTheta();
  straightSetpoint = angle;
  straightPID.Compute();

  if (drivePID.getError() > 0) {
     upSlew = driveSlewRate;
     downSlew = driveNegativeSlewRate;
  } else {
    downSlew = driveSlewRate;
    upSlew = driveNegativeSlewRate;
  }

  if(enabled){
  if (driveOutputDesired > driveOutput){
    driveOutput += driveOutputDesired - driveOutput > upSlew ? upSlew : driveOutputDesired - driveOutput;
  } else {
    driveOutput -= driveOutput - driveOutputDesired > downSlew ? downSlew : driveOutput - driveOutputDesired;
  }
} else {
  driveOutput = 0;
}
  arcadeDrive(driveOutput , straightOutput);

  bool done = booleanDelay(abs(drivePID.getError()) < driveTolerance , 500);

  // sprintf(buffer, "IN %f  SET %f\n",driveInput,driveSetpoint );
  // Serial.print(buffer);
  // Serial.print(done);
  // Serial.print(" ");
  // Serial.print(driveInput);
  // Serial.print(" ");
  Serial.println(driveOutput);

  // lcd.setCursor(0,0);
  // lcd.print(driveInput);
  // lcd.setCursor(0,1);
  // lcd.print(driveSetpoint);
  // lcd.setCursor(6,1);
  // lcd.print(driveOutput);

  if(done){
    driveStarting = true;
    driveStartingPoint = odom.getAverageEncoder();
    driveOutput = 0;
  }
  return done;
}

bool Drive::driveToLine(double angle){
  straightInput = getTheta();
  straightSetpoint = angle;
  straightPID.Compute();

  arcadeDrive(0.375, straightOutput);

  return analogRead(lineFollowerFrontPort) > lineFollowerTolerance;
}

bool Drive::turnToLine(bool right){
  arcadeDrive(0, 0.25 * right ? 1.0 : -1.0);

  return analogRead(lineFollowerBackPort) > lineFollowerTolerance;
}

bool Drive::driveToButton(double angle){
  straightInput = getTheta();
  straightSetpoint = angle;
  straightPID.Compute();

  arcadeDrive(0.375, straightOutput);

  return !digitalRead(limitSwitch);
}


bool Drive::turnToAngle(double angle, bool enabled){
  turnInput = getTheta();
  turnSetpoint = angle;
  turnPID.Compute();

  if (turnPID.getError() > 0) {
     upSlew = turnSlewRate;
     downSlew = turnNegativeSlewRate;
  } else {
    downSlew = turnSlewRate;
    upSlew = turnNegativeSlewRate;
  }

  if(enabled){
  if (turnOutputDesired > turnOutput){
    turnOutput += turnOutputDesired - turnOutput > upSlew ? upSlew : turnOutputDesired - turnOutput;
  } else {
    turnOutput -= turnOutput - turnOutputDesired > downSlew ? downSlew : turnOutput - turnOutputDesired;
  }
  } else {
  turnOutput = 0;
  }

  arcadeDrive(0, turnOutput);

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
  odom.track();
}

void Drive::reset(double newX,double newY, double newTheta){
  odom.reset(newX,newY,newTheta);
}

double Drive::getX(){return odom.getX();}
double Drive::getY(){return odom.getY();}
double Drive::getTheta(){return odom.getTheta();}
