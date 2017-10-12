#include "drive.h"

Drive::Drive() :
drivePID(&driveInput,&driveOutputDesired,&driveSetpoint,Kp_drive,Ki_drive,Kd_drive,DIRECT),
straightPID(&straightInput,&straightOutput,&straightSetpoint,Kp_straight,Ki_straight,Kd_straight,DIRECT),
turnPID(&turnInput,&turnOutputDesired,&turnSetpoint,Kp_turn,Ki_turn,Kd_turn,DIRECT),
odom(0,0,0)
{ }

void Drive::initialize(){
  pinMode(limitSwitch,INPUT_PULLUP);
  pinMode(beamBreak,INPUT_PULLUP);

  //Init PIDs with output limits, mode, and integration ranges
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

  odom.setScale(0.024, 5.4);  //drive scale , turn scale to map encoder tics to inches and degrees

  odom.reset(0,32,0);  //init robot location to x=0 y =32 theta=0
}

/* drive forward a set distance using PID
*  @param setpoint the target distance to drive in inches
*  @param angle    the target angle to hold to while driving
*  @param enabled  turns the loop on or off
*  @return true when within target encoder range for certain time
*/
bool Drive::driveDistance(double setpoint, double angle, bool enabled){
  if(driveStarting){ //reset if new setpoint
    driveStartingPoint = odom.getAverageEncoder();
    driveOutput = 0;
    drivePID.flush();
    straightPID.flush();
    driveStarting = false;
  }

  //scale input if driving at a -180 degreee angle
  straightInput = odom.getTheta() < -170 ? 180 - abs(odom.getTheta()) + 180  : odom.getTheta();

  driveInput = odom.getAverageEncoder() - driveStartingPoint;
  driveSetpoint = setpoint;
  drivePID.Compute();

  straightSetpoint = angle;
  straightPID.Compute();

  if (drivePID.getError() > 0) { //invert slew gains if setpoint is backwards
    upSlew = driveSlewRate;
    downSlew = driveNegativeSlewRate;
  } else {
    downSlew = driveSlewRate;
    upSlew = driveNegativeSlewRate;
  }

  if(enabled){
    if (driveOutputDesired > driveOutput){ //ramp the motor values up and down to avoid jerky motion and wheel slip
      driveOutput += driveOutputDesired - driveOutput > upSlew ? upSlew : driveOutputDesired - driveOutput;
    } else {
      driveOutput -= driveOutput - driveOutputDesired > downSlew ? downSlew : driveOutput - driveOutputDesired;
    }
  } else {
    driveOutput = 0;
  }

  arcadeDrive(driveOutput , straightOutput);

  Serial.print(abs(drivePID.getError()));
  Serial.print(" ");
  Serial.println(driveTolerance);

  //end condition met if in target rangle for 0.5 seconds
  bool done = booleanDelay(abs(drivePID.getError()) < driveTolerance , 500);

  if(done){ //if done reset for next setpoint
    driveStarting = true;
    driveStartingPoint = odom.getAverageEncoder();
    driveOutput = 0;
  }
  return done;
}

/* drive forward until line is seen within target range
*  @param distance the target distance to drive in inches of where the line should be
*  @param angle    the target angle to hold to while driving
*  @return true when within target encoder range and line is seen
*/
bool Drive::driveToLine(double distance, double angle){
  if(lineStarting){ //reset variables and sets target distance
    lineGoal = odom.getAverageEncoder() + distance;
    lineStarting = false;
  }

  straightInput = getTheta();
  straightSetpoint = angle;
  straightPID.Compute();

  arcadeDrive(0.25, straightOutput); //drive at fixed speed while using PID to go straight

  //returs done when front line sensor sees black within a certain range of encoder values
  bool done = (analogRead(lineFollowerFrontPort) > lineFollowerTolerance) && (abs(lineGoal - odom.getAverageEncoder()) < lineFindingRange);
  if(done){
    lineStarting = true;
  }
  return done;
}

/* drive forward until line is seen within target range
*  @param angle    the target angle to hold to while driving
*  @return true when odometry value shows your near reactor and line is seen
*/
bool Drive::driveToReactorLine(double angle){
  straightInput = getTheta();
  straightSetpoint = angle;
  straightPID.Compute();

  arcadeDrive(0.25, straightOutput);

  return (analogRead(lineFollowerFrontPort) > lineFollowerTolerance) && (abs(odom.getY()) > 15);   //(abs(lineGoal - odom.getAverageEncoder()) < reactorlineFindingRange);
}

/* turn until back line sesnor sees line
*  @param angle    -1 or 1 depending on which way to turn
*  @return true when back line sensor sees a line
*/
bool Drive::turnToLine(double angle){
  arcadeDrive(0, 0.25 * angle);
  return analogRead(lineFollowerBackPort) > lineFollowerTolerance;
}

/* drive forward holding a setpoint angle until button is hit
*  @param angle    the target angle to hold to while driving
*  @return true when button is hit for 1 second
*/
bool Drive::driveToButton(double angle){
  straightInput = getTheta();
  straightSetpoint = angle;
  straightPID.Compute();

  arcadeDrive(0.375, straightOutput);

  return booleanDelay(!digitalRead(limitSwitch),1000);
}

/* drive forward holding a setpoint angle until beam break is triggered
*  @param angle    the target angle to hold to while driving
*  @return true when beam break is triggerd for 1 second
*/
bool Drive::driveToBeamBreak(double angle){
  //change angle to not wrap theta at -180 (PID gets mad because it's driving at the reactor)
  straightInput = odom.getTheta() < -90 ? 180 - abs(odom.getTheta()) + 180  : odom.getTheta();
  straightSetpoint = angle;
  straightPID.Compute();

  arcadeDrive(0.375, straightOutput);

  return booleanDelay(!digitalRead(beamBreak),1000);
}

/* drive to a specific coordinate point
*  @param _x    the target x coordinate
*  @param _y    the target y coordinate
*  @return true when driveDistance PID is at setpoint
*/
bool Drive::driveToPoint(double _x, double _y){
  switch (pointState) {
    case 0: //init all values and calculate distance and angle to point
    pointX = odom.getX() - _x;
    pointY = odom.getY() - _y;
    pointDistance = sqrt(pointX*pointX+pointY*pointY);
    pointAngle =  pointX < 0 ? (_x > 0 ? atan(abs(pointX)/abs(pointY))* 57.2958 : 180 - atan(abs(pointX)/abs(pointY))* 57.2958) : atan(pointX/pointY)* 57.2958 - 180 ;

    Serial.print(_x);
    Serial.print(" ");
    Serial.print(_y);
    Serial.print(" ");
    Serial.print(pointX);
    Serial.print(" ");
    Serial.print(pointY);
    Serial.print(" ");
    Serial.print(pointDistance);
    Serial.print(" ");
    Serial.println(pointAngle);

    pointState++;
    break;
    case 1: //Turn to setpoint angle
    if(turnToAngle(pointAngle, true)) {
      pointState++;
      arcadeDrive(0, 0);
    }
    break;
    case 2: //Drive to setpoint distance
    if(driveDistance(pointDistance, pointAngle, true)){
      pointState++;
      arcadeDrive(0, 0);
    }
    break;
  }
  bool done = pointState > 2;
  if(done){ //reset state for next time
    pointState = 0;
  }
  return done;
}


/* drive to a line at a specific coordinate point and turn into the line till at the wall
*  @param _x    the target x coordinate
*  @param _y    the target y coordinate
*  @return true when the button is hit
*/
bool Drive::driveToPeg(double _x, double _y){
  switch (pegState) {
    case 0: //compute values to drive and turn to get to point
    pegX = odom.getX() - _x;
    pegY = odom.getY() - _y;
    pegDistance = sqrt(pegX*pegX+pegY*pegY);
    pegAngle = atan(pegX/pegY)* 57.2958 + (pegX > 0 ? -180 :180);

    if(_x > 0){
      buttonDirection = 1;
    } else {
      buttonDirection = -1;
    }

    Serial.print(odom.getX());
    Serial.print(" ");
    Serial.print(odom.getY());
    Serial.print(" ");
    Serial.print(_x);
    Serial.print(" ");
    Serial.print(_y);
    Serial.print(" ");
    Serial.print(pegX);
    Serial.print(" ");
    Serial.print(pegY);
    Serial.print(" ");
    Serial.print(pegDistance);
    Serial.print(" ");
    Serial.println(pegAngle);

    pegState++;
    break;
    case 1: //turn to angle to navigate to point
    if(turnToAngle(pegAngle, true)) {
      arcadeDrive(0, 0);
      pegState++;
    }
    break;
    case 2: //drive distance until in range of line
    if(driveToLine(pegDistance, pegAngle)){
      arcadeDrive(0, 0);
      //set angle to turn into line for next case
      lineAngle = (((odom.getTheta() > 0 && odom.getTheta() < 90) || (odom.getTheta() < -90 && odom.getTheta() > -180)) ? 1.0 : -1.0);
      pegState++;

    }
    break;
    case 3: //turn into line
    if(turnToLine(lineAngle)){
      arcadeDrive(0, 0);
      pegState++;
    }
    break;
    case 4: //drive into button while holding heading with PID
    if(driveToButton(90 * buttonDirection)){
      arcadeDrive(0, 0);
      pegState++;
    }
    break;
  }
  bool done = pegState > 4;
  if(done){
    pegState = 0;
  }
  return done;
}

/* drive to a line at a specific coordinate point and turn into the line till at the reactor
*  @param _x    the target x coordinate
*  @param _y    the target y coordinate
*  @return true when the button is hit
*/
bool Drive::driveToReactor(double _x, double _y){
  switch (reactorState) {
    case 0: //compute values to drive and turn to get to point
    reactorX = odom.getX() - _x;
    reactorY = odom.getY() - _y;
    reactorAngle =  reactorX < 0 ? (_y > 0 ? atan(abs(reactorX)/abs(reactorY))* 57.2958 : 180 - atan(abs(reactorX)/abs(reactorY))* 57.2958) : -atan(abs(reactorX)/abs(reactorY))* 57.2958;
    if(reactorY > 0){
      buttonDirection = 1;
    } else {
      buttonDirection = 0;
    }

    Serial.print(_x);
    Serial.print(" ");
    Serial.print(_y);
    Serial.print(" ");
    Serial.print(reactorX);
    Serial.print(" ");
    Serial.print(reactorY);
    Serial.print(" ");
    Serial.print(reactorDistance);
    Serial.print(" ");
    Serial.println(reactorAngle);
    reactorState++;
    break;
    case 1: //turn to angle to navigate to point
    if(turnToAngle(reactorAngle, true)) {
      reactorState++;
      arcadeDrive(0, 0);
    }
    break;
    case 2: //drive until in correct odom range and sees line
    if(driveToReactorLine(reactorAngle)){
      reactorState++;
      arcadeDrive(0, 0);
      //calculate distance to turn into line
      lineAngle = (((odom.getTheta() < 180 && odom.getTheta() > 90) || (odom.getTheta() < 0 && odom.getTheta() > -90)) ? 1.0 : -1.0);
    }
    break;
    case 3: //turn into line
    if(turnToLine(lineAngle)){
      reactorState++;
      arcadeDrive(0, 0);
    }
    break;
    case 4: //drive into beam break holding heading with PID
    if(driveToBeamBreak(187 * buttonDirection)){
      reactorState++;
      arcadeDrive(0, 0);
    }
    break;
  }
  bool done = reactorState > 4;
  if(done){
    reactorState = 0;
  }
  return done;
}

/* turn to specific angle using PID
*  @param angle    the target angle to turn
*  @param enabled  if the loop is running
*  @return true when in range half a second
*/
bool Drive::turnToAngle(double angle, bool enabled){
  turnInput = getTheta();
  turnSetpoint = angle;
  turnPID.Compute();

  if (turnPID.getError() > 0) { //invert slew rates if moving backwards
    upSlew = turnSlewRate;
    downSlew = turnNegativeSlewRate;
  } else {
    downSlew = turnSlewRate;
    upSlew = turnNegativeSlewRate;
  }

  if(enabled){
    if (turnOutputDesired > turnOutput){ //ramp up and down speed of motors
      turnOutput += turnOutputDesired - turnOutput > upSlew ? upSlew : turnOutputDesired - turnOutput;
    } else {
      turnOutput -= turnOutput - turnOutputDesired > downSlew ? downSlew : turnOutput - turnOutputDesired;
    }
  } else {
    turnOutput = 0;
  }

  arcadeDrive(0, turnOutput);

  //return true if in target range for 0.5 secons
  return booleanDelay(abs(turnPID.getError()) < turnTolerance, 500);
}

/* control drive motors
*  @param throttle  the forward/backwards speed
*  @param turn      the turning speed (+ is right)
*/
void Drive::arcadeDrive(double throttle, double turn){
  leftDrive.write(frcToServo(throttle + turn));
  rightDrive.write(frcToServo(turn - throttle));
}

//Converts from -1 to 1 scale to a 0 180 sclase
int Drive::frcToServo(double input){
  return 90 + (input * 90.0);
}

/* returns true when input has been true for certian time
*  @param latch  the value of the condition you want to  
*  @param delay  the time the input must be true to return true
*  @return true when in range for specified time
*/bool Drive::booleanDelay(bool latch, unsigned int delay){
  if(!latch){
    lastLatched = millis();
    return false;
  } else {
    return millis() - lastLatched > delay;
  }
}

//run odometry funciton (odom is private to drive)
void Drive::odometry(){
  odom.track();
}
//reset odom data to new location
void Drive::reset(double newX,double newY, double newTheta){
  odom.reset(newX,newY,newTheta);
}

double Drive::getX(){return odom.getX();}
double Drive::getY(){return odom.getY();}
double Drive::getTheta(){return odom.getTheta();}
