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

     odom.reset(0,32,0);  //old 40
}


bool Drive::driveDistance(double setpoint, double angle, bool enabled){
  if(driveStarting){
    driveStartingPoint = odom.getAverageEncoder();
    driveOutput = 0;
    drivePID.flush();
    straightPID.flush();
    driveStarting = false;
  }

  straightInput = odom.getTheta() < -170 ? 180 - abs(odom.getTheta()) + 180  : odom.getTheta();



  driveInput = odom.getAverageEncoder() - driveStartingPoint;
  driveSetpoint = setpoint;
  drivePID.Compute();

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

  Serial.print(abs(drivePID.getError()));
  Serial.print(" ");
  Serial.println(driveTolerance);

  bool done = booleanDelay(abs(drivePID.getError()) < driveTolerance , 500);

  // sprintf(buffer, "IN %f  SET %f\n",driveInput,driveSetpoint );
  // Serial.print(buffer);
  // Serial.print(done);
  // Serial.print(" ");
  // Serial.print(driveInput);
  // Serial.print(" ");
  //Serial.println(driveOutput);

  if(done){
    driveStarting = true;
    driveStartingPoint = odom.getAverageEncoder();
    driveOutput = 0;
  }
  return done;
}

bool Drive::driveToLine(double distance, double angle){
  if(lineStarting){
    lineGoal = odom.getAverageEncoder() + distance;
    lineStarting = false;
  }

  // Serial.print(lineGoal);
  // Serial.print(" ");
  // Serial.println(odom.getAverageEncoder());
  straightInput = getTheta();
  straightSetpoint = angle;
  straightPID.Compute();

  arcadeDrive(0.25, straightOutput);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(abs(lineGoal - odom.getAverageEncoder()));
  lcd.setCursor(0, 1);
  lcd.print(analogRead(lineFollowerFrontPort) > lineFollowerTolerance);

  bool done = (analogRead(lineFollowerFrontPort) > lineFollowerTolerance) && (abs(lineGoal - odom.getAverageEncoder()) < lineFindingRange);
  if(done){
    lineStarting = true;
  }

  return done;
}

bool Drive::driveToReactorLine(double distance, double angle){
  if(lineStarting){
    lineGoal = odom.getAverageEncoder() + distance;
    lineStarting = false;
  }

  // Serial.print(lineGoal);
  // Serial.print(" ");
  // Serial.println(odom.getAverageEncoder());
  straightInput = getTheta();
  straightSetpoint = angle;
  straightPID.Compute();

  arcadeDrive(0.25, straightOutput);

  bool done = (analogRead(lineFollowerFrontPort) > lineFollowerTolerance) && (abs(odom.getY()) > 15);   //(abs(lineGoal - odom.getAverageEncoder()) < reactorlineFindingRange);
  if(done){
    lineStarting = true;
  }

  return done;
}


bool Drive::turnToLine(double angle){
  arcadeDrive(0, 0.25 * angle);

  return analogRead(lineFollowerBackPort) > lineFollowerTolerance;

}

bool Drive::driveToButton(double angle){
  straightInput = getTheta();
  straightSetpoint = angle;
  straightPID.Compute();

  arcadeDrive(0.375, straightOutput);

  return booleanDelay(!digitalRead(limitSwitch),1000);
}

bool Drive::driveToBeamBreak(double angle){
  straightInput = odom.getTheta() < -90 ? 180 - abs(odom.getTheta()) + 180  : odom.getTheta();

  straightSetpoint = angle;

  // Serial.print(straightInput);
  // Serial.print(" ");
  // Serial.println(straightSetpoint);

  straightPID.Compute();

  arcadeDrive(0.375, straightOutput);

  return booleanDelay(!digitalRead(beamBreak),1000);
}

// bool Drive::driveToPoint(double _x, double _y){
//   switch (pointState) {
//     case 0:
//        pointX = odom.getX() - _x;
//        pointY = odom.getY() - _y;
//        pointDistance = sqrt(pointX*pointX+pointY*pointY);
//        //pointAngle =  pointX < 0 ? (_x > 0 ? atan(abs(pointX)/abs(pointY))* 57.2958 : 180 - atan(abs(pointX)/abs(pointY))* 57.2958) : atan(pointX/pointY)* 57.2958 - 180 ; //old - 180 at end
//        pointAngle = atan(pointX/pointY)* 57.2958 + (pointX > 0 ? -180 :180);
//
//        Serial.print(odom.getX());
//        Serial.print(" ");
//        Serial.print(odom.getY());
//        Serial.print(" ");
//       Serial.print(_x);
//       Serial.print(" ");
//       Serial.print(_y);
//       Serial.print(" ");
//        Serial.print(pointX);
//        Serial.print(" ");
//        Serial.print(pointY);
//        Serial.print(" ");
//        Serial.print(pointDistance);
//        Serial.print(" ");
//        Serial.println(pointAngle);
//       pointState++;
//       break;
//     case 1:
//     if(turnToAngle(pointAngle, true)) {
//       pointState++;
//       arcadeDrive(0, 0);
//     }
//     break;
//     case 2:
//     if(driveDistance(pointDistance, pointAngle, true)){
//       pointState++;
//       arcadeDrive(0, 0);
//     }
//     break;
//   }
//     bool done = pointState > 2;
//     if(done){
//       pointState = 0;
//     }
//     //lcd.setCursor(15, 0);
//     //lcd.print(pointState);
//
//     return done;
//   }

bool Drive::driveToPoint(double _x, double _y){
  switch (pointState) {
    case 0:
       pointX = odom.getX() - _x;
       pointY = odom.getY() - _y;
       pointDistance = sqrt(pointX*pointX+pointY*pointY);
       pointAngle =  pointX < 0 ? (_x > 0 ? atan(abs(pointX)/abs(pointY))* 57.2958 : 180 - atan(abs(pointX)/abs(pointY))* 57.2958) : atan(pointX/pointY)* 57.2958 - 180 ; //old - 180 at end

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
    case 1:
    if(turnToAngle(pointAngle, true)) {
      pointState++;
      arcadeDrive(0, 0);
    }
    break;
    case 2:
    if(driveDistance(pointDistance, pointAngle, true)){
      pointState++;
      arcadeDrive(0, 0);
    }
    break;
  }
    bool done = pointState > 2;
    if(done){
      pointState = 0;
    }
    lcd.setCursor(15, 0);
    lcd.print(pointState);

    return done;
  }



bool Drive::driveToPeg(double _x, double _y){
  switch (pegState) {
    case 0:
       pegX = odom.getX() - _x;
       pegY = odom.getY() - _y;
       pegDistance = sqrt(pegX*pegX+pegY*pegY);
       //pegAngle =  pegX < 0 ? (_x > 0 ? atan(abs(pegX)/abs(pegY))* 57.2958 : 180 - atan(abs(pegX)/abs(pegY))* 57.2958) : atan(pegX/pegY)* 57.2958 - 180 ; //old - 180 at end
       //pegAngle =  pegX < 0 ? 180 - atan(abs(pegX)/abs(pegY))* 57.2958 : atan(pegX/pegY)* 57.2958 - 180 ; //old - 180 at end
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

      // lcd.setCursor(0, 0);
      // lcd.print("                      ");
      // lcd.setCursor(0, 0);
      // lcd.print(_x);
      // lcd.setCursor(6, 0);
      // lcd.print(_y);
      pegState++;
      break;
    case 1:
    if(turnToAngle(pegAngle, true)) {
      pegState++;
      arcadeDrive(0, 0);
    }
    break;
    case 2:
    if(driveToLine(pegDistance, pegAngle)){
      pegState++;
      arcadeDrive(0, 0);
      lineAngle = (((odom.getTheta() > 0 && odom.getTheta() < 90) || (odom.getTheta() < -90 && odom.getTheta() > -180)) ? 1.0 : -1.0);
    }
    break;
    case 3:
    if(turnToLine(lineAngle)){
      pegState++;
      arcadeDrive(0, 0);
    }
    break;
    case 4:
    if(driveToButton(90 * buttonDirection)){
      pegState++;
      arcadeDrive(0, 0);
    }
    break;
  }
    bool done = pegState > 4;
    if(done){
      pegState = 0;
    }
    lcd.setCursor(15, 0);
    lcd.print(pegState);

    return done;
  }


  bool Drive::driveToReactor(double _x, double _y){
    switch (reactorState) {
      case 0:
         reactorX = odom.getX() - _x;
         reactorY = odom.getY() - _y;
         reactorDistance = sqrt(reactorX*reactorX+reactorY*reactorY) - 10;
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
      case 1:
      if(turnToAngle(reactorAngle, true)) {
        reactorState++;
        arcadeDrive(0, 0);
      }
      break;
      case 2:
      if(driveToReactorLine(reactorDistance, reactorAngle)){
        reactorState++;
        arcadeDrive(0, 0);
        lineAngle = (((odom.getTheta() < 180 && odom.getTheta() > 90) || (odom.getTheta() < 0 && odom.getTheta() > -90)) ? 1.0 : -1.0);
      }
      break;
      case 3:
      if(turnToLine(lineAngle)){
        reactorState++;
        arcadeDrive(0, 0);
      }
      break;
      case 4:
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
      lcd.setCursor(15, 0);
      lcd.print(reactorState);

      return done;
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
