#include "scheduler.h"

Scheduler::Scheduler()
{
  i = 0;
  radLevel = 0;
  startTime = millis();
}

//return true only when a bool has been true for "delay" amount of time
bool Scheduler::booleanDelay(bool latch, unsigned int delay){
  if(!latch){
    lastLatched = millis();
    return false;
  } else {
    return millis() - lastLatched > delay;
  }
}

task Scheduler::makeTask(tasks _function, double _x, double _y) {
  task mytask = {_function, _x, _y};
  return mytask;
}

task Scheduler::makeRaise(){
  return makeTask(RAISE_ARM,0,0);
}

task Scheduler::makeLower(){
  return makeTask(LOWER_ARM,0,0);
}

task Scheduler::makeGrab(){
  return makeTask(GRAB,0,0);
}

task Scheduler::makeRelease(){
  return makeTask(RELEASE,0,0);
}

task Scheduler::makeDriveDistance(double _distance , double _angle){
  return makeTask(DRIVE_DISTANCE , _distance, _angle);
}

task Scheduler::makeTurnAngle(double _angle){
  return makeTask(TURN_ANGLE,0,_angle);
}

task Scheduler::makeDriveToLine(double _distance, double _angle){
  return makeTask(DRIVE_TO_LINE,_distance,_angle);
}

task Scheduler::makeTurnToLine(double _power){
  return makeTask(TURN_TO_LINE,0,_power);
}

task Scheduler::makeDriveToButton(double _angle){
  return makeTask(DRIVE_TO_BUTTON,0,_angle);
}

task Scheduler::makeDriveToPeg(double _x, double _y){
  return makeTask(DRIVE_TO_PEG,_x,_y);
}

task Scheduler::makeDriveToReactor(double _x, double _y){
  return makeTask(DRIVE_TO_REACTOR,_x,_y);
}

task Scheduler::makeresetOdomXY(double _x, double _y){
  return makeTask(RESET_ODOM_XY,_x,_y);
}
task Scheduler::makeResetOdomTheta(double _t){
  return makeTask(RESET_ODOM_THETA,0,_t);
}
task Scheduler::makeDriveToPoint(double _x, double _y){
  return makeTask(DRIVE_TO_POINT,_x,_y);
}

task Scheduler::makeHIGH(){
  return makeTask(HIGH_RAD,0,0);
}
task Scheduler::makeLOW(){
  return makeTask(LOW_RAD,0,0);
}
task Scheduler::makeOFF(){
  return makeTask(OFF_RAD,0,0);
}
int Scheduler::getRadiation(){
  return radLevel;
}

int Scheduler::storageCoords(int holder){
  return (holder * 6 ) - 9;
}

int* Scheduler::storageOrder(){
  bool *storageArray = msg.getStorageAvailability();
  bool *supplyArray = msg.getSupplyAvailability();
  Serial.print("Storage: ");
  Serial.print(storageArray[0]);
  Serial.print(" ");
  Serial.print(storageArray[1]);
  Serial.print(" ");
  Serial.print(storageArray[2]);
  Serial.print(" ");
  Serial.println(storageArray[3]);
  Serial.print("Supply: ");
  Serial.print(supplyArray[0]);
  Serial.print(" ");
  Serial.print(supplyArray[1]);
  Serial.print(" ");
  Serial.print(supplyArray[2]);
  Serial.print(" ");
  Serial.println(supplyArray[3]);

  int order[4] = {-1,-1,-1,-1};
  for(int i = 0; i < 4; i++){
    if(storageArray[3-i] == 0 && order[0] == -1){
      order[0] = storageCoords(i);
    }
    if(supplyArray[3 - i] == 0 && order[1] == -1){
      order[1] = storageCoords(3 - i);
    }
    if(storageArray[i] == 0 && order[2] == -1){
      order[2] = storageCoords(3-i);
    }
    if(supplyArray[i] == 0 && order[3] == -1){
      order[3] = storageCoords(i);
    }
  }
  // if (order[0] == order[2] || order[1] == order[3]){
  //   int temp = order[0];
  //   order[0] = order[2];
  //   order[2] = temp;
  // }
  Serial.println(order[0]);
  return order;
}

void Scheduler::build(){
  int *pathPlan = storageOrder();
  Serial.print(int(pathPlan[0]));
  Serial.print(" ");
  Serial.print(pathPlan[1]);
  Serial.print(" ");
  Serial.print(pathPlan[2]);
  Serial.print(" ");
  Serial.println(pathPlan[3]);

  schedule.push_back(makeGrab());
  schedule.push_back(makeHIGH());
  schedule.push_back(makeRaise());
  schedule.push_back(makeDriveDistance(-5, 0));
  schedule.push_back(makeDriveToPeg(-8, pathPlan[0]));  //old positive
  schedule.push_back(makeRelease());
  schedule.push_back(makeOFF());
  schedule.push_back(makeResetOdomTheta(-90));
  schedule.push_back(makeDriveDistance(-5, -90));

  schedule.push_back(makeDriveToPeg(5, pathPlan[1])); // old 8,3
  schedule.push_back(makeGrab());
  schedule.push_back(makeLOW());
  schedule.push_back(makeResetOdomTheta(90));
  schedule.push_back(makeDriveDistance(-5, 90));

  schedule.push_back(makeLower());
  schedule.push_back(makeDriveToReactor(0, 25));
  schedule.push_back(makeRelease());
  schedule.push_back(makeOFF());

  schedule.push_back(makeRaise());
  schedule.push_back(makeDriveDistance(-5, 0));
  schedule.push_back(makeLower());
  schedule.push_back(makeDriveToPoint(-10, -15));
  schedule.push_back(makeDriveToReactor(0, -35));
  schedule.push_back(makeGrab());


}


bool Scheduler::run(bool enabled){
  if(enabled){
  switch (schedule[i].function) {
    case GRAB:
      if(arm.grab()) {
        i++; }
      break;
    case RELEASE:
      if(arm.release()) {
        i++; }
      break;
    case LOWER_ARM:
      if(arm.lowerArm()) {
        i++; }
      break;
    case RAISE_ARM:
      if(arm.raiseArm()) {
        i++; }
      break;
    case DRIVE_DISTANCE:
      if(drive.driveDistance(schedule[i].distance, schedule[i].angle, true)){
        i++;
        drive.arcadeDrive(0,0); }
      break;
    case TURN_ANGLE:
      if(drive.turnToAngle(schedule[i].angle, true)){
        i++;
        drive.arcadeDrive(0, 0); }
      break;
    case DRIVE_TO_LINE:
        if(drive.driveToLine(schedule[i].distance,schedule[i].angle)){
          i++;
          drive.arcadeDrive(0, 0); }
        break;
    case TURN_TO_LINE:
        if(drive.turnToLine(schedule[i].angle)){
              i++;
              drive.arcadeDrive(0, 0); }
            break;
    case DRIVE_TO_BUTTON:
        if(drive.driveToButton(schedule[i].angle)){
            i++;
            drive.arcadeDrive(0, 0); }
        break;
    case DRIVE_TO_PEG:
      if(drive.driveToPeg(schedule[i].distance,schedule[i].angle)){
        i++;
        drive.arcadeDrive(0, 0); }
        break;
   case DRIVE_TO_REACTOR:
        if(drive.driveToReactor(schedule[i].distance,schedule[i].angle)){
          i++;
          drive.arcadeDrive(0, 0); }
        break;
   case RESET_ODOM_XY:
        drive.reset(schedule[i].distance, schedule[i].angle, drive.getTheta());
        i++;
        break;
  case RESET_ODOM_THETA:
        drive.reset(drive.getX(),drive.getY(), schedule[i].angle);
        i++;
        break;
  case DRIVE_TO_POINT:
        if(drive.driveToPoint(schedule[i].distance, schedule[i].angle)){
        i++; }
        break;
  case HIGH_RAD:
      radLevel = 1;
      i++;
      break;
  case LOW_RAD:
      radLevel = 2;
      i++;
      break;
  case OFF_RAD:
      radLevel = 0;
      i++;
      break;
  }

  if(lastState != schedule[i].function){
    startTime = millis();
  }

  if(startTime + timeoutTime > millis()){
    i++;
  }

} else {
  drive.arcadeDrive(0, 0);
  startTime = millis();
}

  lastState = schedule[i].function;

  lcd.setCursor(2, 0);
  lcd.print(int(schedule[i].function));
  //Serial.print(enabled);
  //Serial.print(" ");
  //Serial.println(schedule[i].function);
  return false;
}
