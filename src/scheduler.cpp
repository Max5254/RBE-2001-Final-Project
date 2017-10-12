#include "scheduler.h"

Scheduler::Scheduler()
{
  i = 0;
  radLevel = 0;
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

/*
*   make a new task with the given inputs
*/
task Scheduler::makeTask(tasks _function, double _x, double _y) {
  task mytask = {_function, _x, _y};
  return mytask;
}

/*
*   all functions to make a task for all robot functions
*   will be added to the Scheduler array
*/
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
task Scheduler::makeDriveToBeamBreak(double _angle){
  return makeTask(DRIVE_TO_BEAM_BREAK,0,_angle);
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

//return the Y position for the holder array given
int Scheduler::rodCoords(int holder){
  return (holder * 6 ) - 9;
}

//reads in data from the BT and compute where the robot should drive
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
  Serial.print("Supply:  ");
  Serial.print(supplyArray[0]);
  Serial.print(" ");
  Serial.print(supplyArray[1]);
  Serial.print(" ");
  Serial.print(supplyArray[2]);
  Serial.print(" ");
  Serial.println(supplyArray[3]);

  int order[4] = {-1,-1,-1,-1};
  //loop through storage to find closest full rod to either reactor
  for(int i = 0; i < 4; i++){
    if(storageArray[3-i] == 0 && order[0] == -1){
      order[0] = -rodCoords(i);
    }
    if(storageArray[i] == 0 && order[2] == -1){
      order[2] = -rodCoords(3-i);
    }
  }
  //loop through supply to find closest full rod to either reactor
  for(int i = 0; i < 4; i++){
    if(supplyArray[3-i] == 1 && order[1] == -1){
      order[1] = rodCoords(3-i);
    }
    if(supplyArray[i] == 1 && order[3] == -1){
      order[3] = rodCoords(i);
    }
  }

  //  if either set is moving from storage to opposite new rod
  //  switch them because the robot doesn't like that
  if(order[0] == order[1] || order[2] == order[3]){
    int temp = order[1];
    order[1] = order[3];
    order[3] = temp;
  }

  Serial.print("Out: ");
  Serial.print(order[0]);
  Serial.print(" ");
  Serial.print(order[1]);
  Serial.print(" ");
  Serial.print(order[2]);
  Serial.print(" ");
  Serial.println(order[3]);
  return order;
}

//create the order of events
void Scheduler::build(){
  int *pathPlan = storageOrder(); //build the path from BT
  Serial.print(int(pathPlan[0]));
  Serial.print(" ");
  Serial.print(pathPlan[1]);
  Serial.print(" ");
  Serial.print(pathPlan[2]);
  Serial.print(" ");
  Serial.println(pathPlan[3]);

  // add all the makeXXX() to the array of tasks
  // is executed in sequence later in run()

  lineSub = 4;
  schedule.push_back(makeDriveToBeamBreak(0));
  schedule.push_back(makeGrab());
  schedule.push_back(makeHIGH());
  schedule.push_back(makeRaise());
  schedule.push_back(makeDriveDistance(-8, 0));
  schedule.push_back(makeDriveToPeg(-3, pathPlan[0]));  //old -8,-3
  schedule.push_back(makeRelease());
  schedule.push_back(makeOFF());
  schedule.push_back(makeResetOdomTheta(-90));
  schedule.push_back(makeDriveDistance(-14, -90));

  //Driving to first peg
  lineSub = 7;
  schedule.push_back(makeDriveToPeg(13, pathPlan[1])); // old 5,7
  schedule.push_back(makeGrab());
  schedule.push_back(makeLOW());
  schedule.push_back(makeResetOdomTheta(90));
  schedule.push_back(makeDriveDistance(-5, 90));

  //Driving back to reactor 1
  schedule.push_back(makeLower());
  schedule.push_back(makeDriveToReactor(0, 25));
  schedule.push_back(makeRelease());
  schedule.push_back(makeOFF());

  //Driving to reactor 2
  schedule.push_back(makeDriveDistance(-20, 0));
  schedule.push_back(makeDriveToPoint(-10, -10));
  schedule.push_back(makeDriveToReactor(0, -30));

  // //Drive to deposit 2nd rod in storage
  schedule.push_back(makeGrab());
  schedule.push_back(makeHIGH());
  schedule.push_back(makeRaise());
  schedule.push_back(makeDriveDistance(-8, 180));

  //second half that isn't working yet :(

  // schedule.push_back(makeDriveToPeg(-9, pathPlan[2]));  //old -8,-3
  // schedule.push_back(makeRelease());
  // schedule.push_back(makeOFF());
  // schedule.push_back(makeResetOdomTheta(-90));
  // schedule.push_back(makeDriveDistance(-5, -90));
  //
  // //Driving to pick up last rod
  // schedule.push_back(makeDriveToPeg(3, pathPlan[3])); // old 5,7
  // schedule.push_back(makeGrab());
  // schedule.push_back(makeLOW());
  // schedule.push_back(makeResetOdomTheta(90));
  // schedule.push_back(makeDriveDistance(-5, 90));
  //
  // //Driving back to reactor 1
  // schedule.push_back(makeLower());
  // schedule.push_back(makeDriveToReactor(0, -25));
  // schedule.push_back(makeRelease());
  // schedule.push_back(makeOFF());
  //
  //
  // //Back up
  // schedule.push_back(makeRaise());
  // schedule.push_back(makeDriveDistance(-8, 180));
}

/*
* loop through the generated schedule of events in sequence
*
* allows for order of events to be changed easily by having sequence
* defined seperatly from state machine 
*/
bool Scheduler::run(bool enabled){
  if(enabled){
  switch (schedule[i].function) { //switch depending on type of function for current array element
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
    case DRIVE_TO_BEAM_BREAK:
        if(drive.driveToBeamBreak(schedule[i].angle)){
          i++;
          drive.arcadeDrive(0, 0); }
          break;
    case DRIVE_TO_PEG:
      if(drive.driveToPeg(schedule[i].distance - lineSub,schedule[i].angle)){
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
} else {
  drive.arcadeDrive(0, 0);
}
  lcd.setCursor(2, 0);
  lcd.print(int(schedule[i].function));
  //Serial.print(enabled);
  //Serial.print(" ");
  //Serial.println(schedule[i].function);
  return false;
}
