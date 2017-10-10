#include "scheduler.h"

Scheduler::Scheduler()
{
  i = 0;
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

void Scheduler::build(){
  // schedule.push_back(makeGrab());
  // schedule.push_back(makeRaise());
  // schedule.push_back(makeDriveDistance(-8, 0));
  // schedule.push_back(makeDriveToPeg(-8, 3));
  // schedule.push_back(makeRelease());

  schedule.push_back(makeRelease());
  schedule.push_back(makeRaise());
  schedule.push_back(makeDriveDistance(-5, 0));
  schedule.push_back(makeDriveToPeg(-8, -3));  //old positive
  schedule.push_back(makeGrab());
  schedule.push_back(makeResetOdomTheta(-90));
  schedule.push_back(makeDriveDistance(-5, -90));
  schedule.push_back(makeLower());
  schedule.push_back(makeDriveToReactor(0, 30));
  schedule.push_back(makeRelease());



  // schedule.push_back(makeTurnAngle(-160));
  // schedule.push_back(makeDriveDistance(5, -160));
  // schedule.push_back(makeDriveToLine(10,-160));
  // schedule.push_back(makeTurnToLine(0.5));
  // schedule.push_back(makeDriveToButton(-90));
  // schedule.push_back(makeRelease());
  // schedule.push_back(makeDriveDistance(-10, -90));
  // schedule.push_back(makeTurnAngle(93));
  // schedule.push_back(makeDriveToButton(93));
  // schedule.push_back(makeGrab());

  // schedule.push_back(makeDriveDistance(-30, 0));
  // schedule.push_back(makeTurnAngle(-90));
  // schedule.push_back(makeDriveDistance(16, -90));
  // schedule.push_back(makeRelease());
  // schedule.push_back(makeDriveDistance(-16, -90));
  // schedule.push_back(makeLower());
}


bool Scheduler::run(){
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
  }
  //Serial.println(schedule[i].function);

  return false;
}
