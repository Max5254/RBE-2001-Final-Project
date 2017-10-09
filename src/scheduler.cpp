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

void Scheduler::build(){
  schedule.push_back(makeGrab());
  schedule.push_back(makeRaise());
  schedule.push_back(makeDriveDistance(-20, 0));
  schedule.push_back(makeTurnAngle(-130));
  schedule.push_back(makeRelease());
  schedule.push_back(makeLower());
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
  }
  return false;
}
