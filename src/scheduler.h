#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "ReactorControl/Messages.h"
#include "drive.h"
#include "arm.h"
#include <Vector.h>


enum tasks{
  DRIVE_DISTANCE,
  TURN_ANGLE,
  RAISE_ARM,
  LOWER_ARM,
  GRAB,
  RELEASE,
  DRIVE_TO_LINE,
  TURN_TO_LINE,
  DRIVE_TO_BUTTON
};

struct task{
  tasks function;
  double distance;
  double angle;
};


extern Drive drive;
extern Arm arm;
extern Messages messages;

class Scheduler{
public:
  Scheduler();
  void build();
  bool run();

private:

  Vector< task > schedule;


  task makeTask(tasks,double,double);
  task makeRaise();
  task makeLower();
  task makeGrab();
  task makeRelease();
  task makeDriveDistance(double,double);
  task makeTurnAngle(double);
  task makeDriveToLine(double);
  task makeTurnToLine(double);
  task makeDriveToButton(double);

  int i;

};

#endif
