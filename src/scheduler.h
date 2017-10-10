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
  DRIVE_TO_BUTTON,
  DRIVE_TO_PEG,
  DRIVE_TO_REACTOR,
  RESET_ODOM_XY,
  RESET_ODOM_THETA,
  DRIVE_TO_POINT,
  HIGH_RAD,
  LOW_RAD,
  OFF_RAD
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
  bool run(bool);

  int getRadiation();

private:

  char radLevel;

  Vector< task > schedule;

  bool booleanDelay(bool,unsigned int);
  unsigned int lastLatched;

  task makeTask(tasks,double,double);
  task makeRaise();
  task makeLower();
  task makeGrab();
  task makeRelease();
  task makeDriveDistance(double,double);
  task makeTurnAngle(double);
  task makeDriveToLine(double,double);
  task makeDriveToPoint(double,double);
  task makeTurnToLine(double);
  task makeDriveToButton(double);
  task makeDriveToPeg(double,double);
  task makeDriveToReactor(double,double);
  task makeresetOdomXY(double,double);
  task makeResetOdomTheta(double);

  task makeHIGH();
  task makeLOW();
  task makeOFF();


  int i;

};

#endif
