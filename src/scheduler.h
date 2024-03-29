#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "ReactorControl/Messages.h"
#include "drive.h"
#include "arm.h"
#include <Vector.h>


enum tasks{ //all possible robot states
  DRIVE_DISTANCE,
  TURN_ANGLE,
  RAISE_ARM,
  LOWER_ARM,
  GRAB,
  RELEASE,
  DRIVE_TO_LINE,
  TURN_TO_LINE,
  DRIVE_TO_BUTTON,
  DRIVE_TO_BEAM_BREAK,
  DRIVE_TO_PEG,
  DRIVE_TO_REACTOR,
  DRIVE_TO_PEG_REVERSED,
  DRIVE_TO_REACTOR_REVERSED,
  RESET_ODOM_XY,
  RESET_ODOM_THETA,
  DRIVE_TO_POINT,
  HIGH_RAD,
  LOW_RAD,
  OFF_RAD
};

//structure to hold desired function and their parameters
//used for the array of tasks
struct task{
  tasks function;
  double distance;
  double angle;
};

//link to objects in main.cpp
extern Drive drive;
extern Arm arm;
extern Messages msg;

class Scheduler{
public:
  Scheduler();
  void build();
  bool run(bool);
  int getRadiation();

private:

  char radLevel;

  //create a dynamic array of task to hold the desired robot states
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
  task makeDriveToBeamBreak(double);
  task makeDriveToPeg(double,double);
  task makeDriveToReactor(double,double);
  task makeDriveToPegReversed(double,double);
  task makeDriveToReactorReversed(double,double);
  task makeresetOdomXY(double,double);
  task makeResetOdomTheta(double);

  int* storageOrder();
  int rodCoords(int);

  task makeHIGH();
  task makeLOW();
  task makeOFF();

  int i;
  tasks lastState;
  int lineSub;

};

#endif
