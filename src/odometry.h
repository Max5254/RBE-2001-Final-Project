/*

*/

#ifndef odometry_h
#define odometry_h

#include "Arduino.h"

class Odom
{
public:
  //Odom(Encoder,Encoder,double,double,double);
  Odom(double,double,double);
  void setScale(double,double);
  void track(long,long);

  void reset(double,double,double);
  double getX();
  double getY();
  double getTheta();


private:

  double x, y, theta;
  double driveScale, turnScale;
  long lastLeft, lastRight, leftTicks, rightTicks;
  float leftIn, rightIn, avgIn;

};

#endif
