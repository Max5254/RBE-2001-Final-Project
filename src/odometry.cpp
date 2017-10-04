#include "odometry.h"



Odom::Odom(double startX, double startY, double startTheta){
  x = startX;
  y = startY;
  theta = startTheta;
}

void Odom::setScale(double driveScale, double turnScale){
  this->driveScale = driveScale;
  this->turnScale = turnScale;
}

void Odom::track(long leftCount, long rightCount){
     //Get delta
     leftTicks = leftCount - lastLeft;
     rightTicks = rightCount - lastRight;

     //Save last vals
     lastLeft = leftCount;
     lastRight = rightCount;

     leftIn = leftTicks * driveScale;
     rightIn = rightTicks * driveScale;

     avgIn = (leftIn + rightIn) / 2.0;

    //  Serial.print(leftIn);
    //  Serial.print(" ");
    //  Serial.print(rightIn);
    //  Serial.print(" ");
    //  Serial.print(avgIn);
    //  Serial.print(" ");

     //Get theta
    //theta += (leftTicks - rightTicks) / turnScale;
    theta = (leftCount - rightCount) / turnScale;


    //Wrap theta
    if(theta > 180)
      theta = theta - 360;
    if(theta < -180)
      theta = 360 + theta;

      //Do the odom math
      y += avgIn * cos((theta * 3.14) / 180);
      x += avgIn * sin((theta * 3.14) / 180);

      // Serial.print(theta);
      // Serial.print(" ");
      // Serial.print(cos((theta * 3.14) / 180));
      // Serial.print(" ");
      // Serial.print(sin((theta * 3.14) / 180));
      // Serial.println(" ");
      //x=y=0;
}

void Odom::reset(double newX,double newY, double newTheta){
  x = newX;
  y = newY;
  theta = newTheta;
}

double Odom::getX(){return x;}
double Odom::getY(){return y;}
double Odom::getTheta(){return theta;}
