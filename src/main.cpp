#include <Arduino.h>
#include <LiquidCrystal.h>
#include <Servo.h>
#include "ReactorControl/Messages.h"
#include "Odometry.h"
#include <Encoder.h>
#include "drive.h"

LiquidCrystal lcd(40,41,42,43,44,45);
Messages msg;
Odom odom(0,0,0);

////////
// IO //
////////

//Motors

const int armPort = 8;
const int gripperPort = 9;
//Digital IO
const int leftEncoderA = 18;
const int leftEncoderB = 19;
const int rightEncoderA = 2;
const int rightencoderB = 3;
const int startPort = 22;
const int beamBreakPort = 12;
const int limitSwitchPort = 13;
//Analog Input
const int armPotPort = A2;
const int lineFollowerFrontPort = A1;
const int lineFollowerBackPort = A0;


//Encoders
Encoder leftEncoder(leftEncoderA,leftEncoderB);
Encoder rightEncoder(rightEncoderA,rightencoderB);

//Motors
Servo arm;
Servo gripper;

Drive drive;

/////////
// PID //
/////////

double driveInput, driveOutput, driveSetpoint;
double Kp_drive = 1, Ki_drive = 0, Kd_drive = 0;
//PID drivePID(&driveInput,&driveOutput,&driveSetpoint,1,0,0,DIRECT);



///////////
// SETUP //
///////////
void setup() {
  Serial.begin(9600);
  lcd.begin(16,2);
  msg.setup();
  odom.setScale(0.024, 5.4);
  drive.initialize();

  pinMode(startPort,INPUT_PULLUP);

  leftEncoder.write(0);
  rightEncoder.write(0);


  arm.attach(armPort, 1000, 2000);
  gripper.attach(gripperPort, 1000, 2000);

  //pinMode(reactorSwitchPort, INPUT_PULLUP);
  //pinMode(startPort, INPUT_PULLUP);
}

void printToLCD(){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("X:   Y:   T:");
  //char buffer[16];
  //sprintf(buffer, "%.1f %.1f %.1f", odom.getX(), odom.getY(),odom.getTheta());
  //lcd.print(buffer);
  lcd.setCursor(0, 1);
  lcd.print(odom.getX());
  lcd.setCursor(6, 1);
  lcd.print(odom.getY());
  lcd.setCursor(12, 1);
  lcd.print(odom.getTheta());
}

void printStorageSupplytoLCD(){
lcd.setCursor(0,0);
bool *storageArray = msg.getStorageAvailability();
char buffer[20];
sprintf(buffer,"Storage: %d %d %d %d",storageArray[0],storageArray[1],storageArray[2],storageArray[3]);
lcd.print(buffer);
lcd.setCursor(0,1);
bool *supplyArray = msg.getSupplyAvailability();
sprintf(buffer,"Supply:  %d %d %d %d",supplyArray[0],supplyArray[1],supplyArray[2],supplyArray[3]);
lcd.print(buffer);
}

///////////////
// MAIN LOOP //
///////////////
void loop() {
  msg.heartbeat();
  msg.sendRadiationAlert(0x01);
  odom.track(leftEncoder.read(), rightEncoder.read());
  //printToLCD();

  printStorageSupplytoLCD();



  if(!digitalRead(startPort)){
    leftEncoder.write(0);
    rightEncoder.write(0);
    odom.reset(0,0,0);
  }

  delay(2000);
}
