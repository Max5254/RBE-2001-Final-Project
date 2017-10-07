#include <Arduino.h>
#include <LiquidCrystal.h>
#include <Servo.h>
#include "ReactorControl/Messages.h"
#include "drive.h"

LiquidCrystal lcd(40,41,42,43,44,45);
Messages msg;

////////
// IO //
////////

//Motors

const int armPort = 8;
const int gripperPort = 9;
//Digital IO

const int startPort = 22;
const int beamBreak = 12;
const int limitSwitch = 13;
//Analog Input
const int armPotPort = A2;
const int lineFollowerFrontPort = A1;
const int lineFollowerBackPort = A0;


//Motors
Servo arm;
Servo gripper;

Drive drive;

///////////
// SETUP //
///////////
void setup() {
  Serial.begin(9600);
  lcd.begin(16,2);
  msg.setup();


  pinMode(startPort,INPUT_PULLUP);
  pinMode(beamBreak,INPUT_PULLUP);
  pinMode(limitSwitch,INPUT_PULLUP);

  arm.attach(armPort, 1000, 2000);
  gripper.attach(gripperPort, 1000, 2000);
  drive.initialize();

}

void printToLCD(){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("X:   Y:   T:");
  //char buffer[16];
  //sprintf(buffer, "%.1f %.1f %.1f", odom.getX(), odom.getY(),odom.getTheta());
  //lcd.print(buffer);
  lcd.setCursor(0, 1);
  lcd.print(drive.getX());
  lcd.setCursor(6, 1);
  lcd.print(drive.getY());
  lcd.setCursor(12, 1);
  lcd.print(drive.getTheta());
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
  arm.write(90);
  gripper.write(90);

  msg.heartbeat();
  //msg.sendRadiationAlert(0x01);
  drive.odometry();
  printToLCD();

  //printStorageSupplytoLCD();

if(digitalRead(startPort)){
  if(drive.driveDistance(12)){
      drive.arcadeDrive(0, 0);
    }
  //drive.arcadeDrive(0.75, 0);
  } else {
    drive.arcadeDrive(0, 0);
  }

  //drive.arcadeDrive(0, 0);

  if(!digitalRead(limitSwitch)){
    drive.reset(0,0,0);
  }

  delay(20);
}
