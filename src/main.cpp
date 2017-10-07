#include <Arduino.h>
#include <LiquidCrystal.h>
#include <Servo.h>
#include "ReactorControl/Messages.h"
#include "drive.h"
#include "arm.h"
#include "helpers.h"


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

Drive drive;
Arm arm;

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

  drive.initialize();
  arm.initialize(armPort, gripperPort, armPotPort);

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
  if (!digitalRead(startPort)) arm.grab();
  else if (!digitalRead(limitSwitch)) arm.release();
  arm.updateArm(!digitalRead(startPort) || !digitalRead(limitSwitch));
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(arm.intakeInput);
  lcd.setCursor(0,1);
  lcd.print(arm.intakeOutput);
  lcd.setCursor(8,1);
  lcd.print(arm.Kp_intake);
  //arm.write(90);
  //gripper.write(90);

  msg.heartbeat();
  //msg.sendRadiationAlert(0x01);
  drive.odometry();
  // printToLCD();

  //printStorageSupplytoLCD();

if(digitalRead(startPort) && false){
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
