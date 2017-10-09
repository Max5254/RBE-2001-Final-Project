#include <Arduino.h>
#include <LiquidCrystal.h>
#include <Servo.h>
#include "ReactorControl/Messages.h"
#include "drive.h"
#include "arm.h"
#include "helpers.h"
#include "scheduler.h"


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

const int debugA = 23;
const int debugB = 24;
//Analog Input
const int armPotPort = A2;



//Motors

Drive drive;
Arm arm;
Scheduler scheduler;

///////////
// SETUP //
///////////
void setup() {

  scheduler.build();

  Serial.begin(9600);
  lcd.begin(16,2);
  msg.setup();


  pinMode(startPort,INPUT_PULLUP);
  pinMode(beamBreak,INPUT_PULLUP);
  pinMode(debugA,INPUT_PULLUP);
  pinMode(debugB,INPUT_PULLUP);

  drive.initialize();
  arm.initialize(armPort, gripperPort, armPotPort);

}



void printOdomToLCD(){
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

void printStorageSupplyToLCD(){
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

void printIntakeToLCD(){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(arm.intakeInput);
  lcd.setCursor(0,1);
  lcd.print(arm.intakeOutput);
  lcd.setCursor(8,1);
  lcd.print(arm.Kp_intake);
}

void printArmToLCD(){
  lcd.setCursor(0,0);
  lcd.print(arm.armInput);
  lcd.setCursor(0,1);
  lcd.print(arm.armOutput);
  lcd.setCursor(8,0);
  lcd.print(arm.armSetpoint);
}


///////////////
// MAIN LOOP //
///////////////
void loop() {
  msg.heartbeat();
  msg.PeriodicRadiationStatus(0x01);
  drive.odometry();

  // if (!digitalRead(startPort)) arm.raiseArm();
  // else if (!digitalRead(limitSwitch)) arm.lowerArm();
  // if (!digitalRead(debugA)) arm.grab();
  // else if (!digitalRead(debugB)) arm.release();
  //arm.updateArm(!digitalRead(startPort) || !digitalRead(limitSwitch) || !digitalRead(debugA) || !digitalRead(debugB));

  bool pressed = digitalRead(startPort);

  if(pressed){
    //drive.driveDistance(-20, 0, pressed);
    //drive.turnToAngle(-90, pressed);
    scheduler.run();
  } else {
    drive.arcadeDrive(0,0);
  }
  arm.updateArm(pressed);
  printOdomToLCD();


  // if(!digitalRead(13)){
  //   drive.reset(0,0,0);
  // }

  delay(20);
}
