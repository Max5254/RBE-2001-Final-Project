#include <Arduino.h>
#include <LiquidCrystal.h>
#include <Servo.h>
#include "ReactorControl/Messages.h"
#include "drive.h"
#include "arm.h"
#include "helpers.h"
#include <Vector.h>



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

bool driveDistance(double a, double b, bool c){
  return false;
}

//bool (*functptr[])(double,double,bool)= {&Drive::driveDistance,driveDistance} ;

Vector<bool(*)(double,double,bool)> vof;

void addThingy(bool (*function)(double,double,bool))
{
    //Don't take the address of the address:
    vof.push_back(function);
}

using namespace std;

///////////
// SETUP //
///////////
void setup() {
  addThingy(*driveDistance(0,0,false));

  vof[0];

  Serial.begin(9600);
  lcd.begin(16,2);
  msg.setup();


  pinMode(startPort,INPUT_PULLUP);
  pinMode(beamBreak,INPUT_PULLUP);
  pinMode(limitSwitch,INPUT_PULLUP);

  drive.initialize();
  arm.initialize(armPort, gripperPort, armPotPort);

 //bool (*fn[])(double, double, bool);
 //static fn funcs[] = {drive.driveDistance, drive.driveDistance};

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

int i = 0;
///////////////
// MAIN LOOP //
///////////////
void loop() {
    msg.heartbeat();
    msg.PeriodicRadiationStatus(0x01);
    drive.odometry();

  // if (!digitalRead(startPort)) arm.grab();
  // else if (!digitalRead(limitSwitch)) arm.release();
  // arm.updateArm(!digitalRead(startPort) || !digitalRead(limitSwitch));

  printOdomToLCD();
  //printStorageSupplyToLCD();
  //printIntakeToLCD();

// if(digitalRead(startPort)){
//   if(drive.driveDistance(12,digitalRead(startPort))){
//       drive.arcadeDrive(0, 0);
//     }
//   //drive.arcadeDrive(0.75, 0);
//   } else {
//     drive.arcadeDrive(0, 0);
//   }

// if(digitalRead(startPort)){
//   if(drive.turnToAngle(90,digitalRead(startPort))){
//       drive.arcadeDrive(0, 0);
//     }
//   //drive.arcadeDrive(0.75, 0);
//   } else {
//     drive.arcadeDrive(0, 0);
//   }

bool pressed = digitalRead(startPort);

if(pressed){
switch(i){
  case 0:
  if(drive.driveDistance(12,0, pressed)){ i++; }
  break;
  case 1:
  if(drive.turnToAngle(90, pressed)){ i++; }
  break;
  case 2:
  if(drive.driveDistance(24,90, pressed)){ i++; }
  break;
}
} else {
  drive.arcadeDrive(0, 0);
}

// if(digitalRead(startPort)){
//   drive.arcadeDrive(0.75, 0);
// } else {
//   drive.arcadeDrive(0, 0);
// }




  if(!digitalRead(limitSwitch)){
    drive.reset(0,0,0);
  }

  delay(20);
}
