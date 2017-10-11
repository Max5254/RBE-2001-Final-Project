#include <Arduino.h>
#include <LiquidCrystal.h>
#include <Servo.h>
#include "ReactorControl/Messages.h"
#include "drive.h"
#include "arm.h"
#include "helpers.h"
#include "scheduler.h"

#include "Adafruit_NeoPixel.h"
#ifdef __AVR__
  #include <avr/power.h>
#endif


LiquidCrystal lcd(40,41,42,43,44,45);
Messages msg;

#define LED_PIN 24
#define NUM_PIXELS 4
Adafruit_NeoPixel strip = Adafruit_NeoPixel(4, 24, NEO_GRB + NEO_KHZ800);

uint32_t RED = strip.Color(255, 0, 0);
uint32_t GREEN = strip.Color(0, 255, 0);
uint32_t BLUE = strip.Color(0, 0, 255);
uint32_t YELLOW = strip.Color(255, 200, 0);
uint32_t PURPLE = strip.Color(200, 0, 200);
uint32_t ORANGE = strip.Color(255, 100, 0);
uint32_t NO_COLOR = strip.Color(0, 0, 0);

void setLEDs(uint32_t color){
  strip.setPixelColor(0,color);
  strip.setPixelColor(1,color);
  strip.setPixelColor(2,color);
  strip.setPixelColor(3,color);
  strip.show();
}

int lastRadiation, currentRadiation = 0;

////////
// IO //
////////

//Motors
const int armPort = 8;
const int gripperPort = 9;
//Digital IO
const int startPort = 22;
const int debugA = 23;
//const int debugB = 24;
//Analog Input
const int armPotPort = A2;

bool enabled = false;
bool lastPressed = true;


Drive drive;
Arm arm;
Scheduler scheduler;

///////////
// SETUP //
///////////
void setup() {


  Serial.begin(9600);
  lcd.begin(16,2);
  msg.setup();


  //Neopixel Init
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  pinMode(startPort,INPUT_PULLUP);
  pinMode(debugA,INPUT_PULLUP);
  //pinMode(debugB,INPUT_PULLUP);

  drive.initialize();
  arm.initialize(armPort, gripperPort, armPotPort);

  while(msg.isStopped()){
  msg.read();
  }
  msg.buttonStop();
  scheduler.build();
  setLEDs(BLUE);
}



void printOdomToLCD(){
  //lcd.clear();
  lcd.setCursor(0,0);
  //lcd.print("X:   Y:   T:");
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
  msg.read();
  msg.heartbeat();
  //Serial.println(scheduler.getRadiation());

  // currentRadiation = scheduler.getRadiation();
  // msg.PeriodicRadiationStatus(currentRadiation);
  //
  // if(currentRadiation != lastRadiation){
  //   if(currentRadiation == 1){
  //     setLEDs(RED);
  //   } else if(currentRadiation == 2){
  //     setLEDs(ORANGE);
  //   } else {
  //     setLEDs(BLUE);
  //   }
  // }
  // lastRadiation = currentRadiation;

  drive.odometry();


  // if (!digitalRead(startPort)) arm.raiseArm();
  // else if (!digitalRead(limitSwitch)) arm.lowerArm();
  // if (!digitalRead(debugA)) arm.grab();
  // else if (!digitalRead(debugB)) arm.release();
  //arm.updateArm(!digitalRead(startPort) || !digitalRead(limitSwitch) || !digitalRead(debugA) || !digitalRead(debugB));

  bool pressed = digitalRead(startPort);

  // Serial.print(msg.isStopped());
  // Serial.print(" ");
  // Serial.println(lastStopped);


  if(msg.isStopped()){
    enabled = false;
  }
  if(!msg.isStopped()){
    enabled = true;
  }
  if(pressed && !lastPressed){
    msg.buttonStop();
  }
  lastPressed = pressed;


  scheduler.run(enabled);
  arm.updateArm(true);
  printOdomToLCD();


  // if(!digitalRead(13)){
  //   drive.reset(0,0,0);
  // }

  delay(20);
}
