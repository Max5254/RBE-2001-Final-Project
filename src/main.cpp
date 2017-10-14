#include <Arduino.h>
#include <LiquidCrystal.h>
#include <Servo.h>
#include "ReactorControl/Messages.h"
#include "drive.h"
#include "arm.h"
#include "helpers.h"
#include "scheduler.h"

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

LiquidCrystal lcd(40,41,42,43,44,45);
Messages msg;

#define LED_PIN 24
#define NUM_PIXELS 4
Adafruit_NeoPixel strip = Adafruit_NeoPixel(4, 24, NEO_GRB + NEO_KHZ800);

//color constants
uint32_t RED = strip.Color(255, 0, 0);
uint32_t GREEN = strip.Color(0, 255, 0);
uint32_t BLUE = strip.Color(0, 0, 255);
uint32_t YELLOW = strip.Color(255, 200, 0);
uint32_t PURPLE = strip.Color(200, 0, 200);
uint32_t ORANGE = strip.Color(255, 100, 0);
uint32_t NO_COLOR = strip.Color(0, 0, 0);

void setLEDs(uint32_t color){ //set all 4 LEDs to specific color
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

  setLEDs(PURPLE);
  while(msg.isStopped() && digitalRead(startPort)){ //wait for you to enter pegs into BT
    msg.read();
    msg.heartbeat();
  }

  //init rest of robot
  drive.initialize();
  arm.initialize(armPort, gripperPort, armPotPort);
  msg.buttonStop();
  scheduler.build();
  setLEDs(GREEN);
}



void printOdomToLCD(){
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

///////////////
// MAIN LOOP //
///////////////
void loop() {
  msg.read(); //read BT for stop messages
  msg.heartbeat(); //send hearbeat periodically
  currentRadiation = scheduler.getRadiation();
  msg.PeriodicRadiationStatus(currentRadiation); //send radiation alert periodically

  if(currentRadiation != lastRadiation){ //if new status of radiation update leds
    if(currentRadiation == 1){
      setLEDs(ORANGE);
    } else if(currentRadiation == 2){
      setLEDs(RED);
    } else {
      setLEDs(BLUE);
    }
  }
  lastRadiation = currentRadiation;

  drive.odometry(); //calculate position

  //checks to see if robot should be running
  enabled = !msg.isStopped(); //sets if robot should be running
  bool pressed = digitalRead(startPort);
  if(pressed && !lastPressed){
    msg.buttonStop();
  }
  lastPressed = pressed;

  //runs state machine and updates arm
  scheduler.run(enabled);
  arm.updateArm(true);

  printOdomToLCD();

  delay(20);
}
