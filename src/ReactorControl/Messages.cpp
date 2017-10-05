/*
 * Messages.cpp
 *
 *  Created on: Sep 15, 2016
 *      Author: bradmiller
 */
#include "Arduino.h"
#include "Messages.h"
#include "BTComms.h"

BTComms comms;

/**
 * Constuctor
 * Initialize everything here when the class is created
 * Note: you cannot call methods that depend on other classes having already been created
 */
Messages::Messages() {
	stopped = false;
}

void Messages::heartbeat(){
	if (Messages::read()) {
		Messages::printMessage();
	}
	if (millis() > timeForHeartbeat) {
		timeForHeartbeat = millis() + 1000;
		Messages::sendHeartbeat();
	}
}

/**
 * Setup class code that is called from the Arduino sketch setup() function. This doesn't
 * get called until all the other classes have been created.
 */
void Messages::setup() {
	comms.setup();
	timeForHeartbeat = millis() + 1000;
}

/**
 * Check if the field is currently in the "stop" state
 * @returns bool value that is true if the robot should be stopped
 */
bool Messages::isStopped() {
	return stopped;
}

/**
 * Send a heartbeat message to the field to let it know that your code is alive
 * This should be called by your robot program periodically, say once per second. This
 * timing can easily be done in the loop() function of your program.
 */
void Messages::sendHeartbeat() {
	comms.writeMessage(kHeartbeat, 0x0a, 0x00);
}

/**
 * Print message for debugging
 * This method prints the message as a string of hex numbers strictly for debugging
 * purposes and is not required to be called for any other purpose.
 */
void Messages::printMessage() {
    for (int i = 0; i < comms.getMessageLength(); i++) {
      Serial.print(comms.getMessageByte(i), HEX);
      Serial.print(" ");
    }
    Serial.println();
}

bool* Messages::getStorageAvailability() {
	return storageArray;
}

bool* Messages::getSupplyAvailability() {
	return supplyArray;
}

/*
1 0 0 1 F7
2 0 0 0 F7
1 0 0 1 F7
2 0 0 0 F7
1 0 0 1 F7
2 0 0 0 F7
1 0 0 1 F7
2 0 0 0 F7
1 0 0 1 F7
2 0 0 0 F7
1 0 0 1 F7
2 0 0 0 F7
1 0 0 1 F7
2 0 0 0 F7
1 0 0 1 F7
4 0 3 F3
Robot stop!
*/


/**
 * Read messages from the Bluetooth serial connection
 * This method should be called from the loop() function in your arduino code. It will check
 * to see if the lower level comms object has received a complete message, and run the appropriate
 * code to handle the message type. This should just save the state of the message inside this class
 * inside member variables. Then add getters/setters to retrieve the status from your program.
 */
bool Messages::read() {
	if (comms.read()) {
		switch (comms.getMessageByte(0)) {
		case kStorageAvailability:
			storageArray[0] = comms.getMessageByte(3) & BIT0; //need find correct index for message byte!!
			storageArray[1] = comms.getMessageByte(3) & BIT1;
			storageArray[2] = comms.getMessageByte(3) & BIT2;
			storageArray[3] = comms.getMessageByte(3) & BIT3;
			break;
		case kSupplyAvailability:
		supplyArray[0] = comms.getMessageByte(3) & BIT0; //need find correct index for message byte!!
		supplyArray[1] = comms.getMessageByte(3) & BIT1;
		supplyArray[2] = comms.getMessageByte(3) & BIT2;
		supplyArray[3] = comms.getMessageByte(3) & BIT3;
			break;
		case kRadiationAlert:
			break;
		case kStopMovement:
			stopped = true;
			break;
		case kResumeMovement:
			stopped = false;
			break;
		case kRobotStatus:
			break;
		case kHeartbeat:
			break;
		}
		return true;
	}
	return false;
}
