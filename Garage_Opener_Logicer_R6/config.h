#ifndef _CONFIG_H_
#define _CONFIG_H_

#include "Arduino.h"

const int lightPin = A0;
const int guiBtn1 = 2;
const int guiBtn2 = 3;
const int microswitchPin = 4;
const int doorPin = 5;
const int echoPin = 6;
const int trigPin = 7;
const int doorStatus = 12;
const int carStatus = 13;

const int LIGHT_LEVEL_THRESHOLD = 400;
const int LIGHT_OFF_THRESHOLD = 200;
const int LIGHT_TIMEOUT = 5000;
const int DOOR_DELAY = 10000;
const int CAR_DISTANCE = 100;

static void setup_pins() {
	pinMode(trigPin, OUTPUT);
	pinMode(echoPin, INPUT);
	pinMode(lightPin, INPUT);
	pinMode(doorPin, OUTPUT);
	//pinMode(microswitchPin, INPUT);
	//pinMode(carStatus, OUTPUT);
	//pinMode(doorStatus, OUTPUT);
	//pinMode(guiBtn1, INPUT);
	//pinMode(guiBtn2, INPUT);
}

#endif