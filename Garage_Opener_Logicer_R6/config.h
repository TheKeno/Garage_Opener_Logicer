#ifndef _CONFIG_H_
#define _CONFIG_H_

#include "Arduino.h"

// Serial output is a desk-development aid; comment out DEBUG_SERIAL for prod.
// The periodic status block is ~78 bytes against a 64-byte TX buffer, and
// write() busy-waits once that fills, so the baud has to be high enough for
// the burst to drain faster than it is produced: 115200 costs ~7 ms of wire
// time, 9600 would block the loop for ~15 ms every time.
#define DEBUG_SERIAL

#ifdef DEBUG_SERIAL
	#define DBG_BEGIN()     Serial.begin(DEBUG_BAUD)
	#define DBG_PRINT(x)    Serial.print(x)
	#define DBG_PRINTLN(x)  Serial.println(x)
#else
	#define DBG_BEGIN()     ((void)0)
	#define DBG_PRINT(x)    ((void)0)
	#define DBG_PRINTLN(x)  ((void)0)
#endif

const unsigned long DEBUG_BAUD = 115200;

const int lightPin = A0;
const int guiBtn1 = 2;
const int guiBtn2 = 3;
const int microswitchPin = 4;
const int doorPin = 5;
const int echoPin = 6;
const int trigPin = 7;
const int externalDoorPin = 11;
const int doorStatus = 12;
const int carStatus = 13;

const int16_t LIGHT_LEVEL_THRESHOLD = 300;
const int16_t LIGHT_OFF_THRESHOLD = 200;
const int16_t CAR_DISTANCE = 45;

// Durations are unsigned long: as int they would be 16-bit on AVR, so
// anything past 32767 ms would silently go negative.
const unsigned long LIGHT_TIMEOUT = 1500;
const unsigned long LIGHT_PULSE_TIMEOUT = 500;
const unsigned long DOOR_DELAY = 19000;
const unsigned long LCD_UPDATE_INTERVAL = 250;
const unsigned long DEBUG_PRINT_INTERVAL = 5000;
const unsigned long CONFIG_SAVE_HOLD = 2000;

static void setup_pins() {
	pinMode(trigPin, OUTPUT);
	pinMode(echoPin, INPUT);
	pinMode(lightPin, INPUT);
	pinMode(doorPin, OUTPUT);
	pinMode(carStatus, OUTPUT);
	pinMode(doorStatus, OUTPUT);
	// The buttons (microswitch, gui1, gui2, externalDoor) set their own pin
	// modes in Button::begin(), which enables INPUT_PULLUP.
}

#endif
