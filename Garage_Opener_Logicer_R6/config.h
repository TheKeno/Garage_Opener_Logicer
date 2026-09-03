#ifndef _CONFIG_H_
#define _CONFIG_H_

#include "Arduino.h"

// The UART is the control channel. The ESP8266 - or a terminal on the PC
// pretending to be it - sends one command per line and reads back one reply
// line: OPEN, CLOSE, STATE, SET <name> <value>, SAVE. Replies start with OK or
// ERR; anything the board says on its own initiative starts with '#', so the
// far end can skip those without mistaking one for a reply.
//
// The longest reply, STATE, runs to about 108 bytes against a 64-byte TX
// buffer, and write() busy-waits once that fills - hence a baud high enough
// for the tail to drain in a few milliseconds rather than tens of them.
const unsigned long SERIAL_BAUD = 115200;

#define LOG_EVENT(x)  do { Serial.print(F("# ")); Serial.println(x); } while(0)

const int lightPin = A0;
const int guiBtn1 = 2;
const int microswitchPin = 4;
const int doorPin = 5;
const int echoPin = 6;
const int trigPin = 7;
const int encoderClkPin = 8;
const int encoderDtPin = 9;
const int externalDoorPin = 11;
const int doorStatus = 12;
const int carStatus = 13;
const int ledRedPin = A1;
const int ledGreenPin = A2;
const int buzzerPin = A3;

const int16_t LIGHT_LEVEL_THRESHOLD = 300;
const int16_t LIGHT_OFF_THRESHOLD = 200;
const int16_t CAR_DISTANCE = 45;
const int16_t PARK_FAR_DISTANCE_DEFAULT = 50;
const int16_t PARK_NEAR_DISTANCE_DEFAULT = 15;

// Durations are unsigned long: as int they would be 16-bit on AVR, so
// anything past 32767 ms would silently go negative.
const unsigned long LIGHT_TIMEOUT = 1500;
const unsigned long LIGHT_PULSE_TIMEOUT = 500;
const unsigned long DOOR_DELAY = 19000;
const unsigned long LCD_UPDATE_INTERVAL = 250;
const unsigned long CONFIG_SAVE_HOLD = 2000;
const unsigned long PARK_BEEP_MAX_INTERVAL = 800;  // slowest clicks, at the edge of the zone
const unsigned long PARK_BEEP_MIN_INTERVAL = 100;  // fastest clicks, just short of the goal
const unsigned long PARK_BEEP_ON_DURATION = 60;    // click length
const unsigned long PARK_HOLD_TIME = 1500;         // continuous buzzer required before confirming "parked"
const unsigned long PARKING_TIMEOUT = 120000;      // give up and return to IDLE if the goal is never reached

static void setup_pins() {
	pinMode(trigPin, OUTPUT);
	pinMode(echoPin, INPUT);
	pinMode(lightPin, INPUT);
	pinMode(doorPin, OUTPUT);
	pinMode(carStatus, OUTPUT);
	pinMode(doorStatus, OUTPUT);
	// The buttons (microswitch, gui1, gui2, externalDoor) set their own pin
	// modes in Button::begin(), which enables INPUT_PULLUP. ParkAssist sets
	// its own pins (led red/green, buzzer) in begin() too.
}

#endif
