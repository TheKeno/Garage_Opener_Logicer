#ifndef PARK_ASSIST_H
#define PARK_ASSIST_H

#include "Arduino.h"

class ParkAssist {
public:
	ParkAssist(int led_red_pin, int led_green_pin, int buzzer_pin,
	           unsigned long min_beep_interval, unsigned long max_beep_interval,
	           unsigned long beep_on_duration, unsigned long hold_time);

	void begin();
	// Call once on entering STATE_PARKING: red on, buzzer/timers cleared.
	void reset();
	// Call once on leaving STATE_PARKING: LEDs + buzzer off.
	void off();

	// Call every loop while in STATE_PARKING. Drives the LED/buzzer from the
	// current distance reading. Returns true once the near-distance goal has
	// been held continuously for hold_time.
	bool update(int distance, int16_t far_distance, int16_t near_distance);

private:
	int led_red_pin;
	int led_green_pin;
	int buzzer_pin;

	unsigned long min_beep_interval;
	unsigned long max_beep_interval;
	unsigned long beep_on_duration;
	unsigned long hold_time;

	bool buzzer_on = false;
	unsigned long last_beep_toggle = 0;
	unsigned long parked_since = 0; // 0 = not currently holding at the goal
};

#endif
