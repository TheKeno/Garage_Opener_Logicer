#include "ParkAssist.h"
#include "DistanceSensor.h"

ParkAssist::ParkAssist(int led_red_pin, int led_green_pin, int buzzer_pin,
                       unsigned long min_beep_interval, unsigned long max_beep_interval,
                       unsigned long beep_on_duration, unsigned long hold_time)
	: led_red_pin(led_red_pin), led_green_pin(led_green_pin), buzzer_pin(buzzer_pin),
	  min_beep_interval(min_beep_interval), max_beep_interval(max_beep_interval),
	  beep_on_duration(beep_on_duration), hold_time(hold_time) {
}

void ParkAssist::begin() {
	pinMode(led_red_pin, OUTPUT);
	pinMode(led_green_pin, OUTPUT);
	pinMode(buzzer_pin, OUTPUT);
	off();
}

void ParkAssist::off() {
	digitalWrite(led_red_pin, LOW);
	digitalWrite(led_green_pin, LOW);
	digitalWrite(buzzer_pin, LOW);
}

void ParkAssist::reset() {
	buzzer_on = false;
	last_beep_toggle = millis();
	parked_since = 0;

	digitalWrite(led_red_pin, HIGH);
	digitalWrite(led_green_pin, LOW);
	digitalWrite(buzzer_pin, LOW);
}

bool ParkAssist::update(int distance, int16_t far_distance, int16_t near_distance) {
	if(distance == DistanceSensor::NO_READING || distance > far_distance) {
		// No car in range yet - stay silent rather than beep about nothing.
		digitalWrite(led_red_pin, HIGH);
		digitalWrite(led_green_pin, LOW);
		digitalWrite(buzzer_pin, LOW);
		parked_since = 0;
		return false;
	}

	// far_distance <= near_distance only happens with a misconfigured pair -
	// collapse straight to the goal rather than feed an empty range to map().
	if(distance <= near_distance || far_distance <= near_distance) {
		digitalWrite(led_red_pin, LOW);
		digitalWrite(led_green_pin, HIGH);
		digitalWrite(buzzer_pin, HIGH);

		if(parked_since == 0) {
			parked_since = millis();
		}
		return millis() - parked_since >= hold_time;
	}

	digitalWrite(led_red_pin, HIGH);
	digitalWrite(led_green_pin, LOW);
	parked_since = 0;

	unsigned long off_interval = map(distance, near_distance, far_distance, min_beep_interval, max_beep_interval);
	unsigned long phase_duration = buzzer_on ? beep_on_duration : off_interval;

	if(millis() - last_beep_toggle >= phase_duration) {
		buzzer_on = !buzzer_on;
		digitalWrite(buzzer_pin, buzzer_on ? HIGH : LOW);
		last_beep_toggle = millis();
	}

	return false;
}
