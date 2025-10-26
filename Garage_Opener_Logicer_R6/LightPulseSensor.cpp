#include "LightPulseSensor.h"
#include "Arduino.h"

LightPulseSensor::LightPulseSensor(int pin, int pulse_timeout, int upper_threshold, int lower_threshold) : pin(pin), pulse_timeout(pulse_timeout), upper_threshold(upper_threshold), lower_threshold(lower_threshold) {}

bool LightPulseSensor::did_pulse() {
	if(detected_pulse) {
		detected_pulse = false;
		return true;
	}

	return false;
}

void LightPulseSensor::update() {
	int value = analogRead(pin);

	if(seeking_high_value) {
		if(value >= upper_threshold) {
			seeking_high_value = false;
			time_of_max_value = millis();
			detected_pulse = false;
		}
	} else {
		if(millis() > time_of_max_value + pulse_timeout) {
			seeking_high_value = true;
			return;
		}
		if(value <= lower_threshold) {
			seeking_high_value = true;
			detected_pulse = true;
		}
	}
}