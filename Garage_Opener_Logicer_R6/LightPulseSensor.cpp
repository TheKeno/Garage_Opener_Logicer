#include "LightPulseSensor.h"
#include "Arduino.h"

LightPulseSensor::LightPulseSensor(int pin, int pulse_timeout, int upper_threshold, int lower_threshold) 
	: pin(pin)
	, pulse_timeout(pulse_timeout)
	, upper_threshold(upper_threshold)
	, lower_threshold(lower_threshold) {}

void LightPulseSensor::begin() {
	int value = analogRead(pin);
	for(int i = 0; i < NUM_SAMPLES; ++i) {
		samples[i] = value;
	}
	average = value;
	time_since_last_sample = millis();
}

bool LightPulseSensor::did_pulse() {
	if(detected_pulse) {
		detected_pulse = false;
		return true;
	}

	return false;
}

void LightPulseSensor::update() {
	int value = analogRead(pin);

	if(millis() > time_since_last_sample + TIME_BETWEEN_SAMPLES) {
		average = 0;
		for(int i = LightPulseSensor::NUM_SAMPLES - 1; i > 0; --i) {
			samples[i] = samples[i - 1];
			average += samples[i];
		}
		samples[0] = value;
		average += value;
		average /= NUM_SAMPLES;

		time_since_last_sample = millis();
	}

	if(seeking_high_value) {
		if(value >= average + upper_threshold) {
			seeking_high_value = false;
			detected_pulse = false;
			time_of_max_value = millis();
		}
	} else {
		if(millis() > time_of_max_value + pulse_timeout) {
			seeking_high_value = true;
			return;
		}
		if(value <= average + lower_threshold) {
			seeking_high_value = true;
			detected_pulse = true;
		}
	}
}