#ifndef LIGHT_PULSE_SENSOR_H
#define LIGHT_PULSE_SENSOR_H

#include <inttypes.h>

class LightPulseSensor {
public:
	LightPulseSensor(int pin, unsigned long pulse_timeout, int upper_threshold, int lower_threshold);
	void begin();
	bool did_pulse();

	// Pass detect_pulses = false while the caller is in a state that must not
	// react to light (the door cycle, config). The rolling ambient average
	// keeps updating either way, so it never goes stale, but the pulse
	// detector is held in its resting state instead of latching the garage
	// light as a pulse to be consumed on the way back to idle.
	void update(bool detect_pulses = true);

	int pin;
	unsigned long pulse_timeout;
	int upper_threshold;
	int lower_threshold;
	
	int32_t average;

private:
	bool seeking_high_value = true;
	unsigned long time_of_max_value;
	bool detected_pulse = false;

	static const int NUM_SAMPLES = 32;
	static const unsigned long TIME_BETWEEN_SAMPLES = 2000;
	int16_t samples[NUM_SAMPLES];
	unsigned long time_since_last_sample;
};

#endif
