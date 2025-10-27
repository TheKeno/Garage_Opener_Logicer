#ifndef LIGHT_PULSE_SENSOR_H
#define LIGHT_PULSE_SENSOR_H

#include <inttypes.h>

class LightPulseSensor {
public:
	LightPulseSensor(int pin, int pulse_timeout, int upper_threshold, int lower_threshold);
	bool did_pulse();
	void update();

	int pin;
	int pulse_timeout;
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
