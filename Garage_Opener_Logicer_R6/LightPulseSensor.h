#ifndef LIGHT_PULSE_SENSOR_H
#define LIGHT_PULSE_SENSOR_H

class LightPulseSensor {
public:
	LightPulseSensor(int pin, int pulse_timeout, int upper_threshold, int lower_threshold);
	bool did_pulse();
	void update();

	int pin;
	int pulse_timeout;
	int upper_threshold;
	int lower_threshold;

private:
	bool seeking_high_value = true;
	unsigned long time_of_max_value;
	bool detected_pulse = false;
};

#endif
