#ifndef DISTANCE_SENSOR_H
#define DISTANCE_SENSOR_H

#include "Arduino.h"

class DistanceSensor {
public:
	DistanceSensor(int trigger, int sensor);
	int get_distance();

private:
	const float SOUND_VELOCITY = 0.034f;

	int trigger_pin;
	int sensor_pin;
};

#endif
