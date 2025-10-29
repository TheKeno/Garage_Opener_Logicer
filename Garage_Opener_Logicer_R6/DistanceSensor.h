#ifndef DISTANCE_SENSOR_H
#define DISTANCE_SENSOR_H

class DistanceSensor {
public:
	DistanceSensor(int trigger, int sensor);
	int get_distance();

private:
	const float SOUND_VELOCITY = 0.034f;
	const int TIME_BETWEEN_MEASUREMENTS = 1000;


	int trigger_pin;
	int sensor_pin;

	int cached_value;

	unsigned long time_of_last_measurement = 0;
};

#endif
