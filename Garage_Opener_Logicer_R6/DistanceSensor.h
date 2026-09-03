#ifndef DISTANCE_SENSOR_H
#define DISTANCE_SENSOR_H

class DistanceSensor {
public:
	// Returned by get_distance() when the echo never came back. Callers must
	// treat it as "no information" - it is NOT a short distance.
	static const int NO_READING = -1;

	DistanceSensor(int trigger, int sensor);

	// A fresh reading is only taken once the cached one is at least
	// min_interval old; defaults to TIME_BETWEEN_MEASUREMENTS. Callers that
	// need more responsive feedback (e.g. parking assist) can pass a shorter
	// interval - the cache is shared, so whichever caller asks first with
	// the shortest interval drives when the next measurement actually happens.
	int get_distance(unsigned long min_interval = TIME_BETWEEN_MEASUREMENTS);

private:
	static constexpr float SOUND_VELOCITY = 0.034f;
	static const unsigned long TIME_BETWEEN_MEASUREMENTS = 1000;

	// ~510 cm of range, well past the HC-SR04's 400 cm. pulseIn() defaults to
	// a one second timeout, which would stall the whole loop for a second
	// every time the sensor gets no echo.
	static const unsigned long ECHO_TIMEOUT_US = 30000;

	int trigger_pin;
	int sensor_pin;

	int cached_value;
	bool has_measured;

	unsigned long time_of_last_measurement = 0;
};

#endif
