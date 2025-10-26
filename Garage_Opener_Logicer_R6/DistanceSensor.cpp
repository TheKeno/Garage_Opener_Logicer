#include "DistanceSensor.h"

DistanceSensor::DistanceSensor(int trigger, int sensor) : trigger_pin(trigger), sensor_pin(sensor) {}
int DistanceSensor::get_distance() {
	digitalWrite(trigger_pin, LOW);
	delayMicroseconds(2);

	digitalWrite(trigger_pin, HIGH);
	delayMicroseconds(10);
	digitalWrite(trigger_pin, LOW);

	unsigned long duration = pulseIn(sensor_pin, HIGH);

	return duration * SOUND_VELOCITY/2;
}
