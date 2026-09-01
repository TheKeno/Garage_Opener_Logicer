#include "DistanceSensor.h"
#include "Arduino.h"

DistanceSensor::DistanceSensor(int trigger, int sensor) : trigger_pin(trigger), sensor_pin(sensor) {
	time_of_last_measurement = 0;
	cached_value = NO_READING;
	has_measured = false;
}

int DistanceSensor::get_distance() {
	if(!has_measured || millis() - time_of_last_measurement >= TIME_BETWEEN_MEASUREMENTS) {
		digitalWrite(trigger_pin, LOW);
		delayMicroseconds(2);

		digitalWrite(trigger_pin, HIGH);
		delayMicroseconds(10);
		digitalWrite(trigger_pin, LOW);

		unsigned long duration = pulseIn(sensor_pin, HIGH, ECHO_TIMEOUT_US);

		// pulseIn() returns 0 on timeout. Reporting that as a distance of 0 cm
		// would read as "something is right in front of the sensor".
		cached_value = duration > 0 ? (int)(duration * SOUND_VELOCITY / 2) : NO_READING;

		// Set even on failure, so a dead sensor is retried once per interval
		// rather than on every call.
		has_measured = true;
		time_of_last_measurement = millis();
	}

	return cached_value;
}
