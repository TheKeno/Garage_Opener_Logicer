#include "DistanceSensor.h"
#include "Arduino.h"

DistanceSensor::DistanceSensor(int trigger, int sensor) : trigger_pin(trigger), sensor_pin(sensor) {
	time_of_last_measurement = 0;
	cached_value = -1;
}

int DistanceSensor::get_distance() {
	if(millis() > time_of_last_measurement + TIME_BETWEEN_MEASUREMENTS || cached_value == -1) {
		digitalWrite(trigger_pin, LOW);
		delayMicroseconds(2);

		digitalWrite(trigger_pin, HIGH);
		delayMicroseconds(10);
		digitalWrite(trigger_pin, LOW);

		unsigned long duration = pulseIn(sensor_pin, HIGH);

		cached_value = duration * SOUND_VELOCITY/2;

		time_of_last_measurement = millis();
	}

	return cached_value;
}
