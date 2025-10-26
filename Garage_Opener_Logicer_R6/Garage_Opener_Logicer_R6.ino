#include "Arduino.h"
#include "Button.h"
#include <LiquidCrystal_I2C.h>

class DistanceSensor {
public:
	DistanceSensor(int trigger, int sensor) : trigger_pin(trigger), sensor_pin(sensor) {
	}
	int get_distance() {
		digitalWrite(trigger_pin, LOW);
		delayMicroseconds(2);
		
		digitalWrite(trigger_pin, HIGH);
		delayMicroseconds(10);
		digitalWrite(trigger_pin, LOW);

		unsigned long duration = pulseIn(sensor_pin, HIGH);

		return duration * SOUND_VELOCITY/2;
	}

private:
	const float SOUND_VELOCITY = 0.034f;

	int trigger_pin;
	int sensor_pin;
};

enum STATES {
	STATE_IDLE,
	STATE_WAIT_FOR_DARKNESS,
	STATE_WAIT_FOR_SECOND_SIGNAL,
	STATE_OPEN_DOOR,
	STATE_CLOSE_DOOR,
};

struct StateData {
	STATES current_state;
	unsigned long entered_state_time;
};



const int lightPin = A0;
const int carStatus = D0;
//D1 & D2 used by LCD
const int microswitchPin = D3;
const int doorStatus = D4;
const int doorPin = D5;
const int echoPin = D6;
const int trigPin = D7;
const int guiBtn1 = D8;
const int guiBtn2 = D9;


const int LIGHT_LEVEL_THRESHOLD = 700;
const int LIGHT_OFF_THRESHOLD = 400;
const int LIGHT_TIMEOUT = 5000;
const int DOOR_DELAY = 10000;
const int CAR_DISTANCE = 100;



Button microSwitch(microswitchPin);
DistanceSensor ultraSensor(trigPin, echoPin);

bool is_car_inside() {
	int distance = ultraSensor.get_distance();
	if(distance < CAR_DISTANCE) {
		return true;
	}

	return false;
}

bool is_door_closed() {
	return !microSwitch.read();
}

void send_door_signal() {
	digitalWrite(doorPin, LOW);
	delay(10);
	digitalWrite(doorPin, HIGH);
	delay(500);
	digitalWrite(doorPin, LOW);
}

void switch_state(StateData* data, STATES new_state) {
	data->current_state = new_state;
	data->entered_state_time = millis();

	update_lcd(data);

	switch(new_state) {
		case STATE_OPEN_DOOR:
			Serial.println("State switch: OPEN DOOR");
			send_door_signal();
			break;

		case STATE_CLOSE_DOOR:
			Serial.println("State switch: CLOSE DOOR");
			send_door_signal();
			break;
		
		case STATE_IDLE:
			Serial.println("State switch: IDLE");
			break;

		case STATE_WAIT_FOR_DARKNESS:
			Serial.println("State switch: WAIT_FOR_DARKNESS");
			break;

		case STATE_WAIT_FOR_SECOND_SIGNAL:
			Serial.println("State switch: WAIT_FOR_SECOND_SIGNAL");
			break;

		default:
			break;
	}
}

void update_idle(StateData* data) {
	if(!is_car_inside())
		return;

	int light_level = analogRead(lightPin);
	if(light_level > LIGHT_LEVEL_THRESHOLD) {
		if(is_door_closed()) {
			switch_state(data, STATE_WAIT_FOR_SECOND_SIGNAL);
		} else {
			switch_state(data, STATE_CLOSE_DOOR);
		}
	}
}

void update_wait_for_darkness(StateData* data) {
	if(millis() > data->entered_state_time + LIGHT_TIMEOUT) {
		switch_state(data, STATE_IDLE);
		return;
	}

	int light_level = analogRead(lightPin);
	if(light_level < LIGHT_OFF_THRESHOLD) {
		switch_state(data, STATE_WAIT_FOR_SECOND_SIGNAL);
	}
}

void update_wait_for_second_signal(StateData* data) {
	if(millis() > data->entered_state_time + LIGHT_TIMEOUT) {
		switch_state(data, STATE_IDLE);
		return;
	}

	int light_level = analogRead(lightPin);
	if(light_level > LIGHT_LEVEL_THRESHOLD) {
		if(is_door_closed()) {
			switch_state(data, STATE_OPEN_DOOR);
		}
	}
}

void update_open_door(StateData* data) {
	if(millis() > data->entered_state_time + DOOR_DELAY) {
		switch_state(data, STATE_IDLE);
		return;
	}
}

void update_close_door(StateData* data) {
	if(millis() > data->entered_state_time + DOOR_DELAY) {
		switch_state(data, STATE_IDLE);
		return;
	}
}

void update(StateData* data) {
	switch(data->current_state) {
		case STATE_IDLE:
			update_idle(data);
			break;

		case STATE_WAIT_FOR_DARKNESS:
			update_wait_for_darkness(data);
			break;
		
		case STATE_WAIT_FOR_SECOND_SIGNAL:
			update_wait_for_second_signal(data);
			break;

		case STATE_OPEN_DOOR:
			update_open_door(data);
			break;

		case STATE_CLOSE_DOOR:
			update_close_door(data);
			break;
	}

	delay(10);
}

StateData data;
unsigned long time_of_last_print = 0;
unsigned long time_of_lcd_update = 0;
LiquidCrystal_I2C lcd(0x3F,16,2);

const char* state_to_name(STATES state) {
	switch(state) {
		case STATE_IDLE:
			return "Idle";
			break;

		case STATE_WAIT_FOR_DARKNESS:
			return "Wait dark";
			break;
		
		case STATE_WAIT_FOR_SECOND_SIGNAL:
			return "Wait second";
			break;

		case STATE_OPEN_DOOR:
			return "Open door";
			break;

		case STATE_CLOSE_DOOR:
			return "Close door";
			break;
	}

	return "";
}

void update_lcd(StateData* data) {
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(state_to_name(data->current_state));

	switch(data->current_state) {
		case STATE_IDLE: {
			int dist = ultraSensor.get_distance();
			int light_level = analogRead(lightPin);

			lcd.setCursor(0, 1);
			lcd.printf("D:%i", dist);
			lcd.setCursor(10, 1);
			lcd.printf("L:%i", light_level);
		} break;
	}
}

void setup() {
	lcd.init();
	lcd.clear();
	lcd.backlight();

	data.current_state = STATE_IDLE;

	pinMode(trigPin, OUTPUT);
	pinMode(echoPin, INPUT);
	pinMode(lightPin, INPUT);
	pinMode(doorPin, OUTPUT);
	pinMode(microswitchPin, INPUT);
	pinMode(carStatus, OUTPUT);
	pinMode(doorStatus, OUTPUT);
	pinMode(guiBtn1, INPUT);
	pinMode(guiBtn2, INPUT);

	microSwitch.begin();
	Serial.begin(9600);
	time_of_last_print = millis();
	time_of_lcd_update = millis();
}

void loop() {
	if(millis() > time_of_last_print + 5000) {
		Serial.print("Distance: ");
		Serial.println(ultraSensor.get_distance());

		Serial.print("Light level: ");
		Serial.println(analogRead(lightPin));

		Serial.print("Microswitch: ");
		Serial.println(microSwitch.read());

		time_of_last_print = millis();
	}

	if(millis() > time_of_lcd_update + 250) {
		update_lcd(&data);
		time_of_lcd_update = millis();
	}

	update(&data);
}
