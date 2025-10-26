#include "Arduino.h"
#include "Button.h"
#include "DistanceSensor.h"
#include "LightPulseSensor.h"
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

enum STATES {
	STATE_IDLE,
	STATE_WAIT_FOR_SECOND_SIGNAL,
	STATE_OPEN_DOOR,
	STATE_CLOSE_DOOR,
	STATE_CONFIG,
};

enum CONFIG_STATES {
	CS_CHANGING_UPPER_THRESHOLD,
	CS_CHANGING_LOWER_THRESHOLD,
	CS_CHANGING_CAR_DISTANCE,
	CS_STATE_NUM,
};

enum THRESHOLDS {
	THRESHOLD_LIGHT_ON,
	THRESHOLD_LIGHT_OFF,
	THRESHOLD_CAR_DISTANCE,
	THRESHOLD_NUM,
};

struct StateData {
	STATES current_state;
	unsigned long entered_state_time;

	int16_t thresholds[THRESHOLD_NUM];

	struct {
		unsigned long pressed_key_time;
		bool pressing_key;
		CONFIG_STATES config_state;
	} config;
};

void update_lcd(StateData* data);

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

const int16_t threshold_increments[THRESHOLD_NUM] = {
	25,
	25,
	5,
};

const int16_t threshold_max[THRESHOLD_NUM] = {
	1024,
	1024,
	400,
};


Button microSwitch(microswitchPin);
Button guiButton1(guiBtn1);
Button guiButton2(guiBtn2);
DistanceSensor ultraSensor(trigPin, echoPin);
LightPulseSensor lightPulseSensor(lightPin, 2000, 800, 500);

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

		case STATE_WAIT_FOR_SECOND_SIGNAL:
			Serial.println("State switch: WAIT_FOR_SECOND_SIGNAL");
			break;

		case STATE_CONFIG:
			Serial.println("State switch: CONFIG");
			break;

		default:
			break;
	}
}

void update_idle(StateData* data) {
	lightPulseSensor.update();

	if(guiButton1.pressed()) {
		switch_state(data, STATE_CONFIG);
		return;
	}

	if(!is_car_inside()) {
		return;
	}
	
	if(lightPulseSensor.did_pulse()) {
		if(is_door_closed()) {
			switch_state(data, STATE_WAIT_FOR_SECOND_SIGNAL);
		} else {
			switch_state(data, STATE_CLOSE_DOOR);
		}
	}
}

void update_wait_for_second_signal(StateData* data) {
	lightPulseSensor.update();

	if(millis() > data->entered_state_time + LIGHT_TIMEOUT) {
		switch_state(data, STATE_IDLE);
		return;
	}

	if(lightPulseSensor.did_pulse()) {
		if(is_door_closed()) {
			switch_state(data, STATE_OPEN_DOOR);
		}
	}
}

void update_open_door(StateData* data) {
	if(millis() > data->entered_state_time + DOOR_DELAY) {
		switch_state(data, STATE_IDLE);
	}
}

void update_close_door(StateData* data) {
	if(millis() > data->entered_state_time + DOOR_DELAY) {
		switch_state(data, STATE_IDLE);
	}
}

void update_config(StateData* data) {
	if(guiButton1.pressed()) {
		data->config.pressed_key_time = millis();
		data->config.pressing_key = true;
	}

	if(guiButton1.read() && data->config.pressing_key && millis() >= data->config.pressed_key_time + 2000) {
			data->config.pressing_key = false;
			switch_state(data, STATE_IDLE);
			EEPROM.write(0, 1); // Mark that we have written any data
			for(int i = 0; i < THRESHOLD_NUM; ++i) {
				EEPROM.put(1 + i * sizeof(int16_t), data->thresholds[i]);
			}
			return;
	}

	if(guiButton1.released()) {
		CONFIG_STATES& state = data->config.config_state;
		state = (CONFIG_STATES)((int)state + 1);
		if(state >= CS_STATE_NUM) {
			state = (CONFIG_STATES)0;
		}
	}

	if(guiButton2.pressed()) {
		CONFIG_STATES& state = data->config.config_state;
		data->thresholds[(int)state] += threshold_increments[(int)state];
		if(data->thresholds[(int)state] >= threshold_max[(int)state]) {
			data->thresholds[(int)state] = 0;
		}
	}
}

void update(StateData* data) {
	switch(data->current_state) {
		case STATE_IDLE:
			update_idle(data);
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

		case STATE_CONFIG:
			update_config(data);
			break;
	}

	delay(10);
}

StateData data{};
unsigned long time_of_last_print = 0;
unsigned long time_of_lcd_update = 0;
LiquidCrystal_I2C lcd(0x3F,16,2);

const char* state_to_name(STATES state) {
	switch(state) {
		case STATE_IDLE: return "Idle";
		case STATE_WAIT_FOR_SECOND_SIGNAL: return "Wait second";
		case STATE_OPEN_DOOR: return "Open door";
		case STATE_CLOSE_DOOR: return "Close door";
		case STATE_CONFIG: return "Config";
		default: return "ERROR";
	}
}

const char* get_config_name(CONFIG_STATES state) {
	switch(state) {
		case CS_CHANGING_UPPER_THRESHOLD: return "Upper";
		case CS_CHANGING_LOWER_THRESHOLD: return "Lower";
		case CS_CHANGING_CAR_DISTANCE: return "Dist";
		default: return "ERROR";
	}
} 

int16_t get_config_value(StateData* data) {
	return data->thresholds[(int)data->config.config_state];
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

		case STATE_CONFIG: {
			lcd.setCursor(0, 1);
			lcd.printf("%s: %i", get_config_name(data->config.config_state), get_config_value(data));
		}

		default:
		break;
	}
}

void setup() {
	lcd.init();
	lcd.clear();
	lcd.backlight();

	data.current_state = STATE_IDLE;
	data.entered_state_time = 0;
	data.config.config_state = CS_CHANGING_UPPER_THRESHOLD;
	data.config.pressed_key_time = 0;
	data.config.pressing_key = false;

	if(EEPROM.read(0) == 255) {
		// We have not written data to EEPROM yet, initialize with constants
		data.thresholds[THRESHOLD_LIGHT_ON] = LIGHT_LEVEL_THRESHOLD;
		data.thresholds[THRESHOLD_LIGHT_OFF] = LIGHT_OFF_THRESHOLD;
		data.thresholds[THRESHOLD_CAR_DISTANCE] = CAR_DISTANCE;
	} else {
			for(int i = 0; i < THRESHOLD_NUM; ++i) {
				EEPROM.get(1 + i * sizeof(int16_t), data.thresholds[i]);
			}
	}

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
