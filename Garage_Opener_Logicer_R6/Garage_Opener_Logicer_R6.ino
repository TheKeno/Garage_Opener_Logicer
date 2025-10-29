#include "Arduino.h"
#include "Button.h"
#include "DistanceSensor.h"
#include "LightPulseSensor.h"
#include "config.h"
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <stdlib.h>

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

const int16_t threshold_increments[THRESHOLD_NUM] = {
	50,
	-50,
	25,
};

const int16_t threshold_max[THRESHOLD_NUM] = {
	1024,
	1024,
	300,
};


Button microSwitch(microswitchPin);
Button guiButton1(guiBtn1);
Button guiButton2(guiBtn2);
DistanceSensor ultraSensor(trigPin, echoPin);
LightPulseSensor lightPulseSensor(lightPin, 500, 800, 500);

bool is_car_inside(StateData* data) {
	int distance = ultraSensor.get_distance();
	if(distance < data->thresholds[CS_CHANGING_CAR_DISTANCE]) {
		return true;
	}

	return false;
}

bool is_door_closed() {
	return microSwitch.read() == Button::RELEASED;
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

	if(!is_car_inside(data)) {
		lightPulseSensor.did_pulse(); // Discard any pulses
		return;
	}
	
	if(lightPulseSensor.did_pulse()) {
		switch_state(data, STATE_WAIT_FOR_SECOND_SIGNAL);
	}
}

void update_wait_for_second_signal(StateData* data) {
	lightPulseSensor.update();

	if(millis() > data->entered_state_time + LIGHT_TIMEOUT) {
		if(is_door_closed()) {
			switch_state(data, STATE_IDLE);
		} else {
			switch_state(data, STATE_CLOSE_DOOR);
		}
		return;
	}

	if(lightPulseSensor.did_pulse()) {
		if(is_door_closed()) {
			switch_state(data, STATE_OPEN_DOOR);
		} else {
			switch_state(data, STATE_IDLE);
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

	if(guiButton1.read() == Button::PRESSED && data->config.pressing_key && millis() >= data->config.pressed_key_time + 2000) {
			data->config.pressing_key = false;
			switch_state(data, STATE_IDLE);
			EEPROM.write(0, 1); // Mark that we have written any data
			for(int i = 0; i < THRESHOLD_NUM; ++i) {
				EEPROM.put(1 + i * sizeof(int16_t), data->thresholds[i]);
			}
			lightPulseSensor.upper_threshold = data->thresholds[CS_CHANGING_UPPER_THRESHOLD];
			lightPulseSensor.lower_threshold = data->thresholds[CS_CHANGING_LOWER_THRESHOLD];
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
		if(data->thresholds[(int)state] > threshold_max[(int)state]) {
			data->thresholds[(int)state] = 0;
		}

		if(data->thresholds[(int)state] < 0) {
			data->thresholds[(int)state] = threshold_max[(int)state];
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

	delay(1);
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

int update_count = 0;
unsigned long time_of_last_count = 0;
int current_fps = 0;

void update_lcd(StateData* data) {
	static char buffer[24];

	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(state_to_name(data->current_state));

	switch(data->current_state) {
		case STATE_IDLE: {
			int dist = ultraSensor.get_distance();
			int light_level = analogRead(lightPin);


			unsigned long time = millis() / 5000;
			if(time % 2 == 0) {
				lcd.setCursor(0, 1);
				snprintf(buffer, 24, "D:%i", dist);
				lcd.print(buffer);

				lcd.setCursor(10, 1);
				snprintf(buffer, 24, "L:%i", light_level);
				lcd.print(buffer);
			} else {
				lcd.setCursor(0, 1);
				snprintf(buffer, 24, "F:%i", (int16_t)current_fps);
				lcd.print(buffer);

				lcd.setCursor(10, 1);
				snprintf(buffer, 24, "A:%i", (int16_t)lightPulseSensor.average);
				lcd.print(buffer);
			}

		} break;

		case STATE_CONFIG: {
			lcd.setCursor(0, 1);
			snprintf(buffer, 24, "%s: %i", get_config_name(data->config.config_state), get_config_value(data));
			lcd.print(buffer);
		} break;

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

	lightPulseSensor.upper_threshold = data.thresholds[CS_CHANGING_UPPER_THRESHOLD];
	lightPulseSensor.lower_threshold = data.thresholds[CS_CHANGING_LOWER_THRESHOLD];

	setup_pins();

	microSwitch.begin();
	guiButton1.begin();
	guiButton2.begin();
	lightPulseSensor.begin();

	Serial.begin(9600);
	time_of_last_print = millis();
	time_of_lcd_update = millis();
	time_of_last_count = millis();
}

void loop() {
	update_count++;
	if(millis() > time_of_last_count + 1000) {
		time_of_last_count = millis();
		current_fps = update_count;
		update_count = 0;
	}

	if(millis() > time_of_last_print + 5000) {
		Serial.print("Distance: ");
		Serial.println(ultraSensor.get_distance());

		Serial.print("Light level: ");
		Serial.println(analogRead(lightPin));

		Serial.print("Microswitch: ");
		Serial.println(microSwitch.read() == Button::PRESSED);

		Serial.print("Gui1: ");
		Serial.println(guiButton1.read() == Button::PRESSED);

		Serial.print("Gui2: ");
		Serial.println(guiButton2.read() == Button::PRESSED);

		time_of_last_print = millis();
	}

	if(millis() > time_of_lcd_update + 250) {
		update_lcd(&data);
		time_of_lcd_update = millis();
	}

	update(&data);
}
