#include "Arduino.h"
#include "Button.h"
#include "DistanceSensor.h"
#include "LightPulseSensor.h"
#include "config.h"
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <stdlib.h>
#include <string.h>

/*amb+thresh ska vara mindre än ljuspulsens styrka för att det ska trigga pulskoll

om "släckt" är mindre än amb+low thresh har vi fått en puls*/

enum STATES {
	STATE_IDLE,
	STATE_WAIT_FOR_SECOND_SIGNAL,
	STATE_OPEN_DOOR,
	STATE_CLOSE_DOOR,
	STATE_CONFIG,
};

// Indexes thresholds[], threshold_increments[], threshold_min[] and
// threshold_max[], and doubles as the cursor for the config screen.
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
		THRESHOLDS config_state;
		bool ignore_next_release;
	} config;
};

void update_lcd(StateData* data);

const int16_t threshold_increments[THRESHOLD_NUM] = {
	50,
	-50,
	1,
};

const int16_t threshold_max[THRESHOLD_NUM] = {
	1023,
	1023,
	60,
};

const int16_t threshold_min[THRESHOLD_NUM] = {
	0,
	0,
	40,
};

const int EEPROM_ADDR_MARKER = 0;
const int EEPROM_ADDR_THRESHOLDS = 1;
const uint8_t EEPROM_MARKER = 1;

// EEPROM contents are not trustworthy - a half-finished save or a leftover
// from another sketch has to land somewhere sane.
int16_t clamp_threshold(int index, int16_t value) {
	if(value < threshold_min[index]) return threshold_min[index];
	if(value > threshold_max[index]) return threshold_max[index];
	return value;
}



Button microSwitch(microswitchPin);
Button guiButton1(guiBtn1);
Button guiButton2(guiBtn2);
Button externalDoorButton(externalDoorPin);

DistanceSensor ultraSensor(trigPin, echoPin);
// The thresholds here are only the compiled defaults; setup() replaces them
// with the saved config.
LightPulseSensor lightPulseSensor(lightPin, LIGHT_PULSE_TIMEOUT, LIGHT_LEVEL_THRESHOLD, LIGHT_OFF_THRESHOLD);

bool is_car_inside(StateData* data) {
	int distance = ultraSensor.get_distance();

	// No echo means we know nothing. Treating it as a very short distance would
	// arm the light trigger on an empty garage.
	if(distance == DistanceSensor::NO_READING) {
		return false;
	}

	return distance < data->thresholds[THRESHOLD_CAR_DISTANCE];
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
			DBG_PRINTLN(F("State switch: OPEN DOOR"));
			send_door_signal();
			break;

		case STATE_CLOSE_DOOR:
			DBG_PRINTLN(F("State switch: CLOSE DOOR"));
			send_door_signal();
			break;
		
		case STATE_IDLE:
			DBG_PRINTLN(F("State switch: IDLE"));
			// Drop anything the detector latched on the way in, so a pulse seen
			// just as we left the previous state cannot retrigger immediately.
			lightPulseSensor.did_pulse();
			break;

		case STATE_WAIT_FOR_SECOND_SIGNAL:
			DBG_PRINTLN(F("State switch: WAIT_FOR_SECOND_SIGNAL"));
			break;

		case STATE_CONFIG:
			DBG_PRINTLN(F("State switch: CONFIG"));
			// The press that got us here was consumed by update_idle; its release
			// would otherwise skip straight past the first config item.
			data->config.ignore_next_release = true;
			break;

		default:
			break;
	}
}

void update_idle(StateData* data) {
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
	if(millis() - data->entered_state_time >= LIGHT_TIMEOUT) {
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
	if(millis() - data->entered_state_time >= DOOR_DELAY) {
		switch_state(data, STATE_IDLE);
	}
}

void update_close_door(StateData* data) {
	if(millis() - data->entered_state_time >= DOOR_DELAY) {
		switch_state(data, STATE_IDLE);
	}
}

void update_config(StateData* data) {
	if(guiButton1.pressed()) {
		data->config.pressed_key_time = millis();
		data->config.pressing_key = true;
	}

	if(guiButton1.read() == Button::PRESSED && data->config.pressing_key && millis() - data->config.pressed_key_time >= CONFIG_SAVE_HOLD) {
			data->config.pressing_key = false;
			switch_state(data, STATE_IDLE);
			for(int i = 0; i < THRESHOLD_NUM; ++i) {
				EEPROM.put(EEPROM_ADDR_THRESHOLDS + i * sizeof(int16_t), data->thresholds[i]);
			}
			// Marker last: losing power mid-save then leaves the previous set
			// intact instead of flagging a half-written one as valid.
			EEPROM.write(EEPROM_ADDR_MARKER, EEPROM_MARKER);
			lightPulseSensor.upper_threshold = data->thresholds[THRESHOLD_LIGHT_ON];
			lightPulseSensor.lower_threshold = data->thresholds[THRESHOLD_LIGHT_OFF];
			return;
	}

	if(guiButton1.released()) {
		if(data->config.ignore_next_release) {
			data->config.ignore_next_release = false;
		} else {
			THRESHOLDS& state = data->config.config_state;
			state = (THRESHOLDS)((int)state + 1);
			if(state >= THRESHOLD_NUM) {
				state = (THRESHOLDS)0;
			}
		}
	}

	if(guiButton2.pressed()) {
		THRESHOLDS& state = data->config.config_state;
		data->thresholds[(int)state] += threshold_increments[(int)state];
		if(data->thresholds[(int)state] > threshold_max[(int)state]) {
			data->thresholds[(int)state] = threshold_min[(int)state];
		}

		if(data->thresholds[(int)state] < threshold_min[(int)state]) {
			data->thresholds[(int)state] = threshold_max[(int)state];
		}
	}
}

void update(StateData* data) {
	// Only these two states may react to light. The rest keep the ambient
	// average fresh without arming the detector, so the opener's own light
	// cannot be latched as a pulse and consumed on the way back to idle.
	const bool detect_pulses = data->current_state == STATE_IDLE
	                        || data->current_state == STATE_WAIT_FOR_SECOND_SIGNAL;
	lightPulseSensor.update(detect_pulses);

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

const char* get_config_name(THRESHOLDS state) {
	switch(state) {
		case THRESHOLD_LIGHT_ON: return "Upper";
		case THRESHOLD_LIGHT_OFF: return "Lower";
		case THRESHOLD_CAR_DISTANCE: return "Dist";
		default: return "ERROR";
	}
} 

int16_t get_config_value(StateData* data) {
	return data->thresholds[(int)data->config.config_state];
}

int update_count = 0;
unsigned long time_of_last_count = 0;
int current_fps = 0;

// Writes one space-padded field of fixed width, keeping a shadow copy of the
// screen so unchanged text costs no I2C traffic at all. Padding is what lets
// the refresh skip lcd.clear(), which would blank the display for a visible
// moment and force every cell to be rewritten - expensive at roughly six I2C
// transactions per character in 4-bit mode.
char lcd_shadow[2][17];

void lcd_field(uint8_t col, uint8_t row, uint8_t width, const char* text) {
	char field[17];
	uint8_t i = 0;
	while(i < width && text[i]) {
		field[i] = text[i];
		++i;
	}
	while(i < width) {
		field[i++] = ' ';
	}

	if(memcmp(&lcd_shadow[row][col], field, width) == 0) {
		return;
	}
	memcpy(&lcd_shadow[row][col], field, width);

	lcd.setCursor(col, row);
	for(uint8_t k = 0; k < width; ++k) {
		lcd.write(field[k]);
	}
}

void update_lcd(StateData* data) {
	char buffer[17];

	lcd_field(0, 0, 16, state_to_name(data->current_state));

	switch(data->current_state) {
		case STATE_IDLE: {
			int dist = ultraSensor.get_distance();
			int light_level = analogRead(lightPin);

			unsigned long time = millis() / 5000;
			if(time % 2 == 0) {
				if(dist == DistanceSensor::NO_READING) {
					snprintf(buffer, sizeof(buffer), "D:--");
				} else {
					snprintf(buffer, sizeof(buffer), "D:%i", dist);
				}
				lcd_field(0, 1, 10, buffer);

				snprintf(buffer, sizeof(buffer), "L:%i", light_level);
				lcd_field(10, 1, 6, buffer);
			} else {
				snprintf(buffer, sizeof(buffer), "F:%i", (int16_t)current_fps);
				lcd_field(0, 1, 10, buffer);

				snprintf(buffer, sizeof(buffer), "A:%i", (int16_t)lightPulseSensor.average);
				lcd_field(10, 1, 6, buffer);
			}

		} break;

		case STATE_CONFIG: {
			snprintf(buffer, sizeof(buffer), "%s: %i", get_config_name(data->config.config_state), get_config_value(data));
			lcd_field(0, 1, 16, buffer);
		} break;

		default:
			lcd_field(0, 1, 16, "");
			break;
	}
}

void setup() {
	lcd.init();
	lcd.clear();
	lcd.backlight();

	data.current_state = STATE_IDLE;
	data.entered_state_time = 0;
	data.config.config_state = THRESHOLD_LIGHT_ON;
	data.config.pressed_key_time = 0;
	data.config.pressing_key = false;
	data.config.ignore_next_release = false;

	if(EEPROM.read(EEPROM_ADDR_MARKER) != EEPROM_MARKER) {
		// We have not written data to EEPROM yet, initialize with constants
		data.thresholds[THRESHOLD_LIGHT_ON] = LIGHT_LEVEL_THRESHOLD;
		data.thresholds[THRESHOLD_LIGHT_OFF] = LIGHT_OFF_THRESHOLD;
		data.thresholds[THRESHOLD_CAR_DISTANCE] = CAR_DISTANCE;
	} else {
		for(int i = 0; i < THRESHOLD_NUM; ++i) {
			EEPROM.get(EEPROM_ADDR_THRESHOLDS + i * sizeof(int16_t), data.thresholds[i]);
			data.thresholds[i] = clamp_threshold(i, data.thresholds[i]);
		}
	}

	lightPulseSensor.upper_threshold = data.thresholds[THRESHOLD_LIGHT_ON];
	lightPulseSensor.lower_threshold = data.thresholds[THRESHOLD_LIGHT_OFF];

	setup_pins();

	microSwitch.begin();
	guiButton1.begin();
	guiButton2.begin();
	lightPulseSensor.begin();
	externalDoorButton.begin();

	DBG_BEGIN();
	time_of_last_print = millis();
	time_of_lcd_update = millis();
	time_of_last_count = millis();
}

void loop() {
	update_count++;
	if(millis() - time_of_last_count >= 1000) {
		time_of_last_count = millis();
		current_fps = update_count;
		update_count = 0;

		// Set reporting pins
		digitalWrite(doorStatus, is_door_closed() ? HIGH : LOW);
		digitalWrite(carStatus, is_car_inside(&data) ? HIGH : LOW);
	}

	// Only from idle: mid-cycle the door is already moving, and in config a
	// stray press should not blow away the edit. pressed() still runs first so
	// the edge is consumed rather than saved up for later.
	if(externalDoorButton.pressed() && data.current_state == STATE_IDLE) {
		DBG_PRINTLN(F("External Button Triggered"));
		// The state machine owns the 19 second door cycle and keeps the loop
		// running through it. The opener takes one toggle signal either way, so
		// the two states differ only in what they report.
		switch_state(&data, is_door_closed() ? STATE_OPEN_DOOR : STATE_CLOSE_DOOR);
	}

#ifdef DEBUG_SERIAL
	if(millis() - time_of_last_print >= DEBUG_PRINT_INTERVAL) {
		Serial.print(F("Distance: "));
		Serial.println(ultraSensor.get_distance());

		Serial.print(F("Light level: "));
		Serial.println(analogRead(lightPin));

		Serial.print(F("Microswitch: "));
		Serial.println(microSwitch.read() == Button::PRESSED);

		Serial.print(F("Gui1: "));
		Serial.println(guiButton1.read() == Button::PRESSED);

		Serial.print(F("Gui2: "));
		Serial.println(guiButton2.read() == Button::PRESSED);

		Serial.print(F("FPS: "));
		Serial.println(current_fps);

		time_of_last_print = millis();
	}
#endif

	if(millis() - time_of_lcd_update >= LCD_UPDATE_INTERVAL) {
		update_lcd(&data);
		time_of_lcd_update = millis();
	}

	update(&data);
}
