#include "Arduino.h"
#include "Button.h"
#include "DistanceSensor.h"
#include "LightPulseSensor.h"
#include "AccelDial.h"
#include "ParkAssist.h"
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
	STATE_PARKING,
	STATE_CLOSE_DOOR,
	STATE_CONFIG,
};

// Indexes thresholds[], threshold_min[] and threshold_max[], and doubles as
// the cursor for the config screen.
enum THRESHOLDS {
	THRESHOLD_LIGHT_ON,
	THRESHOLD_LIGHT_OFF,
	THRESHOLD_CAR_DISTANCE,
	THRESHOLD_PARK_FAR,
	THRESHOLD_PARK_NEAR,
	THRESHOLD_NUM,
};

struct StateData {
	STATES current_state;
	unsigned long entered_state_time;

	int16_t thresholds[THRESHOLD_NUM];

	// Tracks the microswitch across ticks so a closed->open transition can be
	// caught even when the Arduino did not command it itself (the car's own
	// remote opens the door directly; the switch is the only signal of that).
	bool door_was_closed;

	struct {
		unsigned long pressed_key_time;
		bool pressing_key;
		THRESHOLDS config_state;
		bool ignore_next_release;
	} config;
};

void update_lcd(StateData* data);
void aim_dial_at_config_value(StateData* data);

const int16_t threshold_max[THRESHOLD_NUM] = {
	1023,
	1023,
	60,
	100,
	50,
};

const int16_t threshold_min[THRESHOLD_NUM] = {
	0,
	0,
	5,
	15,
	5,
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
Button externalDoorButton(externalDoorPin);

DistanceSensor ultraSensor(trigPin, echoPin);
// The thresholds here are only the compiled defaults; setup() replaces them
// with the saved config.
LightPulseSensor lightPulseSensor(lightPin, LIGHT_PULSE_TIMEOUT, LIGHT_LEVEL_THRESHOLD, LIGHT_OFF_THRESHOLD);

// Only live while the config screen is up: begin() reseeds it from the resting
// click on the way in, so a knob turned in the meantime cannot bank a step.
AccelDial configDial(encoderClkPin, encoderDtPin);

ParkAssist parkAssist(ledRedPin, ledGreenPin, buzzerPin,
                       PARK_BEEP_MIN_INTERVAL, PARK_BEEP_MAX_INTERVAL,
                       PARK_BEEP_ON_DURATION, PARK_HOLD_TIME);

void apply_thresholds(StateData* data) {
	lightPulseSensor.upper_threshold = data->thresholds[THRESHOLD_LIGHT_ON];
	lightPulseSensor.lower_threshold = data->thresholds[THRESHOLD_LIGHT_OFF];
}

void save_thresholds(StateData* data) {
	for(int i = 0; i < THRESHOLD_NUM; ++i) {
		EEPROM.put(EEPROM_ADDR_THRESHOLDS + i * sizeof(int16_t), data->thresholds[i]);
	}
	// Marker last: losing power mid-save then leaves the previous set intact
	// instead of flagging a half-written one as valid.
	EEPROM.write(EEPROM_ADDR_MARKER, EEPROM_MARKER);
}

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
			LOG_EVENT(F("State switch: OPEN DOOR"));
			send_door_signal();
			break;

		case STATE_CLOSE_DOOR:
			LOG_EVENT(F("State switch: CLOSE DOOR"));
			send_door_signal();
			break;

		case STATE_PARKING:
			LOG_EVENT(F("State switch: PARKING"));
			parkAssist.reset();
			break;

		case STATE_IDLE:
			LOG_EVENT(F("State switch: IDLE"));
			// Drop anything the detector latched on the way in, so a pulse seen
			// just as we left the previous state cannot retrigger immediately.
			lightPulseSensor.did_pulse();
			// The only path into IDLE that could have come from PARKING; turn
			// its LED/buzzer off regardless of which state we actually left.
			parkAssist.off();
			break;

		case STATE_WAIT_FOR_SECOND_SIGNAL:
			LOG_EVENT(F("State switch: WAIT_FOR_SECOND_SIGNAL"));
			break;

		case STATE_CONFIG:
			LOG_EVENT(F("State switch: CONFIG"));
			// The press that got us here was consumed by update_idle; its release
			// would otherwise skip straight past the first config item.
			data->config.ignore_next_release = true;
			configDial.begin();
			aim_dial_at_config_value(data);
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
		switch_state(data, STATE_PARKING);
	}
}

void update_close_door(StateData* data) {
	if(millis() - data->entered_state_time >= DOOR_DELAY) {
		switch_state(data, STATE_IDLE);
	}
}

void update_parking(StateData* data) {
	int distance = ultraSensor.get_distance();
	if(parkAssist.update(distance, data->thresholds[THRESHOLD_PARK_FAR], data->thresholds[THRESHOLD_PARK_NEAR])) {
		switch_state(data, STATE_IDLE);
		return;
	}

	// Nobody arrived (or the sensor never saw them) - don't sit here forever,
	// since the light-pulse close flow only runs from STATE_IDLE.
	if(millis() - data->entered_state_time >= PARKING_TIMEOUT) {
		switch_state(data, STATE_IDLE);
	}
}

// The dial edits whichever threshold the cursor is on, so it takes that
// threshold's bounds as its range. A narrow range like Dist (40..60) works out
// to a maxStep of 1, which is what a 21-count span wants anyway.
void aim_dial_at_config_value(StateData* data) {
	int index = (int)data->config.config_state;
	configDial.setRange(threshold_min[index], threshold_max[index]);
	configDial.setValue(data->thresholds[index]);
}

void set_config_value(StateData* data, int16_t value) {
	int index = (int)data->config.config_state;
	data->thresholds[index] = clamp_threshold(index, value);
	apply_thresholds(data);

	// Redraw now rather than waiting for the next refresh - at a quarter second
	// of lag the knob feels disconnected from the number.
	update_lcd(data);
}

void update_config(StateData* data) {
	if(guiButton1.pressed()) {
		data->config.pressed_key_time = millis();
		data->config.pressing_key = true;
	}

	if(guiButton1.read() == Button::PRESSED && data->config.pressing_key && millis() - data->config.pressed_key_time >= CONFIG_SAVE_HOLD) {
			data->config.pressing_key = false;
			switch_state(data, STATE_IDLE);
			save_thresholds(data);
			apply_thresholds(data);
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
			aim_dial_at_config_value(data);
		}
	}

	if(configDial.update()) {
		set_config_value(data, (int16_t)configDial.value());
	}
}

void update(StateData* data) {
	// The door is most often opened by the car's own remote, straight to the
	// opener - the Arduino never sees that command, only the microswitch
	// releasing. Catch that here, gated to idle so an in-progress door cycle
	// or config edit can't be interrupted, and to an empty garage so a car
	// already parked and about to leave doesn't trigger a false "parked" beep.
	bool door_closed_now = is_door_closed();
	if(!door_closed_now && data->door_was_closed && data->current_state == STATE_IDLE && !is_car_inside(data)) {
		switch_state(data, STATE_PARKING);
	}
	data->door_was_closed = door_closed_now;

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

		case STATE_PARKING:
			update_parking(data);
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
unsigned long time_of_lcd_update = 0;
LiquidCrystal_I2C lcd(0x3F,16,2);

const char* state_to_name(STATES state) {
	switch(state) {
		case STATE_IDLE: return "Idle";
		case STATE_WAIT_FOR_SECOND_SIGNAL: return "Wait second";
		case STATE_OPEN_DOOR: return "Open door";
		case STATE_PARKING: return "Parking";
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
		case THRESHOLD_PARK_FAR: return "ParkFar";
		case THRESHOLD_PARK_NEAR: return "ParkNear";
		default: return "ERROR";
	}
}

int16_t get_config_value(StateData* data) {
	return data->thresholds[(int)data->config.config_state];
}

int update_count = 0;
unsigned long time_of_last_count = 0;
int current_fps = 0;

// Single tokens, unlike state_to_name(): these go into key=value output, so a
// space in one would split it into two fields.
const char* state_to_token(STATES state) {
	switch(state) {
		case STATE_IDLE: return "idle";
		case STATE_WAIT_FOR_SECOND_SIGNAL: return "wait_second";
		case STATE_OPEN_DOOR: return "opening";
		case STATE_PARKING: return "parking";
		case STATE_CLOSE_DOOR: return "closing";
		case STATE_CONFIG: return "config";
		default: return "error";
	}
}

// Everything the far end can observe, on one line, in reply to STATE.
// Threshold names match the config screen's labels, so whatever the LCD shows
// is also what SET accepts.
void print_state_payload(StateData* data) {
	int range = ultraSensor.get_distance();

	Serial.print(F("state="));  Serial.print(state_to_token(data->current_state));
	Serial.print(F(" door="));  Serial.print(is_door_closed() ? F("closed") : F("open"));
	Serial.print(F(" car="));   Serial.print(is_car_inside(data) ? F("yes") : F("no"));
	Serial.print(F(" range="));
	if(range == DistanceSensor::NO_READING) {
		Serial.print(F("--"));
	} else {
		Serial.print(range);
	}
	Serial.print(F(" light=")); Serial.print(analogRead(lightPin));
	Serial.print(F(" amb="));   Serial.print(lightPulseSensor.average);
	Serial.print(F(" gui="));   Serial.print(guiButton1.read() == Button::PRESSED);
	Serial.print(F(" fps="));   Serial.print(current_fps);

	for(int i = 0; i < THRESHOLD_NUM; ++i) {
		Serial.print(' ');
		Serial.print(get_config_name((THRESHOLDS)i));
		Serial.print('=');
		Serial.print(data->thresholds[i]);
	}
}

// The opener only understands one toggle signal, so which way the door will
// travel comes from the microswitch, and asking for the position it is already
// in is answered rather than acted on.
void cmd_door(StateData* data, bool want_open) {
	if(data->current_state != STATE_IDLE) {
		Serial.print(F("ERR busy state="));
		Serial.println(state_to_token(data->current_state));
		return;
	}

	if(is_door_closed() != want_open) {
		Serial.println(want_open ? F("OK already open") : F("OK already closed"));
		return;
	}

	switch_state(data, want_open ? STATE_OPEN_DOOR : STATE_CLOSE_DOOR);
	Serial.println(want_open ? F("OK opening") : F("OK closing"));
}

int threshold_by_name(const char* name) {
	for(int i = 0; i < THRESHOLD_NUM; ++i) {
		if(strcasecmp(name, get_config_name((THRESHOLDS)i)) == 0) {
			return i;
		}
	}

	return -1;
}

// SET changes the live value only; SAVE is what commits it to EEPROM.
void cmd_set(StateData* data) {
	const char* name = strtok(NULL, " ");
	const char* value = strtok(NULL, " ");
	if(!name || !value) {
		Serial.println(F("ERR usage: SET <name> <value>"));
		return;
	}

	int index = threshold_by_name(name);
	if(index < 0) {
		Serial.print(F("ERR no setting called "));
		Serial.println(name);
		return;
	}

	data->thresholds[index] = clamp_threshold(index, (int16_t)atoi(value));
	apply_thresholds(data);

	// If someone is standing at the config screen while this arrives, the dial
	// is still holding the old number and its next click would write it back.
	if(data->current_state == STATE_CONFIG) {
		aim_dial_at_config_value(data);
		update_lcd(data);
	}

	Serial.print(F("OK "));
	Serial.print(get_config_name((THRESHOLDS)index));
	Serial.print('=');
	Serial.println(data->thresholds[index]);
}

void handle_command(StateData* data, char* line) {
	const char* cmd = strtok(line, " ");
	if(!cmd) {
		return;
	}

	// Exact matches only. The ESP8266 dumps a ROM boot log at 74880 baud on
	// every reset, which arrives here as random bytes, and a loose match could
	// find a door command in the noise.
	if(strcasecmp(cmd, "OPEN") == 0) {
		cmd_door(data, true);
	} else if(strcasecmp(cmd, "CLOSE") == 0) {
		cmd_door(data, false);
	} else if(strcasecmp(cmd, "STATE") == 0) {
		Serial.print(F("OK "));
		print_state_payload(data);
		Serial.println();
	} else if(strcasecmp(cmd, "SET") == 0) {
		cmd_set(data);
	} else if(strcasecmp(cmd, "SAVE") == 0) {
		save_thresholds(data);
		Serial.println(F("OK saved"));
	} else {
		Serial.println(F("ERR unknown, try OPEN CLOSE STATE SET SAVE"));
	}
}

char cmd_buffer[32];
uint8_t cmd_length = 0;
bool cmd_overflow = false;

void read_commands(StateData* data) {
	while(Serial.available()) {
		char c = (char)Serial.read();

		if(c == '\n' || c == '\r') {
			bool overflowed = cmd_overflow;
			uint8_t length = cmd_length;
			cmd_overflow = false;
			cmd_length = 0;

			if(overflowed) {
				Serial.println(F("ERR line too long"));
			} else if(length > 0) {
				cmd_buffer[length] = '\0';
				handle_command(data, cmd_buffer);
			}

			// One command per pass through loop(), so a fast talker cannot hold
			// the state machine here.
			return;
		}

		if(cmd_length >= sizeof(cmd_buffer) - 1) {
			// Drop the whole line rather than truncate it into a command nobody
			// sent.
			cmd_overflow = true;
			continue;
		}

		cmd_buffer[cmd_length++] = c;
	}
}

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

	// Only the span that actually changed goes over the wire: editing one digit
	// of a threshold costs one character rather than sixteen, which matters
	// because the encoder is polled from the same loop this blocks.
	uint8_t first = 0;
	while(first < width && field[first] == lcd_shadow[row][col + first]) {
		++first;
	}
	if(first == width) {
		return;
	}

	uint8_t last = width - 1;
	while(last > first && field[last] == lcd_shadow[row][col + last]) {
		--last;
	}

	lcd.setCursor(col + first, row);
	for(uint8_t k = first; k <= last; ++k) {
		lcd_shadow[row][col + k] = field[k];
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

		case STATE_PARKING: {
			int dist = ultraSensor.get_distance();
			if(dist == DistanceSensor::NO_READING) {
				snprintf(buffer, sizeof(buffer), "D:-- ->%i", data->thresholds[THRESHOLD_PARK_NEAR]);
			} else {
				snprintf(buffer, sizeof(buffer), "D:%i ->%i", dist, data->thresholds[THRESHOLD_PARK_NEAR]);
			}
			lcd_field(0, 1, 16, buffer);
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
		data.thresholds[THRESHOLD_PARK_FAR] = PARK_FAR_DISTANCE_DEFAULT;
		data.thresholds[THRESHOLD_PARK_NEAR] = PARK_NEAR_DISTANCE_DEFAULT;
	} else {
		for(int i = 0; i < THRESHOLD_NUM; ++i) {
			EEPROM.get(EEPROM_ADDR_THRESHOLDS + i * sizeof(int16_t), data.thresholds[i]);
			data.thresholds[i] = clamp_threshold(i, data.thresholds[i]);
		}
	}

	apply_thresholds(&data);

	setup_pins();

	microSwitch.begin();
	guiButton1.begin();
	lightPulseSensor.begin();
	externalDoorButton.begin();
	parkAssist.begin();

	// Needs the microswitch pin mode set up (microSwitch.begin(), above) to
	// read a meaningful value.
	data.door_was_closed = is_door_closed();

	Serial.begin(SERIAL_BAUD);
	time_of_lcd_update = millis();
	time_of_last_count = millis();

	// Also tells the far end that the board just restarted.
	LOG_EVENT(F("Garage opener R6 ready"));
}

void loop() {
	read_commands(&data);

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
		LOG_EVENT(F("External button"));
		// The state machine owns the 19 second door cycle and keeps the loop
		// running through it. The opener takes one toggle signal either way, so
		// the two states differ only in what they report.
		switch_state(&data, is_door_closed() ? STATE_OPEN_DOOR : STATE_CLOSE_DOOR);
	}

	if(millis() - time_of_lcd_update >= LCD_UPDATE_INTERVAL) {
		update_lcd(&data);
		time_of_lcd_update = millis();
	}

	update(&data);
}
