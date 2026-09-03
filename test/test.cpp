// Host tests for the R6 serial command protocol. The sketch is included
// verbatim below and compiled against the stubs in stub/, so these checks run
// the same code the board does - only Arduino.h, EEPROM and the LCD are faked.
// Button, DistanceSensor and LightPulseSensor are the real implementations.
//
// Run with ./run_tests.sh

#include "Arduino.h"
#include "EEPROM.h"
#include <algorithm>
#include <cstdio>

uint32_t g_millis = 100000;
uint8_t  g_pin[32];   // set HIGH in main(), as INPUT_PULLUP would
uint16_t g_analog[32] = {0};
uint32_t g_pulse = 2471;          // ~42 cm
FakeSerial Serial;
FakeEEPROM EEPROM;

#include "../Garage_Opener_Logicer_R6/Garage_Opener_Logicer_R6.ino"

struct DialSim {
	uint8_t rest = 3;                      // both switches open, as the pull-ups leave them
	void click(int dir, uint32_t gap_ms);
};
void DialSim::click(int dir, uint32_t gap_ms) {
	uint8_t first_mask = (dir > 0) ? 0x02 : 0x01;   // clockwise moves CLK first
	uint8_t code = rest ^ first_mask;
	g_pin[encoderClkPin] = (code >> 1) & 1;
	g_pin[encoderDtPin]  = code & 1;
	g_millis += gap_ms / 2;
	loop();

	rest ^= 3;
	g_pin[encoderClkPin] = (rest >> 1) & 1;
	g_pin[encoderDtPin]  = rest & 1;
	g_millis += gap_ms - gap_ms / 2;
	loop();
}
static DialSim dial;
static void spin(int dir, int clicks, uint32_t gap_ms) {
	for (int i = 0; i < clicks; i++) dial.click(dir, gap_ms);
}

static void press(int pin, uint32_t hold_ms) {
	g_pin[pin] = LOW;
	g_millis += 60; loop();
	g_millis += hold_ms; loop();
	g_pin[pin] = HIGH;
	g_millis += 60; loop();
	g_millis += 60; loop();
}

static int fails = 0;
static void expect(const char* what, const std::string& got, const char* want) {
	bool ok = got.find(want) != std::string::npos;
	if (!ok) fails++;
	printf("  %-42s %-34s %s\n", what, ("[" + got.substr(0, got.find('\n') == std::string::npos ? got.size() : got.find('\n')) + "]").c_str(),
	       ok ? "ok" : "FAIL");
	if (!ok) printf("      wanted to contain: %s\n", want);
}
static void check(const char* what, long got, long want) {
	bool ok = got == want;
	if (!ok) fails++;
	printf("  %-42s %-34ld %s\n", what, got, ok ? "ok" : "FAIL");
}

// Let the Button debounce settle on a new pin level.
static void settle() { for (int i = 0; i < 3; i++) { g_millis += 60; loop(); } }
static std::string send(const char* line) {
	Serial.take();
	Serial.feed(std::string(line) + "\n");
	g_millis += 2;
	loop();                       // one command per pass
	return Serial.take();
}
static void set_door_closed(bool closed) { g_pin[microswitchPin] = closed ? HIGH : LOW; settle(); }

int main() {
	for (int i = 0; i < 32; i++) g_pin[i] = HIGH;
	g_analog[A0] = 318;
	setup();
	Serial.take();
	g_pin[doorPin] = LOW;
	set_door_closed(true);

	printf("--- STATE ---\n");
	expect("STATE reply", send("STATE"), "OK state=idle door=closed car=yes range=42");
	expect("...carries the thresholds", send("STATE"), "Upper=300 Lower=200 Dist=45");
	g_pulse = 0;                  // no echo
	g_millis += 1100;
	expect("no echo reports range=-- car=no", send("STATE"), "range=-- light=318");
	expect("...and not a close car", send("STATE"), "car=no");
	g_pulse = 2471; g_millis += 1100; send("STATE");

	printf("--- OPEN / CLOSE ---\n");
	g_pin[doorPin] = 0;
	expect("OPEN when closed", send("OPEN"), "OK opening");
	check("...pulsed the door relay", g_pin[doorPin], LOW);   // pulse ends low
	expect("OPEN while the cycle runs", send("OPEN"), "ERR busy state=opening");
	expect("CLOSE while the cycle runs", send("CLOSE"), "ERR busy state=opening");

	g_millis += DOOR_DELAY + 10; loop();      // door cycle finishes
	set_door_closed(false);
	expect("OPEN when already open", send("open"), "OK already open");
	expect("CLOSE when open", send("CLOSE"), "OK closing");
	g_millis += DOOR_DELAY + 10; loop();
	set_door_closed(true);
	expect("CLOSE when already closed", send("close"), "OK already closed");

	printf("--- SET / SAVE ---\n");
	expect("SET Upper 400", send("SET Upper 400"), "OK Upper=400");
	check("...applied to the sensor", lightPulseSensor.upper_threshold, 400);
	expect("SET is case insensitive", send("set lower 150"), "OK Lower=150");
	expect("SET clamps high", send("SET Dist 999"), "OK Dist=60");
	expect("SET clamps low", send("SET Dist 1"), "OK Dist=40");
	expect("SET unknown name", send("SET bogus 5"), "ERR no setting called bogus");
	expect("SET missing value", send("SET Upper"), "ERR usage");
	check("nothing saved to EEPROM yet", EEPROM.read(EEPROM_ADDR_MARKER), 255);
	expect("SAVE", send("SAVE"), "OK saved");
	check("...marker written", EEPROM.read(EEPROM_ADDR_MARKER), EEPROM_MARKER);

	printf("--- persistence across a reboot ---\n");
	setup();
	Serial.take();
	check("Upper survived", data.thresholds[THRESHOLD_LIGHT_ON], 400);
	check("Lower survived", data.thresholds[THRESHOLD_LIGHT_OFF], 150);
	check("Dist survived", data.thresholds[THRESHOLD_CAR_DISTANCE], 40);

	printf("--- garbage and malformed input ---\n");
	set_door_closed(true);
	g_pin[doorPin] = 0;
	expect("unknown command lists the real ones", send("HELLO"), "ERR unknown, try OPEN CLOSE STATE SET SAVE");
	// A burst of ESP8266 boot-log noise read at the wrong baud.
	Serial.take();
	Serial.feed(std::string("\xff\x1b\x80OPENISH\x03 rst:0x1 ap\xf0\n"));
	g_millis += 2; loop();
	expect("noise containing OPEN is not a command", Serial.take(), "ERR unknown");
	check("...door relay untouched by noise", g_pin[doorPin], 0);
	check("...state still idle", data.current_state, STATE_IDLE);

	std::string overlong(60, 'x');
	expect("overlong line rejected", send(overlong.c_str()), "ERR line too long");
	expect("...and the next command still works", send("STATE"), "OK state=idle");
	Serial.take(); Serial.feed("STATE\r\n"); g_millis += 2; loop();
	expect("CRLF line endings", Serial.take(), "OK state=idle");

	printf("--- pacing ---\n");
	Serial.rx.clear();
	Serial.take();
	Serial.feed("STATE\nSTATE\n");
	g_millis += 2; loop();
	std::string one = Serial.take();
	check("one command per loop pass", (long)std::count(one.begin(), one.end(), '\n'), 1);
	g_millis += 2; loop();
	expect("second command handled next pass", Serial.take(), "OK state=idle");

	printf("--- encoder in config mode ---\n");
	send("SET Upper 300"); send("SET Lower 200"); send("SET Dist 45");
	press(guiBtn1, 10);                     // enter config, cursor on Upper
	check("entered config", data.current_state, STATE_CONFIG);
	check("cursor starts on Upper", data.config.config_state, THRESHOLD_LIGHT_ON);

	spin(+1, 5, 300);                       // slow: one count per click
	check("5 slow clicks CW", data.thresholds[THRESHOLD_LIGHT_ON], 305);
	spin(-1, 3, 300);
	check("3 slow clicks CCW", data.thresholds[THRESHOLD_LIGHT_ON], 302);
	check("...pushed to the light sensor", lightPulseSensor.upper_threshold, 302);

	spin(+1, 12, 12);                       // fast: acceleration kicks in
	check("a fast spin moves more than 12", data.thresholds[THRESHOLD_LIGHT_ON] > 314, 1);
	printf("  %-42s %-34d ok\n", "  (value after 12 fast clicks)", data.thresholds[THRESHOLD_LIGHT_ON]);

	send("SET Upper 3");
	spin(-1, 40, 300);
	check("clamps at the low bound", data.thresholds[THRESHOLD_LIGHT_ON], 0);
	// An Upper of 0 makes the light sensor see a pulse in any reading, which
	// would pull the state machine out of idle the moment config exits.
	send("SET Upper 300");

	press(guiBtn1, 10);                     // cursor -> Lower
	check("cursor advanced", data.config.config_state, THRESHOLD_LIGHT_OFF);
	int16_t upper_before = data.thresholds[THRESHOLD_LIGHT_ON];
	spin(+1, 4, 300);
	check("dial now edits Lower", data.thresholds[THRESHOLD_LIGHT_OFF], 204);
	check("...and leaves Upper alone", data.thresholds[THRESHOLD_LIGHT_ON], upper_before);

	press(guiBtn1, 10);                     // cursor -> Dist (range 40..60)
	check("cursor on Dist", data.config.config_state, THRESHOLD_CAR_DISTANCE);
	spin(+1, 30, 12);                       // fast, but a 21-count span gets maxStep 1
	check("narrow range clamps at 60", data.thresholds[THRESHOLD_CAR_DISTANCE], 60);
	spin(-1, 30, 12);
	check("...and at 40", data.thresholds[THRESHOLD_CAR_DISTANCE], 40);

	spin(+1, 3, 300);
	check("dial steps back up off the bound", data.thresholds[THRESHOLD_CAR_DISTANCE], 43);

	press(guiBtn1, CONFIG_SAVE_HOLD + 200); // long press saves and exits
	check("long press left config", data.current_state, STATE_IDLE);
	int16_t saved = 0;
	EEPROM.get(EEPROM_ADDR_THRESHOLDS + 2 * sizeof(int16_t), saved);
	check("...and wrote the dialled value", saved, 43);

	printf("\nlongest reply line: %zu bytes\n", send("STATE").size());

	printf(fails ? "\n%d FAILURES\n" : "\nall checks passed\n", fails);
	return fails ? 1 : 0;
}
