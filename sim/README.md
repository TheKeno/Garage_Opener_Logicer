# R6 simulator and tests

Runs the real `Garage_Opener_Logicer_R6` sketch on a PC against a simulated
garage, with no board needed. The sketch is compiled unmodified; only the
Arduino core, `EEPROM` and `LiquidCrystal_I2C` are replaced.

```
make test     # the test suite
make run      # interactive simulator in the terminal (or ./build/garage_sim --help)
make avr      # check the sketch still builds for an Uno
```

The build needs `g++` with 32-bit support (`lib32-gcc-libs` / `g++-multilib`)
and the Button library. It looks for Button in `~/Arduino/libraries/Button` and
on the old backup drive; otherwise run `make BUTTON_DIR=/path/to/Button`.

## What is simulated

| | |
|---|---|
| door | single-button opener: a relay pulse on `doorPin` toggles it (open → stop → close → stop); 14 s travel; the microswitch reads HIGH only when fully shut |
| car | position along the garage, a side profile (hood, windshield, roof, trunk) that can't pass a door that isn't up or go through the back wall, headlights |
| HC-SR04 | hangs from the ceiling (165 cm up, 320 cm in from the door) looking down: reads the floor, or the part of the car under it. Distance falls as the windshield passes under and bottoms out at the roof, about 13 cm. `garage.sonar_ok = false` simulates a dead sensor (30 ms `pulseIn()` timeout) |
| light on A0 | dark garage + daylight through the door + the opener's courtesy lamp + headlights + noise |
| panel | GUI button, the ESP's door input, the rotary encoder (with its transient state, so slow polling drops clicks), optional contact bounce |
| LCD | the 16×2 screen contents; each character costs the ~1.6 ms of I2C it really takes |
| UART | 115200 baud with the AVR's 64-byte TX buffer (long replies block) and 63-byte RX ring (overflow drops bytes) |
| EEPROM | 1 KB that survives a power cycle; counts writes; 3.4 ms per write |

Time is virtual. It only advances when the sketch does something that takes
time on the real chip, such as `delay()`, `pulseIn()`, an ADC read or an LCD
write. Scheduled events, like a headlight flash, still land at their exact time
even while the sketch is blocked. Each `power_on()` `dlopen`s a fresh copy of
the firmware, so a power cycle really does reset every global.

## Writing tests

```cpp
TEST(arrive, remote_into_an_empty_garage_starts_park_assist) {
  sim::Sim s;
  boot(s);
  s.car_outside();
  s.remote();
  CHECK(wait_state(s, "Parking", 200));
  CHECK_EQ(s.pin(ledRedPin), HIGH);
}
```

`s.state()` is LCD row 0. `s.lines` holds every serial line with its
timestamp. `s.edges` records every output pin change. `s.global<T>("name")`
gives access to a firmware global for white-box checks. `./build/run_tests -v
leave` runs only tests matching "leave" and echoes the serial output.

`KNOWN_ISSUE(...)` tests assert the behaviour we want and currently fail. They
are reported, but they don't fail the run. If one starts passing, the runner
says so.

## Limits

- `int` is 32 bits here and 16 bits on the AVR. `long` matches, thanks to
  `-m32`, so `millis()` arithmetic wraps correctly. An `int` overflow would
  still go unnoticed, though.
- The light and timing numbers are plausible guesses, not measurements from
  your garage. Adjust `sim::Optics` and `sim::Timing` to match the real thing.
- The ESP8266 sketch (`esp/`) is not simulated. The tests play its part over
  the UART and the external door input.
