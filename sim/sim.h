#pragma once
// A simulated garage for Garage_Opener_Logicer_R6.
//
// The sketch is compiled, unmodified, into firmware.so and loaded with
// dlopen(), so every power_on() starts from freshly constructed globals exactly
// like a reset does on the board. Its calls into the Arduino core (stubs/) land
// here, where they read from and act on a model of the garage:
//
//   door      a single-button opener: the relay on doorPin toggles it, and the
//             microswitch reads HIGH only while the door is fully shut
//   car       a position along the garage and a side profile (hood,
//             windshield, roof); the HC-SR04 in the ceiling looks straight
//             down and reads the floor, or whatever part of the car is under
//             it. Headlights reach the light sensor once the car can see it.
//   light     the LDR on A0: dark garage + daylight through the door + the
//             opener's courtesy lamp + headlights + noise
//   panel     the GUI button, the ESP's external door input, the rotary
//             encoder, the 16x2 LCD, the park-assist LEDs and buzzer
//   UART      115200 baud with the AVR's 64-byte buffers, so long replies
//             block and bytes sent while the sketch is busy queue (or drop)
//   EEPROM    1 KB that survives power_cycle()
//
// Time is virtual and only moves when the sketch does something that takes
// time on a 16 MHz Uno - a pass of loop(), an ADC conversion, an echo,
// a delay(), an I2C write to the LCD, an EEPROM write, a full TX buffer.
// Scheduled world events (a headlight flash, a button press, a knob click)
// fire at their exact time even while the sketch is blocked in a delay().
#include <cstdint>
#include <functional>
#include <deque>
#include <map>
#include <random>
#include <string>
#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#include "config.h"  // the sketch's pin map and timing constants
#pragma GCC diagnostic pop

namespace sim {

// What things cost on the real board, in microseconds.
struct Timing {
  uint32_t loop_us         = 40;    // loop() bookkeeping not modelled below
  uint32_t adc_us          = 112;   // one analogRead()
  uint32_t micros_step     = 4;     // micros() resolution at 16 MHz
  uint32_t lcd_byte_us     = 1600;  // one LCD character/command: 6 PCF8574 writes at 100 kHz
  uint32_t lcd_clear_us    = 2000;  // ...plus the HD44780's own clear time
  uint32_t eeprom_write_us = 3400;
  uint32_t echo_setup_us   = 450;   // HC-SR04 trigger to start of echo
};

struct Door {
  double travel_s = 14;             // fully shut to fully open
  bool   reverse_on_toggle = false; // false: open -> stop -> close -> stop
  double pos      = 0;              // 0 shut .. 1 open
  int    motion   = 0;              // +1 opening, -1 closing, 0 stopped
  int    last_dir = -1;
  int    toggles  = 0;              // relay pulses and remote presses seen
  bool shut() const { return pos <= 0; }
  bool open() const { return pos >= 1; }
};

// Positions run along the garage, in cm from the door plane: negative is out
// in the driveway, garage.depth_cm is the back wall.
struct Car {
  bool   present    = false;  // false: nowhere near
  double pos_cm     = -900;   // where the front bumper is
  double target_cm  = -900;
  double speed      = 30;     // cm/s while driving to target_cm
  bool   headlights = false;
  // Side profile: height above the floor at each distance back from the
  // front bumper - bumper, hood, windshield up to the roof, rear window,
  // trunk. Interpolated linearly; the last point is the rear.
  std::vector<std::pair<double, double>> profile = {
      {0, 60}, {20, 85}, {110, 95}, {190, 152}, {330, 152}, {400, 105}, {450, 95}};
  double length_cm() const { return profile.back().first; }
  double height_at(double from_front_cm) const;  // 0 off the car
};

struct Garage {
  double depth_cm    = 550;   // door plane to back wall
  double sensor_x_cm = 320;   // the HC-SR04's spot in the ceiling, from the door
  double sensor_h_cm = 165;   // ...and its height above the floor
  double sound_cm_us = 0.0343;
  bool   sonar_ok    = true;  // false: unplugged or broken, never an echo
};

// Light at the LDR, in ADC counts.
struct Optics {
  double dark       = 60;   // door shut, lamp off
  double daylight   = 0;    // added in proportion to how open the door is (0 = night)
  double lamp       = 150;  // the opener's courtesy lamp
  double lamp_s     = 180;  // ...which stays on this long after the door stops
  double headlights = 450;  // car headlights reflected onto the sensor
  double noise      = 2;    // Gaussian sigma
};

struct Edge {
  uint64_t at_us;
  uint8_t  pin, level;
};

struct Line {  // one line the sketch printed
  uint64_t    at_us;
  std::string text;
};

class Sim {
 public:
  Timing t;
  Door   door;
  Car    car;
  Garage garage;
  Optics light;
  double bounce_ms = 0;      // contact bounce on buttons and the microswitch

  uint64_t now_us = 0;
  uint64_t loops  = 0;

  // uptime_us: what the firmware's clock reads at the first power_on(), to
  // test the millis()/micros() rollovers without simulating 49 days. Later
  // power cycles start from zero, as a reset does.
  explicit Sim(uint64_t uptime_us = 0, uint32_t seed = 1);
  ~Sim();
  Sim(const Sim &) = delete;

  // ------------------------------------------------------------- the board --
  void power_on();     // fresh firmware image, then setup()
  void power_off();
  void power_cycle() { power_off(); power_on(); }
  bool powered() const { return fw_ != nullptr; }
  void loop_once();
  void run_ms(double ms);
  // Run until done() holds, checked after every loop(). False on timeout.
  bool run_until(const std::function<bool()> &done, double timeout_ms);

  // Time passing outside loop() - used by the stubs. Fires due events.
  void advance(uint64_t us);

  // ------------------------------------------------------- world: schedule --
  void at(double ms_from_now, std::function<void()> fn);

  // ------------------------------------------------------- world: actions --
  void remote();                                // the car's remote, straight to the opener
  void flash(int times = 1, double on_ms = 150, double off_ms = 350);
  void car_outside();                           // car turns up in the driveway
  void car_parked();                            // car at the stop, where a driver would leave it
  void drive_to(double pos_cm, double speed_cm_s = 30);
  void drive_out(double speed_cm_s = 100) { drive_to(-car.length_cm() - 300, speed_cm_s); }
  void car_gone();
  // Where the front bumper has to be, driving in, for the sonar to first read
  // `reading_cm` or less. A driver who stops on the steady tone ends up just
  // past pos_for_reading(ParkNear).
  double pos_for_reading(double reading_cm) const;
  void press(int pin);
  void release(int pin);
  void tap(int pin, double hold_ms = 150);
  // Rotary encoder detents, one every interval_ms, scheduled from now. The
  // knob passes through its transient state for part of each click; if loop()
  // is busy for all of it the sketch's AccelDial drops the click.
  void turn(int dir, int clicks, double interval_ms);
  double turn_duration_ms(int clicks, double interval_ms) const { return clicks * interval_ms; }
  // Bytes arrive at the baud rate starting now, into the 63-byte RX ring.
  void send(const std::string &text);

  // --------------------------------------------------------- observations --
  std::string lcd(int row) const;               // 16 chars
  std::string state() const;                    // LCD row 0, trimmed: "Idle", "Parking", ...
  uint8_t     pin(int p) const { return out_[p]; }
  int         light_now();                      // what analogRead(A0) would see now
  double      sonar_cm() const;                 // true distance below the sensor, -1 for no echo
  int         echo_cm() const;                  // ...rounded
  uint32_t    fw_millis() const { return (uint32_t)(fw_us() / 1000); }
  double      secs() const { return now_us / 1e6; }

  std::string        tx;                        // everything printed, raw
  std::vector<Line>  lines;
  std::function<void(const Line &)> on_line;    // live, e.g. for the TUI
  std::string        take() { std::string s; s.swap(tx); return s; }
  // Send a line, run until the reply (a line starting OK or ERR) comes back.
  std::string command(const std::string &line, double timeout_ms = 200);

  std::vector<Edge>  edges;                     // every output pin change
  uint8_t            eeprom[1024];
  uint32_t           eeprom_writes = 0;
  uint32_t           rx_dropped    = 0;
  uint64_t           longest_loop_us = 0;

  // A global from the running firmware, for white-box checks.
  template <class T> T *global(const char *symbol) { return static_cast<T *>(lookup(symbol)); }

  // ------------------------------------------- the stubs' side of the wire --
  unsigned long hw_millis() const { return fw_millis(); }
  unsigned long hw_micros() const { return (unsigned long)(uint32_t)(fw_us() - fw_us() % t.micros_step); }
  void          hw_pin_mode(uint8_t p, uint8_t mode) { modes_[p] = mode; }
  void          hw_write(uint8_t p, uint8_t level);
  int           hw_read(uint8_t p);
  int           hw_analog(uint8_t p);
  unsigned long hw_pulse_in(uint8_t p, uint8_t state, unsigned long timeout);
  void          hw_lcd_init();
  void          hw_lcd_clear();
  void          hw_lcd_cursor(uint8_t col, uint8_t row);
  void          hw_lcd_write(uint8_t c);
  void          hw_serial_begin(unsigned long baud) { baud_ = baud; }
  int           hw_serial_available();
  int           hw_serial_read(bool consume);
  void          hw_serial_write(uint8_t c);
  void          hw_serial_flush();
  uint8_t       hw_eeprom_read(int addr) const { return eeprom[addr & 1023]; }
  void          hw_eeprom_write(int addr, uint8_t v);

 private:
  struct Event {
    uint64_t at, seq;
    std::function<void()> fn;
  };
  struct Contact {     // a switch input with optional bounce
    bool     level = true;
    uint64_t since = 0;
  };

  void  physics(uint64_t dt_us);
  void  set_contact(Contact &c, bool level);
  bool  read_contact(const Contact &c) const;
  bool  headlights_reach() const;
  void  door_toggle();
  void *lookup(const char *symbol);
  void  pump_rx();

  uint64_t fw_us() const { return now_us - boot_us_ + uptime_us_; }

  uint64_t boot_us_ = 0, uptime_us_ = 0;
  void *fw_ = nullptr;
  void (*setup_)() = nullptr;
  void (*loop_)()  = nullptr;

  std::multimap<uint64_t, Event> events_;
  uint64_t seq_ = 0;

  uint8_t  out_[32]   = {0};
  uint8_t  modes_[32] = {0};
  Contact  microswitch_, gui_, ext_;
  uint8_t  knob_      = 3;      // encoder (CLK << 1) | DT, both open at rest
  uint64_t lamp_until_ = 0;

  char     screen_[2][16];
  uint8_t  col_ = 0, row_ = 0;

  unsigned long baud_ = 115200;
  uint64_t tx_idle_at_ = 0;
  std::string partial_;
  std::deque<std::pair<uint64_t, uint8_t>> rx_wire_;  // in flight
  std::deque<uint8_t> rx_ring_;

  std::mt19937 rng_;
  std::normal_distribution<double> gauss_{0, 1};
};

// The Sim the stubs talk to. Set by the constructor.
extern Sim *current;

// Print every line the sketch sends to stdout as it happens.
extern bool echo_serial;

}  // namespace sim
