#include "sim.h"

#include <Arduino.h>
#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>

#include <dlfcn.h>
#include <unistd.h>

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace sim {

Sim *current = nullptr;
bool echo_serial = false;

static std::string firmware_path() {
  if (const char *env = getenv("GARAGE_FIRMWARE")) return env;
  char exe[4096];
  ssize_t n = readlink("/proc/self/exe", exe, sizeof exe - 1);
  if (n <= 0) return "./firmware.so";
  exe[n] = 0;
  std::string dir(exe);
  return dir.substr(0, dir.rfind('/')) + "/firmware.so";
}

Sim::Sim(uint64_t uptime_us, uint32_t seed) : uptime_us_(uptime_us), rng_(seed) {
  if (current) throw std::logic_error("only one sim::Sim may exist at a time");
  current = this;
  memset(eeprom, 0xFF, sizeof eeprom);  // erased, as shipped
  memset(screen_, ' ', sizeof screen_);
  microswitch_.level = door.shut();
}

Sim::~Sim() {
  power_off();
  current = nullptr;
}

// ------------------------------------------------------------------ board --

void Sim::power_on() {
  if (fw_) return;
  std::string path = firmware_path();
  // A firmware image still mapped from last time would keep its globals.
  if (void *stale = dlopen(path.c_str(), RTLD_NOW | RTLD_NOLOAD)) {
    dlclose(stale);
    throw std::runtime_error("firmware.so did not unload; globals would survive a power cycle");
  }
  fw_ = dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
  if (!fw_) throw std::runtime_error(std::string("cannot load firmware: ") + dlerror());
  setup_ = reinterpret_cast<void (*)()>(lookup("_Z5setupv"));
  loop_  = reinterpret_cast<void (*)()>(lookup("_Z4loopv"));

  memset(out_, 0, sizeof out_);
  memset(modes_, 0, sizeof modes_);
  memset(screen_, ' ', sizeof screen_);
  col_ = row_ = 0;
  rx_ring_.clear();
  partial_.clear();
  tx_idle_at_ = now_us;
  boot_us_ = now_us;

  setup_();
}

void Sim::power_off() {
  if (!fw_) return;
  dlclose(fw_);
  fw_ = nullptr;
  setup_ = loop_ = nullptr;
  uptime_us_ = 0;
  memset(out_, 0, sizeof out_);  // relay, LEDs and buzzer drop out
}

void *Sim::lookup(const char *symbol) {
  void *p = fw_ ? dlsym(fw_, symbol) : nullptr;
  if (!p) throw std::runtime_error(std::string("firmware has no symbol ") + symbol);
  return p;
}

void Sim::loop_once() {
  if (!fw_) {
    advance(t.loop_us);
    return;
  }
  uint64_t t0 = now_us;
  loop_();
  advance(t.loop_us);
  loops++;
  longest_loop_us = std::max(longest_loop_us, now_us - t0);
}

void Sim::run_ms(double ms) {
  uint64_t end = now_us + (uint64_t)(ms * 1000);
  while (now_us < end) loop_once();
}

bool Sim::run_until(const std::function<bool()> &done, double timeout_ms) {
  uint64_t end = now_us + (uint64_t)(timeout_ms * 1000);
  while (now_us < end) {
    loop_once();
    if (done()) return true;
  }
  return done();
}

void Sim::advance(uint64_t us) {
  uint64_t end = now_us + us;
  while (!events_.empty() && events_.begin()->first <= end) {
    auto it = events_.begin();
    uint64_t when = std::max(it->first, now_us);
    std::function<void()> fn = std::move(it->second.fn);
    events_.erase(it);
    uint64_t dt = when - now_us;
    now_us = when;
    physics(dt);
    fn();
  }
  uint64_t dt = end - now_us;
  now_us = end;
  physics(dt);
}

void Sim::at(double ms_from_now, std::function<void()> fn) {
  uint64_t when = now_us + (uint64_t)std::llround(std::max(0.0, ms_from_now) * 1000);
  events_.emplace(when, Event{when, seq_++, std::move(fn)});
}

// ---------------------------------------------------------------- physics --

void Sim::physics(uint64_t dt_us) {
  if (dt_us == 0) return;
  double dt = dt_us / 1e6;

  if (door.motion) {
    bool was_shut = door.shut();
    door.pos += door.motion * dt / door.travel_s;
    if (door.pos >= 1 || door.pos <= 0) {
      door.pos = door.pos >= 1 ? 1 : 0;
      door.motion = 0;
      lamp_until_ = now_us + (uint64_t)(light.lamp_s * 1e6);
    }
    if (door.shut() != was_shut) set_contact(microswitch_, door.shut());
  }

  if (car.present && car.pos_cm != car.target_cm) {
    double step = std::min(car.speed * dt, std::fabs(car.target_cm - car.pos_cm));
    double next = car.pos_cm + (car.target_cm > car.pos_cm ? step : -step);
    // A door that is not fully up is in the way: the bumper cannot pass it
    // coming in, nor the rear going out. And the back wall is the back wall.
    double len = car.length_cm();
    if (door.pos < 0.95) {
      if (car.pos_cm <= 0 && next > 0) next = -2;
      if (car.pos_cm - len >= 0 && next - len < 0) next = len + 2;
    }
    next = std::min(next, garage.depth_cm - 2);
    if (next == car.pos_cm) car.target_cm = car.pos_cm;  // blocked
    car.pos_cm = next;
  }
}

void Sim::door_toggle() {
  door.toggles++;
  if (door.motion != 0) {
    if (door.reverse_on_toggle) {
      door.motion = -door.motion;
      door.last_dir = door.motion;
    } else {
      door.last_dir = door.motion;
      door.motion = 0;
      lamp_until_ = now_us + (uint64_t)(light.lamp_s * 1e6);
    }
    return;
  }
  int dir = door.shut() ? +1 : door.open() ? -1 : -door.last_dir;
  door.motion = dir;
  door.last_dir = dir;
  lamp_until_ = UINT64_MAX;  // on while moving
}

void Sim::set_contact(Contact &c, bool level) {
  if (c.level == level) return;
  c.level = level;
  c.since = now_us;
}

bool Sim::read_contact(const Contact &c) const {
  uint64_t bounce_us = (uint64_t)(bounce_ms * 1000);
  if (bounce_us && now_us - c.since < bounce_us)
    return ((now_us - c.since) / 250) % 2 ? c.level : !c.level;
  return c.level;
}

bool Sim::headlights_reach() const {
  return car.present && (car.pos_cm > 0 || door.pos > 0.3);
}

int Sim::light_now() {
  double v = light.dark + light.daylight * door.pos;
  if (now_us < lamp_until_) v += light.lamp;
  if (car.headlights && headlights_reach()) v += light.headlights;
  if (light.noise > 0) v += light.noise * gauss_(rng_);
  return std::clamp((int)std::lround(v), 0, 1023);
}

double Car::height_at(double x) const {
  if (x < 0 || x > length_cm()) return 0;
  for (size_t i = 1; i < profile.size(); i++) {
    auto a = profile[i - 1], b = profile[i];
    if (x <= b.first) return a.second + (b.second - a.second) * (x - a.first) / (b.first - a.first);
  }
  return profile.back().second;
}

double Sim::sonar_cm() const {
  if (!garage.sonar_ok) return -1;
  double h = car.present ? car.height_at(car.pos_cm - garage.sensor_x_cm) : 0;
  return garage.sensor_h_cm - h;
}

int Sim::echo_cm() const {
  double cm = sonar_cm();
  return cm < 0 ? -1 : (int)std::lround(cm);
}

double Sim::pos_for_reading(double reading_cm) const {
  for (double pos = garage.sensor_x_cm; pos <= garage.sensor_x_cm + car.length_cm(); pos += 0.5)
    if (garage.sensor_h_cm - car.height_at(pos - garage.sensor_x_cm) <= reading_cm) return pos;
  return garage.depth_cm - 2;
}

// ---------------------------------------------------------- world actions --

void Sim::remote() { door_toggle(); }

void Sim::flash(int times, double on_ms, double off_ms) {
  for (int i = 0; i < times; i++) {
    double start = i * (on_ms + off_ms);
    at(start, [this] { car.headlights = true; });
    at(start + on_ms, [this] { car.headlights = false; });
  }
}

void Sim::car_outside() {
  car.present = true;
  car.pos_cm = car.target_cm = -300;
}

void Sim::car_parked() {
  car.present = true;
  car.pos_cm = car.target_cm = pos_for_reading(PARK_NEAR_DISTANCE_DEFAULT) + 5;
}

void Sim::drive_to(double pos_cm, double speed_cm_s) {
  car.present = true;
  car.target_cm = pos_cm;
  car.speed = speed_cm_s;
}

void Sim::car_gone() {
  car.present = false;
  car.headlights = false;
  car.pos_cm = car.target_cm = -1000;
}

void Sim::press(int p) {
  if (p == guiBtn1) set_contact(gui_, false);
  else if (p == externalDoorPin) set_contact(ext_, false);
  else throw std::invalid_argument("not a button pin");
}

void Sim::release(int p) {
  if (p == guiBtn1) set_contact(gui_, true);
  else if (p == externalDoorPin) set_contact(ext_, true);
  else throw std::invalid_argument("not a button pin");
}

void Sim::tap(int p, double hold_ms) {
  press(p);
  at(hold_ms, [this, p] { release(p); });
}

// One click of the datasheet's module: from one rest state (both switches the
// same) through a transient (the leading switch moved) to the opposite rest
// state. CLK leads for clockwise. The transient lasts a fraction of the click,
// so a fast spin gives the sketch less time to see it.
void Sim::turn(int dir, int clicks, double interval_ms) {
  double transient = std::clamp(interval_ms * 0.3, 2.0, 40.0);
  for (int i = 0; i < clicks; i++) {
    double start = i * interval_ms;
    // CLK (bit 1) leads clockwise, DT (bit 0) counter-clockwise; the other
    // switch follows to complete the click.
    uint8_t lead = dir > 0 ? 0x02 : 0x01;
    at(start, [this, lead] { knob_ ^= lead; });
    at(start + transient, [this, lead] { knob_ ^= (uint8_t)(lead ^ 0x03); });
  }
}

void Sim::send(const std::string &text) {
  uint64_t byte_us = 10000000ULL / baud_;
  uint64_t t0 = std::max(now_us, rx_wire_.empty() ? 0 : rx_wire_.back().first);
  for (size_t i = 0; i < text.size(); i++)
    rx_wire_.push_back({t0 + (i + 1) * byte_us, (uint8_t)text[i]});
}

// ---------------------------------------------------------- observations --

std::string Sim::lcd(int row) const { return std::string(screen_[row & 1], 16); }

std::string Sim::state() const {
  std::string s = lcd(0);
  s.erase(s.find_last_not_of(' ') + 1);
  return s;
}

std::string Sim::command(const std::string &line, double timeout_ms) {
  size_t from = lines.size();
  send(line + "\n");
  std::string reply;
  run_until([&] {
    for (size_t i = from; i < lines.size(); i++) {
      const std::string &l = lines[i].text;
      if (l.rfind("OK", 0) == 0 || l.rfind("ERR", 0) == 0) { reply = l; return true; }
    }
    return false;
  }, timeout_ms);
  return reply;
}

// ------------------------------------------------------------------- pins --

void Sim::hw_write(uint8_t p, uint8_t level) {
  level = level ? HIGH : LOW;
  if (out_[p] != level) {
    edges.push_back({now_us, p, level});
    // The opener's wall-button input sees the relay close.
    if (p == doorPin && level == HIGH) door_toggle();
  }
  out_[p] = level;
}

int Sim::hw_read(uint8_t p) {
  if (p == microswitchPin)  return read_contact(microswitch_) ? HIGH : LOW;
  if (p == guiBtn1)         return read_contact(gui_) ? HIGH : LOW;
  if (p == externalDoorPin) return read_contact(ext_) ? HIGH : LOW;
  if (p == encoderClkPin)   return (knob_ >> 1) & 1;
  if (p == encoderDtPin)    return knob_ & 1;
  if (modes_[p] == INPUT_PULLUP) return HIGH;
  return out_[p];
}

int Sim::hw_analog(uint8_t p) {
  advance(t.adc_us);
  return p == lightPin ? light_now() : 0;
}

// The HC-SR04 answers a trigger with an echo pulse as long as the round trip.
// Looking down from the ceiling it always has the floor or the car to hit; a
// dead sensor (garage.sonar_ok = false) holds echo high for ~38 ms, so
// pulseIn() runs into its timeout and returns 0.
unsigned long Sim::hw_pulse_in(uint8_t p, uint8_t state, unsigned long timeout) {
  double cm = (p == echoPin && state == HIGH) ? sonar_cm() : -1;
  double dur = cm >= 0 ? 2.0 * cm / garage.sound_cm_us : 0;
  if (cm < 0 || t.echo_setup_us + dur > timeout) {
    advance(timeout);
    return 0;
  }
  advance(t.echo_setup_us + (uint64_t)dur);
  return (unsigned long)std::lround(dur);
}

// -------------------------------------------------------------------- LCD --

void Sim::hw_lcd_init() {
  advance(100000);  // the library's power-on wait and 4-bit handshake
  memset(screen_, ' ', sizeof screen_);
  col_ = row_ = 0;
}

void Sim::hw_lcd_clear() {
  advance(t.lcd_byte_us + t.lcd_clear_us);
  memset(screen_, ' ', sizeof screen_);
  col_ = row_ = 0;
}

void Sim::hw_lcd_cursor(uint8_t col, uint8_t row) {
  advance(t.lcd_byte_us);
  col_ = col;
  row_ = row & 1;
}

void Sim::hw_lcd_write(uint8_t c) {
  advance(t.lcd_byte_us);
  if (col_ < 16) screen_[row_][col_] = (char)c;
  col_++;
}

// ------------------------------------------------------------------- UART --

// Bytes leave at 10 bits each; write() only blocks once 64 are queued.
void Sim::hw_serial_write(uint8_t c) {
  uint64_t byte_us = 10000000ULL / baud_;
  uint64_t full_at = tx_idle_at_ > 64 * byte_us ? tx_idle_at_ - 64 * byte_us : 0;
  if (now_us < full_at) advance(full_at - now_us);
  tx_idle_at_ = std::max(tx_idle_at_, now_us) + byte_us;

  tx += (char)c;
  if (c == '\n') {
    std::string text = partial_;
    if (!text.empty() && text.back() == '\r') text.pop_back();
    partial_.clear();
    lines.push_back({now_us, text});
    if (echo_serial) printf("      [%9.3f] %s\n", now_us / 1e6, text.c_str());
    if (on_line) on_line(lines.back());
  } else {
    partial_ += (char)c;
  }
}

void Sim::hw_serial_flush() {
  if (tx_idle_at_ > now_us) advance(tx_idle_at_ - now_us);
}

// The AVR core's RX ring holds 63 bytes; anything arriving while it is full
// is lost.
void Sim::pump_rx() {
  while (!rx_wire_.empty() && rx_wire_.front().first <= now_us) {
    if (rx_ring_.size() < 63) rx_ring_.push_back(rx_wire_.front().second);
    else rx_dropped++;
    rx_wire_.pop_front();
  }
}

int Sim::hw_serial_available() {
  pump_rx();
  return (int)rx_ring_.size();
}

int Sim::hw_serial_read(bool consume) {
  pump_rx();
  if (rx_ring_.empty()) return -1;
  int c = rx_ring_.front();
  if (consume) rx_ring_.pop_front();
  return c;
}

void Sim::hw_eeprom_write(int addr, uint8_t v) {
  eeprom[addr & 1023] = v;
  eeprom_writes++;
  advance(t.eeprom_write_us);
}

}  // namespace sim

// ======================================== the Arduino core, as the sketch sees it

using sim::current;

unsigned long millis() { return current->hw_millis(); }
unsigned long micros() { return current->hw_micros(); }
void delay(unsigned long ms) { current->advance((uint64_t)ms * 1000); }
void delayMicroseconds(unsigned int us) { current->advance(us); }
void pinMode(uint8_t p, uint8_t mode) { current->hw_pin_mode(p, mode); }
void digitalWrite(uint8_t p, uint8_t level) { current->hw_write(p, level); }
int  digitalRead(uint8_t p) { return current->hw_read(p); }
int  analogRead(uint8_t p) { return current->hw_analog(p); }
unsigned long pulseIn(uint8_t p, uint8_t state, unsigned long timeout) {
  return current->hw_pulse_in(p, state, timeout);
}

HardwareSerial Serial;
void   HardwareSerial::begin(unsigned long baud) { current->hw_serial_begin(baud); }
int    HardwareSerial::available() { return current->hw_serial_available(); }
int    HardwareSerial::read() { return current->hw_serial_read(true); }
int    HardwareSerial::peek() { return current->hw_serial_read(false); }
void   HardwareSerial::flush() { current->hw_serial_flush(); }
size_t HardwareSerial::write(uint8_t c) { current->hw_serial_write(c); return 1; }
size_t HardwareSerial::print(const char *s) {
  size_t n = 0;
  while (*s) n += write((uint8_t)*s++);
  return n;
}
size_t HardwareSerial::printNum(long long v) { return print(std::to_string(v).c_str()); }

EEPROMClass EEPROM;
uint8_t EEPROMClass::read(int addr) { return current->hw_eeprom_read(addr); }
void    EEPROMClass::write(int addr, uint8_t v) { current->hw_eeprom_write(addr, v); }

void   LiquidCrystal_I2C::init() { current->hw_lcd_init(); }
void   LiquidCrystal_I2C::clear() { current->hw_lcd_clear(); }
void   LiquidCrystal_I2C::backlight() {}
void   LiquidCrystal_I2C::noBacklight() {}
void   LiquidCrystal_I2C::setCursor(uint8_t col, uint8_t row) { current->hw_lcd_cursor(col, row); }
size_t LiquidCrystal_I2C::write(uint8_t c) { current->hw_lcd_write(c); return 1; }
