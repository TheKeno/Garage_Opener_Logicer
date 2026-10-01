#pragma once
// A 16x2 character LCD behind a PCF8574 backpack. The simulator keeps the
// screen contents and charges the I2C time each character really costs.
#include "Arduino.h"

class LiquidCrystal_I2C {
 public:
  LiquidCrystal_I2C(uint8_t addr, uint8_t cols, uint8_t rows) { (void)addr; (void)cols; (void)rows; }
  void   init();
  void   begin(uint8_t cols, uint8_t rows) { (void)cols; (void)rows; init(); }
  void   clear();
  void   home() { setCursor(0, 0); }
  void   backlight();
  void   noBacklight();
  void   setCursor(uint8_t col, uint8_t row);
  size_t write(uint8_t c);
  size_t print(const char *s) { size_t n = 0; while (*s) n += write((uint8_t)*s++); return n; }
};
