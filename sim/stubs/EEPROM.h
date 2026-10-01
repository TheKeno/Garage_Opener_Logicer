#pragma once
// 1 KB of EEPROM that outlives sim::Sim::power_cycle(), like the real thing.
#include "Arduino.h"

struct EEPROMClass {
  uint8_t read(int addr);
  void    write(int addr, uint8_t v);
  void    update(int addr, uint8_t v) { if (read(addr) != v) write(addr, v); }
  uint16_t length() const { return 1024; }

  // The core's put() goes through update(), so unchanged bytes cost no wear.
  template <class T> const T &put(int addr, const T &v) {
    const uint8_t *p = reinterpret_cast<const uint8_t *>(&v);
    for (size_t i = 0; i < sizeof(T); i++) update(addr + (int)i, p[i]);
    return v;
  }
  template <class T> T &get(int addr, T &v) {
    uint8_t *p = reinterpret_cast<uint8_t *>(&v);
    for (size_t i = 0; i < sizeof(T); i++) p[i] = read(addr + (int)i);
    return v;
  }
};
extern EEPROMClass EEPROM;
