#pragma once
// Just enough of the Arduino core for Garage_Opener_Logicer_R6 to compile on a
// PC. Everything is implemented in sim.cpp against the simulated garage, so the
// sketch sees a virtual clock, real-looking sensors and a 115200-baud UART.
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <strings.h>

typedef uint8_t byte;
typedef bool boolean;

#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2

// Uno pin numbering.
#define LED_BUILTIN 13
#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define A4 18
#define A5 19

class __FlashStringHelper;
#define F(s) (reinterpret_cast<const __FlashStringHelper *>(s))

template <class A, class B>
inline auto min(A a, B b) -> decltype(a < b ? a : b) { return a < b ? a : b; }
template <class A, class B>
inline auto max(A a, B b) -> decltype(a > b ? a : b) { return a > b ? a : b; }
template <class T, class L, class H>
inline T constrain(T x, L lo, H hi) { return x < lo ? lo : (x > hi ? hi : x); }

inline long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

unsigned long millis();
unsigned long micros();
void          delay(unsigned long ms);
void          delayMicroseconds(unsigned int us);
void          pinMode(uint8_t pin, uint8_t mode);
void          digitalWrite(uint8_t pin, uint8_t level);
int           digitalRead(uint8_t pin);
int           analogRead(uint8_t pin);
unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout = 1000000UL);
inline void   noInterrupts() {}
inline void   interrupts() {}

class HardwareSerial {
 public:
  void begin(unsigned long baud);
  void end() {}
  int  available();
  int  read();
  int  peek();
  void flush();
  explicit operator bool() const { return true; }

  size_t write(uint8_t c);
  size_t print(const char *s);
  size_t print(const __FlashStringHelper *s) { return print(reinterpret_cast<const char *>(s)); }
  size_t print(char c) { return write((uint8_t)c); }
  size_t print(int v) { return printNum(v); }
  size_t print(unsigned int v) { return printNum(v); }
  size_t print(long v) { return printNum(v); }
  size_t print(unsigned long v) { return printNum((long long)v); }
  size_t println() { return print("\r\n"); }
  template <class T> size_t println(T v) { size_t n = print(v); return n + println(); }

 private:
  size_t printNum(long long v);
};
extern HardwareSerial Serial;
