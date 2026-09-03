#pragma once
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <string>
#include <deque>

#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define HIGH 1
#define LOW 0
#define A0 14
#define F(x) (x)
typedef uint8_t byte;

extern uint32_t g_millis;
extern uint8_t  g_pin[32];
extern uint16_t g_analog[32];
extern uint32_t g_pulse;

inline uint32_t millis() { return g_millis; }
inline uint32_t micros() { return g_millis * 1000UL; }
inline void pinMode(int, int) {}
inline void digitalWrite(int p, int v) { g_pin[p] = (uint8_t)v; }
inline int  digitalRead(int p) { return g_pin[p]; }
inline int  analogRead(int p) { return g_analog[p]; }
inline void delay(unsigned long ms) { g_millis += ms; }
inline void delayMicroseconds(unsigned) {}
inline unsigned long pulseIn(int, int, unsigned long = 1000000UL) { return g_pulse; }

struct FakeSerial {
	std::deque<char> rx;
	std::string tx;
	void begin(unsigned long) {}
	void flush() {}
	int  available() { return (int)rx.size(); }
	int  read() { if (rx.empty()) return -1; char c = rx.front(); rx.pop_front(); return c; }
	void feed(const std::string& s) { for (char c : s) rx.push_back(c); }
	void print(const char* s) { tx += s; }
	void print(char c) { tx += c; }
	void print(bool v) { tx += (v ? "1" : "0"); }
	void print(int v) { tx += std::to_string(v); }
	void print(long v) { tx += std::to_string(v); }
	void print(unsigned long v) { tx += std::to_string(v); }
	void println() { tx += "\n"; }
	template <class T> void println(T v) { print(v); tx += "\n"; }
	std::string take() { std::string s = tx; tx.clear(); return s; }
};
extern FakeSerial Serial;
