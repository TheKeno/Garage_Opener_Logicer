#pragma once
#include "Arduino.h"
struct FakeEEPROM {
	uint8_t data[1024];
	FakeEEPROM() { memset(data, 255, sizeof(data)); }
	uint8_t read(int a) { return data[a]; }
	void write(int a, uint8_t v) { data[a] = v; }
	template <class T> void put(int a, const T& v) { memcpy(&data[a], &v, sizeof(T)); }
	template <class T> void get(int a, T& v) { memcpy(&v, &data[a], sizeof(T)); }
};
extern FakeEEPROM EEPROM;
