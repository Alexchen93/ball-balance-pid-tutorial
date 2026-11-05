#pragma once
#include <stdint.h>
#include <cmath>
#include <cstdlib>
#define F(x) x
#define A0 14
#define A1 15
#define LED_BUILTIN 13
#define OUTPUT 1
#define LOW 0
#define HIGH 1
extern uint32_t fakeMillis;
extern int fakeAnalog[2];
inline uint32_t millis() { return fakeMillis; }
inline uint32_t micros() { return fakeMillis * 1000; }
inline int analogRead(int pin) { return fakeAnalog[pin - A0]; }
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
template<typename T, typename L, typename H> T constrain(T x, L lo, H hi) { return x < lo ? lo : x > hi ? hi : x; }
struct Servo { int angle = -1; void attach(int) {} void write(int value) { angle = value; } };
struct FakeSerial {
  void begin(int) {}
  int available() { return 0; }
  int read() { return -1; }
  template<typename T> void print(T) {}
  template<typename T> void print(T, int) {}
  template<typename T> void println(T) {}
};
extern FakeSerial Serial;
