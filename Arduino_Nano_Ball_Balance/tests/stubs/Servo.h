#pragma once
#include <stdint.h>
#include <cmath>
#include <cstdlib>
#include <sstream>
#include <string>
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
  std::ostringstream out;
  void begin(int) {}
  int available() { return 0; }
  int read() { return -1; }
  void print(uint8_t value) { out << static_cast<unsigned>(value); }
  template<typename T> void print(T value) { out << value; }
  template<typename T> void print(T value, int) { out << value; }
  template<typename T> void println(T value) { out << value << '\n'; }
  void clear() { out.str(""); out.clear(); }
  std::string text() const { return out.str(); }
};
extern FakeSerial Serial;
