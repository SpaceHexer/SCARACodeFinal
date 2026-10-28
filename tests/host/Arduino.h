#pragma once
#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <string>
#include <sstream>
#include <iomanip>
constexpr int HIGH = 1, LOW = 0, OUTPUT = 1, INPUT_PULLUP = 2;
#define F(text) text
extern unsigned long testMicros;
extern int pins[256], modeCalls[256], readCalls[256], writeCalls[256];
extern bool drivenLow[256], drivenHigh[256], simulateSwitches;
extern int32_t physicalSteps[4];
extern unsigned long edges[256];
extern int txCapacity;
unsigned long micros();
unsigned long millis();
void pinMode(uint8_t, uint8_t);
int digitalRead(uint8_t);
void digitalWrite(uint8_t, uint8_t);
class Print {
 public:
  virtual ~Print() {};
  virtual size_t write(uint8_t) = 0;
  void print(const char* s) {
    while (*s)
      write(uint8_t(*s++));
  }
  void print(char c) {
    write(uint8_t(c));
  }
  template <class T>
  void print(T n) {
    std::ostringstream s;
    s << +n;
    print(s.str().c_str());
  }
  void print(double n, int digits) {
    std::ostringstream s;
    s << std::fixed << std::setprecision(digits) << n;
    print(s.str().c_str());
  }
  template <class T>
  void println(T n) {
    print(n);
    println();
  }
  void println(double n, int digits) {
    print(n, digits);
    println();
  }
  void println() {
    write('\n');
  }
};
class FakeSerial : public Print {
 public:
  std::string input, output;
  void begin(unsigned long) {}
  int available() const {
    return int(input.size());
  }
  int read() {
    char c = input.front();
    input.erase(0, 1);
    return uint8_t(c);
  }
  int availableForWrite() const {
    return txCapacity;
  }
  size_t write(uint8_t c) override {
    if (txCapacity <= 0)
      return 0;
    --txCapacity;
    output += char(c);
    return 1;
  }
};
extern FakeSerial Serial;
