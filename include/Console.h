#pragma once
#include <Arduino.h>
namespace scara {
// Print formatiert wie Serial.print, aber schreibt nur in einen festen Ringpuffer.
// Das Entleeren ist begrenzt und wartet niemals auf freie UART-Plätze.
class Console : public Print {
 public:
  size_t write(uint8_t value) override;
  void update();
  uint16_t availableForWrite() const {
    return SIZE - count_;
  }

 private:
  static constexpr uint16_t SIZE = 512;
  char data_[SIZE];
  uint16_t read_ = 0, write_ = 0, count_ = 0;
  bool dropped_ = false;
};
}  // namespace scara
