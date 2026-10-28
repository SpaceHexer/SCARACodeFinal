#include "Console.h"

namespace scara {
  size_t Console::write(uint8_t c) {
    if (count_ == SIZE) {
      dropped_ = true;
      return 0;
    }
    data_[write_] = char(c);
    write_ = (write_ + 1) % SIZE;
    ++count_;
    return 1;
  }

  void Console::update() {
    uint8_t budget = 16;
    while (budget-- && count_ && Serial.availableForWrite() > 0) {
      Serial.write(uint8_t(data_[read_]));
      read_ = (read_ + 1) % SIZE;
      --count_;
    }
    if (dropped_ && count_ == 0) {
      dropped_ = false;
      print(F("\nERR TX_OVERFLOW; STATUS abfragen\n"));
    }
  }
}  // namespace scara
