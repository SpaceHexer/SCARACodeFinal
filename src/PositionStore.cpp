#include "PositionStore.h"
#include <math.h>

#ifdef ARDUINO_ARCH_AVR
#include <EEPROM.h>
#include <avr/eeprom.h>
#endif

namespace scara {
  namespace {
    bool validPose(const Pose& p) {
      return isfinite(p.x) && isfinite(p.y) && isfinite(p.z) && isfinite(p.psi);
    }

#ifdef ARDUINO_ARCH_AVR
    constexpr uint8_t VALID = 0xA1; // Gültig, Speicherformat 1.
    constexpr int SLOT_SIZE = 1 + sizeof(Pose) + sizeof(uint16_t);
    static_assert(sizeof(Pose) == 16, "EEPROM-Format erwartet vier 32-Bit-Floats");
    static_assert(PositionStore::CAPACITY * SLOT_SIZE <= E2END + 1,
                  "Nicht genug EEPROM fuer die Speicherplaetze");

    int slotAddress(uint8_t id) {
      return id * SLOT_SIZE;
    }

    uint16_t poseCrc(const Pose& p) {
      const auto* bytes = reinterpret_cast<const unsigned char*>(&p);
      uint16_t crc = 0xFFFF;
      for (unsigned i = 0; i < sizeof(Pose); ++i) {
        crc ^= uint16_t(bytes[i]) << 8;
        for (uint8_t bit = 0; bit < 8; ++bit)
          crc = uint16_t((crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0));
      }
      return crc;
    }
#endif
  }

  void PositionStore::begin() {
    used_ = 0;
#ifdef ARDUINO_ARCH_AVR
    for (uint8_t id = 0; id < CAPACITY; ++id) {
      const int address = slotAddress(id);
      if (EEPROM.read(address) != VALID)
        continue;

      Pose p;
      uint16_t storedCrc;
      EEPROM.get(address + 1, p);
      EEPROM.get(address + 1 + sizeof(Pose), storedCrc);
      if (poseCrc(p) == storedCrc && validPose(p)) {
        poses_[id] = p;
        used_ |= uint16_t(1) << id;
      }
    }
#endif
  }

  Error PositionStore::save(uint8_t id, const Pose& p) {
    if (id >= CAPACITY || !validPose(p))
      return Error::BadArgs;

#ifdef ARDUINO_ARCH_AVR
    if (occupied(id)) {
      const Pose& old = poses_[id];
      if (p.x == old.x && p.y == old.y && p.z == old.z && p.psi == old.psi)
        return Error::None;
    }

    const int address = slotAddress(id);
    const uint16_t crc = poseCrc(p);
    EEPROM.update(address, 0);
    EEPROM.put(address + 1, p);
    EEPROM.put(address + 1 + sizeof(Pose), crc);
    EEPROM.update(address, VALID);
    eeprom_busy_wait();
#endif

    poses_[id] = p;
    used_ |= uint16_t(1) << id;
    return Error::None;
  }

  Error PositionStore::load(uint8_t id, Pose& p) const {
    if (id >= CAPACITY)
      return Error::BadArgs;
    if (!occupied(id))
      return Error::EmptySlot;
    p = poses_[id];
    return Error::None;
  }

  Error PositionStore::erase(uint8_t id) {
    if (id >= CAPACITY)
      return Error::BadArgs;
#ifdef ARDUINO_ARCH_AVR
    EEPROM.update(slotAddress(id), 0);
    eeprom_busy_wait();
#endif
    used_ &= uint16_t(~(uint16_t(1) << id));
    return Error::None;
  }
}  // namespace scara
