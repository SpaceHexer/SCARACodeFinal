#pragma once
#include "Types.h"
#include <math.h>

#ifndef SCARA_TEST_CONFIRMED
#define SCARA_TEST_CONFIRMED 0
#endif

#if defined(__AVR__) && SCARA_TEST_CONFIRMED
#error Simulationseinstellungen sind auf echter Hardware verboten.
#endif

namespace scara {
  namespace config {
    constexpr Real PI_VALUE = 3.14159265358979323846f, RAD_PER_DEG = PI_VALUE / 180;
    constexpr uint32_t SERIAL_BAUD = 115200;
    constexpr Real L1 = 228, L2 = 136.5f;   // Vom Benutzer bestätigte Maße in mm.
    constexpr Real TOOL_X = 0, TOOL_Y = 0;  // Benutzer: aktueller Werkzeugversatz 0.
    constexpr bool GEOMETRY_CONFIRMED = true;
    constexpr bool HOME_CONFIRMED[4] = {true, true, true, true};

    // Benutzerkonfiguration: false sperrt paralleles Homing am Roboter.
    // Der bestehende Testschalter gilt nur im Hosttest (auf AVR oben verboten).
    constexpr bool PARALLEL_CONFIRMED = false || SCARA_TEST_CONFIRMED;
    constexpr uint8_t PARALLEL_GROUPS[] = {0x0f};  // Erst nach mechanischer Prüfung freigeben.
    constexpr Real K1 = 200.0f * 4 * 20 / (2 * PI_VALUE);
    constexpr Real K2 = 200.0f * 4 * 16 / (2 * PI_VALUE);
    constexpr Real K3 = 200.0f * 4 * 4 / (2 * PI_VALUE);
    constexpr Real KZ = 100;

    // Benutzer: Referenzwinkel derzeit 0 Grad. Pins/Getriebe/Offsets übernommen;
    // Homing-Einstellungen am Roboter kalibrieren und HOME_CONFIRMED danach setzen.
    constexpr AxisConfig AXIS[4] = {
      {54, 55, 38, 3,  K1, 0, 1, -175 * RAD_PER_DEG, 168 * RAD_PER_DEG, 2000 / K1, 1000 / K1, 0, -1, 800, 200, 40, 7900 },      // J1   
      {60, 61, 56, 2,  K2, 0, 1, -151.5 * RAD_PER_DEG, 177 * RAD_PER_DEG, 2000 / K2, 1000 / K2, 0, -1, 1000, 200, 40, 5400  },  // J2   
      {26, 28, 24, 14, K3, 0, 1, -200 * RAD_PER_DEG, 166 * RAD_PER_DEG, 2000 / K3, 1000 / K3, 0, 1,  500,  100, 40, -1500},     // J3   
      {46, 48, 62, 15, KZ, 150, -1, 0,         308,      4000 / KZ, 1000 / KZ, 0, 1, 1100, 200, 40, -16000}                     // Z

      // J1, Joint 1, X   // brown & white Limit  (auf RAMPS an S und GND/- sonst raucht es),   X-
      // J2, Joint 2, Y   // blue & white Limit   (auf RAMPS an S und GND/- sonst raucht es),   X+
      // J3, Joint 3, Z   // green & white Limit  (auf RAMPS an S und GND/- sonst raucht es),   Y-
      // Z,  Z-Axis,  E0  // orange & white Limit (auf RAMPS an S und GND/- sonst raucht es),   Y+
    };

    // Zusätzliche Softgrenze am bekannten Referenz-Endschalter. Nach dem Homing
    // liegt dessen Koordinate bei q_ref - homeOffsetSteps/K. Eine Fahrt darüber
    // hinaus würde ohnehin den Endschalter auslösen. Die Benutzermin/max bleiben
    // bindend; diese Grenze kann sie nur enger machen, niemals erweitern.
    constexpr uint16_t HOME_SOFT_MARGIN_STEPS = 5;

    inline AxisConfig limits(uint8_t i) {
      AxisConfig a = AXIS[i];
      Real switchQ = a.qRef - a.homeOffsetSteps / a.stepsPerUnit;
      if (a.homeOffsetSteps > 0) {
        Real lower = switchQ + HOME_SOFT_MARGIN_STEPS / a.stepsPerUnit;
        if (lower > a.minimum)
          a.minimum = lower;
      } else if (a.homeOffsetSteps < 0) {
        Real upper = switchQ - HOME_SOFT_MARGIN_STEPS / a.stepsPerUnit;
        if (upper < a.maximum)
          a.maximum = upper;
      }
      return a;
    }

#ifndef SCARA_STOP_BUTTON_1_ENABLED
    constexpr bool STOP_BUTTON_1_ENABLED = true;  // Hier einzeln true/false einstellen, um STOP BUTTON 1 zu aktivieren/deaktivieren.
#else
    constexpr bool STOP_BUTTON_1_ENABLED = SCARA_STOP_BUTTON_1_ENABLED;
#endif

#ifndef SCARA_STOP_BUTTON_2_ENABLED
    constexpr bool STOP_BUTTON_2_ENABLED = true;  // Hier einzeln true/false einstellen, um STOP BUTTON 2 zu aktivieren/deaktivieren.
#else
    constexpr bool STOP_BUTTON_2_ENABLED = SCARA_STOP_BUTTON_2_ENABLED;
#endif
    constexpr uint8_t STOP_BUTTON_1_PIN = 22, STOP_BUTTON_2_PIN = 23;  // In obiger Belegung frei.
    constexpr uint32_t RELEASE_STABLE_MS = 20, TICK_US = 100;

    // Mindestzeiten des Treibers in µs; Datenblatt prüfen. Der Backend verwendet
    // längere 100-us-Pulse. Geänderte Anforderungen dürfen nicht unbemerkt verletzt werden.
    constexpr uint16_t MIN_STEP_HIGH_US = 2, MIN_STEP_LOW_US = 2, MIN_DIR_SETUP_US = 2, MIN_DIR_HOLD_US = 2;
    static_assert(TICK_US >= MIN_STEP_HIGH_US && TICK_US >= MIN_STEP_LOW_US && TICK_US >= MIN_DIR_SETUP_US && TICK_US >= MIN_DIR_HOLD_US, "Treiberzeiten erfordern groesseren Timer-Takt");
    constexpr uint16_t TICKS_PER_FRAME = 200;
    constexpr Real FRAME_SECONDS = TICK_US * TICKS_PER_FRAME / 1000000.0f;

    inline uint16_t maxFramePulses(uint8_t i) {
      Real pps = AXIS[i].maxSpeed * AXIS[i].stepsPerUnit;
      if (!isfinite(pps) || pps <= 0)
        return 0;
      Real spacing = ceilf((1000000.0f / TICK_US) / fminf(pps, 1000000.0f / TICK_US / 2) - 2e-5f);
      return uint16_t(TICKS_PER_FRAME / spacing);
    }

    constexpr uint8_t FRAME_QUEUE_SIZE = 4, MAX_SEGMENTS = 8;
    constexpr uint32_t HOME_TIMEOUT_MS = 60000, SWITCH_DEBOUNCE_MS = 5;
    constexpr int32_t HOME_MAX_STEPS = 100000;
    constexpr Real DEFAULT_DURATION = 1, MAX_DURATION = 120;

    constexpr uint8_t fixedPin(uint8_t i) {
      return i % 4 == 0 ? AXIS[i / 4].stepPin : i % 4 == 1 ? AXIS[i / 4].dirPin : i % 4 == 2 ? AXIS[i / 4].enablePin : AXIS[i / 4].limitPin;
    }

    constexpr bool pinFree(uint8_t pin, uint8_t n = 0) {
      return n == 16 ? true : (pin != fixedPin(n) && pinFree(pin, n + 1));
    }

    constexpr bool fixedPinsValid(uint8_t i = 0, uint8_t j = 1) {
      return i == 16 ? true : (fixedPin(i) > 1 && fixedPin(i) <= 69 && (j == 16 ? fixedPinsValid(i + 1, i + 2) : (fixedPin(i) != fixedPin(j) && fixedPinsValid(i, j + 1))));
    }

    static_assert(fixedPinsValid(), "Feste Pinbelegung ungueltig/doppelt oder kollidiert mit Serial0");
    static_assert(!STOP_BUTTON_1_ENABLED || (STOP_BUTTON_1_PIN > 1 && STOP_BUTTON_1_PIN <= 69), "STOP1 kein gueltiger freier Mega-Pin");
    static_assert(!STOP_BUTTON_2_ENABLED || (STOP_BUTTON_2_PIN > 1 && STOP_BUTTON_2_PIN <= 69), "STOP2 kein gueltiger freier Mega-Pin");
    static_assert(!STOP_BUTTON_1_ENABLED || pinFree(STOP_BUTTON_1_PIN), "STOP1 Pin-Konflikt");
    static_assert(!STOP_BUTTON_2_ENABLED || pinFree(STOP_BUTTON_2_PIN), "STOP2 Pin-Konflikt");
    static_assert(!STOP_BUTTON_1_ENABLED || !STOP_BUTTON_2_ENABLED || STOP_BUTTON_1_PIN != STOP_BUTTON_2_PIN, "STOP Pins doppelt");
  }  // namespace config
}  // namespace scara
