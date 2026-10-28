#include <Arduino.h>
#include "SerialCommands.h"
#include <assert.h>
#include <stdio.h>

using namespace scara;
extern StopInputs stops;
extern MotorController motors;
extern RobotState robot;
extern Movement movement;
extern Homing homing;
extern PositionStore positions;
void setup();
void loop();

void cycle() {
  testMicros += config::TICK_US;
  txCapacity = 64;
  motors.tick();
  loop();
}

void flush() {
  for (int n = 0; n < 150; ++n)
    cycle();
}

void send(const std::string& s) {
  Serial.output.clear();
  Serial.input = s;
  while (Serial.available())
    cycle();
  flush();
}

void homeAll() {
  send("HOME ALL\n");
  assert(Serial.output.find("OK") != std::string::npos);
  for (uint32_t n = 0; n < 4000000 && homing.busy(); ++n)
    cycle();
  flush();
  if (homing.state() != Homing::Done)
    fprintf(stderr, "home failed: %s %s\n", errorName(homing.error()), Serial.output.c_str());
  assert(homing.state() == Homing::Done && robot.referenced(15));
  assert(Serial.output.find("DONE HOME") != std::string::npos);
}

void finishMove() {
  for (uint32_t n = 0; n < 4000000 && movement.busy(); ++n)
    cycle();
  flush();
  if (movement.state() != Movement::Done)
    fprintf(stderr, "move failed: %s output=%s\n", errorName(movement.error()), Serial.output.c_str());
  assert(movement.state() == Movement::Done);
  assert(Serial.output.find("DONE MOVE") != std::string::npos);
}

void reset() {
  drivenLow[config::STOP_BUTTON_1_PIN] = !config::STOP_BUTTON_1_ENABLED;
  drivenLow[config::STOP_BUTTON_2_PIN] = !config::STOP_BUTTON_2_ENABLED;
  for (uint16_t n = 0; n < config::RELEASE_STABLE_MS * 1000 / config::TICK_US + 5; ++n)
    cycle();
  send("RESETSTOP\n");
  assert(motors.status().error == Error::None);
  assert(!robot.referenced(15));
}

int main() {
  simulateSwitches = true;
  drivenLow[config::STOP_BUTTON_1_PIN] = !config::STOP_BUTTON_1_ENABLED;
  drivenLow[config::STOP_BUTTON_2_PIN] = !config::STOP_BUTTON_2_ENABLED;
  setup();
  flush();
  assert(Serial.output.find("SCARA JOINT CONTROLLER") != std::string::npos);
  send("SETJ J1 nan\n");
  assert(Serial.output.find("BAD_ARGS") != std::string::npos);
  send(std::string(270, 'a') + "STOP\nPING\n");
  assert(Serial.output.find("LINE_TOO_LONG") != std::string::npos);
  assert(motors.status().error == Error::None);
  // Homing ueberspringen: SETREF setzt nur die bekannte Referenz, ohne GPIO-
  // Impulse oder Treiberfreigabe. Ungueltige ALL-Auftraege wirken nicht teilweise.
  send("SETREF ALL 0 0 0 999\n");
  assert(Serial.output.find("JOINT_LIMIT") != std::string::npos);
  assert(robot.referenceMask() == 0);
  send("SETREF J1 nan\n");
  assert(Serial.output.find("BAD_ARGS") != std::string::npos);
  send("SETREF J1 5\n");
  assert(robot.referenced(1) && !robot.referenced(15));
  assert(fabsf(motors.position().q[0] / config::RAD_PER_DEG - 5) < .05f);
  for (uint8_t i = 0; i < 4; ++i) {
    assert(!edges[config::AXIS[i].stepPin]);
    assert(pins[config::AXIS[i].enablePin] == HIGH);
  }
  send("MOVEXY 210 145\n");
  assert(Serial.output.find("NOT_HOMED") != std::string::npos);
  send("SETREF ALL 0 0 0 0\n");
  assert(robot.referenced(15));
  send("BEGIN\n");
  send("SETREF ALL 0 0 0 0\n");
  assert(Serial.output.find("BUSY") != std::string::npos);
  send("CANCEL\n");
  send("MOVEJ 1 1 1 0 2\n");
  send("SETREF ALL 0 0 0 0\n");
  assert(Serial.output.find("BUSY") != std::string::npos);
  finishMove();
  send("STOP\n");
  send("SETREF ALL 0 0 0 0\n");
  assert(Serial.output.find("ERR STOPPED") != std::string::npos);
  assert(!robot.referenced(15));
  reset();
  send("HOME J1\n");
  for (uint32_t n = 0; n < 4000000 && homing.busy(); ++n)
    cycle();
  flush();
  assert(robot.referenced(1) && !robot.referenced(15));
  send("SETJ J1 5\n");
  finishMove();
  send("MOVEXY 210 145\n");
  assert(Serial.output.find("NOT_HOMED") != std::string::npos);
  homeAll();
  // Ein abgelehntes HOME darf den alten erfolgreichen Abschluss nicht erneut
  // melden. Ein Timingfehler verriegelt alle weiteren Motorauftraege.
  motors.emergencyStop(Error::Timing);
  flush();
  send("HOME ALL SEQUENTIAL\n");
  assert(Serial.output.find("ERR TIMING_ERROR") != std::string::npos);
  assert(Serial.output.find("DONE HOME") == std::string::npos);
  assert(!robot.referenced(15));
  send("SETREF ALL 0 0 0 0\n");
  assert(Serial.output.find("ERR TIMING_ERROR") != std::string::npos);
  assert(!robot.referenced(15));
  reset();
  homeAll();
  // Direkt nach Home: X/Y funktionieren ohne vorheriges MOVEJ, obwohl q2=0.
  send("MOVEXY 210 145 Z 20 PSI 0 T 3\n");
  finishMove();
  Kinematics k;
  Pose actual = k.forward(motors.position());
  assert(fabsf(actual.x - 210) < .25f && fabsf(actual.y - 145) < .25f);
  send("SAVE 0\n");
  assert(positions.occupied(0));
  send("MOVEXY 220 145\n");
  finishMove();
  send("Position 0\n");
  finishMove();
  actual = k.forward(motors.position());
  assert(fabsf(actual.x - 210) < .25f);
  send("MOVEL 0 0 0 0 1\n");
  assert(Serial.output.find("REMOVED") != std::string::npos);
  send("CHECKLINE 0 0 0 0 1\n");
  assert(Serial.output.find("REMOVED") != std::string::npos);
  // Alle sechs Randwerte sind über Serial ausführbar, nicht nur COEFF.
  send("BEGIN\n");
  send("SEG 10 90 -90 20 3 V0 0 0 0 0 A0 0 0 0 0 V1 2 0 0 0 A1 1 0 0 0\n");
  send("RUN\n");
  assert(Serial.output.find("NEED_SUCCESSOR") != std::string::npos);
  send("SEG 20 90 -90 20 3 V0 2 0 0 0 A0 1 0 0 0 V1 0 0 0 0 A1 0 0 0 0\n");
  send("RUN\n");
  finishMove();
  assert(fabsf(motors.position().q[0] / config::RAD_PER_DEG - 20) < .05f);
  // LIST muss auch mit 16 vollen Slots vollständig und nicht blockierend bleiben.
  for (uint8_t id = 0; id < PositionStore::CAPACITY; ++id)
    positions.save(id, actual);
  send("LIST\n");
  for (int n = 0; n < 1000; ++n)
    cycle();
  for (uint8_t id = 0; id < PositionStore::CAPACITY; ++id)
    assert(Serial.output.find("POSITION " + std::to_string(id) + " ") != std::string::npos);
  assert(Serial.output.find("TX_OVERFLOW") == std::string::npos);
  // Software-STOP bleibt auch bei beiden deaktivierten Tastern wirksam.
  send("MOVEJ 30 90 -90 20 3\n");
  for (int n = 0; n < 1000; ++n)
    cycle();
  assert(motors.status().running);
  send("STOP\n");
  assert(motors.status().error == Error::Stopped && !robot.referenced(15));
  int32_t count = motors.steps(0);
  flush();
  assert(motors.steps(0) == count);
  assert(Serial.output.find("DONE MOVE") == std::string::npos);
  reset();
  send("MOVEJ 0 0 0 0 1\n");
  assert(Serial.output.find("NOT_HOMED") != std::string::npos);
  // Je aktivem Taster: Anschlag während Homing, Planung und Fahrt testen.
  for (int which = 0; which < 2; ++which) {
    bool enabled = which ? config::STOP_BUTTON_2_ENABLED : config::STOP_BUTTON_1_ENABLED;
    uint8_t pin = which ? config::STOP_BUTTON_2_PIN : config::STOP_BUTTON_1_PIN;
    if (!enabled) {
      assert(!modeCalls[pin] && !readCalls[pin] && !writeCalls[pin]);
      continue;
    }
    send("HOME J1\n");
    assert(homing.busy());
    drivenLow[pin] = true;
    cycle();
    flush();
    assert(motors.status().error == Error::Stopped && !homing.busy());
    send("RESETSTOP\n");
    assert(Serial.output.find("HELD") != std::string::npos);
    send("SETREF ALL 0 0 0 0\n");
    assert(Serial.output.find("HELD") != std::string::npos);
    assert(!robot.referenced(15));
    send("HOME ALL\n");
    assert(!homing.busy());
    reset();
    homeAll();
    MoveRequest r;
    r.target = motors.position();
    r.target.q[0] = .1f;
    r.T_wunsch = 2;
    assert(movement.request(r) == Error::None);
    assert(movement.state() == Movement::Planning);
    drivenLow[pin] = true;
    cycle();
    flush();
    assert(motors.status().error == Error::Stopped);
    assert(movement.state() != Movement::Done);
    reset();
    homeAll();
    send("MOVEJ 10 40 -40 20 3\n");
    for (int n = 0; n < 1000; ++n)
      cycle();
    assert(motors.status().running);
    drivenLow[pin] = true;
    cycle();
    flush();
    assert(motors.status().error == Error::Stopped);
    assert(movement.state() != Movement::Done);
    assert(Serial.output.find("DONE MOVE") == std::string::npos);
    reset();
  }
  if (config::STOP_BUTTON_1_ENABLED && config::STOP_BUTTON_2_ENABLED) {
    homeAll();
    send("MOVEJ 10 40 -40 20 3\n");
    for (int n = 0; n < 1000; ++n)
      cycle();
    drivenLow[config::STOP_BUTTON_1_PIN] = drivenLow[config::STOP_BUTTON_2_PIN] = true;
    cycle();
    flush();
    assert(motors.status().error == Error::Stopped);
    drivenLow[config::STOP_BUTTON_1_PIN] = false;
    flush();
    send("RESETSTOP\n");
    assert(Serial.output.find("HELD") != std::string::npos);
    reset();
  }
  // Aktivierte Endschalter führen ebenfalls zu Fehler, nie zu DONE.
  homeAll();
  send("MOVEJ 10 40 -40 20 3\n");
  for (int n = 0; n < 1000; ++n)
    cycle();
  drivenLow[config::AXIS[0].limitPin] = true;
  cycle();
  flush();
  assert(motors.status().error == Error::Endstop);
  assert(Serial.output.find("DONE MOVE") == std::string::npos);
  drivenLow[config::AXIS[0].limitPin] = false;
  reset();
  // Kein GPIO-Zugriff auf deaktivierte STOP-Eingänge, auch nach kompletter main-Ausführung.
  if (!config::STOP_BUTTON_1_ENABLED)
    assert(!modeCalls[config::STOP_BUTTON_1_PIN] && !readCalls[config::STOP_BUTTON_1_PIN] && !writeCalls[config::STOP_BUTTON_1_PIN]);
  if (!config::STOP_BUTTON_2_ENABLED)
    assert(!modeCalls[config::STOP_BUTTON_2_PIN] && !readCalls[config::STOP_BUTTON_2_PIN] && !writeCalls[config::STOP_BUTTON_2_PIN]);
  puts(
      "PASS main/Serial integration: partial HOME, direct MOVEXY, RAM positions, six-boundary "
      "chain, LIST, STOP in all phases");
}
