#include <Arduino.h>
#include "SerialCommands.h"

// Genau eine Hardwareinstanz. Die Subsysteme erhalten Referenzen darauf.
scara::StopInputs stops;
scara::MotorController motors(stops);
scara::RobotState robot;
scara::Movement movement(motors, robot);
scara::Homing homing(motors, robot);
scara::PositionStore positions;
scara::SerialCommands commands(motors, robot, movement, homing, positions, stops);

void setup() {
  stops.begin();
  motors.begin();
  commands.begin();
}

void loop() {
  stops.poll();  // Auch im Leerlauf sofort verriegeln, nicht erst bei Bewegung.
  if (stops.pressedMask() && motors.status().error != scara::Error::Stopped)
    motors.emergencyStop();
  if (motors.status().error != scara::Error::None)
    robot.invalidate();
  commands.update();
  homing.update();
  movement.update();
  commands.reportEvents();
}
