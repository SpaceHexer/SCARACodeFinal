#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
scara_test_binary=$(mktemp /tmp/scara-joint-test.XXXXXX)
trap 'rm -f "$scara_test_binary"' EXIT
for first in 0 1; do
 for second in 0 1; do
  for target in subsystems integration; do
   scara_main_sources=()
   if [ "$target" = integration ]; then scara_main_sources=(src/main.cpp); fi
   g++ -std=c++11 -Wall -Wextra -Werror -fsanitize=address,undefined -g \
    -DSCARA_TEST_CONFIRMED=1 -DSCARA_STOP_BUTTON_1_ENABLED="$first" -DSCARA_STOP_BUTTON_2_ENABLED="$second" \
    -Itests/host -Iinclude tests/host/Hardware.cpp "tests/host/$target.cpp" \
    src/Types.cpp src/QuinticProfile.cpp src/Kinematics.cpp src/StopInputs.cpp \
    src/MotorController.cpp src/Homing.cpp src/Movement.cpp src/PositionStore.cpp \
    src/Console.cpp src/SerialCommands.cpp "${scara_main_sources[@]}" -o "$scara_test_binary"
   echo "STOP1=$first STOP2=$second $target"
   "$scara_test_binary"
  done
 done
done
