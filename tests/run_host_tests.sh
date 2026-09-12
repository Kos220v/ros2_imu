#!/bin/sh
# Хост-тесты прошивки (gcc, без железа). Запуск: ./tests/run_host_tests.sh
set -e
cd "$(dirname "$0")/.."

echo "== C host tests =="
gcc -std=c11 -Wall -Wextra -Werror -DIMU_HOST_TEST \
    -ICore/Inc -Itests/host/hal_stub \
    Core/Src/imu_protocol.c \
    Core/Src/madgwick_ahrs.c \
    Core/Src/imu_fusion.c \
    Core/Src/mpu6050.c \
    Core/Src/qmc5883l.c \
    Core/Src/imu_flash.c \
    Core/Src/imu_app.c \
    Core/Src/debug_log.c \
    tests/host/hal_stub/hal_stub.c \
    tests/host/test_all.c \
    -o /tmp/imu_host_test -lm
/tmp/imu_host_test

echo "== Python protocol tests =="
if command -v pytest >/dev/null 2>&1; then
    pytest -q ros2/imu_stm32_bridge/test/test_protocol.py
elif python3 -m pytest --version >/dev/null 2>&1; then
    python3 -m pytest -q ros2/imu_stm32_bridge/test/test_protocol.py
else
    python3 ros2/imu_stm32_bridge/test/test_protocol.py
fi
echo "== Windows viewer protocol mirror test =="
python3 tests/test_windows_protocol_mirror.py

echo "ALL OK"
