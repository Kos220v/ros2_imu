# ros2_imu — инерционный модуль на STM32F303 для ROS 2 Jazzy

Датчик угла на основе гироскопа и акселерометра **MPU6050** + магнитометра
**QMC5883L**. Плата STM32F303 вычисляет **кватернион ориентации** и **азимут**,
робот забирает их по UART в ROS 2.

## Состав

- `Core/` — прошивка STM32F303 (HAL, CubeMX-структура):
  - `mpu6050.c`, `qmc5883l.c` — драйверы датчиков по I2C;
  - `madgwick_ahrs.c` — фильтр ориентации 9/6 осей;
  - `imu_fusion.c` — калибровки, азимут с компенсацией наклона;
  - `imu_protocol.c` — бинарный протокол с CRC16;
  - `imu_app.c` — опрос 50 Гц, команды, выдача в USART2;
  - `imu_flash.c` — хранение калибровки во flash.
- `ros2/imu_stm32_bridge/` — ROS 2 Jazzy пакет (Python): топики
  `imu/data` (Imu), `imu/mag` (MagneticField), `imu/azimuth` (Float32),
  сервисы калибровки.
- `tests/` — хост-тесты прошивки (gcc + стаб HAL) и протокола Python.
- `docs/` — [протокол](docs/PROTOCOL.md), [подключение](docs/WIRING.md),
  [калибровка](docs/CALIBRATION.md).

## Быстрый старт

1. Подключите датчики и UART по [docs/WIRING.md](docs/WIRING.md).
2. Соберите прошивку в STM32CubeIDE (проект CubeMX: I2C1 + USART1 + USART2)
   и залейте в STM32F303. USART2 автоматически на 115200.
3. Не трогайте плату ~2 с после включения (автокалибровка гироскопа).
4. Откалибруйте магнитометр ([инструкция](docs/CALIBRATION.md)).
5. На роботе соберите и запустите мост:

```bash
cd ~/ros2_ws
cp -r <repo>/ros2/imu_stm32_bridge src/
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select imu_stm32_bridge
source install/setup.bash
ros2 launch imu_stm32_bridge imu.launch.py
```

## Тестовое приложение Windows

Готовый просмотрщик для наладки без робота: `tools/windows_imu_viewer`.
Подключается по COM-порту (USB-UART адаптер) и показывает компас с азимутом,
искусственный горизонт, кватернион, все сенсоры и команды калибровки.

```bat
cd tools\windows_imu_viewer
run.bat        :: установит pyserial при первом запуске
```

## Тесты без железа

```bash
./tests/run_host_tests.sh   # C-тесты (132 проверки) + Python-тесты протокола
```

## Лицензия

BSD-3-Clause.
