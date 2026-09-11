"""Проверка зеркала протокола tools/windows_imu_viewer/imu_protocol.py
против эталонной реализации из пакета ROS 2 (ros2/imu_stm32_bridge)
+ смоук-тест геометрии тестового приложения.

Запуск из корня репозитория:  python3 tests/test_windows_protocol_mirror.py
"""
import importlib.util
import os
import random
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CHECKS = 0


def ok(cond, msg):
    global CHECKS
    assert cond, msg
    CHECKS += 1


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, os.path.join(ROOT, path))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


w = load("tools/windows_imu_viewer/imu_protocol.py", "win_proto")
sys.path.insert(0, os.path.join(ROOT, "ros2", "imu_stm32_bridge"))
from imu_stm32_bridge import protocol as r  # noqa: E402

# --- 1. Вектор CRC16 -------------------------------------------------------
ok(w.crc16_ccitt(b"123456789") == 0x29B1, "crc16 вектор 0x29B1")
ok(w.crc16_ccitt(b"123456789") == r.crc16_ccitt(b"123456789"), "crc16 == ROS")

# --- 2. Команды байт-в-байт ------------------------------------------------
cmd_cases = [
    ("cmd_ping", ()), ("cmd_set_rate", (50,)), ("cmd_set_rate", (100,)),
    ("cmd_mag_calib_start", ()), ("cmd_mag_calib_stop", (True,)),
    ("cmd_mag_calib_stop", (False,)), ("cmd_gyro_calib", ()),
    ("cmd_zero_yaw", ()), ("cmd_zero_yaw", (True,)),
    ("cmd_set_declination", (11.5,)), ("cmd_set_declination", (-5.25,)),
    ("cmd_save_flash", ()), ("cmd_get_info", ()), ("cmd_get_calib", ()),
]
for name, args in cmd_cases:
    ok(getattr(w, name)(*args) == getattr(r, name)(*args),
       f"команда {name}{args}")

# --- 3. Кадр ORIENTATION в обе стороны -------------------------------------
o = r.Orientation(
    ts_ms=123456, qw=0.7071, qx=0.12, qy=-0.34, qz=0.61,
    roll_deg=1.5, pitch_deg=-2.25, yaw_deg=123.4, azimuth_deg=45.0,
    wx=0.1, wy=0.2, wz=0.3, ax=0.0, ay=0.0, az=9.81,
    mx=30.1, my=-12.4, mz=41.7, temp_c=35.5,
    status=r.STATUS_MPU_OK | r.STATUS_MAG_OK | r.STATUS_FUSED_9X,
    calib_state=0, rate_hz=50)
pr = o.to_payload()
ok(len(pr) == 80, "длина payload ориентации 80")
ow = w.Orientation.from_payload(pr)
o_rt = r.Orientation.from_payload(pr)  # эталонный round-trip (float32)
import dataclasses  # noqa: E402
for f in dataclasses.fields(o):
    ok(getattr(ow, f.name) == getattr(o_rt, f.name), f"поле {f.name}")
ok(w.encode_frame(w.MSG_ORIENTATION, pr) == r.encode_frame(r.MSG_ORIENTATION, pr),
   "кадр ориентации байт-в-байт")

# --- 4. Декодер: шум, битые кадры, произвольная нарезка ---------------------
f1 = w.encode_frame(0x01, pr)
pi = r.INFO_STRUCT.pack(1, 0, 1, 0, 99999, b"IMU-STM32", 1, 1, 1, 0, 11.5, 50, 0, 0, 0)
f2 = w.encode_frame(0x02, pi)
bad = bytearray(f1)
bad[-1] ^= 0xFF
noise = b"\x00\x11\x22\xaa\x55\x55\x12\x34"
stream = f1 + noise + bytes(bad) + b"\xaa\x55" + f2 + f1
rng = random.Random(42)
dw, dr = w.Decoder(), r.Decoder()
res_w, res_r = [], []
i = 0
while i < len(stream):
    n = rng.randint(1, 7)
    chunk = stream[i:i + n]
    i += n
    res_w += dw.feed(chunk)
    res_r += dr.feed(chunk)
ok(res_w == res_r, "декодеры дают одинаковый результат")
ok([m[0] for m in res_w] == [0x01, 0x02, 0x01],
   f"ids кадров {tuple(m[0] for m in res_w)}")
ok(res_w[0][1] == pr, "payload первого кадра")
info = r.Info.from_payload(res_w[1][1])
ok(info.fw_major == 1 and info.board == "IMU-STM32", "info декодирован")

# --- 5. Геометрия приложения -----------------------------------------------
sys.path.insert(0, os.path.join(ROOT, "tools", "windows_imu_viewer"))
viewer = load("tools/windows_imu_viewer/imu_viewer.py", "imu_viewer")

x, y = viewer.polar(125, 125, 100, 0)
ok(abs(x - 125) < 1e-9 and abs(y - 25) < 1e-9, "polar: север = вверх")
x, y = viewer.polar(125, 125, 100, 90)
ok(abs(x - 225) < 1e-9 and abs(y - 125) < 1e-9, "polar: восток = вправо")

tip, b1, b2 = viewer.needle_points(125, 125, 90, 0)
ok(abs(tip[0] - 125) < 1e-9 and abs(tip[1] - 35) < 1e-9, "стрелка на север")
tip, _, _ = viewer.needle_points(125, 125, 90, 90)
ok(abs(tip[0] - 215) < 1e-9 and abs(tip[1] - 125) < 1e-9, "стрелка на восток")

sky, ground, marks, h1, h2 = viewer.horizon_scene(0.0, 0.0, 250, 250)
ok(abs(h1[1] - 125) < 1e-6 and abs(h2[1] - 125) < 1e-6,
   "горизонт по центру при pitch=0")
ok(all(p[1] <= 125.001 for p in sky), "небо над горизонтом")
ok(all(p[1] >= 124.999 for p in ground), "земля под горизонтом")
ok(len(marks) == 10, "10 штрихов тангажа")

_, _, _, h1, h2 = viewer.horizon_scene(0.0, 20.0, 250, 250)
ok(abs(h1[1] - 185) < 1e-6 and abs(h2[1] - 185) < 1e-6,
   "тангаж +20 -> горизонт вниз на 60 px")

_, _, _, h1, h2 = viewer.horizon_scene(90.0, 0.0, 250, 250)
ok(abs(h1[0] - 125) < 1e-6 and abs(h2[0] - 125) < 1e-6
   and abs(h2[1] - h1[1]) > 100, "крен 90 -> горизонт вертикален")

print(f"windows protocol mirror: {CHECKS} checks OK")
