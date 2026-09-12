#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IMU viewer — тестовое приложение для Windows.

Подключается к инерционному модулю STM32F303 (MPU6050 + QMC5883L)
по последовательному порту (USART2 115200, бинарный протокол с
CRC16-CCITT, спецификация: docs/PROTOCOL.md) и отображает:

  * компас — азимут (0° = север, по часовой стрелке),
  * указатель положения (искусственный горизонт) — крен/тангаж,
  * кватернион — столбиковая диаграмма компонент w/x/y/z,
  * числовые значения: углы, гироскоп (рад/с), акселерометр (м/с²),
    магнитное поле (мкТл), температура, частота, статус, прошивка.

Запуск:
    python imu_viewer.py        # или просто run.bat

Зависимости:
    Python 3.9+ (tkinter входит в стандартный инсталлятор Windows),
    pip install pyserial
"""
from __future__ import annotations

import math
import queue
import threading
import time
from collections import deque

import imu_protocol as proto

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # модуль импортируется юнит-тестами без pyserial
    serial = None

try:
    import tkinter as tk
    from tkinter import ttk, messagebox
except ImportError:  # модуль импортируется юнит-тестами без tkinter
    tk = None


# ---------------------------------------------------------------------------
# Чистая геометрия (без tkinter) — юнит-тесты:
# tests/test_windows_protocol_mirror.py
# ---------------------------------------------------------------------------

def polar(cx: float, cy: float, r: float, angle_deg: float):
    """Точка на радиусе r под углом angle_deg от вертикали вверх (0 = север),
    по часовой стрелке."""
    a = math.radians(angle_deg)
    return cx + r * math.sin(a), cy - r * math.cos(a)


def needle_points(cx: float, cy: float, r: float, az_deg: float,
                  half_w: float = 0.05):
    """Полигон стрелки компаса: [кончик, основание1, основание2]."""
    a = math.radians(az_deg)
    ux, uy = math.sin(a), -math.cos(a)      # направление азимута
    px, py = -uy, ux                        # перпендикуляр
    bx, by = cx - ux * r * 0.20, cy - uy * r * 0.20
    tip = (cx + ux * r, cy + uy * r)
    return [tip, (bx + px * r * half_w, by + py * r * half_w),
            (bx - px * r * half_w, by - py * r * half_w)]


def horizon_scene(roll_deg: float, pitch_deg: float, w: int, h: int,
                  pitch_scale: float = 3.0):
    """Сцена «искусственного горизонта» для холста w x h.

    Возвращает (sky, ground, marks, h1, h2):
      sky/ground — полигоны [(x, y), ...];
      marks — [(p1, p2, pitch_value), ...] штрихи тангажа;
      h1, h2 — концы линии горизонта.
    Соглашение: крен вправо > 0, тангаж носом вверх > 0
    (горизонт при подъёме смещается вниз).
    """
    cx, cy = w / 2.0, h / 2.0
    big = 4.0 * max(w, h)
    off = max(min(pitch_deg, 60.0), -60.0) * pitch_scale
    r = math.radians(roll_deg)
    cr, sr = math.cos(r), math.sin(r)

    def ts(x: float, y: float):
        return (cx + x * cr - y * sr, cy + x * sr + y * cr)

    sky = [ts(-big, -big), ts(big, -big), ts(big, off), ts(-big, off)]
    ground = [ts(-big, off), ts(big, off), ts(big, big), ts(-big, big)]
    marks = []
    for val in (10, 20, 30, 45, 60):
        for s in (-1, 1):
            y = off - s * val * pitch_scale
            half = 16 if val % 20 == 0 else 10
            marks.append((ts(-half, y), ts(half, y), s * val))
    return sky, ground, marks, ts(-big, off), ts(big, off)


def sim_orientation(t: float) -> 'proto.Orientation':
    """Кадр симуляции на момент времени t секунд (чистая функция, без GUI).

    Движение: медленное вращение по азимуту (20 град/с, по часовой),
    плавные качания крена (±8°) и тангажа (±6°), гравитация 9.81 м/с²,
    магнитное поле 43 мкТл, температура ~36.5 °C. Все статусы «OK»,
    фьюжн 9 осей. Используется режимом --sim и юнит-тестами.
    """
    g = 9.81
    az = (20.0 * t) % 360.0
    roll = math.radians(8.0 * math.sin(0.4 * t))
    pitch = math.radians(6.0 * math.sin(0.27 * t + 1.3))
    yaw = -math.radians(az)  # азимут по часовой => yaw против часовой

    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    # Гравитация в координатах платформы (платформа неподвижна)
    ax = g * math.sin(pitch)
    ay = -g * math.sin(roll) * math.cos(pitch)
    az_a = g * math.cos(roll) * math.cos(pitch)

    # Магнитное поле мира (N, E, D) в координатах платформы
    fn, fe, fd = 20.0, 27.0, -45.0
    mx = fn * math.cos(yaw) + fe * math.sin(yaw)
    my = -fn * math.sin(yaw) + fe * math.cos(yaw)
    mz = fd

    return proto.Orientation(
        ts_ms=int(t * 1000.0) % (2 ** 32),
        qw=qw, qx=qx, qy=qy, qz=qz,
        roll_deg=math.degrees(roll), pitch_deg=math.degrees(pitch),
        yaw_deg=math.degrees(yaw), azimuth_deg=az,
        wx=math.radians(8.0 * 0.4 * math.cos(0.4 * t)),
        wy=math.radians(6.0 * 0.27 * math.cos(0.27 * t + 1.3)),
        wz=-math.radians(20.0),
        ax=ax, ay=ay, az=az_a,
        mx=mx, my=my, mz=mz,
        temp_c=36.5 + 0.3 * math.sin(t / 7.0),
        status=(proto.STATUS_MPU_OK | proto.STATUS_MAG_OK | proto.STATUS_MAG_CAL
                | proto.STATUS_GYRO_CAL | proto.STATUS_FUSED_9X),
        calib_state=0, rate_hz=50)


# ---------------------------------------------------------------------------
# GUI
# ---------------------------------------------------------------------------

CMD_NAMES = {
    proto.CMD_PING: "Пинг",
    proto.CMD_SET_RATE: "Смена частоты",
    proto.CMD_MAG_CALIB_START: "Калибровка маг.: старт",
    proto.CMD_MAG_CALIB_STOP: "Калибровка маг.: стоп",
    proto.CMD_GYRO_CALIB: "Калибровка гиро",
    proto.CMD_ZERO_YAW: "Ноль азимута",
    proto.CMD_SET_DECLINATION: "Склонение",
    proto.CMD_SAVE_FLASH: "Сохранение во flash",
    proto.CMD_GET_INFO: "Запрос инфа",
    proto.CMD_GET_CALIB: "Запрос калибровки",
}
ACK_NAMES = {0: "OK", 1: "ошибка аргумента", 2: "ошибка состояния",
             3: "ошибка железа", 4: "неизвестная команда"}
CALIB_STATE_NAMES = {0: "ожидание", 1: "идёт калибровка магнитометра",
                     2: "идёт калибровка гироскопа"}

SKY = "#79b8e6"
GROUND = "#8a6b42"
DIAL_BG = "#10243e"
DIAL_EDGE = "#3a5a8a"
NEEDLE = "#e74c3c"


class ImuViewer:
    TICK_MS = 30            # период опроса/отрисовки UI, мс
    COMPASS_SIZE = 250
    HORIZON_SIZE = 250

    def __init__(self, root: 'tk.Tk', sim: bool = False) -> None:
        self.root = root
        title = "IMU viewer — STM32F303 (MPU6050 + QMC5883L)"
        if sim:
            title += " [симуляция]"
        root.title(title)
        root.geometry("1080x660")
        self._sim_mode = sim

        # Состояние
        self._rx_q: 'queue.Queue' = queue.Queue()
        self._cmd_q: 'queue.Queue' = queue.Queue()
        self._ui_q: 'queue.Queue' = queue.Queue()
        self._ser = None
        self._thread = None
        self._want_connect = False
        self._port = None
        self._baud = 115200
        self._decoder = proto.Decoder()
        self._frame_times: deque = deque()
        self._last_frame_at = None

        self._build_ui()
        self._refresh_ports()
        self._draw_compass(None)
        self._draw_horizon(0.0, 0.0, placeholder=True)
        self._draw_quat(0.0, 0.0, 0.0, 0.0)
        root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.after(self.TICK_MS, self.tick)

    # ----------------------------- построение UI --------------------------

    def _build_ui(self) -> None:
        # --- Панель подключения ---
        bar = ttk.Frame(self.root, padding=6)
        bar.grid(row=0, column=0, sticky="ew")
        ttk.Label(bar, text="Порт:").pack(side="left")
        self.cb_port = ttk.Combobox(bar, width=18, values=[], state="readonly")
        self.cb_port.pack(side="left", padx=(4, 2))
        ttk.Button(bar, text="↻", width=3,
                   command=self._refresh_ports).pack(side="left")
        ttk.Label(bar, text="  Бит/с:").pack(side="left")
        self.cb_baud = ttk.Combobox(bar, width=8,
                                    values=["9600", "115200", "230400"],
                                    state="readonly")
        self.cb_baud.set("115200")
        self.cb_baud.pack(side="left", padx=(4, 12))
        self.btn_connect = ttk.Button(bar, text="Подключиться",
                                      command=self._toggle_connect)
        self.btn_connect.pack(side="left")
        self.lbl_conn = tk.Label(bar, text="● не подключено", fg="#888")
        self.lbl_conn.pack(side="left", padx=12)

        # --- Основная область ---
        body = ttk.Panedwindow(self.root, orient="horizontal")
        body.grid(row=1, column=0, sticky="nsew")
        self.root.rowconfigure(1, weight=1)
        self.root.columnconfigure(0, weight=1)

        # Левая часть: два «прибора»
        left = ttk.Frame(body)
        body.add(left, weight=1)
        ttk.Label(left, text="Азимут (компас)").grid(row=0, column=0, pady=(0, 2))
        self.cv_compass = tk.Canvas(left, width=self.COMPASS_SIZE,
                                    height=self.COMPASS_SIZE, bg="white",
                                    highlightthickness=0)
        self.cv_compass.grid(row=1, column=0, padx=(10, 6))
        ttk.Label(left, text="Крен / тангаж").grid(row=0, column=1, pady=(0, 2))
        self.cv_horizon = tk.Canvas(left, width=self.HORIZON_SIZE,
                                    height=self.HORIZON_SIZE, bg="white",
                                    highlightthickness=0)
        self.cv_horizon.grid(row=1, column=1, padx=(6, 10))

        # Правая часть: кватернион, значения, статус, команды, журнал
        right = ttk.Frame(body, padding=8)
        body.add(right, weight=0)

        self.cv_quat = tk.Canvas(right, width=340, height=118, bg="white",
                                 highlightthickness=0)
        self.cv_quat.pack(fill="x")

        vals = ttk.LabelFrame(right, text="Значения")
        vals.pack(fill="x", pady=(8, 0))
        self.vals = {}
        for i, name in enumerate(("Азимут", "Крен", "Тангаж", "Курс (yaw)",
                                  "Температура", "Частота")):
            ttk.Label(vals, text=name, anchor="e", width=12
                      ).grid(row=i, column=0, sticky="ew", padx=(4, 4), pady=1)
            var = tk.StringVar(value="—")
            self.vals[name] = var
            ttk.Label(vals, textvariable=var, anchor="w",
                      font=("Consolas", 10)
                      ).grid(row=i, column=1, sticky="ew", padx=(0, 4), pady=1)
        self.v_gyro = self._sensor_row(vals, "Гиро, рад/с")
        self.v_accel = self._sensor_row(vals, "Ускр., м/с²")
        self.v_mag = self._sensor_row(vals, "Магн., мкТл")

        st = ttk.LabelFrame(right, text="Статус")
        st.pack(fill="x", pady=(8, 0))
        self.v_status = self._labeled_row(st, "Датчики")
        self.v_fw = self._labeled_row(st, "Плата")

        cmd = ttk.LabelFrame(right, text="Команды (на STM32)")
        cmd.pack(fill="x", pady=(8, 0))
        buttons = [
            ("Пинг", proto.cmd_ping(), "Пинг"),
            ("Ноль азимута", proto.cmd_zero_yaw(), "Ноль азимута"),
            ("Сброс смещения", proto.cmd_zero_yaw(True), "Сброс смещения азимута"),
            ("Калибровка гиро", proto.cmd_gyro_calib(), "Калибровка гиро"),
            ("Калибровка маг: старт", proto.cmd_mag_calib_start(),
             "Калибровка маг.: старт"),
            ("Калибровка маг: сохранить", proto.cmd_mag_calib_stop(True),
             "Калибровка маг.: стоп+сохранить"),
            ("Сохранить во flash", proto.cmd_save_flash(), "Сохранение во flash"),
            ("Запрос инфо", proto.cmd_get_info(), "Запрос инфо"),
            ("Запрос калибровки", proto.cmd_get_calib(), "Запрос калибровки"),
        ]
        for i, (text, frame, label) in enumerate(buttons):
            ttk.Button(cmd, text=text,
                       command=lambda f=frame, l=label: self._send(f, l)
                       ).grid(row=i // 3, column=i % 3, sticky="ew",
                              padx=2, pady=2)
        for c in range(3):
            cmd.columnconfigure(c, weight=1)
        self.pbar = ttk.Progressbar(cmd, maximum=100, length=300)
        self.pbar.grid(row=3, column=0, columnspan=3, sticky="ew", pady=(6, 0))
        self.v_calib = tk.Label(cmd, text="", fg="#444")
        self.v_calib.grid(row=4, column=0, columnspan=3)

        self.log = tk.Text(right, height=5, width=46, state="disabled",
                           font=("Consolas", 9))
        self.log.pack(fill="both", expand=True, pady=(8, 0))

        # --- Строка состояния ---
        sb = ttk.Frame(self.root, padding=(10, 3))
        sb.grid(row=2, column=0, sticky="ew")
        self.v_rate = tk.Label(sb, text="—", font=("Consolas", 9))
        self.v_rate.pack(side="left")
        self.v_last = tk.Label(sb, text="", font=("Consolas", 9))
        self.v_last.pack(side="left", padx=16)
        self.v_link = tk.Label(sb, text="", font=("Consolas", 9), fg="#666")
        self.v_link.pack(side="right")

    def _sensor_row(self, parent, name: str):
        ttk.Label(parent, text=name, anchor="e", width=12
                  ).grid(sticky="ew", padx=(4, 4), pady=1)
        var = tk.StringVar(value="—")
        ttk.Label(parent, textvariable=var, anchor="w",
                  font=("Consolas", 9)
                  ).grid(sticky="ew", columnspan=2, padx=(0, 4), pady=1)
        return var

    def _labeled_row(self, parent, name: str):
        ttk.Label(parent, text=name, anchor="e", width=12
                  ).grid(sticky="ew", padx=(4, 4), pady=1)
        var = tk.StringVar(value="—")
        ttk.Label(parent, textvariable=var, anchor="w",
                  font=("Consolas", 9)
                  ).grid(sticky="ew", columnspan=2, padx=(0, 4), pady=1)
        return var

    # ----------------------------- порт / поток ---------------------------

    def _refresh_ports(self) -> None:
        if self._sim_mode:
            self.cb_port.config(values=["(симуляция)"], state="readonly")
            self.cb_port.current(0)
            return
        if serial is None:
            self.cb_port.config(values=[], state="normal")
            return
        try:
            ports = [p.device for p in list_ports.comports()]
        except Exception:
            ports = []
        self.cb_port.config(values=ports, state="readonly")
        if ports and self.cb_port.get() not in ports:
            self.cb_port.current(0)

    def _toggle_connect(self) -> None:
        if self._want_connect:
            self._want_connect = False
            self._apply_conn("не подключено", "#888")
            self.btn_connect.config(text="Подключиться")
            return
        if self._sim_mode:
            port, baud = "(симуляция)", 115200
        else:
            if serial is None:
                messagebox.showerror(
                    "Нет pyserial",
                    "Не установлен pyserial.\n\nВыполните:\n    pip install pyserial")
                return
            port = (self.cb_port.get() or "").strip()
            if not port:
                messagebox.showerror("Нет порта", "Выберите COM-порт.")
                return
            try:
                baud = int(self.cb_baud.get())
            except ValueError:
                messagebox.showerror("Бит/с", "Некорректная скорость.")
                return
        self._port, self._baud = port, baud
        self._want_connect = True
        self._decoder = proto.Decoder()
        self._frame_times = deque()
        self._last_frame_at = None
        self.btn_connect.config(text="Отключиться")
        self._ui_q.put(("conn", "подключение…", "#d4a017"))
        self._log(f"Подключение к {port} @ {baud}…")
        loop = self._sim_loop if self._sim_mode else self._reader_loop
        self._thread = threading.Thread(target=loop, daemon=True)
        self._thread.start()

    def _reader_loop(self) -> None:
        """Поток чтения: только очередь, никаких виджетов."""
        while True:
            try:
                self._ser = serial.Serial(self._port, self._baud, timeout=0.2)
            except Exception as e:
                if not self._want_connect:
                    break
                self._log(f"Ошибка подключения: {e}")
                self._ui_q.put(("conn", "ошибка порта", "#c0392b"))
                time.sleep(1.5)
                if not self._want_connect:
                    break
                continue
            self._ui_q.put(("conn", f"подключено: {self._port}", "#27ae60"))
            self._log(f"Подключено: {self._port} @ {self._baud}")
            self._cmd_q.put(proto.cmd_get_info())  # автозапрос инфо
            try:
                while self._want_connect:
                    try:
                        while True:
                            self._ser.write(self._cmd_q.get_nowait())
                    except queue.Empty:
                        pass
                    data = self._ser.read(256)
                    if data:
                        for msg_id, payload in self._decoder.feed(data):
                            self._rx_q.put((msg_id, payload))
            except Exception as e:
                self._log(f"Ошибка UART: {e}")
            finally:
                try:
                    self._ser.close()
                except Exception:
                    pass
                self._ser = None
            if self._want_connect:
                self._ui_q.put(("conn", "переподключение…", "#d4a017"))
                time.sleep(1.0)
        self._ui_q.put(("conn", "не подключено", "#888"))

    def _sim_loop(self) -> None:
        """Симуляция (без железа): INFO при старте, кадры 50 Гц,
        ответы ACK на команды из панели. Только очередь, без виджетов."""
        t0 = time.monotonic()
        time.sleep(0.3)
        if self._want_connect:
            info = proto.Info(fw_major=0, fw_minor=1, fw_patch=0,
                              uptime_ms=3600000, board="SIM-IMU",
                              mpu_ok=1, mag_ok=1, mag_cal=1, gyro_cal=1,
                              declination_deg=11.5, rate_hz=50)
            self._rx_q.put((proto.MSG_INFO, info.to_payload()))
            self._log("Симуляция: генерация кадров 50 Гц (без железа)")
        next_t = t0
        while self._want_connect:
            now = time.monotonic()
            try:
                while True:
                    frame = self._cmd_q.get_nowait()
                    # Кадр: AA 55 LEN ID ... CRC16 — команда в 4-м байте
                    cmd_id = frame[3] if len(frame) > 3 else 0
                    ack = proto.Ack(cmd_id=cmd_id, result=proto.ACK_OK, info=0)
                    self._rx_q.put((proto.MSG_ACK, ack.to_payload()))
            except queue.Empty:
                pass
            if now >= next_t:
                next_t = max(next_t + 0.02, now)
                o = sim_orientation(now - t0)
                self._rx_q.put((proto.MSG_ORIENTATION, o.to_payload()))
            time.sleep(0.005)
        self._ui_q.put(("conn", "не подключено", "#888"))

    def _send(self, frame: bytes, label: str) -> None:
        if not self._want_connect or (self._ser is None and not self._sim_mode):
            messagebox.showinfo("Нет подключения", "Порт не подключён.")
            return
        self._cmd_q.put(frame)
        self._log(f"→ {label}")

    # ----------------------------- цикл UI --------------------------------

    def _log(self, msg: str) -> None:
        self._append_log(msg)

    def _append_log(self, msg: str) -> None:
        self.log.config(state="normal")
        self.log.insert("end", time.strftime("%H:%M:%S ") + msg + "\n")
        self.log.see("end")
        while int(self.log.index("end-1c").split(".")[0]) > 40:
            self.log.delete("1.0", "1.0")
        self.log.config(state="disabled")

    def _apply_conn(self, text: str, color: str) -> None:
        self.lbl_conn.config(text="● " + text, fg=color)
        if self._port:
            self.v_link.config(text=f"{self._port} @ {self._baud}")

    def _apply_info(self, info: 'proto.Info') -> None:
        fw = f"{info.fw_major}.{info.fw_minor}.{info.fw_patch}"
        self.v_fw.set(f"FW {fw} · {info.board} · аптайм {info.uptime_ms/1000:.0f} с"
                      f" · склонение {info.declination_deg:+.1f}° · {info.rate_hz} Гц")
        self._log(f"Инфо: FW {fw}, {info.board}, "
                  f"mpu={info.mpu_ok} mag={info.mag_ok} "
                  f"cal(mag)={info.mag_cal} cal(gyro)={info.gyro_cal}")

    def _apply_calib(self, cal: 'proto.Calib') -> None:
        self.pbar["value"] = cal.progress_pct
        text = CALIB_STATE_NAMES.get(cal.state, f"состояние {cal.state}")
        if cal.state:
            text += f" · {cal.progress_pct}%"
        self.v_calib.config(text=text)
        self._log(f"Калибровка: {text}")

    def _apply_orientation(self, o: 'proto.Orientation') -> None:
        self._draw_compass(o.azimuth_deg)
        self._draw_horizon(o.roll_deg, o.pitch_deg)
        self._draw_quat(o.qw, o.qx, o.qy, o.qz)
        self.vals["Азимут"].set(f"{o.azimuth_deg:6.1f} °")
        self.vals["Крен"].set(f"{o.roll_deg:+6.1f} °")
        self.vals["Тангаж"].set(f"{o.pitch_deg:+6.1f} °")
        self.vals["Курс (yaw)"].set(f"{o.yaw_deg:+7.1f} °")
        self.vals["Температура"].set(f"{o.temp_c:5.1f} °C")
        self.vals["Частота"].set(f"{o.rate_hz:3d} Гц")
        self.v_gyro.set(f"{o.wx:+8.3f} {o.wy:+8.3f} {o.wz:+8.3f}")
        self.v_accel.set(f"{o.ax:+8.3f} {o.ay:+8.3f} {o.az:+8.3f}")
        self.v_mag.set(f"{o.mx:+8.1f} {o.my:+8.1f} {o.mz:+8.1f}")
        flags = []
        if o.status & proto.STATUS_MPU_OK:
            flags.append("MPU")
        if o.status & proto.STATUS_MAG_OK:
            flags.append("MAG")
        if o.status & proto.STATUS_FUSED_9X:
            flags.append("9 осей")
        elif not o.status & proto.STATUS_MAG_OK:
            flags.append("6 осей (нет мага)")
        if o.status & proto.STATUS_MAG_CAL:
            flags.append("маг.калеб")
        if o.status & proto.STATUS_GYRO_CAL:
            flags.append("гирокалеб")
        self.v_status.set(" · ".join(flags) or "—")

    def tick(self) -> None:
        # События из потока чтения
        try:
            while True:
                kind, a, b = self._ui_q.get_nowait()
                if kind == "conn":
                    self._apply_conn(a, b)
        except (queue.Empty, ValueError):
            pass
        # Кадры
        last_o = None
        try:
            while True:
                msg_id, payload = self._rx_q.get_nowait()
                if msg_id == proto.MSG_ORIENTATION:
                    o = proto.Orientation.from_payload(payload)
                    last_o = o
                    self._frame_times.append(time.monotonic())
                    self._last_frame_at = time.monotonic()
                elif msg_id == proto.MSG_INFO:
                    self._apply_info(proto.Info.from_payload(payload))
                elif msg_id == proto.MSG_CALIB:
                    self._apply_calib(proto.Calib.from_payload(payload))
                elif msg_id == proto.MSG_ACK:
                    a = proto.Ack.from_payload(payload)
                    name = CMD_NAMES.get(a.cmd_id, f"0x{a.cmd_id:02X}")
                    res = ACK_NAMES.get(a.result, str(a.result))
                    self._log(f"ACK {name}: {res}"
                              + (f" (info={a.info})" if a.info else ""))
        except queue.Empty:
            pass
        if last_o is not None:
            self._apply_orientation(last_o)
        # Строка состояния
        now = time.monotonic()
        while self._frame_times and now - self._frame_times[0] > 1.0:
            self._frame_times.popleft()
        self.v_rate.config(text=f"{len(self._frame_times)} кадров/с")
        if self._last_frame_at:
            self.v_last.config(text=f"последний кадр: "
                                    f"{(now - self._last_frame_at)*1000:.0f} мс назад")
        self.root.after(self.TICK_MS, self.tick)

    def _on_close(self) -> None:
        self._want_connect = False
        self.root.destroy()

    # ----------------------------- отрисовка ------------------------------

    def _draw_compass(self, az) -> None:
        c = self.cv_compass
        c.delete("all")
        s = self.COMPASS_SIZE
        cx = cy = s / 2.0
        r = s / 2.0 - 10
        c.create_oval(cx - r, cy - r, cx + r, cy + r,
                      fill=DIAL_BG, outline=DIAL_EDGE, width=2)
        for deg in range(0, 360, 10):
            r1 = r - 6 if deg % 30 else r - 12
            x1, y1 = polar(cx, cy, r1, deg)
            x2, y2 = polar(cx, cy, r, deg)
            c.create_line(x1, y1, x2, y2, fill="white",
                          width=1 if deg % 30 else 2)
        for d, t in ((0, "N"), (90, "E"), (180, "S"), (270, "W")):
            x, y = polar(cx, cy, r - 24, d)
            c.create_text(x, y, text=t, fill="#ffd24a" if t == "N" else "white",
                          font=("Segoe UI", 11, "bold"))
        if az is None:
            c.create_text(cx, cy, text="нет данных", fill="#8899aa",
                          font=("Segoe UI", 11))
            return
        pts = [p for pt in needle_points(cx, cy, r * 0.9, az) for p in pt]
        c.create_polygon(*pts, fill=NEEDLE, outline="")
        c.create_oval(cx - 5, cy - 5, cx + 5, cy + 5, fill="white")
        c.create_text(cx, cy + r * 0.45, text=f"{az:.1f}°", fill="white",
                      font=("Consolas", 16, "bold"))

    def _draw_horizon(self, roll: float, pitch: float,
                      placeholder: bool = False) -> None:
        c = self.cv_horizon
        c.delete("all")
        w = h = self.HORIZON_SIZE
        cx = cy = w / 2.0
        if placeholder:
            c.create_rectangle(8, 8, w - 8, h - 8, fill=SKY, outline="")
            c.create_text(cx, cy, text="нет данных", fill="#5b7fa6",
                          font=("Segoe UI", 11))
            return
        sky, ground, marks, h1, h2 = horizon_scene(roll, pitch, w, h)
        pts = [p for pt in sky for p in pt]
        c.create_polygon(*pts, fill=SKY, outline="")
        pts = [p for pt in ground for p in pt]
        c.create_polygon(*pts, fill=GROUND, outline="")
        c.create_line(*h1, *h2, fill="white", width=2)
        for p1, p2, _v in marks:
            c.create_line(*p1, *p2, fill="white", width=1)
        # Невертирующаяся рамка: круг и «крылья»
        c.create_oval(6, 6, w - 6, h - 6, outline="#223", width=2)
        c.create_line(cx - 42, cy, cx - 12, cy, fill="#ff5722", width=4)
        c.create_line(cx + 12, cy, cx + 42, cy, fill="#ff5722", width=4)
        c.create_oval(cx - 3, cy - 3, cx + 3, cy + 3, fill="#ff5722")
        c.create_text(cx, h - 16,
                      text=f"крен {roll:+.0f}°   тангаж {pitch:+.0f}°",
                      fill="#223", font=("Consolas", 10, "bold"))

    def _draw_quat(self, qw: float, qx: float, qy: float, qz: float) -> None:
        c = self.cv_quat
        c.delete("all")
        W = 340
        x0, x1 = 34, W - 70
        xm = (x0 + x1) / 2.0
        half = (x1 - x0) / 2.0
        for i, (name, v, col) in enumerate(
                (("w", qw, "#4caf50"), ("x", qx, "#2980ff"),
                 ("y", qy, "#2980ff"), ("z", qz, "#2980ff"))):
            y = 16 + i * 27
            c.create_text(8, y, anchor="w", text=name,
                          font=("Consolas", 10, "bold"))
            c.create_line(x0, y, x1, y, fill="#ccc")
            c.create_line(xm, y - 8, xm, y + 8, fill="#777")
            c.create_rectangle(xm, y - 5, xm + v * half, y + 5,
                               fill=col, outline="")
            c.create_text(x1 + 6, y, anchor="w", text=f"{v:+.3f}",
                          font=("Consolas", 10))


def main(argv=None) -> int:
    import argparse
    ap = argparse.ArgumentParser(
        description="IMU viewer — тестовое приложение для инерционного модуля")
    ap.add_argument("--sim", action="store_true",
                    help="симуляция: данные генерируются внутри приложения "
                         "(50 Гц), порт и pyserial не нужны")
    args = ap.parse_args(argv)
    if tk is None:
        print("В этом Python нет tkinter (в стандартном инсталляторе "
              "Windows он входит по умолчанию).")
        return 1
    if serial is None and not args.sim:
        print("Не установлен pyserial. Выполните:  pip install pyserial\n"
              "(для режима --sim pyserial не нужен)")
        return 1
    try:  # чёткая отрисовка на HiDPI (Windows)
        import ctypes
        ctypes.windll.shcore.SetProcessDpiAwareness(1)
    except Exception:
        pass
    root = tk.Tk()
    ImuViewer(root, sim=args.sim)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
