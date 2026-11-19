#!/usr/bin/env python3
"""Always-on-top battery percent for the Ubuntu desktop. One instance.

Voltage comes from MCU auto-report (FUNC_REPORT_SPEED byte / 10).
Keeps a receive thread on /dev/myserial unless mapping/patrol needs the port
(/tmp/r2_serial_for_driver). Then it switches to ROS /voltage.
"""

from __future__ import annotations

import fcntl
import os
import threading
import time
from pathlib import Path

LOCK_PATH = '/tmp/r2_battery_osd.lock'
DRIVER_FLAG = '/tmp/r2_serial_for_driver'
STATUS_FILE = '/tmp/r2_status.txt'

# 3셀 리튬(3S) 대략 곡선. 전압(팩) → %.
_LIPO_3S = (
    (9.90, 0), (10.20, 8), (10.50, 20), (10.80, 38),
    (11.10, 52), (11.40, 66), (11.70, 78), (12.00, 88),
    (12.30, 96), (12.60, 100),
)


def pct_from_volts(v: float) -> int:
    if v <= 0.1:
        return -1
    if v <= _LIPO_3S[0][0]:
        return 0
    if v >= _LIPO_3S[-1][0]:
        return 100
    for (v0, p0), (v1, p1) in zip(_LIPO_3S, _LIPO_3S[1:]):
        if v <= v1:
            t = (v - v0) / (v1 - v0)
            return int(round(p0 + t * (p1 - p0)))
    return 100


def format_line(v: float, charge: str) -> str:
    p = pct_from_volts(v)
    if p < 0:
        return '배터리 --'
    if charge == 'charging':
        tag = '충전 중'
    elif charge == 'idle':
        tag = '충전 안 함'
    else:
        tag = '상태 확인 중'
    return f'배터리 {p:3d}%   {v:4.1f}V   {tag}'


def read_status() -> str:
    try:
        return Path(STATUS_FILE).read_text(encoding='utf-8').strip()
    except Exception:
        return ''


def take_lock():
    fh = open(LOCK_PATH, 'w')
    try:
        fcntl.flock(fh, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        fh.close()
        return None
    fh.write(str(os.getpid()))
    fh.flush()
    return fh


class VoltageSource:
    def __init__(self) -> None:
        self.volts = 0.0
        self.updated_at = 0.0
        self.charge = 'unknown'
        self._lock = threading.Lock()
        self._ros_ok = False
        self._ros_seen = 0.0
        self._car = None
        self._hist: list[tuple[float, float]] = []
        threading.Thread(target=self._ros_loop, daemon=True).start()
        threading.Thread(target=self._serial_loop, daemon=True).start()

    def get(self) -> tuple[float, float, str]:
        with self._lock:
            return self.volts, self.updated_at, self.charge

    def _set(self, v: float) -> None:
        v = float(v)
        now = time.time()
        with self._lock:
            self._hist.append((now, v))
            cutoff = now - 240.0
            self._hist = [(t, x) for t, x in self._hist if t >= cutoff]
            vals = [x for _, x in self._hist[-20:]]
            self.volts = sorted(vals)[len(vals) // 2]
            self.updated_at = now
            self.charge = self._charge_locked(now)

    def _charge_locked(self, now: float) -> str:
        """Latch charge state. MCU voltage is 0.1V steps; charging can sit still."""
        prev = self.charge
        if len(self._hist) < 10:
            return prev
        old = [x for t, x in self._hist if now - 180.0 <= t <= now - 90.0]
        new = [x for t, x in self._hist if t >= now - 25.0]
        if not new:
            return prev
        new_m = sorted(new)[len(new) // 2]
        if new_m >= 12.35:
            return 'charging'
        if not old:
            return prev
        old_m = sorted(old)[len(old) // 2]
        if new_m - old_m >= 0.09:
            return 'charging'
        if old_m - new_m >= 0.09:
            return 'idle'
        return prev

    def _close_car(self) -> None:
        car = self._car
        self._car = None
        if car is None:
            return
        try:
            del car
        except Exception:
            pass

    def _ros_loop(self) -> None:
        try:
            import rclpy
            from rclpy.node import Node
            from std_msgs.msg import Float32
        except Exception:
            return
        while True:
            try:
                if not rclpy.ok():
                    rclpy.init()
                node = Node('battery_osd')

                def on_v(msg: Float32) -> None:
                    if msg.data > 0.2:
                        self._set(msg.data)
                        self._ros_ok = True
                        self._ros_seen = time.time()

                node.create_subscription(Float32, '/voltage', on_v, 10)
                while rclpy.ok():
                    rclpy.spin_once(node, timeout_sec=0.5)
                    if time.time() - self._ros_seen > 8.0:
                        self._ros_ok = False
            except Exception:
                self._ros_ok = False
                time.sleep(2.0)

    def _serial_loop(self) -> None:
        while True:
            if os.path.exists(DRIVER_FLAG) or self._ros_ok:
                self._close_car()
                time.sleep(0.5)
                continue
            try:
                if self._car is None:
                    from Rosmaster_Lib import Rosmaster
                    car = Rosmaster(car_type=5, com='/dev/myserial')
                    car.create_receive_threading()
                    try:
                        car.set_auto_report_state(True, False)
                    except Exception:
                        pass
                    time.sleep(0.4)
                    self._car = car
                v = self._car.get_battery_voltage()
                if v and v > 0.2:
                    self._set(v)
            except Exception:
                self._close_car()
            time.sleep(0.5)


def run_gui(src: VoltageSource) -> None:
    import tkinter as tk

    root = tk.Tk()
    root.title('R2 배터리')
    root.attributes('-topmost', True)
    root.resizable(False, False)
    root.configure(bg='#111827')
    try:
        root.attributes('-type', 'utility')
    except tk.TclError:
        pass
    bat = tk.Label(
        root, text=format_line(0.0, 'unknown'), font=('Noto Sans CJK KR', 16, 'bold'),
        fg='#f8fafc', bg='#111827', padx=14, pady=6,
    )
    bat.pack()
    err = tk.Label(
        root, text='', font=('Noto Sans CJK KR', 13, 'bold'),
        fg='#fbbf24', bg='#111827', padx=14, pady=4, wraplength=360, justify='left',
    )
    err.pack()

    def place() -> None:
        root.update_idletasks()
        w = root.winfo_width()
        sw = root.winfo_screenwidth()
        root.geometry(f'+{max(24, sw - w - 28)}+18')

    color_mode = {'v': 'green'}

    def tick() -> None:
        v, ts, charge = src.get()
        p = pct_from_volts(v)
        age = time.time() - ts if ts else 1e9
        if p < 0:
            fg = '#94a3b8'
        elif p <= 12:
            fg = '#f87171'
            color_mode['v'] = 'red'
        else:
            # Hysteresis so 26%↔32% does not flip yellow/green every 0.1V.
            if color_mode['v'] == 'yellow':
                if p >= 34:
                    color_mode['v'] = 'green'
            else:
                if p <= 24:
                    color_mode['v'] = 'yellow'
            fg = '#fbbf24' if color_mode['v'] == 'yellow' else '#4ade80'
        live = '●' if age < 5.0 else '○'
        bat.config(text=f'{live}  {format_line(v, charge)}', fg=fg)
        st = read_status()
        if st:
            extra = st
        elif age > 8.0 and p >= 0:
            extra = f'마지막 값 · {int(age)}초 전 (실시간 아님)'
        else:
            extra = ''
        err.config(text=extra)
        root.after(400, tick)

    root.after(100, place)
    root.after(200, tick)
    root.mainloop()


def run_text(src: VoltageSource) -> None:
    while True:
        v, _, charge = src.get()
        print(format_line(v, charge), flush=True)
        time.sleep(2.0)


def main() -> int:
    lock = take_lock()
    if lock is None:
        return 0
    src = VoltageSource()
    try:
        run_gui(src)
    except Exception:
        run_text(src)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
