#!/usr/bin/env python3
"""Always-on-top battery percent for the Ubuntu desktop. One instance.

Voltage comes from MCU auto-report (FUNC_REPORT_SPEED byte / 10).
Keeps a receive thread on /dev/myserial unless mapping/patrol needs the port
(/tmp/r2_serial_for_driver). Then it switches to ROS /voltage.
Diagnostic logs: ~/.ros/r2_logs/battery_osd.log and serial_owner.log
(RotatingFileHandler, 1MB x 5). Applied on the next process start.
"""

from __future__ import annotations

import fcntl
import os
import sys
import threading
import time
import traceback
from pathlib import Path

LOCK_PATH = '/tmp/r2_battery_osd.lock'
DRIVER_FLAG = '/tmp/r2_serial_for_driver'
STATUS_FILE = '/tmp/r2_status.txt'
SERIAL_DEVICE = '/dev/myserial'
HOLD_WARN_INTERVAL_S = 10.0

# 3셀 리튬(3S) 대략 곡선. 전압(팩) → %.
_LIPO_3S = (
    (9.90, 0), (10.20, 8), (10.50, 20), (10.80, 38),
    (11.10, 52), (11.40, 66), (11.70, 78), (12.00, 88),
    (12.30, 96), (12.60, 100),
)

try:
    import serial_owner_check as _soc
except Exception:  # pragma: no cover - keep OSD up even if helper is missing
    _soc = None


def _loggers():
    if _soc is None:
        return None, None
    osd = _soc.setup_logger('battery_osd', 'battery_osd.log')
    own = _soc.setup_logger('r2.serial_owner', 'serial_owner.log')
    return osd, own


LOG, OWNER_LOG = _loggers()


def _diag(level: str, msg: str, also_owner: bool = False) -> None:
    log = LOG
    if log is None:
        return
    getattr(log, level, log.info)(msg)
    if also_owner and OWNER_LOG is not None and OWNER_LOG is not log:
        getattr(OWNER_LOG, level, OWNER_LOG.info)(msg)


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
        self._flag_present = os.path.exists(DRIVER_FLAG)
        self._source = 'none'
        self._last_hold_warn = 0.0
        self._serial_thread: threading.Thread | None = None
        self._ros_thread: threading.Thread | None = None
        self._ros_thread = threading.Thread(target=self._ros_loop, name='osd-ros', daemon=True)
        self._serial_thread = threading.Thread(target=self._serial_loop, name='osd-serial', daemon=True)
        self._ros_thread.start()
        self._serial_thread.start()

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

    def _owners(self) -> str:
        if _soc is None:
            return 'helper-unavailable'
        try:
            return _soc.format_owners(_soc.list_serial_owners(SERIAL_DEVICE))
        except Exception as e:
            return f'owner-check-failed:{e}'

    def _shm(self) -> str:
        if _soc is None:
            return '/dev/shm/r2_battery_v unknown'
        try:
            return _soc.format_shm(_soc.shm_battery_state())
        except Exception as e:
            return f'shm-check-failed:{e}'

    def _thread_state(self) -> str:
        def st(t: threading.Thread | None) -> str:
            if t is None:
                return 'none'
            return f'{t.name}:{"alive" if t.is_alive() else "dead"} ident={t.ident}'
        return f'serial({st(self._serial_thread)}) ros({st(self._ros_thread)})'

    def _set_source(self, src: str) -> None:
        prev = self._source
        if src == prev:
            return
        self._source = src
        _diag('info', f'voltage source {prev} -> {src}; shm={self._shm()}', also_owner=True)

    def _warn_hold(self, reason: str) -> None:
        now = time.time()
        if now - self._last_hold_warn < HOLD_WARN_INTERVAL_S:
            return
        self._last_hold_warn = now
        _diag(
            'warning',
            'flag present but serial may still be held: '
            f'{reason}; car={"yes" if self._car is not None else "no"} '
            f'ros_ok={self._ros_ok} owners={self._owners()} {self._thread_state()}',
            also_owner=True,
        )

    def _close_car(self) -> None:
        car = self._car
        self._car = None
        if car is None:
            return
        attrs = []
        for name in ('close', 'ser', '_ser', 'serial', 'usart'):
            if hasattr(car, name):
                attrs.append(name)
        _diag('info', f'serial close begin type={type(car).__name__} attrs={attrs} owners_before={self._owners()}')
        try:
            del car
            close_ok = True
            close_exc = ''
        except Exception as e:
            close_ok = False
            close_exc = f'{type(e).__name__}: {e}'
        # Give the receive thread a moment; then see if our pid still owns the port.
        time.sleep(0.05)
        owners = self._owners()
        still = False
        if _soc is not None:
            try:
                still = _soc.self_holds_port(_soc.list_serial_owners(SERIAL_DEVICE))
            except Exception:
                still = False
        if close_ok and not still:
            _diag('info', f'serial close done owners_after={owners}')
        else:
            _diag(
                'warning',
                f'serial close incomplete ok={close_ok} exc={close_exc or "none"} '
                f'self_still_holds={still} owners_after={owners} {self._thread_state()}',
                also_owner=True,
            )

    def _ros_loop(self) -> None:
        try:
            import rclpy
            from rclpy.node import Node
            from std_msgs.msg import Float32
        except Exception as e:
            _diag('warning', f'rclpy import failed; /voltage source unavailable: {e}')
            return
        _diag('info', 'ros loop starting, will subscribe to /voltage')
        while True:
            try:
                if not rclpy.ok():
                    rclpy.init()
                node = Node('battery_osd')

                def on_v(msg: Float32) -> None:
                    if msg.data > 0.2:
                        self._set(msg.data)
                        was = self._ros_ok
                        self._ros_ok = True
                        self._ros_seen = time.time()
                        if not was:
                            _diag('info', f'/voltage received data={msg.data:.2f}')
                        self._set_source('voltage_topic')

                node.create_subscription(Float32, '/voltage', on_v, 10)
                while rclpy.ok():
                    rclpy.spin_once(node, timeout_sec=0.5)
                    if time.time() - self._ros_seen > 8.0:
                        if self._ros_ok:
                            _diag('info', '/voltage stale >8s, ros_ok -> False')
                        self._ros_ok = False
            except Exception as e:
                if self._ros_ok:
                    _diag('warning', f'ros loop exception: {e}')
                self._ros_ok = False
                time.sleep(2.0)

    def _note_flag(self, present: bool) -> None:
        if present == self._flag_present:
            return
        prev = self._flag_present
        self._flag_present = present
        flag_txt = 'present' if present else 'absent'
        prev_txt = 'present' if prev else 'absent'
        extra = ''
        if _soc is not None:
            try:
                extra = f' { _soc.format_flag(_soc.flag_state()) }'
            except Exception:
                extra = ''
        _diag(
            'info',
            f'driver flag {prev_txt} -> {flag_txt};{extra} car={"yes" if self._car is not None else "no"} '
            f'ros_ok={self._ros_ok} source={self._source} owners={self._owners()} shm={self._shm()} '
            f'{self._thread_state()}',
            also_owner=True,
        )

    def _serial_loop(self) -> None:
        _diag('info', f'serial loop starting device={SERIAL_DEVICE}')
        while True:
            flag_now = os.path.exists(DRIVER_FLAG)
            self._note_flag(flag_now)
            if flag_now or self._ros_ok:
                if flag_now:
                    held = self._car is not None
                    if not held and _soc is not None:
                        try:
                            held = _soc.self_holds_port(_soc.list_serial_owners(SERIAL_DEVICE))
                        except Exception:
                            held = False
                    if held:
                        self._warn_hold('close attempted because flag is present')
                self._close_car()
                time.sleep(0.5)
                continue
            try:
                if self._car is None:
                    _diag('info', f'serial open begin device={SERIAL_DEVICE} owners={self._owners()}')
                    from Rosmaster_Lib import Rosmaster
                    car = Rosmaster(car_type=5, com=SERIAL_DEVICE)
                    car.create_receive_threading()
                    try:
                        car.set_auto_report_state(True, False)
                    except Exception as e:
                        _diag('warning', f'set_auto_report_state failed (continuing): {e}')
                    time.sleep(0.4)
                    self._car = car
                    _diag(
                        'info',
                        f'serial open success owners={self._owners()} flag={os.path.exists(DRIVER_FLAG)}',
                        also_owner=True,
                    )
                v = self._car.get_battery_voltage()
                if v and v > 0.2:
                    self._set(v)
                    self._set_source('serial')
            except Exception as e:
                _diag(
                    'warning',
                    f'serial open/read failed: {type(e).__name__}: {e} owners={self._owners()}\n'
                    + traceback.format_exc(),
                )
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
    src_path = os.path.abspath(__file__)
    flag_now = os.path.exists(DRIVER_FLAG)
    _diag(
        'info',
        f'start pid={os.getpid()} file={src_path} flag={"present" if flag_now else "absent"} '
        f'device={SERIAL_DEVICE} argv={sys.argv!r}',
        also_owner=True,
    )
    if _soc is not None:
        try:
            _diag('info', f'at start owners={_soc.format_owners(_soc.list_serial_owners(SERIAL_DEVICE))} '
                  f'{_soc.format_flag(_soc.flag_state())} {_soc.format_shm(_soc.shm_battery_state())}',
                  also_owner=True)
        except Exception as e:
            _diag('warning', f'owner check at start failed: {e}')
    lock = take_lock()
    if lock is None:
        _diag('info', f'lock busy at {LOCK_PATH}; another instance running, exiting')
        return 0
    src = VoltageSource()
    try:
        run_gui(src)
    except Exception as e:
        _diag('warning', f'gui failed, falling back to text: {e}')
        run_text(src)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
