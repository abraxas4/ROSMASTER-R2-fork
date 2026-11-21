#!/usr/bin/env python3
"""List processes that hold a serial device open.

Used by battery_osd for diagnostics, and as a standalone CLI:
  python3 scripts/serial_owner_check.py [/dev/myserial]
"""
from __future__ import annotations

import argparse
import json
import os
import stat
import sys
import time
from logging.handlers import RotatingFileHandler
from pathlib import Path
from typing import Any

DEVICE_DEFAULT = '/dev/myserial'
FLAG_PATH = '/tmp/r2_serial_for_driver'
SHM_BATTERY = '/dev/shm/r2_battery_v'
LOG_DIR = Path.home() / '.ros' / 'r2_logs'
MAX_BYTES = 1 * 1024 * 1024
BACKUP_COUNT = 5


def ensure_log_dir(path: Path | None = None) -> Path:
    d = path or LOG_DIR
    d.mkdir(parents=True, exist_ok=True)
    return d


def setup_logger(name: str, filename: str, log_dir: Path | None = None):
    import logging
    logger = logging.getLogger(name)
    if logger.handlers:
        return logger
    logger.setLevel(logging.INFO)
    try:
        d = ensure_log_dir(log_dir)
        fh = RotatingFileHandler(
            d / filename, maxBytes=MAX_BYTES, backupCount=BACKUP_COUNT, encoding='utf-8')
        fmt = logging.Formatter(
            fmt='%(asctime)s.%(msecs)03d %(name)s[%(process)d:%(processName)s] %(levelname)s %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S')
        fh.setFormatter(fmt)
        logger.addHandler(fh)
        logger.propagate = False
    except Exception as e:
        sh = logging.StreamHandler(sys.stderr)
        sh.setFormatter(logging.Formatter('%(asctime)s %(name)s %(levelname)s %(message)s'))
        logger.addHandler(sh)
        logger.propagate = False
        logger.warning('file log unavailable: %s', e)
    return logger


def _stat_key(path: str) -> tuple[int, int] | None:
    try:
        st = os.stat(path, follow_symlinks=True)
        if not stat.S_ISCHR(st.st_mode) and not stat.S_ISREG(st.st_mode):
            # still compare character devices; allow any file type for robustness
            pass
        return (st.st_dev, st.st_ino)
    except OSError:
        return None


def _comm(pid: int) -> str:
    try:
        return Path(f'/proc/{pid}/comm').read_text(encoding='utf-8', errors='replace').strip()
    except OSError:
        return '?'


def _exe(pid: int) -> str:
    try:
        return os.readlink(f'/proc/{pid}/exe')
    except OSError:
        return ''


def _cmdline(pid: int) -> str:
    try:
        raw = Path(f'/proc/{pid}/cmdline').read_bytes().replace(b'\x00', b' ').strip()
        return raw.decode('utf-8', 'replace')
    except OSError:
        return ''


def list_serial_owners(device: str = DEVICE_DEFAULT) -> list[dict[str, Any]]:
    """Return processes with an fd open on `device` (symlink-aware, inode compare)."""
    key = _stat_key(device)
    if key is None:
        # device missing: still try the literal path among fd links
        key = None
    try:
        resolved = os.path.realpath(device)
    except OSError:
        resolved = device
    owners: list[dict[str, Any]] = []
    seen: set[tuple[int, str]] = set()
    try:
        pids = [int(n) for n in os.listdir('/proc') if n.isdigit()]
    except OSError:
        return owners
    for pid in pids:
        fd_dir = f'/proc/{pid}/fd'
        try:
            fds = os.listdir(fd_dir)
        except OSError:
            continue
        for fd in fds:
            fd_path = f'{fd_dir}/{fd}'
            try:
                target = os.readlink(fd_path)
            except OSError:
                continue
            match = False
            if target == device or target == resolved or os.path.normpath(target) == os.path.normpath(resolved):
                match = True
            else:
                fk = _stat_key(fd_path)
                if key is not None and fk is not None and fk == key:
                    match = True
            if not match:
                continue
            rec_key = (pid, target)
            if rec_key in seen:
                continue
            seen.add(rec_key)
            owners.append({
                'pid': pid,
                'comm': _comm(pid),
                'exe': _exe(pid),
                'cmdline': _cmdline(pid),
                'fd': fd,
                'fd_target': target,
            })
    owners.sort(key=lambda r: (r['pid'], r['fd']))
    return owners


def format_owners(owners: list[dict[str, Any]]) -> str:
    if not owners:
        return 'none'
    parts = []
    for o in owners:
        parts.append(f"{o['pid']}:{o['comm']}(fd={o['fd']}->{o['fd_target']})")
    return ', '.join(parts)


def flag_state(path: str = FLAG_PATH) -> dict[str, Any]:
    p = Path(path)
    exists = p.exists()
    info: dict[str, Any] = {'path': path, 'exists': exists}
    if exists:
        try:
            st = p.stat()
            info['mtime'] = st.st_mtime
            info['age_s'] = max(0.0, time.time() - st.st_mtime)
            info['size'] = st.st_size
        except OSError as e:
            info['stat_error'] = str(e)
    return info


def shm_battery_state(path: str = SHM_BATTERY) -> dict[str, Any]:
    p = Path(path)
    info: dict[str, Any] = {'path': path, 'exists': p.exists()}
    if not info['exists']:
        return info
    try:
        st = p.stat()
        info['mtime'] = st.st_mtime
        info['age_s'] = max(0.0, time.time() - st.st_mtime)
        txt = p.read_text(encoding='utf-8', errors='replace').strip()
        info['value'] = txt
    except OSError as e:
        info['stat_error'] = str(e)
    return info


def format_flag(info: dict[str, Any]) -> str:
    if not info.get('exists'):
        return f"{info.get('path', FLAG_PATH)} absent"
    age = info.get('age_s')
    age_s = f'{age:.1f}s' if isinstance(age, float) else '?'
    return f"{info.get('path')} present age={age_s}"


def format_shm(info: dict[str, Any]) -> str:
    if not info.get('exists'):
        return f"{info.get('path', SHM_BATTERY)} absent"
    age = info.get('age_s')
    age_s = f'{age:.1f}s' if isinstance(age, float) else '?'
    return f"{info.get('path')} value={info.get('value', '?')} age={age_s}"


def self_holds_port(owners: list[dict[str, Any]], pid: int | None = None) -> bool:
    pid = os.getpid() if pid is None else pid
    return any(o['pid'] == pid for o in owners)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description='Show processes holding a serial device open')
    ap.add_argument('device', nargs='?', default=DEVICE_DEFAULT)
    ap.add_argument('--json', action='store_true')
    args = ap.parse_args(argv)
    owners = list_serial_owners(args.device)
    flag = flag_state()
    shm = shm_battery_state()
    if args.json:
        print(json.dumps({
            'device': args.device,
            'realpath': os.path.realpath(args.device) if os.path.exists(args.device) else None,
            'owners': owners,
            'flag': flag,
            'shm_battery': shm,
        }, indent=2, default=str))
    else:
        try:
            rp = os.path.realpath(args.device)
        except OSError:
            rp = '?'
        print(f'device={args.device} realpath={rp}')
        print(f'flag: {format_flag(flag)}')
        print(f'shm:  {format_shm(shm)}')
        if not owners:
            print('owners: none')
        else:
            print('owners:')
            for o in owners:
                print(f"  pid={o['pid']} comm={o['comm']} fd={o['fd']} -> {o['fd_target']}")
                if o['cmdline']:
                    print(f"    cmdline={o['cmdline']}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
