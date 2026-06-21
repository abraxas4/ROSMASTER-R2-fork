#!/usr/bin/env python3
"""Manage RTAB-Map databases: auto-name by location, rename later, track active map."""

from __future__ import annotations

import argparse
import json
import math
import shlex
import urllib.error
import urllib.request
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


HOME = Path.home()
MAPS_DIR = HOME / 'maps'
DB_DIR = MAPS_DIR / 'databases'
BACKUP_DIR = MAPS_DIR / 'backups'
REGISTRY_PATH = MAPS_DIR / 'registry.json'
ACTIVE_LINK = HOME / '.ros' / 'rtabmap.db'
DEFAULT_DB = HOME / '.ros' / 'rtabmap.db'


def now_iso() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec='seconds')


def load_registry() -> dict[str, Any]:
    if not REGISTRY_PATH.exists():
        return {'version': 1, 'active_id': None, 'maps': {}}
    with REGISTRY_PATH.open(encoding='utf-8') as fh:
        return json.load(fh)


def save_registry(data: dict[str, Any]) -> None:
    MAPS_DIR.mkdir(parents=True, exist_ok=True)
    DB_DIR.mkdir(parents=True, exist_ok=True)
    BACKUP_DIR.mkdir(parents=True, exist_ok=True)
    with REGISTRY_PATH.open('w', encoding='utf-8') as fh:
        json.dump(data, fh, ensure_ascii=False, indent=2)
        fh.write('\n')


def format_coord(value: float) -> str:
    sign = 'm' if value < 0 else 'p'
    return f'{sign}{abs(value):.4f}'


def auto_id(lat: float, lon: float) -> str:
    return f'loc_{format_coord(lat)}_{format_coord(lon)}'


def fetch_location() -> dict[str, Any]:
    """Best-effort location: no GPS hardware on R2, so WiFi/IP geolocation is used."""
    url = 'https://ipinfo.io/json'
    req = urllib.request.Request(url, headers={'User-Agent': 'ROSMASTER-R2-fork/1.0'})
    try:
        with urllib.request.urlopen(req, timeout=8) as resp:
            payload = json.load(resp)
    except (urllib.error.URLError, TimeoutError, json.JSONDecodeError) as exc:
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        return {
            'id': f'unknown_{ts}',
            'lat': None,
            'lon': None,
            'source': 'timestamp_fallback',
            'city': None,
            'region': None,
            'country': None,
            'accuracy_m': None,
            'note': f'location lookup failed: {exc}',
        }

    loc = payload.get('loc')
    lat = lon = None
    if loc and ',' in loc:
        lat_s, lon_s = loc.split(',', 1)
        lat = float(lat_s)
        lon = float(lon_s)

    if lat is None or lon is None:
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        map_id = f'unknown_{ts}'
    else:
        map_id = auto_id(lat, lon)

    return {
        'id': map_id,
        'lat': lat,
        'lon': lon,
        'source': 'ip_geolocation',
        'city': payload.get('city'),
        'region': payload.get('region'),
        'country': payload.get('country'),
        'accuracy_m': 3000,
        'note': 'WiFi/IP 기반 대략 위치(수 km 오차). 집/회사 구분은 나중에 rename 권장.',
    }


def ensure_active_symlink(db_path: Path) -> None:
    db_path.parent.mkdir(parents=True, exist_ok=True)
    DEFAULT_DB.parent.mkdir(parents=True, exist_ok=True)
    if ACTIVE_LINK.exists() or ACTIVE_LINK.is_symlink():
        ACTIVE_LINK.unlink()
    ACTIVE_LINK.symlink_to(db_path)


def register_map(
    map_id: str,
    db_path: Path,
    location: dict[str, Any],
    *,
    display_name: str | None = None,
    set_active: bool = True,
) -> dict[str, Any]:
    reg = load_registry()
    entry = {
        'id': map_id,
        'display_name': display_name or '',
        'db_path': str(db_path),
        'lat': location.get('lat'),
        'lon': location.get('lon'),
        'location_source': location.get('source'),
        'city': location.get('city'),
        'region': location.get('region'),
        'country': location.get('country'),
        'accuracy_m': location.get('accuracy_m'),
        'note': location.get('note'),
        'created_at': now_iso(),
        'updated_at': now_iso(),
    }
    if map_id in reg['maps']:
        old = reg['maps'][map_id]
        entry['created_at'] = old.get('created_at', entry['created_at'])
        if old.get('display_name') and not display_name:
            entry['display_name'] = old['display_name']
    reg['maps'][map_id] = entry
    if set_active:
        reg['active_id'] = map_id
        ensure_active_symlink(db_path)
    save_registry(reg)
    return entry


def adopt_existing_default_db() -> None:
    """Import legacy ~/.ros/rtabmap.db into registry if present."""
    if not DEFAULT_DB.exists() and not DEFAULT_DB.is_symlink():
        return
    real = DEFAULT_DB.resolve() if DEFAULT_DB.exists() else DEFAULT_DB
    if not real.is_file():
        return
    reg = load_registry()
    if reg.get('active_id'):
        return

    loc = fetch_location()
    map_id = loc['id'] if loc.get('lat') is not None else 'legacy_import'
    DB_DIR.mkdir(parents=True, exist_ok=True)
    target = DB_DIR / f'{map_id}.db'
    if not target.exists():
        import shutil
        shutil.copy2(real, target)
    register_map(map_id, target, loc, display_name='', set_active=True)


def cmd_location(_: argparse.Namespace) -> int:
    loc = fetch_location()
    print(json.dumps(loc, ensure_ascii=False, indent=2))
    return 0


def cmd_list(_: argparse.Namespace) -> int:
    adopt_existing_default_db()
    reg = load_registry()
    active = reg.get('active_id')
    print(f'Active: {active or "(none)"}')
    for map_id, entry in reg.get('maps', {}).items():
        name = entry.get('display_name') or '(이름 없음)'
        marker = '*' if map_id == active else ' '
        db = entry.get('db_path')
        city = entry.get('city') or ''
        lat = entry.get('lat')
        lon = entry.get('lon')
        coord = f'{lat},{lon}' if lat is not None else 'n/a'
        print(f"{marker} {map_id} | {name} | {city} | {coord} | {db}")
    return 0


def cmd_rename(args: argparse.Namespace) -> int:
    adopt_existing_default_db()
    reg = load_registry()
    map_id = args.map_id or reg.get('active_id')
    if not map_id or map_id not in reg['maps']:
        print('ERROR: map not found. Use: python3 map_registry.py list')
        return 1
    reg['maps'][map_id]['display_name'] = args.name
    reg['maps'][map_id]['updated_at'] = now_iso()
    save_registry(reg)
    print(f'Renamed {map_id} -> "{args.name}"')
    return 0


def cmd_new(args: argparse.Namespace) -> int:
    adopt_existing_default_db()
    loc = fetch_location()
    map_id = loc['id']
    db_path = DB_DIR / f'{map_id}.db'

    if db_path.exists() and not args.force:
        print(f'NOTE: database already exists: {db_path}')
        print('      Existing file will be deleted for a fresh map (-d).')

    entry = register_map(map_id, db_path, loc, set_active=True)
    print('NEW_MAP_ID=' + shlex.quote(map_id))
    print('NEW_MAP_DB=' + shlex.quote(str(db_path)))
    print('NEW_MAP_LAT=' + shlex.quote(str(entry.get('lat'))))
    print('NEW_MAP_LON=' + shlex.quote(str(entry.get('lon'))))
    if entry.get('city'):
        print('NEW_MAP_CITY=' + shlex.quote(str(entry.get('city'))))
    print('DELETE_DB_ON_START=1')
    return 0


def cmd_continue(_: argparse.Namespace) -> int:
    adopt_existing_default_db()
    reg = load_registry()
    map_id = reg.get('active_id')
    if not map_id:
        print('ERROR: no active map. Start "새로 매핑" first.')
        return 1
    entry = reg['maps'][map_id]
    db_path = Path(entry['db_path'])
    ensure_active_symlink(db_path)
    name = entry.get('display_name') or map_id
    print('ACTIVE_MAP_ID=' + shlex.quote(map_id))
    print('ACTIVE_MAP_NAME=' + shlex.quote(name))
    print('ACTIVE_MAP_DB=' + shlex.quote(str(db_path.resolve())))
    print('DELETE_DB_ON_START=0')
    if not db_path.exists():
        print('NOTE: database file does not exist yet; RTAB-Map will create it.')
    return 0


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    r = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


def cmd_set_export(args: argparse.Namespace) -> int:
    adopt_existing_default_db()
    reg = load_registry()
    map_id = args.map_id or reg.get('active_id')
    if not map_id or map_id not in reg['maps']:
        print('ERROR: map not found')
        return 1
    reg['maps'][map_id]['export_yaml'] = str(Path(args.yaml).resolve())
    reg['maps'][map_id]['export_pgm'] = str(Path(args.pgm).resolve())
    reg['maps'][map_id]['updated_at'] = now_iso()
    save_registry(reg)
    print(f'Export registered for {map_id}')
    return 0


def cmd_nearby(args: argparse.Namespace) -> int:
    """Find existing map near current location (for hints only)."""
    loc = fetch_location()
    reg = load_registry()
    if loc.get('lat') is None:
        print('No current coordinates available.')
        return 0
    lat, lon = loc['lat'], loc['lon']
    for map_id, entry in reg.get('maps', {}).items():
        if entry.get('lat') is None:
            continue
        dist = haversine_m(lat, lon, entry['lat'], entry['lon'])
        name = entry.get('display_name') or map_id
        print(f'{map_id} ({name}): ~{dist:.0f} m')
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description='R2 RTAB-Map registry')
    sub = parser.add_subparsers(dest='cmd', required=True)

    sub.add_parser('location', help='Show estimated lat/lon').set_defaults(func=cmd_location)
    sub.add_parser('list', help='List registered maps').set_defaults(func=cmd_list)
    sub.add_parser('continue', help='Select active map for resume').set_defaults(func=cmd_continue)

    p_new = sub.add_parser('new', help='Register a new map from current location')
    p_new.add_argument('--force', action='store_true', help='Allow reusing same coordinate id')
    p_new.set_defaults(func=cmd_new)

    p_ren = sub.add_parser('rename', help='Set display name (e.g. 집, 회사)')
    p_ren.add_argument('name')
    p_ren.add_argument('--map-id', default=None)
    p_ren.set_defaults(func=cmd_rename)

    sub.add_parser('nearby', help='Distance to registered maps').set_defaults(func=cmd_nearby)

    p_exp = sub.add_parser('set-export', help='Register exported nav map paths')
    p_exp.add_argument('map_id', nargs='?', default=None)
    p_exp.add_argument('yaml')
    p_exp.add_argument('pgm')
    p_exp.set_defaults(func=cmd_set_export)

    args = parser.parse_args()
    return args.func(args)


if __name__ == '__main__':
    raise SystemExit(main())