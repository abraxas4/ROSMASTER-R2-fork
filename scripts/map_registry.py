#!/usr/bin/env python3
"""Track Cartographer maps (pbstream + Nav2 yaml/pgm) and the active map."""

from __future__ import annotations

import argparse
import json
import shlex
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


HOME = Path.home()
MAPS_DIR = HOME / 'maps'
CARTO_DIR = MAPS_DIR / 'cartographer'
REGISTRY_PATH = MAPS_DIR / 'registry.json'


def now_iso() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec='seconds')


def load_registry() -> dict[str, Any]:
    if not REGISTRY_PATH.exists():
        return {'version': 2, 'active_id': None, 'maps': {}}
    with REGISTRY_PATH.open(encoding='utf-8') as fh:
        data = json.load(fh)
    data.setdefault('version', 2)
    data.setdefault('active_id', None)
    data.setdefault('maps', {})
    return data


def save_registry(data: dict[str, Any]) -> None:
    MAPS_DIR.mkdir(parents=True, exist_ok=True)
    CARTO_DIR.mkdir(parents=True, exist_ok=True)
    with REGISTRY_PATH.open('w', encoding='utf-8') as fh:
        json.dump(data, fh, ensure_ascii=False, indent=2)
        fh.write('\n')


def print_active(entry: dict[str, Any]) -> None:
    map_id = entry['id']
    name = entry.get('display_name') or map_id
    print('ACTIVE_MAP_ID=' + shlex.quote(map_id))
    print('ACTIVE_MAP_NAME=' + shlex.quote(name))
    print('ACTIVE_MAP_PBSTREAM=' + shlex.quote(str(entry.get('pbstream') or '')))
    print('ACTIVE_MAP_YAML=' + shlex.quote(str(entry.get('export_yaml') or '')))
    print('ACTIVE_MAP_PGM=' + shlex.quote(str(entry.get('export_pgm') or '')))


def cmd_list(_: argparse.Namespace) -> int:
    reg = load_registry()
    active = reg.get('active_id')
    print(f'Active: {active or "(none)"}')
    for map_id, entry in reg.get('maps', {}).items():
        name = entry.get('display_name') or '(이름 없음)'
        marker = '*' if map_id == active else ' '
        yaml_path = entry.get('export_yaml') or ''
        print(f'{marker} {map_id} | {name} | {yaml_path}')
    return 0


def cmd_continue(_: argparse.Namespace) -> int:
    reg = load_registry()
    map_id = reg.get('active_id')
    if not map_id or map_id not in reg.get('maps', {}):
        print('ERROR: no active Cartographer map. Map first, then save.')
        return 1
    print_active(reg['maps'][map_id])
    yaml_path = Path(reg['maps'][map_id].get('export_yaml') or '')
    if not yaml_path.is_file():
        print('ERROR: Nav2 yaml missing. Run save_cartographer_map.sh')
        return 1
    return 0


def cmd_rename(args: argparse.Namespace) -> int:
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


def cmd_register(args: argparse.Namespace) -> int:
    pbstream = Path(args.pbstream).expanduser().resolve()
    yaml_path = Path(args.yaml).expanduser().resolve() if args.yaml else pbstream.with_suffix('.yaml')
    pgm_path = Path(args.pgm).expanduser().resolve() if args.pgm else pbstream.with_suffix('.pgm')
    map_id = args.map_id or pbstream.stem
    display_name = args.name or ''

    if not pbstream.is_file():
        print(f'ERROR: pbstream not found: {pbstream}')
        return 1
    if not yaml_path.is_file():
        print(f'ERROR: yaml not found: {yaml_path}')
        return 1

    reg = load_registry()
    entry = reg.get('maps', {}).get(map_id, {})
    created = entry.get('created_at', now_iso())
    if entry.get('display_name') and not display_name:
        display_name = entry['display_name']
    reg['maps'][map_id] = {
        'id': map_id,
        'display_name': display_name,
        'backend': 'cartographer',
        'pbstream': str(pbstream),
        'export_yaml': str(yaml_path),
        'export_pgm': str(pgm_path if pgm_path.is_file() else yaml_path.with_suffix('.pgm')),
        'created_at': created,
        'updated_at': now_iso(),
    }
    if args.set_active:
        reg['active_id'] = map_id
    save_registry(reg)
    print_active(reg['maps'][map_id])
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description='R2 Cartographer map registry')
    sub = parser.add_subparsers(dest='cmd', required=True)

    sub.add_parser('list', help='List registered maps').set_defaults(func=cmd_list)
    sub.add_parser('continue', help='Print active map paths').set_defaults(func=cmd_continue)

    p_ren = sub.add_parser('rename', help='Set display name (e.g. 집)')
    p_ren.add_argument('name')
    p_ren.add_argument('--map-id', default=None)
    p_ren.set_defaults(func=cmd_rename)

    p_reg = sub.add_parser('register', help='Register a saved Cartographer map')
    p_reg.add_argument('--id', dest='map_id', default=None)
    p_reg.add_argument('--pbstream', required=True)
    p_reg.add_argument('--yaml', default=None)
    p_reg.add_argument('--pgm', default=None)
    p_reg.add_argument('--name', default='')
    p_reg.add_argument('--no-activate', dest='set_active', action='store_false')
    p_reg.set_defaults(func=cmd_register, set_active=True)

    args = parser.parse_args()
    return args.func(args)


if __name__ == '__main__':
    raise SystemExit(main())
