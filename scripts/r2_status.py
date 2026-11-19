#!/usr/bin/env python3
"""Publish a Korean status line for the desktop OSD."""

from __future__ import annotations

import sys
from pathlib import Path

STATUS_FILE = Path('/tmp/r2_status.txt')


def set_status(text: str) -> None:
    STATUS_FILE.write_text((text or '').strip() + '\n', encoding='utf-8')


def clear_status() -> None:
    set_status('')


if __name__ == '__main__':
    set_status(' '.join(sys.argv[1:]) if len(sys.argv) > 1 else '')
