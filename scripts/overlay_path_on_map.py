#!/usr/bin/env python3
"""Draw drive direction on the occupancy-grid PNG."""
from __future__ import annotations

import json
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

SAMPLES = Path('/tmp/r2_samples')
DRIVE = Path('/tmp/r2_last_drive.json')
DOCS = Path.home() / 'ROSMASTER-R2-fork/docs/drive_test_full'
SRC = DOCS / 'result_map.png'
if not SRC.exists():
    SRC = SAMPLES / 'result_map.png'
OUT = DOCS / 'result_map_with_path.png'
DOCS.mkdir(parents=True, exist_ok=True)


def world_to_px(x, y, origin, res, gw, gh, scale):
    gx = (x - origin[0]) / res
    gy = (y - origin[1]) / res
    px = gx * scale
    # PNG was flipped so +Y is up in the image
    py = (gh - 1 - gy) * scale
    return px, py


def arrow(draw, x, y, yaw, size, fill):
    c, s = math.cos(yaw), math.sin(yaw)
    # image y is up after flip, so screen-y decreases with +world-y
    # pixel +x = +world x, pixel +y = -world y
    # yaw 0 = +x world = right on image
    dx, dy = c, -s
    tip = (x + dx * size, y + dy * size)
    left = (x - dx * size * 0.5 + (-dy) * size * 0.4,
            y - dy * size * 0.5 + dx * size * 0.4)
    right = (x - dx * size * 0.5 - (-dy) * size * 0.4,
             y - dy * size * 0.5 - dx * size * 0.4)
    draw.polygon([tip, left, right], fill=fill)


def main():
    meta = json.loads((SAMPLES / 'map.json').read_text())
    drive = json.loads(DRIVE.read_text())
    im = Image.open(SRC).convert('RGB')
    gw, gh = meta['width'], meta['height']
    scale = im.width / gw
    origin = meta['origin_xy']
    res = meta['resolution_m']
    draw = ImageDraw.Draw(im)

    trace = drive.get('trace') or []
    if not trace:
        raise SystemExit('no drive trace')

    # split outbound / inbound at max along
    i_max = max(range(len(trace)), key=lambda i: trace[i]['along'])
    out_tr = trace[: i_max + 1]
    back_tr = trace[i_max:]

    def pts(seq):
        return [world_to_px(p['x'], p['y'], origin, res, gw, gh, scale) for p in seq]

    def draw_poly(seq, color, width):
        p = pts(seq)
        if len(p) >= 2:
            draw.line(p, fill=color, width=width)

    draw_poly(out_tr, (29, 78, 216), 4)      # blue outbound
    draw_poly(back_tr, (217, 119, 6), 3)     # orange return

    # direction ticks
    def ticks(seq, color, step=12):
        for i in range(step, len(seq) - 1, step):
            x0, y0 = world_to_px(seq[i]['x'], seq[i]['y'], origin, res, gw, gh, scale)
            x1, y1 = world_to_px(seq[i + 1]['x'], seq[i + 1]['y'], origin, res, gw, gh, scale)
            yaw = math.atan2(-(y1 - y0), x1 - x0)  # already in image; convert to world-like
            # use pose yaw: 0 = +x
            wyaw = seq[i].get('yaw', 0.0)
            # outbound uses yaw; inbound is yaw+pi if reversing along same line
            if color == (217, 119, 6):
                wyaw = wyaw + math.pi
            arrow(draw, x0, y0, wyaw, 7, color)

    ticks(out_tr, (29, 78, 216), 14)
    ticks(back_tr, (217, 119, 6), 14)

    sx, sy = world_to_px(out_tr[0]['x'], out_tr[0]['y'], origin, res, gw, gh, scale)
    ex, ey = world_to_px(out_tr[-1]['x'], out_tr[-1]['y'], origin, res, gw, gh, scale)
    rx, ry = world_to_px(back_tr[-1]['x'], back_tr[-1]['y'], origin, res, gw, gh, scale)
    r = 5
    draw.ellipse((sx - r, sy - r, sx + r, sy + r), fill=(5, 150, 105), outline=(255, 255, 255))
    draw.ellipse((ex - r, ey - r, ex + r, ey + r), fill=(185, 28, 28), outline=(255, 255, 255))
    draw.ellipse((rx - r - 1, ry - r - 1, rx + r + 1, ry + r + 1), outline=(5, 150, 105), width=2)

    # legend
    try:
        font = ImageFont.truetype('/usr/share/fonts/truetype/nanum/NanumGothic.ttf', 12)
        font_s = ImageFont.truetype('/usr/share/fonts/truetype/nanum/NanumGothic.ttf', 11)
    except Exception:
        font = ImageFont.load_default()
        font_s = font
    box = (6, 6, 168, 78)
    draw.rectangle(box, fill=(255, 255, 255), outline=(51, 65, 85))
    draw.line((14, 22, 40, 22), fill=(29, 78, 216), width=4)
    draw.text((46, 14), '전진 (출발→장애물)', fill=(29, 78, 216), font=font_s)
    draw.line((14, 40, 40, 40), fill=(217, 119, 6), width=3)
    draw.text((46, 32), '후진 (복귀)', fill=(180, 83, 9), font=font_s)
    draw.ellipse((16, 52, 26, 62), fill=(5, 150, 105))
    draw.text((32, 50), '출발', fill=(6, 95, 70), font=font_s)
    draw.ellipse((78, 52, 88, 62), fill=(185, 28, 28))
    draw.text((92, 50), '정지(장애물)', fill=(153, 27, 27), font=font_s)

    im.save(OUT)
    print('WROTE', OUT, 'scale', scale)
    print('start px', sx, sy, 'end px', ex, ey)


if __name__ == '__main__':
    main()
