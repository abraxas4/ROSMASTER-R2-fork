#!/usr/bin/env python3
"""Build Korean HTML report from last 3m drive + RTAB-Map test DB."""
from __future__ import annotations

import json
import math
import sqlite3
from datetime import datetime
from pathlib import Path

from PIL import Image
import io

HOME = Path.home()
TEST_DB = HOME / 'maps/databases/drive_test_sensors.db'
HOME_DB = HOME / 'maps/databases/loc_p37.4386_p127.1378.db'
DRIVE_JSON = Path('/tmp/r2_last_drive.json')
OUT_DIR = HOME / 'ROSMASTER-R2-fork/docs'
IMG_DIR = OUT_DIR / 'drive_test_images'
OUT_HTML = OUT_DIR / 'R2_3m_왕복_테스트.html'


def load_drive():
    if DRIVE_JSON.exists():
        return json.loads(DRIVE_JSON.read_text(encoding='utf-8'))
    return {}


def db_stats(path: Path) -> dict:
    if not path.exists():
        return {'exists': False}
    con = sqlite3.connect(f'file:{path}?mode=ro', uri=True)
    cur = con.cursor()
    tables = {r[0] for r in cur.execute("SELECT name FROM sqlite_master WHERE type='table'")}
    out = {
        'exists': True,
        'bytes': path.stat().st_size,
        'tables': sorted(tables),
        'nodes': 0,
        'data': 0,
        'links': 0,
        'poses': [],
    }
    if 'Node' in tables:
        cols = [r[1] for r in cur.execute('PRAGMA table_info(Node)')]
        out['node_cols'] = cols
        out['nodes'] = cur.execute('SELECT COUNT(*) FROM Node').fetchone()[0]
        # common pose columns vary by version
        want = [c for c in ('id', 'map_id', 'stamp', 'pose', 'x', 'y', 'z') if c in cols]
        if {'id'} <= set(cols):
            rows = cur.execute('SELECT * FROM Node ORDER BY id').fetchall()
            col_i = {c: i for i, c in enumerate(cols)}
            for row in rows:
                item = {'id': row[col_i['id']]}
                if 'stamp' in col_i:
                    item['stamp'] = row[col_i['stamp']]
                if 'map_id' in col_i:
                    item['map_id'] = row[col_i['map_id']]
                out['poses'].append(item)
    if 'Data' in tables:
        out['data'] = cur.execute('SELECT COUNT(*) FROM Data').fetchone()[0]
        out['data_cols'] = [r[1] for r in cur.execute('PRAGMA table_info(Data)')]
    if 'Link' in tables:
        out['links'] = cur.execute('SELECT COUNT(*) FROM Link').fetchone()[0]
    con.close()
    return out


def extract_images(db: Path, dest: Path, limit: int = 12) -> list[dict]:
    dest.mkdir(parents=True, exist_ok=True)
    for old in dest.glob('*'):
        old.unlink()
    con = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
    cur = con.cursor()
    cols = [r[1] for r in cur.execute('PRAGMA table_info(Data)')]
    # RTAB-Map 0.21: id, image, depth, calibration, scan, compressed...
    blob_candidates = [c for c in cols if c.lower() in (
        'image', 'rgb', 'data', 'image_compressed', 'compressed'
    ) or 'image' in c.lower()]
    id_col = 'id' if 'id' in cols else cols[0]
    # pick first non-null blob column
    chosen = None
    for c in blob_candidates:
        n = cur.execute(f'SELECT COUNT(*) FROM Data WHERE {c} IS NOT NULL AND length({c})>100').fetchone()[0]
        if n:
            chosen = c
            break
    if not chosen:
        # try any blob-like
        for c in cols:
            if c == id_col:
                continue
            try:
                n = cur.execute(f'SELECT COUNT(*) FROM Data WHERE {c} IS NOT NULL AND length({c})>1000').fetchone()[0]
            except sqlite3.Error:
                continue
            if n:
                chosen = c
                break
    saved = []
    if not chosen:
        con.close()
        return saved
    ids = [r[0] for r in cur.execute(f'SELECT {id_col} FROM Data WHERE {chosen} IS NOT NULL ORDER BY {id_col}')]
    if not ids:
        con.close()
        return saved
    # even sample
    if len(ids) <= limit:
        pick = ids
    else:
        pick = [ids[int(i * (len(ids) - 1) / (limit - 1))] for i in range(limit)]
    for i, did in enumerate(pick, 1):
        blob = cur.execute(f'SELECT {chosen} FROM Data WHERE {id_col}=?', (did,)).fetchone()[0]
        if not blob:
            continue
        raw = bytes(blob)
        name = f'frame_{i:02d}_id{did}.jpg'
        path = dest / name
        try:
            im = Image.open(io.BytesIO(raw))
            im.convert('RGB').save(path, 'JPEG', quality=85)
            saved.append({
                'file': f'drive_test_images/{name}',
                'id': did,
                'width': im.width,
                'height': im.height,
                'bytes': path.stat().st_size,
            })
        except Exception:
            # maybe already jpeg file bytes
            if raw[:3] == b'\xff\xd8\xff' or raw[:8] == b'\x89PNG\r\n\x1a\n':
                path.write_bytes(raw)
                saved.append({
                    'file': f'drive_test_images/{name}',
                    'id': did,
                    'width': None,
                    'height': None,
                    'bytes': path.stat().st_size,
                })
    con.close()
    return saved


def score_row(label, value, ok: bool, note=''):
    klass = 'ok' if ok else 'bad'
    return f'<tr class="{klass}"><td>{label}</td><td class="num">{value}</td><td>{note}</td></tr>'


def html_escape(s):
    return str(s).replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;')


def main():
    drive = load_drive()
    stats = db_stats(TEST_DB)
    home = db_stats(HOME_DB)
    images = extract_images(TEST_DB, IMG_DIR, 12) if stats.get('exists') else []

    fwd = drive.get('forward_m')
    final = drive.get('final_m')
    ok_drive = bool(drive.get('ok')) and fwd is not None and fwd >= 2.7
    ok_map = stats.get('nodes', 0) >= 5 and stats.get('data', 0) >= 5
    ok_home = home.get('exists') and home.get('bytes', 0) > 500_000_000
    overall = ok_drive and ok_map and ok_home

    mb = lambda n: f'{n/1024/1024:.1f} MB' if n else '—'

    # path svg from drive trace
    svg = ''
    trace = drive.get('trace') or []
    if trace:
        xs = [p['along'] for p in trace]
        # use along as x, keep 0 y (odom y was ~0)
        w, h = 720, 160
        pad = 30
        xmin, xmax = min(xs), max(xs)
        span = max(0.2, xmax - xmin)
        def X(a):
            return pad + (a - xmin) / span * (w - 2 * pad)
        y0 = h // 2
        pts = ' '.join(f'{X(p["along"]):.1f},{y0}' for p in trace)
        svg = f'''<svg viewBox="0 0 {w} {h}" xmlns="http://www.w3.org/2000/svg" role="img">
  <rect width="{w}" height="{h}" fill="#f8fafc"/>
  <line x1="{pad}" y1="{y0}" x2="{w-pad}" y2="{y0}" stroke="#cbd5e1" stroke-width="2"/>
  <polyline points="{pts}" fill="none" stroke="#1d4ed8" stroke-width="3"/>
  <circle cx="{X(xs[0])}" cy="{y0}" r="5" fill="#047857"/>
  <circle cx="{X(xs[-1])}" cy="{y0}" r="5" fill="#b91c1c"/>
  <text x="{X(xs[0])}" y="{y0+22}" font-size="12" text-anchor="middle" fill="#047857">출발</text>
  <text x="{X(max(xs))}" y="{y0-14}" font-size="12" text-anchor="middle" fill="#1d4ed8">최대 {max(xs):.2f} m</text>
  <text x="{X(xs[-1])}" y="{y0+22}" font-size="12" text-anchor="middle" fill="#b91c1c">복귀 {xs[-1]:.2f} m</text>
</svg>'''

    img_html = ''
    if images:
        cells = []
        for im in images:
            wh = ''
            if im.get('width'):
                wh = f'{im["width"]}×{im["height"]} · '
            cells.append(
                f'<figure><img src="{html_escape(im["file"])}" alt="node {im["id"]}">'
                f'<figcaption>노드 {im["id"]} · {wh}{im["bytes"]//1024} KB</figcaption></figure>'
            )
        img_html = '<div class="gallery">' + '\n'.join(cells) + '</div>'
    else:
        img_html = '<p>DB에서 RGB 프레임을 꺼내지 못했습니다. 표의 노드/이미지 개수를 보세요.</p>'

    now = datetime.now().strftime('%Y-%m-%d %H:%M')
    verdict = '잘 됨' if overall else '부분만 됨'
    verdict_cls = 'ok' if overall else 'warn'

    html = f'''<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>R2 3m 왕복 테스트 결과</title>
<style>
  body {{ font-family: "Noto Sans CJK KR","NanumGothic",sans-serif; margin:0; background:#f4f6f8; color:#1a1d26; }}
  header {{ background:#0f172a; color:#fff; padding:1.6rem 1.4rem; }}
  header h1 {{ margin:0 0 .4rem; font-size:1.5rem; }}
  .wrap {{ max-width:960px; margin:0 auto; padding:1.4rem; }}
  section {{ background:#fff; border:1px solid #d8dee6; padding:1.2rem 1.4rem; margin:0 0 1rem; }}
  h2 {{ margin:0 0 .8rem; font-size:1.15rem; border-bottom:3px solid #dbeafe; padding-bottom:.3rem; }}
  table {{ width:100%; border-collapse:collapse; font-size:.94rem; }}
  th,td {{ border:1px solid #d8dee6; padding:.5rem .65rem; text-align:left; }}
  th {{ background:#eef2f7; }}
  td.num {{ font-variant-numeric:tabular-nums; font-weight:700; }}
  tr.ok td {{ background:#ecfdf5; }}
  tr.bad td {{ background:#fef2f2; }}
  .badge {{ display:inline-block; padding:.2rem .7rem; font-weight:700; }}
  .badge.ok {{ background:#d1fae5; color:#065f46; }}
  .badge.warn {{ background:#fef3c7; color:#92400e; }}
  .gallery {{ display:grid; grid-template-columns:repeat(auto-fill,minmax(200px,1fr)); gap:.8rem; }}
  figure {{ margin:0; background:#f8fafc; border:1px solid #e2e8f0; }}
  figure img {{ width:100%; height:140px; object-fit:cover; display:block; }}
  figcaption {{ font-size:.78rem; color:#64748b; padding:.35rem .45rem; }}
  .svgbox {{ border:1px solid #e2e8f0; background:#f8fafc; padding:.4rem; }}
  footer {{ color:#64748b; font-size:.82rem; text-align:center; padding:1rem; }}
</style>
</head>
<body>
<header>
  <div class="wrap" style="padding:0">
    <h1>R2 3 m 왕복 테스트</h1>
    <p>카메라 + 라이다 매핑이 숫자를 남겼는지, 이미지가 쌓였는지</p>
    <p><span class="badge {verdict_cls}">{verdict}</span> · {now}</p>
  </div>
</header>
<div class="wrap">
<section>
  <h2>1. 한눈에 보는 점수</h2>
  <table>
    <tr><th>항목</th><th>숫자</th><th>기준 / 메모</th></tr>
    {score_row('전진 거리', f'{fwd} m' if fwd is not None else '없음', ok_drive, '목표 3.00 m, 합격 ≥ 2.70 m')}
    {score_row('복귀 잔여', f'{final} m' if final is not None else '없음', final is not None and abs(final) <= 0.25, '출발점까지 남긴 거리. 합격 ≤ 0.25 m')}
    {score_row('RTAB 노드', stats.get('nodes', 0), ok_map, '맵에 찍힌 위치 개수')}
    {score_row('이미지/데이터', stats.get('data', 0), stats.get('data', 0) >= 5, '카메라가 남긴 프레임 수')}
    {score_row('그래프 링크', stats.get('links', 0), stats.get('links', 0) >= 4, '노드를 잇는 선')}
    {score_row('테스트 DB 크기', mb(stats.get('bytes')), stats.get('bytes', 0) > 1_000_000, str(TEST_DB))}
    {score_row('집 맵 보존', mb(home.get('bytes')), ok_home, '548 MB대면 안 지워진 것')}
  </table>
</section>
<section>
  <h2>2. 이번 왕복 궤적 (오돔)</h2>
  <div class="svgbox">{svg or "<p>궤적 데이터 없음</p>"}</div>
  <p>파란 선 = 오돔이 본 앞·뒤 이동. 초록 = 출발, 빨강 = 끝난 위치.</p>
</section>
<section>
  <h2>3. 이번 왕복이 남긴 카메라 이미지</h2>
  <p>RTAB-Map이 테스트 DB에 저장한 RGB 프레임을 시간 순으로 고른 것입니다. 집 맵 사진이 아닙니다.</p>
  {img_html}
</section>
<section>
  <h2>4. 파일 위치</h2>
  <table>
    <tr><th>무엇</th><th>경로</th></tr>
    <tr><td>이 보고서</td><td><code>{html_escape(str(OUT_HTML))}</code></td></tr>
    <tr><td>테스트 맵 DB</td><td><code>{html_escape(str(TEST_DB))}</code></td></tr>
    <tr><td>집 맵 (안 건드림)</td><td><code>{html_escape(str(HOME_DB))}</code></td></tr>
  </table>
  <p>매핑은 왕복이 끝나면 스크립트가 끕니다. 따로 끌 필요 없습니다.</p>
</section>
</div>
<footer>ROSMASTER R2 · 테스트 전용 DB · 집 맵 비사용</footer>
</body>
</html>
'''
    # I used %% in now format by mistake in the f-string... wait I used datetime.now().strftime already assigned to `now`
    # In the html I used {now} which is fine.
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    OUT_HTML.write_text(html, encoding='utf-8')
    print('WROTE', OUT_HTML)
    print('IMAGES', len(images))
    print('NODES', stats.get('nodes'), 'DATA', stats.get('data'))
    print('DRIVE', fwd, final, 'OVERALL', overall)


if __name__ == '__main__':
    main()
