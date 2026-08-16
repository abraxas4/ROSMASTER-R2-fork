#!/usr/bin/env python3
"""Full Korean HTML: map, raw sensors, calibration, algorithms."""
from __future__ import annotations

import json
import shutil
import sys
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from report_drive_test import db_stats, extract_images, load_drive

HOME = Path.home()
DOCS = HOME / 'ROSMASTER-R2-fork/docs'
SAMPLES = Path('/tmp/r2_samples')
TEST_DB = HOME / 'maps/databases/drive_test_sensors.db'
HOME_DB = HOME / 'maps/databases/loc_p37.4386_p127.1378.db'
OUT = DOCS / 'R2_3m_맵_센서_알고리즘.html'
ASSET = DOCS / 'drive_test_full'


def jload(p: Path):
    if p.exists():
        return json.loads(p.read_text(encoding='utf-8'))
    return {}


def copy_asset(src: Path, name: str) -> str | None:
    if not src or not Path(src).exists():
        return None
    ASSET.mkdir(parents=True, exist_ok=True)
    dest = ASSET / name
    shutil.copy2(src, dest)
    return f'drive_test_full/{name}'


def box(x, y, w, h, text, fill='#fff', stroke='#334155'):
    lines = text.split('\n')
    t = ''
    for i, line in enumerate(lines):
        t += f'<text x="{x+w/2}" y="{y+18+i*16}" text-anchor="middle" font-size="12" font-family="Noto Sans KR,sans-serif">{line}</text>'
    return f'<rect x="{x}" y="{y}" width="{w}" height="{h}" fill="{fill}" stroke="{stroke}" stroke-width="2"/>{t}'


def hline(x1, y, x2):
    return f'<polyline points="{x1},{y} {x2},{y}" fill="none" stroke="#475569" stroke-width="2"/>'


def vline(x, y1, y2):
    return f'<polyline points="{x},{y1} {x},{y2}" fill="none" stroke="#475569" stroke-width="2"/>'


def arrow(x, y, d='down'):
    if d == 'down':
        return f'<polygon points="{x-4},{y-6} {x+4},{y-6} {x},{y}" fill="#475569"/>'
    if d == 'right':
        return f'<polygon points="{x-6},{y-4} {x-6},{y+4} {x},{y}" fill="#475569"/>'
    return ''


def occupancy_explain_html(
    res, gw, gh, origin, n_free, n_occ, n_unk,
    paper_w, paper_h, area_free, area_occ, fwd, blocked,
) -> str:
    ox, oy = origin[0], origin[1]
    flow = '''<svg viewBox="0 0 900 260" xmlns="http://www.w3.org/2000/svg" role="img">
  <rect width="900" height="260" fill="#f8fafc"/>
  <rect x="20" y="20" width="200" height="48" fill="#d1fae5" stroke="#047857" stroke-width="2"/>
  <text x="120" y="49" text-anchor="middle" font-size="13" font-family="Noto Sans KR,sans-serif">/scan 거리 배열</text>
  <rect x="20" y="100" width="200" height="48" fill="#d1fae5" stroke="#047857" stroke-width="2"/>
  <text x="120" y="129" text-anchor="middle" font-size="13" font-family="Noto Sans KR,sans-serif">로봇이 맵 위 어디인가</text>
  <rect x="20" y="180" width="200" height="48" fill="#fff" stroke="#334155" stroke-width="2"/>
  <text x="120" y="209" text-anchor="middle" font-size="13" font-family="Noto Sans KR,sans-serif">laser TF · RangeMin 0.2</text>
  <rect x="350" y="100" width="220" height="48" fill="#fef3c7" stroke="#b45309" stroke-width="2"/>
  <text x="460" y="129" text-anchor="middle" font-size="13" font-family="Noto Sans KR,sans-serif">한 광선을 칸에 투영</text>
  <rect x="680" y="40" width="200" height="48" fill="#fff" stroke="#334155" stroke-width="2"/>
  <text x="780" y="69" text-anchor="middle" font-size="13" font-family="Noto Sans KR,sans-serif">중간 칸 = 비어 있음</text>
  <rect x="680" y="160" width="200" height="48" fill="#1a1d26" stroke="#111" stroke-width="2"/>
  <text x="780" y="189" text-anchor="middle" font-size="13" fill="#fff" font-family="Noto Sans KR,sans-serif">끝 칸 = 막힘</text>
  <polyline points="220,44 280,44 280,124 350,124" fill="none" stroke="#475569" stroke-width="2"/>
  <polyline points="220,124 350,124" fill="none" stroke="#475569" stroke-width="2"/>
  <polyline points="220,204 280,204 280,124" fill="none" stroke="#475569" stroke-width="2"/>
  <polyline points="570,124 620,124 620,64 680,64" fill="none" stroke="#475569" stroke-width="2"/>
  <polyline points="620,124 620,184 680,184" fill="none" stroke="#475569" stroke-width="2"/>
  <polygon points="346,120 350,124 346,128" fill="#475569"/>
  <polygon points="676,60 680,64 676,68" fill="#475569"/>
  <polygon points="676,180 680,184 676,188" fill="#475569"/>
</svg>'''
    ray = '''<svg viewBox="0 0 720 180" xmlns="http://www.w3.org/2000/svg" role="img">
  <rect width="720" height="180" fill="#f8fafc"/>
  <rect x="20" y="70" width="28" height="28" fill="#2563eb"/>
  <text x="34" y="120" text-anchor="middle" font-size="11" font-family="Noto Sans KR,sans-serif">로봇</text>
  <rect x="70" y="70" width="28" height="28" fill="#ffffff" stroke="#94a3b8"/>
  <rect x="110" y="70" width="28" height="28" fill="#ffffff" stroke="#94a3b8"/>
  <rect x="150" y="70" width="28" height="28" fill="#ffffff" stroke="#94a3b8"/>
  <rect x="190" y="70" width="28" height="28" fill="#ffffff" stroke="#94a3b8"/>
  <rect x="230" y="70" width="28" height="28" fill="#1a1d26"/>
  <rect x="270" y="70" width="28" height="28" fill="#c4c4c4"/>
  <rect x="310" y="70" width="28" height="28" fill="#c4c4c4"/>
  <polyline points="48,84 230,84" fill="none" stroke="#0f172a" stroke-width="2"/>
  <text x="150" y="50" text-anchor="middle" font-size="12" font-family="Noto Sans KR,sans-serif">광선이 지나간 칸 → 하양 (비어 있음)</text>
  <text x="244" y="130" text-anchor="middle" font-size="12" font-family="Noto Sans KR,sans-serif">맞은 칸 → 검정</text>
  <text x="360" y="130" text-anchor="middle" font-size="12" font-family="Noto Sans KR,sans-serif">아직 광선 없음 → 회색</text>
</svg>'''
    return f'''
<section id="occupancy" style="border:none;padding:0;margin:1.2rem 0 0;background:transparent">
  <h3>점유 격자 <code>/map</code>이 무엇인가</h3>
  <p>사진은 아닙니다. 위에서 내려다본 <strong>칸 모음( OccupancyGrid )</strong>입니다.
  이번 그림은 가로 {gw}칸 × 세로 {gh}칸이고, 한 칸이 <strong>{res:.3f} m (5 cm)</strong>입니다.
  종이 크기는 약 {paper_w:.2f} m × {paper_h:.2f} m 입니다. 그중 실제로 “봤다”고 칠한 빈 공간은 약 {area_free:.1f} m², 벽으로 칠한 면적은 약 {area_occ:.1f} m²입니다.</p>
  <table>
    <tr><th>색</th><th>칸 값</th><th>뜻</th><th>이번 장 개수</th></tr>
    <tr><td style="background:#fff">하양</td><td>0</td><td>레이저가 여기를 <em>지나갔다</em>. 그때는 비어 있었다 (지나갈 수 있는 바닥).</td><td>{n_free}</td></tr>
    <tr><td style="background:#1a1d26;color:#fff">검정</td><td>100 근처</td><td>레이저가 여기에서 <em>멈췄다</em>. 벽, 가구, 사람, 기둥.</td><td>{n_occ}</td></tr>
    <tr><td style="background:#c8c8c8">회색</td><td>-1</td><td>한 번도 광선이 안 지나감. 모름. 비었는지 막혔는지 이 그림만으로는 모름.</td><td>{n_unk}</td></tr>
  </table>

  <h3>그런 그림을 그리려면 무엇을 알고 있나</h3>
  <p>한 칸을 칠하려면 두 가지가 동시에 있어야 합니다. 카메라 픽셀 색깔이 직접 하양/검정을 칠하지는 않습니다.</p>
  <table>
    <tr><th>알아야 하는 것</th><th>어디서 오나</th><th>이 그림에서 하는 일</th></tr>
    <tr>
      <td>지금 로봇이 맵 좌표 어디에 있나 (x, y, 각도)</td>
      <td>RTAB-Map이 고친 자세. 재료는 <code>/odom</code> + 라이다 맞춤 + 카메라 루프</td>
      <td>광선을 “어디서부터” 쏠지 정함</td>
    </tr>
    <tr>
      <td>라이다 한 바퀴의 거리 배열</td>
      <td><code>/scan</code> (이번 센서 약 2020개 빔)</td>
      <td>각 방향마다 “몇 미터에서 맞았나”</td>
    </tr>
    <tr>
      <td>라이다가 차체에서 얼마나 앞에 있나</td>
      <td>고정 TF: 앞으로 4.35 cm, 위로 11 cm</td>
      <td>광선 시작점을 바퀴 중심이 아니라 센서 위치로</td>
    </tr>
    <tr>
      <td>너무 가까운 점은 버릴 것</td>
      <td><code>Grid/RangeMin = 0.2 m</code></td>
      <td>차 자기 몸·근처 반사로 검정을 칠하지 않음</td>
    </tr>
    <tr>
      <td>칸 크기</td>
      <td>RTAB-Map 격자 해상도 5 cm</td>
      <td>세상을 바둑판으로 자름</td>
    </tr>
  </table>
  <p>RGB·깊이 사진은 <strong>로봇이 같은 장소를 다시 알아보게</strong> 도와 자세를 바로잡습니다. 2D <code>/map</code>의 흰/검은 칠은 거의 전부 <strong>라이다 광선</strong>입니다. 카메라가 꺼지면 자세가 틀어져 벽이 이중으로 겹칠 수 있고, 라이다가 꺼지면 이 격자 그림 자체가 안 채워집니다.</p>
  <div class="fig">{flow}</div>
  <p>한 줄기만 보면 이렇게 됩니다.</p>
  <div class="fig">{ray}</div>

  <h3>이 그림에서 어딜 보면 무엇을 아는가</h3>
  <table>
    <tr><th>보는 곳</th><th>알게 되는 것</th><th>이번 주행에 대입</th></tr>
    <tr>
      <td>회색 바다</td>
      <td>차가 아직 안 본 공간. 집이 없는 게 아니라, 이번 왕복의 레이저가 안 닿음.</td>
      <td>3 m 남짓만 왕복해서 종이의 대부분이 회색입니다.</td>
    </tr>
    <tr>
      <td>하양 덩어리의 모양</td>
      <td>레이저가 “뚫고 본” 바닥. 대략 복도·방의 열린 부분.</td>
      <td>가늘고 긴 하양은 전진한 통로입니다. 옆으로 팬 하양은 그때 라이다가 옆문·옆공간을 본 것.</td>
    </tr>
    <tr>
      <td>하양을 둘러싼 검정 테두리</td>
      <td>그때 시야를 막은 것. 벽, 가구, 문틀, 사람.</td>
      <td>끝에서 막힌 검정 근처가 약 {fwd} m 지점, 앞 거리 {blocked or "장애물"} 로 멈춘 곳과 맞습니다.</td>
    </tr>
    <tr>
      <td>하양 한가운데</td>
      <td>차가 지나간 자리. 여기를 비었다고 여러 번 확인함.</td>
      <td>왕복이라 같은 통로를 두 번 봐서 하양이 비교적 확실합니다.</td>
    </tr>
    <tr>
      <td>검정 선이 두껍거나 이중</td>
      <td>자세가 조금 흔들렸거나, 다른 각도에서 같은 벽을 다시 봄.</td>
      <td>짧게 한 번만 본 맵에서 흔합니다. 나쁜 센서라는 뜻은 아닙니다.</td>
    </tr>
    <tr>
      <td>하양 안의 검정 점</td>
      <td>얇은 다리, 노이즈, 잠깐 나타난 물체.</td>
      <td>한두 점은 무시해도 됩니다. 줄로 이어지면 실제 장애물.</td>
    </tr>
    <tr>
      <td>격자 원점 ({ox:.2f}, {oy:.2f}) m</td>
      <td>그림 왼쪽 아래(맵 좌표)가 세상 원점이 아니라, RTAB이 잡은 맵 원점.</td>
      <td>숫자 자체보다 “칸 5 cm”가 중요합니다.</td>
    </tr>
  </table>

  <h3>이 그림이 뜻하는 것 / 뜻하지 않는 것</h3>
  <table>
    <tr><th>뜻하는 것</th><th>뜻하지 않는 것</th></tr>
    <tr>
      <td>이번 왕복에서 레이저가 본 2D 평면도</td>
      <td>천장, 높이, 색깔, “저게 의자다” 같은 이름</td>
    </tr>
    <tr>
      <td>지금 시점에서 비었다고 / 막혔다고 <em>관측된</em> 칸</td>
      <td>앞으로 사람이 들어와도 영원히 하양이라는 보장</td>
    </tr>
    <tr>
      <td>자율주행이 “어디를 피할지” 쓰는 밑그림</td>
      <td>집 전체 도면. 회색은 미탐사지입니다</td>
    </tr>
    <tr>
      <td>자세가 맞으면 벽이 한 줄로 모임</td>
      <td>오돔 y=0 이므로 물리적으로 완벽히 직선이었다는 증거. 이 그림의 굽음은 방 모양 + 자세 오차입니다</td>
    </tr>
  </table>
  <p>한 줄로 말하면: <strong>로봇이 “나는 여기서 광선을 쐈고, 저쪽에서 맞았다”를 5 cm 칸에 누적한 결과</strong>가 <code>/map</code>입니다.</p>
</section>
'''


def main():
    drive = load_drive()
    sensors = jload(SAMPLES / 'sensors.json')
    mmap = jload(SAMPLES / 'map.json')
    stats = db_stats(TEST_DB)
    home = db_stats(HOME_DB)
    frames = extract_images(TEST_DB, DOCS / 'drive_test_images', 8)

    map_rel = copy_asset(Path(mmap['file']) if mmap.get('file') else Path(), 'result_map.png')
    path_png = DOCS / 'drive_test_full' / 'result_map_with_path.png'
    try:
        import overlay_path_on_map
        overlay_path_on_map.main()
    except Exception as exc:
        print('path overlay skip', exc)
    map_path_rel = 'drive_test_full/result_map_with_path.png' if path_png.exists() else map_rel
    rgb_rel = copy_asset(Path(sensors.get('rgb_image', {}).get('file', '')), 'sample_rgb.jpg')
    depth_rel = copy_asset(Path(sensors.get('depth_image', {}).get('preview', '')), 'sample_depth.png')

    fwd = drive.get('forward_m')
    final = drive.get('final_m')
    blocked = drive.get('blocked')
    now = datetime.now().strftime('%Y-%m-%d %H:%M')

    scan = sensors.get('scan', {})
    imu = sensors.get('imu_raw', {})
    odom = sensors.get('odom', {})
    cam = sensors.get('camera_info', {})
    vol = sensors.get('voltage', {})
    rgb = sensors.get('rgb_image', {})
    depth = sensors.get('depth_image', {})

    def pre(obj):
        return '<pre>' + json.dumps(obj, ensure_ascii=False, indent=2) + '</pre>'

    K = cam.get('K') or []
    K3 = ''
    if len(K) == 9:
        K3 = f'''<pre>K = [ {K[0]:.2f}  {K[1]:.2f}  {K[2]:.2f} ]
    [ {K[3]:.2f}  {K[4]:.2f}  {K[5]:.2f} ]
    [ {K[6]:.2f}  {K[7]:.2f}  {K[8]:.2f} ]
fx={K[0]:.1f}  fy={K[4]:.1f}  cx={K[2]:.1f}  cy={K[5]:.1f}</pre>'''

    shown = map_path_rel or map_rel
    map_img = (
        f'<p>파란 화살표는 출발→장애물 전진, 주황은 후진 복귀, 초록=출발, 빨강=정지입니다. 왕복이라 선이 겹칩니다.</p>'
        f'<img src="{shown}" alt="occupancy map with path" style="max-width:100%;border:1px solid #d8dee6">'
        + (f'<p style="font-size:.85rem;color:#64748b">격자만: <a href="{map_rel}">경로 없는 그림</a></p>' if map_rel else '')
        if shown else '<p>맵 이미지를 저장하지 못했습니다.</p>'
    )

    res = float(mmap.get('resolution_m') or 0.05)
    gw = int(mmap.get('width') or 0)
    gh = int(mmap.get('height') or 0)
    origin = mmap.get('origin_xy') or [0, 0]
    n_free = int(mmap.get('free') or 0)
    n_occ = int(mmap.get('occupied') or 0)
    n_unk = int(mmap.get('unknown') or 0)
    paper_w = gw * res
    paper_h = gh * res
    area_free = n_free * res * res
    area_occ = n_occ * res * res
    occ_explain = occupancy_explain_html(
        res=res, gw=gw, gh=gh, origin=origin,
        n_free=n_free, n_occ=n_occ, n_unk=n_unk,
        paper_w=paper_w, paper_h=paper_h,
        area_free=area_free, area_occ=area_occ,
        fwd=fwd, blocked=blocked,
    )
    rgb_img = f'<img src="{rgb_rel}" alt="rgb" style="max-width:100%">' if rgb_rel else ''
    depth_img = f'<img src="{depth_rel}" alt="depth" style="max-width:100%">' if depth_rel else ''

    gallery = ''
    if frames:
        gallery = '<div class="gallery">' + ''.join(
            f'<figure><img src="{f["file"]}" alt="n{f["id"]}"><figcaption>노드 {f["id"]}</figcaption></figure>'
            for f in frames
        ) + '</div>'

    # flowcharts
    flow_top = f'''<svg viewBox="0 0 900 340" xmlns="http://www.w3.org/2000/svg">
  <rect width="900" height="340" fill="#f8fafc"/>
  {box(40,20,180,48,'센서 입력','#d1fae5','#047857')}
  {box(360,20,180,48,'위치 추정 EKF','#fff','#334155')}
  {box(680,20,180,48,'RTAB-Map SLAM','#dbeafe','#1d4ed8')}
  {hline(220,44,360)}{arrow(360,44,'right')}
  {hline(540,44,680)}{arrow(680,44,'right')}
  {box(40,120,180,48,'카메라 RGB+깊이','#fff','#334155')}
  {box(40,190,180,48,'라이다 /scan','#fff','#334155')}
  {box(40,260,180,48,'IMU + 바퀴오돔','#fff','#334155')}
  {vline(130,68,120)}{arrow(130,120,'down')}
  {vline(130,168,190)}{arrow(130,190,'down')}
  {vline(130,238,260)}{arrow(130,260,'down')}
  {box(360,190,180,48,'/odom','#fef3c7','#b45309')}
  {hline(220,284,270)}{vline(270,214,284)}{hline(270,214,360)}{arrow(360,214,'right')}
  {box(680,190,180,48,'그래프 + 점유격자','#dbeafe','#1d4ed8')}
  {vline(770,68,190)}{arrow(770,190,'down')}
  {box(680,270,180,48,'테스트 DB / 맵 PNG','#fff','#334155')}
  {vline(770,238,270)}{arrow(770,270,'down')}
</svg>'''

    flow_rtab = f'''<svg viewBox="0 0 900 520" xmlns="http://www.w3.org/2000/svg">
  <rect width="900" height="520" fill="#f8fafc"/>
  {box(340,16,220,44,'RGB + Depth + Scan + Odom','#d1fae5','#047857')}
  {vline(450,60,90)}{arrow(450,90,'down')}
  {box(340,90,220,44,'rgbd_sync 시간 맞춤','#fff','#334155')}
  {vline(450,134,164)}{arrow(450,164,'down')}
  {box(340,164,220,44,'새 키프레임인가?','#fef3c7','#b45309')}
  {hline(560,186,700)}{vline(700,186,230)}{box(600,230,220,44,'버림 / 대기','#fff','#334155')}
  {vline(450,208,250)}{arrow(450,250,'down')}
  {box(340,250,220,44,'시각 특징 + 깊이','#fff','#334155')}
  {vline(450,294,324)}{arrow(450,324,'down')}
  {box(340,324,220,44,'라이다 ICP 정합','#fff','#334155')}
  {vline(450,368,398)}{arrow(450,398,'down')}
  {box(340,398,220,44,'그래프에 노드 추가','#dbeafe','#1d4ed8')}
  {vline(450,442,472)}{arrow(450,472,'down')}
  {box(340,472,220,44,'2D 격자 맵 갱신','#dbeafe','#1d4ed8')}
</svg>'''

    flow_ekf = f'''<svg viewBox="0 0 900 280" xmlns="http://www.w3.org/2000/svg">
  <rect width="900" height="280" fill="#f8fafc"/>
  {box(40,30,200,48,'바퀴 /odom_raw','#fff','#334155')}
  {box(40,150,200,48,'Madgwick /imu/data','#fff','#334155')}
  {box(350,90,200,48,'EKF 예측-수정','#fef3c7','#b45309')}
  {box(660,90,200,48,'/odom + TF','#dbeafe','#1d4ed8')}
  {hline(240,54,300)}{vline(300,54,114)}{hline(300,114,350)}{arrow(350,114,'right')}
  {hline(240,174,300)}{vline(300,114,174)}
  {hline(550,114,660)}{arrow(660,114,'right')}
  <text x="450" y="240" text-anchor="middle" font-size="12" font-family="Noto Sans KR,sans-serif">2D 모드 · IMU는 yaw만 · 가속도는 중력 제거</text>
</svg>'''

    flow_stop = f'''<svg viewBox="0 0 900 220" xmlns="http://www.w3.org/2000/svg">
  <rect width="900" height="220" fill="#f8fafc"/>
  {box(40,80,180,48,'/scan 한 바퀴','#d1fae5','#047857')}
  {box(320,80,220,48,'앞 또는 뒤 25도 최소거리','#fff','#334155')}
  {box(640,20,220,48,'40 cm 미만 → 즉시 정지','#fee2e2','#b91c1c')}
  {box(640,140,220,48,'아니면 0.15 m/s 유지','#d1fae5','#047857')}
  {hline(220,104,320)}{arrow(320,104,'right')}
  {hline(540,104,590)}{vline(590,44,164)}{hline(590,44,640)}{arrow(640,44,'right')}
  {hline(590,164,640)}{arrow(640,164,'right')}
</svg>'''

    html = f'''<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>R2 3m 맵 · 센서 · 보정 · 알고리즘</title>
<style>
  body {{ font-family:"Noto Sans CJK KR","NanumGothic",sans-serif; margin:0; background:#f4f6f8; color:#1a1d26; }}
  header {{ background:#0f172a; color:#fff; padding:1.8rem 1.4rem; }}
  header h1 {{ margin:0 0 .4rem; font-size:1.55rem; }}
  .wrap {{ max-width:980px; margin:0 auto; padding:1.4rem; }}
  nav.toc {{ background:#fff; border:1px solid #d8dee6; padding:1rem 1.3rem; margin-bottom:1rem; }}
  section {{ background:#fff; border:1px solid #d8dee6; padding:1.25rem 1.4rem; margin:0 0 1rem; }}
  h2 {{ margin:0 0 .8rem; font-size:1.2rem; border-bottom:3px solid #dbeafe; padding-bottom:.3rem; }}
  h3 {{ margin:1.1rem 0 .5rem; font-size:1.02rem; }}
  table {{ width:100%; border-collapse:collapse; font-size:.92rem; }}
  th,td {{ border:1px solid #d8dee6; padding:.5rem .65rem; vertical-align:top; text-align:left; }}
  th {{ background:#eef2f7; }}
  pre {{ background:#0f172a; color:#e2e8f0; padding:.8rem 1rem; overflow:auto; font-size:.82rem; }}
  code {{ background:#eef2f6; padding:.1em .35em; }}
  .fig {{ border:1px solid #e2e8f0; background:#f8fafc; padding:.5rem; overflow:auto; }}
  .gallery {{ display:grid; grid-template-columns:repeat(auto-fill,minmax(180px,1fr)); gap:.7rem; }}
  figure {{ margin:0; border:1px solid #e2e8f0; }}
  figure img {{ width:100%; height:120px; object-fit:cover; display:block; }}
  figcaption {{ font-size:.75rem; color:#64748b; padding:.3rem; }}
  .ok {{ background:#ecfdf5; }}
  .two {{ display:grid; grid-template-columns:1fr 1fr; gap:.8rem; }}
  @media (max-width:700px) {{ .two {{ grid-template-columns:1fr; }} }}
</style>
</head>
<body>
<header>
  <div class="wrap" style="padding:0">
    <h1>이번에 그린 맵과 그 재료</h1>
    <p>3 m 왕복 테스트 · 센서 raw · 보정 · 알고리즘 (위에서 아래로)</p>
    <p style="opacity:.8">{now} · 집 맵은 사용하지 않음 · 끝나면 자동 종료</p>
  </div>
</header>
<div class="wrap">
<nav class="toc">
  <ol>
    <li><a href="#top">한 장 요약</a></li>
    <li><a href="#map">결과 맵의 생김새</a></li>
    <li><a href="#occupancy">점유 격자 /map 읽는 법</a></li>
    <li><a href="#sensors">맵을 그린 센서</a></li>
    <li><a href="#raw">raw 값 예제</a></li>
    <li><a href="#calib">보정 데이터와 기법</a></li>
    <li><a href="#algo">알고리즘 (큰 그림 → 세부)</a></li>
    <li><a href="#stop">장애물 정지 (맵 없음)</a></li>
  </ol>
</nav>

<section id="top">
  <h2>1. 한 장 요약</h2>
  <table>
    <tr><th>항목</th><th>이번 값</th></tr>
    <tr class="ok"><td>전진</td><td>{fwd} m (목표 3.00)</td></tr>
    <tr class="ok"><td>복귀 잔여</td><td>{final} m</td></tr>
    <tr><td>장애물 정지</td><td>{blocked or "없음 (40 cm 안 물체 없음)"}</td></tr>
    <tr><td>RTAB 노드 / 이미지</td><td>{stats.get("nodes",0)} / {stats.get("data",0)}</td></tr>
    <tr><td>테스트 DB</td><td>{TEST_DB} · {stats.get("bytes",0)/1024/1024:.1f} MB</td></tr>
    <tr><td>집 맵</td><td>그대로 {home.get("bytes",0)/1024/1024:.0f} MB</td></tr>
    <tr><td>배터리 샘플</td><td>{vol.get("volts","?")} V</td></tr>
  </table>
  <p>맵을 그리는 입력은 <strong>카메라 RGB+깊이, 라이다 스캔, EKF 오돔</strong>입니다. 앞조향·뒷구동은 움직이기만 하고, IMU는 오돔 융합에만 들어갑니다.</p>
</section>

<section id="map">
  <h2>2. 결과 맵의 생김새</h2>
  <p>RTAB-Map이 만든 2D 점유 격자 <code>/map</code>입니다. 흰색=비어 있음, 검정=벽/장애물, 회색=아직 모름.</p>
  <div class="fig">{map_img}</div>
  <table>
    <tr><th>격자 가로</th><td>{mmap.get("width","?")}</td></tr>
    <tr><th>격자 세로</th><td>{mmap.get("height","?")}</td></tr>
    <tr><th>칸 크기</th><td>{mmap.get("resolution_m","?")} m</td></tr>
    <tr><th>원점 (x,y)</th><td>{mmap.get("origin_xy","?")}</td></tr>
    <tr><th>빈 칸 / 벽 / 모름</th><td>{mmap.get("free","?")} / {mmap.get("occupied","?")} / {mmap.get("unknown","?")}</td></tr>
  </table>
  {occ_explain}
  <h3>키프레임 사진 (DB에서 꺼냄)</h3>
  {gallery}
</section>

<section id="sensors">
  <h2>3. 맵을 그리기 위해 쓴 센서</h2>
  <table>
    <tr><th>센서</th><th>토픽</th><th>맵에서의 역할</th><th>맵 필수?</th></tr>
    <tr><td>Astra Plus RGB</td><td><code>/camera/color/image_raw</code></td><td>장소 생김새, 루프 클로저</td><td>예</td></tr>
    <tr><td>Astra Plus Depth</td><td><code>/camera/depth/image_raw</code></td><td>거리 있는 점, 3D</td><td>예</td></tr>
    <tr><td>카메라 내부변수</td><td><code>/camera/color/camera_info</code></td><td>픽셀 → 광선</td><td>예</td></tr>
    <tr><td>YDLIDAR 4ROS</td><td><code>/scan</code></td><td>2D 벽, ICP 정합</td><td>예</td></tr>
    <tr><td>바퀴+IMU EKF</td><td><code>/odom</code></td><td>이번 프레임이 어디쯤인지 초기값</td><td>예</td></tr>
    <tr><td>IMU 원시</td><td><code>/imu/data_raw</code></td><td>오돔용. RTAB은 직접 안 봄</td><td>간접</td></tr>
    <tr><td>앞조향 / 뒷구동</td><td><code>/cmd_vel</code></td><td>움직임만. 맵 그림 아님</td><td>아니오</td></tr>
  </table>
</section>

<section id="raw">
  <h2>4. raw 값 예제 (이번 세션에서 한 장씩 저장)</h2>
  <h3>라이다 /scan</h3>
  {pre(scan) if scan else "<p>없음</p>"}
  <h3>IMU 원시 /imu/data_raw</h3>
  {pre(imu) if imu else "<p>없음</p>"}
  <h3>오돔 /odom</h3>
  {pre(odom) if odom else "<p>없음</p>"}
  <h3>카메라 내부변수 K</h3>
  {K3 or pre(cam)}
  <h3>RGB / 깊이 한 장</h3>
  <div class="two">
    <div><p>RGB {rgb.get("width","?")}×{rgb.get("height","?")} {rgb.get("encoding","")}</p>{rgb_img}</div>
    <div><p>깊이 center={depth.get("center_pixel","?")} ({depth.get("encoding","")})</p>{depth_img}</div>
  </div>
</section>

<section id="calib">
  <h2>5. 보정에 관계된 모든 데이터와 기법</h2>
  <p>이번 스택은 <strong>매핑 시작 때 캘리브레이션 루틴을 돌리지 않습니다.</strong> 아래는 실제로 적용 중인 값입니다.</p>
  <table>
    <tr><th>대상</th><th>기법</th><th>지금 값 / 상태</th></tr>
    <tr>
      <td>카메라 내부변수</td>
      <td>제조사 <code>camera_info</code> (체스보드 재촬영 없음)</td>
      <td>{cam.get("width","?")}×{cam.get("height","?")} · {cam.get("distortion_model","?")}<br>D={cam.get("D","?")}</td>
    </tr>
    <tr>
      <td>카메라-로봇 위치</td>
      <td>URDF / 드라이버 TF</td>
      <td>Astra launch가 color/depth optical frame 발행</td>
    </tr>
    <tr>
      <td>라이다 위치</td>
      <td>고정 static TF</td>
      <td>base_link → laser : x=0.0435 m, z=0.11 m, yaw=0 (4ROS)</td>
    </tr>
    <tr>
      <td>라이다 자체 거리</td>
      <td>센서 펌웨어. 추가 보정 없음</td>
      <td>range_min={scan.get("range_min","?")} · range_max={scan.get("range_max","?")}</td>
    </tr>
    <tr>
      <td>IMU</td>
      <td>Madgwick 필터 (지자계 끔) + EKF</td>
      <td>use_mag=false · imu0_relative=true · 중력 제거 · yaw만 사용</td>
    </tr>
    <tr>
      <td>바퀴 오돔 스케일</td>
      <td>linear_scale_x/y 파라미터</td>
      <td>둘 다 1.0. calibrate_linear_R2.py 는 자동 실행 안 함</td>
    </tr>
    <tr>
      <td>조향 중립</td>
      <td>보드 set_akm_default_angle</td>
      <td>기본각 약 100 (60–120). 소프트웨어 0도 = 이 각. 손으로 맞춘 바퀴는 다음 명령에 덮일 수 있음</td>
    </tr>
    <tr>
      <td>RGB-D 동기</td>
      <td>approx_sync, 최대 간격 0.01 s</td>
      <td>색과 깊이가 10 ms 넘게 어긋나면 그 쌍은 버림</td>
    </tr>
    <tr>
      <td>RTAB ↔ IMU</td>
      <td>사용 안 함</td>
      <td>subscribe_imu 없음 · Optimizer/GravitySigma=0</td>
    </tr>
    <tr>
      <td>격자에서 로봇 몸체</td>
      <td>Grid/RangeMin</td>
      <td>0.2 m 안 스캔은 맵에 안 넣음</td>
    </tr>
    <tr>
      <td>장애물 정지</td>
      <td>라이다 전방/후방 원뿔 최소거리</td>
      <td>25° · 40 cm. 맵 불필요</td>
    </tr>
  </table>
</section>

<section id="algo">
  <h2>6. 알고리즘 — 위에서 아래로</h2>
  <h3>6.1 전체</h3>
  <div class="fig">{flow_top}</div>
  <h3>6.2 위치 추정 (EKF)</h3>
  <div class="fig">{flow_ekf}</div>
  <p>바퀴는 짧은 시간에 믿을 만하고, IMU yaw가 미끄러짐을 줄입니다. 결과는 <code>/odom</code>입니다. RTAB-Map은 이 값을 “대충 여기”로 받은 뒤 라이다·카메라로 고칩니다.</p>
  <h3>6.3 RTAB-Map (이번에 켠 설정)</h3>
  <div class="fig">{flow_rtab}</div>
  <table>
    <tr><th>파라미터</th><th>값</th><th>뜻</th></tr>
    <tr><td>Reg/Strategy</td><td>1</td><td>라이다 ICP로 이웃 프레임 맞춤</td></tr>
    <tr><td>Reg/Force3DoF</td><td>true</td><td>평면만 (x, y, yaw)</td></tr>
    <tr><td>RGBD/NeighborLinkRefining</td><td>true</td><td>이웃 링크를 스캔으로 다듬음</td></tr>
    <tr><td>subscribe_rgbd / subscribe_scan</td><td>true / true</td><td>카메라+라이다 융합</td></tr>
  </table>
  <p>학습(신경망)은 없습니다. 특징점 + ICP + 그래프 최적화입니다.</p>
</section>

<section id="stop">
  <h2>7. 장애물에서 멈추는 방법 (맵 없음)</h2>
  <div class="fig">{flow_stop}</div>
  <p>라이다 한 프레임만 봅니다. 앞(전진) 또는 뒤(후진) 25° 안에서 40 cm보다 가까운 유효 거리가 있으면 <code>cmd_vel=0</code>입니다. 피해서 돌아가지는 않습니다.</p>
</section>
</div>
<footer style="text-align:center;color:#64748b;padding:1.2rem;font-size:.85rem">
  파일: {OUT} · 테스트 DB만 사용 · 집 맵 비사용
</footer>
</body>
</html>
'''
    DOCS.mkdir(parents=True, exist_ok=True)
    OUT.write_text(html, encoding='utf-8')
    print('WROTE', OUT)


if __name__ == '__main__':
    main()
