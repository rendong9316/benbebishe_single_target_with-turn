#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
build_span_scene.py — 距离分层场景构建器（配准验证专用）

问题背景
--------
现有场景 adsb_0002 的目标到 R2 的距离只有 1014-1365 km（跨度 342 km），
而电离层虚高 h 失配引起的群距离偏差随距离单调衰减：

    R2 距离 1000 km → 偏差 37.4 km
    R2 距离 1365 km → 偏差 30.0 km
    R2 距离 1700 km → 偏差 25.2 km

跨度只有 342 km 时，偏差的变化仅 7.2 km，近似为常数，
因此"两参数均值标定"和"三参数 MLE"在数学上等价（实测完全相同的 RMSE）。
只有把距离跨度拉开，均值标定的"常数补偿"才会在两端留下残余误差，
三参数方法的价值才能显现。

设计
----
1. 空间窗放大到 [116.0, 122.2]E x [26.5, 34.0]N —— 这是双站共视区
   （R1/R2 波束 93°±22.5°、距离 1000-2000 km）与大陆边界的交集。
   122.2E 已是浙江/福建近海，再往东即出海，按合规要求不再扩展。
2. 按目标到 R2 的平均距离分 5 带，每带取若干条最长/跨度最大的航迹，
   使场景内各距离带都有被探测目标，覆盖 1000-1550 km。
3. 时间窗可选，默认 UTC 00-02（凌晨低密度，控制计算量）。

输出与 build_adsb_scene.py 完全一致，load_adsb_scenario.m 可直接读取。
"""
import csv, sys, json, math, datetime, os
from collections import defaultdict

SRC = r'D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn/fr24_adsb_20261024.csv'
OUTDIR = r'D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn/data/adsb'
DT = 30.0
MAX_GAP = 120.0
MIN_FRAMES = 10

R1 = (104.0, 30.7); R2 = (106.5, 29.6)
BC = 93.0; HB = 22.5
RMIN, RMAX = 1000.0, 2000.0

# 双站共视区与大陆边界的交集
LON0, LON1 = 116.0, 122.2
LAT0, LAT1 = 26.5, 34.0

# 距离分层（到 R2，km）。上界 1560 是实测可用极限：1500-1600 仅 139 段，1600+ 为 0
BANDS = [(1000, 1100), (1100, 1200), (1200, 1300), (1300, 1400), (1400, 1560)]


def hav(lon1, lat1, lon2, lat2):
    R = 6371.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1); dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R * math.asin(math.sqrt(a))


def brg(lon1, lat1, lon2, lat2):
    p1, p2 = math.radians(lat1), math.radians(lat2); dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return (math.degrees(math.atan2(y, x)) + 360) % 360


def in_cov(lon, lat):
    d1 = hav(*R1, lon, lat); d2 = hav(*R2, lon, lat)
    a1 = brg(*R1, lon, lat); a2 = brg(*R2, lon, lat)

    def ok(d, a):
        dd = abs(a - BC); dd = 360 - dd if dd > 180 else dd
        return RMIN <= d <= RMAX and dd <= HB
    return ok(d1, a1), ok(d2, a2), d1, d2


def parse(ts):
    return datetime.datetime.strptime(ts[:19], '%Y-%m-%d %H:%M:%S')


def read_region(t0, t1):
    tracks = defaultdict(list)
    n = 0
    with open(SRC, newline='', encoding='utf-8', errors='ignore') as fh:
        rd = csv.reader(fh)
        for row in rd:
            n += 1
            if n % 2000000 == 0:
                print(f'    ...{n/1e6:.1f}M', file=sys.stderr, flush=True)
            if len(row) < 11:
                continue
            try:
                lat = float(row[1]); lon = float(row[2]); hdg = float(row[3])
                alt = float(row[4]); spd = float(row[5]); ts = row[10]
            except (ValueError, IndexError):
                continue
            if not (LON0 <= lon <= LON1 and LAT0 <= lat <= LAT1):
                continue
            try:
                t = parse(ts)
            except ValueError:
                continue
            if not (t0 <= t <= t1):
                continue
            o1, o2, d1, d2 = in_cov(lon, lat)
            if not (o1 and o2):
                continue
            tracks[row[0]].append((t, lat, lon, alt, spd, hdg, row[16] if len(row) > 16 else '',
                                   d1, d2))
    for k in tracks:
        tracks[k].sort(key=lambda x: x[0])
    return tracks


def split_segments(pts):
    segs = []
    cur = []; prev = None
    for p in pts:
        if prev is not None and (p[0] - prev).total_seconds() > MAX_GAP:
            if len(cur) >= MIN_FRAMES:
                segs.append(cur)
            cur = []
        cur.append(p); prev = p[0]
    if len(cur) >= MIN_FRAMES:
        segs.append(cur)
    return segs


def resample(seg, t0, n_frames):
    """重采样到 t0 + k*DT 网格；只保留落在该段覆盖时间内的帧"""
    out = []
    for k in range(n_frames):
        tg = t0 + datetime.timedelta(seconds=k * DT)
        if tg < seg[0][0] or tg > seg[-1][0]:
            continue
        lo = hi = None
        for i in range(len(seg) - 1):
            if seg[i][0] <= tg <= seg[i + 1][0]:
                lo, hi = seg[i], seg[i + 1]
                break
        if lo is None:
            continue
        span = (hi[0] - lo[0]).total_seconds()
        if span <= 0 or span > MAX_GAP:
            continue
        w = (tg - lo[0]).total_seconds() / span
        lat = lo[1] + w * (hi[1] - lo[1]); lon = lo[2] + w * (hi[2] - lo[2])
        alt = lo[3] + w * (hi[3] - lo[3]); spd = lo[4] + w * (hi[4] - lo[4])
        d = ((hi[5] - lo[5] + 540) % 360) - 180
        hdg = (lo[5] + w * d) % 360
        out.append((k, k * DT, lat, lon, alt, spd, hdg, lo[6]))
    return out if len(out) >= MIN_FRAMES else None


def build(scene_id, h0, h1, per_band=3, min_span=80.0):
    t0 = datetime.datetime(2026, 10, 24, h0, 0, 0)
    t1 = datetime.datetime(2026, 10, 24, h1, 0, 0)
    n_frames = int((t1 - t0).total_seconds() / DT)
    print(f'\n[{scene_id}] {LON0}-{LON1}E x {LAT0}-{LAT1}N, UTC {h0:02d}-{h1:02d}, {n_frames} 帧')

    tracks = read_region(t0, t1)
    print(f'  共视区内航迹 {len(tracks)} 架')

    # 收集所有段及其距离统计
    cand = []
    for icao, pts in tracks.items():
        segs = split_segments(pts)
        for si, seg in enumerate(segs, 1):
            rows = resample(seg, t0, n_frames)
            if rows is None:
                continue
            spd_m = sum(r[5] for r in rows) / len(rows)
            if not (120 <= spd_m <= 620):
                continue
            # 原始点元组为 (t, lat, lon, alt, spd, hdg, cs, d1, d2)，d2 在下标 8
            d2s = [p[8] for p in seg]
            d2_mean = sum(d2s) / len(d2s)
            span = max(d2s) - min(d2s)
            tag = f'{icao}' if len(segs) == 1 else f'{icao}_s{si}'
            cand.append({'tag': tag, 'rows': rows, 'd2_mean': d2_mean,
                         'span': span, 'nf': len(rows), 'spd': spd_m,
                         'cs': rows[0][7]})

    # 按距离带分层，每带取跨度最大（同跨度取帧数多）的 per_band 条
    chosen = []
    print(f'  {"带(km)":<12}{"可用":>5}{"入选":>5}{"入选跨度":>10}{"帧数":>6}')
    for lo, hi in BANDS:
        pool = [c for c in cand if lo <= c['d2_mean'] < hi and c['span'] >= min_span]
        pool.sort(key=lambda c: (-c['span'], -c['nf']))
        pick = pool[:per_band]
        chosen.extend(pick)
        sp = '/'.join(f"{c['span']:.0f}" for c in pick) if pick else '-'
        nf = '/'.join(str(c['nf']) for c in pick) if pick else '-'
        print(f'  {lo}-{hi:<7}{len(pool):>5}{len(pick):>5}{sp:>10}{nf:>6}')

    if not chosen:
        print('ERROR: 没有入选航迹'); sys.exit(1)

    rows_all = []
    meta_targets = []
    for c in chosen:
        for r in c['rows']:
            rows_all.append((c['tag'], 0, r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7]))
        meta_targets.append({'id': c['tag'], 'frames': c['nf'],
                             'f0': c['rows'][0][0], 'f1': c['rows'][-1][0],
                             'spd_kt': round(c['spd'], 1), 'callsign': c['cs'],
                             'd2_mean_km': round(c['d2_mean'], 1),
                             'd2_span_km': round(c['span'], 1)})

    # 重新编号 seg（同一 tag 只有一个段）
    rows_all = [(r[0], 1) + r[2:] for r in rows_all]

    act = defaultdict(int)
    for r in rows_all:
        act[r[2]] += 1
    actv = [act.get(k, 0) for k in range(n_frames)]

    os.makedirs(OUTDIR, exist_ok=True)
    out_csv = os.path.join(OUTDIR, f'{scene_id}.csv')
    with open(out_csv, 'w', newline='', encoding='utf-8') as fh:
        w = csv.writer(fh)
        w.writerow(['icao', 'seg', 'frame', 't_sec', 'lat', 'lon',
                    'alt_ft', 'spd_kt', 'hdg_deg', 'callsign'])
        for r in rows_all:
            w.writerow(r)

    d2_all = sorted(t['d2_mean_km'] for t in meta_targets)
    meta = {'scene_id': scene_id, 'source': os.path.basename(SRC),
            'region': [LON0, LON1, LAT0, LAT1], 'alt_min_ft': 0,
            'utc_window': [f'{h0:02d}:00', f'{h1:02d}:00'],
            'dt_sec': DT, 'n_frames': n_frames,
            'n_targets': len(meta_targets), 'n_rows': len(rows_all),
            'active_mean': round(sum(actv) / n_frames, 2), 'active_max': max(actv),
            'design': 'distance-stratified',
            'd2_band_km': [round(d2_all[0], 0), round(d2_all[-1], 0)],
            'targets': meta_targets}
    with open(os.path.join(OUTDIR, f'{scene_id}_meta.json'), 'w', encoding='utf-8') as fh:
        json.dump(meta, fh, ensure_ascii=False, indent=2)

    print(f'  目标 {len(meta_targets)} 个, 行数 {len(rows_all)}, '
          f'活跃均值 {meta["active_mean"]}, 最大 {meta["active_max"]}')
    print(f'  R2 距离带覆盖 {d2_all[0]:.0f} - {d2_all[-1]:.0f} km')
    print(f'  -> {out_csv}')
    return meta


if __name__ == '__main__':
    # 主场景：凌晨低密度，5 带 x 3 条
    build('adsb_span', 0, 2, per_band=3)
    # 备用：略高密度，5 带 x 4 条（用于密度鲁棒性对照）
    build('adsb_span_d', 2, 4, per_band=4)
