"""
ADS-B 场景构建器
================
从 fr24_adsb_20261024.csv 提取指定区域/时段/高度层的真实航迹，
重采样到严格 30 s 网格，断点分割，输出 MATLAB 可直接读取的 CSV。

输出列：icao, seg, frame, t_sec, lat, lon, alt_ft, spd_kt, hdg_deg, callsign
"""
import csv, sys, json, math, datetime, os
from collections import defaultdict

SRC = 'D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn/fr24_adsb_20261024.csv'
OUTDIR = 'D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn/data/adsb'
DT = 30.0
MAX_GAP = 120.0     # 相邻实测点间隔超过此值则断开
MIN_SEG_FRAMES = 10 # 每段至少 10 帧（5 分钟）

def parse(ts):
    return datetime.datetime.strptime(ts[:19], '%Y-%m-%d %H:%M:%S')

def read_region(lon0, lon1, lat0, lat1, alt_min, t0, t1):
    """流式读取，返回 {icao: [(t, lat, lon, alt, spd, hdg, cs)]}"""
    tracks = defaultdict(list)
    n = 0
    with open(SRC, newline='', encoding='utf-8', errors='ignore') as fh:
        r = csv.reader(fh)
        for row in r:
            n += 1
            if n % 3000000 == 0:
                print(f'    ...{n}', file=sys.stderr, flush=True)
            try:
                lat = float(row[1]); lon = float(row[2])
                alt = float(row[4]); spd = float(row[5]); hdg = float(row[3])
                ts = row[10]; icao = row[0]; cs = row[16]
            except (ValueError, IndexError):
                continue
            if not (lon0 <= lon <= lon1 and lat0 <= lat <= lat1):
                continue
            if alt < alt_min:
                continue
            t = parse(ts)
            if not (t0 <= t <= t1):
                continue
            tracks[icao].append((t, lat, lon, alt, spd, hdg, cs))
    for k in tracks:
        tracks[k].sort(key=lambda x: x[0])
    return tracks

def resample_segments(pts, t0, n_frames):
    """把一条实测航迹重采样到 t0 + k*DT 网格，按 MAX_GAP 断开"""
    segs = []
    cur = []
    prev_t = None
    for p in pts:
        if prev_t is not None and (p[0]-prev_t).total_seconds() > MAX_GAP:
            if cur: segs.append(cur)
            cur = []
        cur.append(p)
        prev_t = p[0]
    if cur: segs.append(cur)

    out = []
    for seg in segs:
        rows = []
        for k in range(n_frames):
            tg = t0 + datetime.timedelta(seconds=k*DT)
            if tg < seg[0][0] or tg > seg[-1][0]:
                continue
            # 找夹住 tg 的两个点
            lo = None; hi = None
            for i in range(len(seg)-1):
                if seg[i][0] <= tg <= seg[i+1][0]:
                    lo, hi = seg[i], seg[i+1]; break
            if lo is None:
                continue
            span = (hi[0]-lo[0]).total_seconds()
            if span <= 0:
                continue
            if span > MAX_GAP:      # 插值跨度太大，视为缺失
                continue
            w = (tg-lo[0]).total_seconds()/span
            lat = lo[1]+w*(hi[1]-lo[1]); lon = lo[2]+w*(hi[2]-lo[2])
            alt = lo[3]+w*(hi[3]-lo[3]); spd = lo[4]+w*(hi[4]-lo[4])
            # 航向圆周插值
            d = ((hi[5]-lo[5]+540) % 360) - 180
            hdg = (lo[5]+w*d) % 360
            rows.append((k, k*DT, lat, lon, alt, spd, hdg, lo[6]))
        if len(rows) >= MIN_SEG_FRAMES:
            out.append(rows)
    return out

def build(scene_id, lon0, lon1, lat0, lat1, alt_min, h0, h1, max_targets=None):
    t0 = datetime.datetime(2026, 10, 24, h0, 0, 0)
    t1 = datetime.datetime(2026, 10, 24, h1, 0, 0)
    n_frames = int((t1-t0).total_seconds()/DT)
    print(f'\n[{scene_id}] {lon0}-{lon1}E x {lat0}-{lat1}N, alt>={alt_min}ft, UTC {h0:02d}-{h1:02d}, {n_frames} 帧')
    tracks = read_region(lon0, lon1, lat0, lat1, alt_min, t0, t1)
    print(f'  命中 {len(tracks)} 架')

    rows = []
    meta_targets = []
    for icao, pts in tracks.items():
        segs = resample_segments(pts, t0, n_frames)
        for si, seg in enumerate(segs, 1):
            # 每段平均航向与速度，用于筛选合理性
            spd_m = sum(r[5] for r in seg)/len(seg)
            if not (120 <= spd_m <= 620):     # 剔除异常地速
                continue
            tag = f'{icao}' if len(segs) == 1 else f'{icao}_s{si}'
            for r in seg:
                rows.append((tag, si, r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7]))
            meta_targets.append({'id': tag, 'frames': len(seg),
                                 'f0': seg[0][0], 'f1': seg[-1][0],
                                 'spd_kt': round(spd_m, 1),
                                 'callsign': seg[0][7]})

    if max_targets and len(meta_targets) > max_targets:
        meta_targets.sort(key=lambda x: -x['frames'])
        keep = {m['id'] for m in meta_targets[:max_targets]}
        rows = [r for r in rows if r[0] in keep]
        meta_targets = [m for m in meta_targets if m['id'] in keep]
        print(f'  截断到 {max_targets} 个最长目标')

    # 活跃目标数
    act = defaultdict(int)
    for r in rows: act[r[2]] += 1
    actv = [act.get(k, 0) for k in range(n_frames)]

    os.makedirs(OUTDIR, exist_ok=True)
    out_csv = os.path.join(OUTDIR, f'{scene_id}.csv')
    with open(out_csv, 'w', newline='', encoding='utf-8') as fh:
        w = csv.writer(fh)
        w.writerow(['icao','seg','frame','t_sec','lat','lon','alt_ft','spd_kt','hdg_deg','callsign'])
        for r in rows: w.writerow(r)

    meta = {'scene_id': scene_id, 'source': os.path.basename(SRC),
            'region': [lon0, lon1, lat0, lat1], 'alt_min_ft': alt_min,
            'utc_window': [f'{h0:02d}:00', f'{h1:02d}:00'],
            'dt_sec': DT, 'n_frames': n_frames,
            'n_targets': len(meta_targets), 'n_rows': len(rows),
            'active_mean': round(sum(actv)/n_frames, 2), 'active_max': max(actv),
            'targets': meta_targets}
    with open(os.path.join(OUTDIR, f'{scene_id}_meta.json'), 'w', encoding='utf-8') as fh:
        json.dump(meta, fh, ensure_ascii=False, indent=2)

    print(f'  目标 {len(meta_targets)} 个, 行数 {len(rows)}, '
          f'活跃均值 {meta["active_mean"]}, 最大 {meta["active_max"]}')
    print(f'  -> {out_csv}')
    return meta

if __name__ == '__main__':
    REG = (117.0, 120.5, 29.0, 32.5)
    # 主实验：凌晨低密度（北京 08-10 时）
    build('adsb_0002', *REG, 20000, 0, 2)
    # 中密度
    build('adsb_0608', *REG, 20000, 6, 8)
    # 压力测试：晚高峰
    build('adsb_0911', *REG, 20000, 9, 11)
