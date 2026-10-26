#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
build_calib_set.py — 从 FR24 全天数据中提取"专用标校集"

设计要点：
  1. 时间窗 UTC 12:00-18:00，与评估集（00-02 / 06-08 / 09-11）零重叠，
     避免"用真值标定、再用真值评估"的循环论证。
  2. 空间窗放大到 [105,125]E x [20,40]N，使目标到两站的距离跨度尽量大。
     这是分离电离层虚高 h 与群距离偏差 dR 的必要条件：
     ∂Rg/∂h 随距离衰减，只有跨度大才能把两者拉开（否则 corr(h,dR) → -1）。
  3. 输出仅保留 3 列（ICAO, lat, lon），与 calibrate_bias 的 Var1/Var2/Var3 读取方式一致。
  4. 等间隔抽样，控制文件大小（避免每次标定读 776MB）。
"""
import csv, os, sys

SRC = r"D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn/fr24_adsb_20261024.csv"
DST = r"D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn/data/adsb/calib_utc12_18.csv"

LON_MIN, LON_MAX = 105.0, 125.0
LAT_MIN, LAT_MAX = 20.0, 40.0
HOUR_MIN, HOUR_MAX = 12, 18          # UTC
TARGET_ROWS = 60000

os.makedirs(os.path.dirname(DST), exist_ok=True)

kept = []
total = 0
with open(SRC, newline='', encoding='utf-8', errors='ignore') as fh:
    rd = csv.reader(fh)
    for row in rd:
        total += 1
        if total % 500000 == 0:
            print(f"  扫描 {total/1e6:.1f}M 行，已保留 {len(kept)}", flush=True)
        if len(row) < 11:
            continue
        ts = row[10]
        # 时间戳形如 2026-10-24 13:45:12.345
        try:
            hh = int(ts[11:13])
        except Exception:
            continue
        if hh < HOUR_MIN or hh >= HOUR_MAX:
            continue
        try:
            lat = float(row[1]); lon = float(row[2])
        except Exception:
            continue
        if not (LAT_MIN <= lat <= LAT_MAX and LON_MIN <= lon <= LON_MAX):
            continue
        icao = row[0].strip()
        if not icao:
            continue
        kept.append((icao, lat, lon))

print(f"扫描完成：总行 {total}，命中 {len(kept)}")

if not kept:
    print("ERROR: 没有命中任何点，检查时间窗/空间窗"); sys.exit(1)

step = max(1, len(kept) // TARGET_ROWS)
sub = kept[::step]
print(f"抽样步长 {step}，输出 {len(sub)} 行")

with open(DST, 'w', newline='', encoding='utf-8') as fh:
    wr = csv.writer(fh)
    for r in sub:
        wr.writerow([r[0], f"{r[1]:.5f}", f"{r[2]:.5f}"])

# 统计距离跨度（对两站）
import math
def hav(lon1, lat1, lon2, lat2):
    R = 6371.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1); dl = math.radians(lon2 - lon1)
    a = math.sin(dp/2)**2 + math.cos(p1)*math.cos(p2)*math.sin(dl/2)**2
    return 2*R*math.asin(math.sqrt(a))

R1 = (104.0, 30.7); R2 = (106.5, 29.6)
d1 = sorted(hav(R1[0], R1[1], r[2], r[1]) for r in sub)
d2 = sorted(hav(R2[0], R2[1], r[2], r[1]) for r in sub)
n = len(sub)
print(f"\n到 R1 距离: min={d1[0]:.0f} p5={d1[n//20]:.0f} 中位={d1[n//2]:.0f} p95={d1[n*19//20]:.0f} max={d1[-1]:.0f} km")
print(f"  跨度(p5~p95) = {d1[n*19//20]-d1[n//20]:.0f} km")
print(f"到 R2 距离: min={d2[0]:.0f} p5={d2[n//20]:.0f} 中位={d2[n//2]:.0f} p95={d2[n*19//20]:.0f} max={d2[-1]:.0f} km")
print(f"  跨度(p5~p95) = {d2[n*19//20]-d2[n//20]:.0f} km")
print(f"\n已写出 {DST}")
