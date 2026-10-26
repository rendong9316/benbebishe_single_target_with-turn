# -*- coding: utf-8 -*-
"""聚合 m_sweep.csv：按 M 求平均误配/漏配/P/R/F1，并对比 Sequential 基线。"""
import csv
import os
import statistics as st
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))

def read(p):
    for enc in ("gbk", "utf-8-sig", "utf-8"):
        try:
            with open(p, encoding=enc, newline="") as f:
                return list(csv.DictReader(f))
        except UnicodeDecodeError:
            continue
    return []

rows = read(os.path.join(HERE, "m_sweep.csv"))
by = defaultdict(list)
for r in rows:
    by[int(r["M"])].append(r)

print("## 按 M 聚合（密集 5 组：adsb_0608×3 + adsb_0911×2）\n")
print("| M | 平均误配 | 平均漏配 | 平均 P | 平均 R | F1 |")
print("|---|---|---|---|---|---|")
for M in sorted(by):
    rs = by[M]
    w = st.mean(float(r["n_wrong"]) for r in rs)
    m = st.mean(float(r["n_missed"]) for r in rs)
    P = st.mean(float(r["precision"]) for r in rs)
    R = st.mean(float(r["recall"]) for r in rs)
    f1 = 2 * P * R / (P + R) if P + R else 0
    print("| %d | %.2f | %.2f | %.4f | %.4f | **%.4f** |" % (M, w, m, P, R, f1))

# Sequential 基线（来自 pairing_dense.csv）
dense = read(os.path.join(HERE, "pairing_dense.csv"))
seq = [r for r in dense if r["method"] == "Sequential"]
if seq:
    w = st.mean(float(r["n_wrong"]) for r in seq)
    m = st.mean(float(r["n_missed"]) for r in seq)
    P = st.mean(float(r["precision"]) for r in seq)
    R = st.mean(float(r["recall"]) for r in seq)
    f1 = 2 * P * R / (P + R)
    print("\n**Sequential 基线**（M 无关）：误配 %.2f | 漏配 %.2f | P %.4f | R %.4f | F1 **%.4f**" % (w, m, P, R, f1))

# NN 基线
nn = [r for r in dense if r["method"] == "NN"]
if nn:
    w = st.mean(float(r["n_wrong"]) for r in nn)
    m = st.mean(float(r["n_missed"]) for r in nn)
    P = st.mean(float(r["precision"]) for r in nn)
    R = st.mean(float(r["recall"]) for r in nn)
    f1 = 2 * P * R / (P + R)
    print("**NN 基线**：误配 %.2f | 漏配 %.2f | P %.4f | R %.4f | F1 **%.4f**" % (w, m, P, R, f1))
