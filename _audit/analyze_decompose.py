# -*- coding: utf-8 -*-
"""拆解 MEWF 优势来源：读 mewf_decompose.csv 做精确聚合与归因"""
import csv
import os
import statistics as st

HERE = os.path.dirname(os.path.abspath(__file__))


def load(name):
    with open(os.path.join(HERE, name), encoding="utf-8-sig", newline="") as f:
        return list(csv.DictReader(f))


def f(r, k):
    try:
        return float(r[k])
    except (TypeError, ValueError):
        return 0.0


rows = load("mewf_decompose.csv")
by = {}
for r in rows:
    by.setdefault(r["config"], []).append(r)

order = ["Seq_r0.80_greedy", "Seq_r0.50_greedy", "Seq_r0.30_greedy",
         "Seq_r0.80_hung", "Seq_r0.50_hung", "Seq_r0.30_hung",
         "MEWF_S0.40", "MEWF_S0.45"]

print("=" * 88)
print("优势来源拆解（密集 5 组平均，n_true_both 合计 1547）")
print("=" * 88)
print(f"{'配置':<20}{'误配':>7}{'漏配':>7}{'P':>9}{'R':>9}{'F1':>9}{'加权代价':>10}")
print("-" * 88)
agg = {}
for k in order:
    rs = by[k]
    w = st.mean(f(r, "n_wrong") for r in rs)
    m = st.mean(f(r, "n_missed") for r in rs)
    P = st.mean(f(r, "precision") for r in rs)
    R = st.mean(f(r, "recall") for r in rs)
    F1 = 2 * P * R / (P + R) if P + R else 0
    C = 5 * w + m
    agg[k] = dict(w=w, m=m, P=P, R=R, F1=F1, C=C, n=len(rs))
    print(f"{k:<20}{w:>7.2f}{m:>7.2f}{P:>9.4f}{R:>9.4f}{F1:>9.4f}{C:>10.2f}")

print("\n" + "=" * 88)
print("归因：逐项拆解（以 Seq_r0.80_greedy 为基准）")
print("=" * 88)
b = agg["Seq_r0.80_greedy"]
for k, desc in [("Seq_r0.80_hung", "只换指派（贪心→匈牙利），门限不变"),
                ("Seq_r0.50_greedy", "只放松门限（0.80→0.50），指派不变"),
                ("Seq_r0.50_hung", "两者都改"),
                ("MEWF_S0.40", "MEWF 软门限 + 匈牙利")]:
    a = agg[k]
    print(f"{desc:<34} 误配 {b['w']:5.2f}→{a['w']:5.2f} ({a['w']-b['w']:+5.2f})  "
          f"漏配 {b['m']:5.2f}→{a['m']:5.2f} ({a['m']-b['m']:+5.2f})  "
          f"代价 {b['C']:5.2f}→{a['C']:5.2f} ({a['C']-b['C']:+5.2f})")

print("\n" + "=" * 88)
print("关键判据：MEWF 软门限 vs 极限松硬门限（同指派、同漏配水平）")
print("=" * 88)
a = agg["Seq_r0.30_hung"]
c = agg["MEWF_S0.40"]
print(f"Seq_r0.30_hung : 误配 {a['w']:.2f} 漏配 {a['m']:.2f} P={a['P']:.4f} R={a['R']:.4f} 代价 {a['C']:.2f}")
print(f"MEWF_S0.40     : 误配 {c['w']:.2f} 漏配 {c['m']:.2f} P={c['P']:.4f} R={c['R']:.4f} 代价 {c['C']:.2f}")
dP = c["P"] - a["P"]
dR = c["R"] - a["R"]
print(f"差值           : ΔP={dP:+.4f} ΔR={dR:+.4f} Δ误配={c['w']-a['w']:+.2f} Δ代价={c['C']-a['C']:+.2f}")
if dP >= -1e-9 and dR >= -1e-9 and (dP > 1e-9 or dR > 1e-9):
    print("结论：MEWF 在 P、R 上均不劣于「极限松门限+匈牙利」，软门限设计有独立（但有限）的贡献。")
elif abs(dP) < 1e-9 and abs(dR) < 1e-9:
    print("结论：两者等价 → 应诚实表述为「软门限 ≈ 自适应松门限 + 全局指派」，不宜强调特征融合的独立价值。")
else:
    print("结论：MEWF 未占优 → 须在论文中诚实说明软门限的边际贡献有限。")
