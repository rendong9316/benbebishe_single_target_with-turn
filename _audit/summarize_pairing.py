# -*- coding: utf-8 -*-
"""汇总 pairing_baseline_v2.csv：按方法 / 场景 / 密度分组聚合
用法: python summarize_pairing.py
输出 Markdown 表格，可直接贴论文 4.4 节。
"""
import csv
import os
import statistics as st
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
CSV = os.path.join(HERE, "pairing_baseline_v2.csv")

# 场景密度分组
DENSE = {"adsb_0608", "adsb_0911"}
ORDER = ["NN", "Sequential", "DualGate(三判据)", "DualGate(双门限)"]


def read_csv(path):
    for enc in ("gbk", "utf-8", "utf-8-sig"):
        try:
            with open(path, "r", encoding=enc, newline="") as f:
                rows = list(csv.DictReader(f))
            if rows and any("method" in k for k in rows[0].keys()):
                return rows
        except (UnicodeDecodeError, KeyError):
            continue
    raise RuntimeError("无法解析 CSV: %s" % path)


def fnum(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return float("nan")


def ms(vals):
    vals = [v for v in vals if v == v]
    if not vals:
        return "-"
    if len(vals) == 1:
        return "%.4f" % vals[0]
    return "%.4f±%.4f" % (st.mean(vals), st.pstdev(vals))


def main():
    rows = read_csv(CSV)
    print("读入 %d 行\n" % len(rows))
    has_fusion = "SCC" in rows[0].keys()
    methods = [m for m in ORDER if any(r["method"] == m for r in rows)]
    methods += [m for m in {r["method"] for r in rows} if m not in methods]

    # ---------- 1. 按密度分组：核心结论 ----------
    for label, keep in (("稀疏场景 (adsb_0002 / adsb_span)", lambda r: r["scene"] not in DENSE),
                        ("密集场景 (adsb_0608 / adsb_0911)", lambda r: r["scene"] in DENSE)):
        sub = [r for r in rows if keep(r)]
        if not sub:
            continue
        print("## %s\n" % label)
        print("| 方法 | 配对数 | 正确 | 误配 | 漏配 | 查准率 P | 查全率 R | 误配率 |")
        print("|---|---|---|---|---|---|---|---|")
        for m in methods:
            rs = [r for r in sub if r["method"] == m]
            if not rs:
                continue
            g = lambda k: [fnum(r[k]) for r in rs]
            print("| %s | %s | %s | **%s** | %s | %s | %s | %s |" % (
                m, ms(g("n_pairs")), ms(g("n_correct")), ms(g("n_wrong")),
                ms(g("n_missed")), ms(g("precision")), ms(g("recall")),
                ms(g("wrong_rate"))))
        if has_fusion:
            print("\n| 方法 | SCC | BC | CI | FCI |")
            print("|---|---|---|---|---|")
            for m in methods:
                rs = [r for r in sub if r["method"] == m]
                if not rs:
                    continue
                g = lambda k: [fnum(r[k]) for r in rs]
                print("| %s | %s | %s | %s | %s |" % (
                    m, ms(g("SCC")), ms(g("BC")), ms(g("CI")), ms(g("FCI"))))
        print()

    # ---------- 2. 逐场景误配数（最直观） ----------
    print("## 逐场景误配数\n")
    scenes = sorted({r["scene"] for r in rows})
    print("| 场景 | " + " | ".join(methods) + " |")
    print("|---" * (len(methods) + 1) + "|")
    for sc in scenes:
        cells = []
        for m in methods:
            rs = [r for r in rows if r["scene"] == sc and r["method"] == m]
            cells.append(ms([fnum(r["n_wrong"]) for r in rs]) if rs else "-")
        print("| %s | %s |" % (sc, " | ".join(cells)))
    print()

    # ---------- 3. 逐场景查准率 ----------
    print("## 逐场景查准率 P\n")
    print("| 场景 | " + " | ".join(methods) + " |")
    print("|---" * (len(methods) + 1) + "|")
    for sc in scenes:
        cells = []
        for m in methods:
            rs = [r for r in rows if r["scene"] == sc and r["method"] == m]
            cells.append(ms([fnum(r["precision"]) for r in rs]) if rs else "-")
        print("| %s | %s |" % (sc, " | ".join(cells)))
    print()

    # ---------- 4. 密集场景：加门限对 NN 的误配抑制 ----------
    dense = [r for r in rows if r["scene"] in DENSE]
    if dense:
        print("## 密集场景：加门限对 NN 的误配抑制\n")
        base = [fnum(r["n_wrong"]) for r in dense if r["method"] == "NN"]
        if base and st.mean(base) > 0:
            for m in methods[1:]:
                cur = [fnum(r["n_wrong"]) for r in dense if r["method"] == m]
                if cur:
                    red = (st.mean(base) - st.mean(cur)) / st.mean(base) * 100
                    print("- %s：平均误配 %.2f → %.2f（降低 %.1f%%）" % (
                        m, st.mean(base), st.mean(cur), red))


if __name__ == "__main__":
    main()
