# -*- coding: utf-8 -*-
"""汇总 pairing_baseline_results.csv：按方法聚合配对质量与融合 RMSE（mean±std）
用法: python summarize_pairing.py
输出 Markdown 表格，可直接贴论文 4.4 节。
"""
import csv
import os
import statistics as st
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
CSV = os.path.join(HERE, "pairing_baseline_results.csv")


def read_csv(path):
    # MATLAB 在中文 Windows 默认 GBK 落盘，逐个尝试
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


def main():
    rows = read_csv(CSV)
    print("读入 %d 行" % len(rows))
    cols = rows[0].keys()
    has_fusion = "SCC" in cols

    # 按 method 聚合
    agg = defaultdict(lambda: defaultdict(list))
    for r in rows:
        m = r["method"]
        for k in ("n_pairs", "n_correct", "n_wrong", "n_missed",
                  "precision", "recall", "wrong_rate", "n_true_both"):
            if k in r:
                agg[m][k].append(fnum(r[k]))
        if has_fusion:
            for k in ("SCC", "BC", "CI", "FCI"):
                agg[m][k].append(fnum(r[k]))

    order = ["NN", "Sequential", "DualGate(三判据)", "DualGate(双门限)"]
    methods = [m for m in order if m in agg] + [m for m in agg if m not in order]

    def ms(vals):
        vals = [v for v in vals if v == v]  # 去 nan
        if not vals:
            return "-"
        if len(vals) == 1:
            return "%.4f" % vals[0]
        return "%.4f±%.4f" % (st.mean(vals), st.pstdev(vals))

    print("\n## 配对质量（跨场景均值）\n")
    print("| 方法 | 正确对 | 误配对 | 漏配 | 查准率 P | 查全率 R | 误配率 |")
    print("|---|---|---|---|---|---|---|")
    for m in methods:
        a = agg[m]
        print("| %s | %s | %s | %s | %s | %s | %s |" % (
            m,
            ms(a["n_correct"]), ms(a["n_wrong"]), ms(a["n_missed"]),
            ms(a["precision"]), ms(a["recall"]), ms(a["wrong_rate"])))

    if has_fusion:
        print("\n## 融合 RMSE（km，跨场景均值）\n")
        print("| 方法 | SCC | BC | CI | FCI |")
        print("|---|---|---|---|---|")
        for m in methods:
            a = agg[m]
            print("| %s | %s | %s | %s | %s |" % (
                m, ms(a["SCC"]), ms(a["BC"]), ms(a["CI"]), ms(a["FCI"])))

        # 三判据相对 NN 的改进幅度
        if "NN" in agg and "DualGate(三判据)" in agg:
            print("\n## 三判据 vs NN（SCC RMSE 相对改进）\n")
            for m in ("Sequential", "DualGate(三判据)", "DualGate(双门限)"):
                if m not in agg:
                    continue
                nn = [v for v in agg["NN"]["SCC"] if v == v]
                mm = [v for v in agg[m]["SCC"] if v == v]
                if nn and mm:
                    n_mean, m_mean = st.mean(nn), st.mean(mm)
                    imp = (n_mean - m_mean) / n_mean * 100
                    print("- %s: %.3f → %.3f km (%+.2f%%)" % (m, n_mean, m_mean, imp))


if __name__ == "__main__":
    main()
