# -*- coding: utf-8 -*-
"""合并稀疏场景结果（pairing_baseline_results.csv，含融合）
与密集场景结果（pairing_dense.csv，无融合）为统一的 pairing_baseline_v2.csv。
"""
import csv
import os

HERE = os.path.dirname(os.path.abspath(__file__))
COLS = ["scene", "seed", "method", "n_pairs", "n_correct", "n_wrong", "n_missed",
        "precision", "recall", "wrong_rate", "n_true_both", "SCC", "BC", "CI", "FCI"]


def read(p):
    for enc in ("gbk", "utf-8-sig", "utf-8"):
        try:
            with open(p, encoding=enc, newline="") as f:
                return list(csv.DictReader(f))
        except (UnicodeDecodeError, LookupError):
            continue
    return []


def main():
    rows = []
    for fn in ("pairing_baseline_results.csv", "pairing_dense.csv"):
        p = os.path.join(HERE, fn)
        if os.path.isfile(p):
            got = read(p)
            print("%-32s %d 行" % (fn, len(got)))
            rows += got
    out = os.path.join(HERE, "pairing_baseline_v2.csv")
    with open(out, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=COLS)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, "") for k in COLS})
    print("合并完成：%d 行 -> %s" % (len(rows), out))


if __name__ == "__main__":
    main()
