# -*- coding: utf-8 -*-
"""
用「融合任务真正关心的记分牌」重新评估各配对方法。

核心论点：
  等权 F1 把「误配」和「漏配」看成等价代价，但在航迹融合里二者代价差 1 个数量级：
    - 漏配：少融合一条航迹，损失是「覆盖/可用性」
    - 误配：把两个不同目标融成一条假航迹，输出的是**错误状态**，会污染后续
      指挥/制导决策，且无法事后自动发现
  因此引入代价加权：C = alpha * 误配数 + beta * 漏配数，alpha >> beta。
"""
import csv
import os
import statistics as st

HERE = os.path.dirname(os.path.abspath(__file__))
DENSE = {"adsb_0608", "adsb_0911"}


def read_csv(name):
    p = os.path.join(HERE, name)
    with open(p, encoding="utf-8-sig", newline="") as f:
        return list(csv.DictReader(f))


def fnum(r, k, default=0.0):
    v = r.get(k, "")
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def weighted_cost(wrong, missed, alpha):
    """误配代价 alpha 倍于漏配"""
    return alpha * wrong + 1.0 * missed


def summarize(rows, label, alpha=5.0):
    """按 method 聚合"""
    by = {}
    for r in rows:
        by.setdefault(r["method"], []).append(r)
    print("\n" + "=" * 92)
    print(f"{label}  （n={len(rows)} 条记录，alpha={alpha:g}：误配代价 = {alpha:g} × 漏配）")
    print("=" * 92)
    hdr = (f"{'方法':<12} {'场景数':>4} {'配对':>6} {'正确':>6} {'误配':>6} {'漏配':>6} "
           f"{'P':>7} {'R':>7} {'误配率%':>8} {'漏配率%':>8} {'F1':>7} {'加权代价':>9}")
    print(hdr)
    print("-" * 92)
    out = {}
    for m, rs in by.items():
        n = len(rs)
        pairs = st.mean(fnum(r, "n_pairs") for r in rs)
        corr = st.mean(fnum(r, "n_correct") for r in rs)
        wrong = st.mean(fnum(r, "n_wrong") for r in rs)
        missed = st.mean(fnum(r, "n_missed") for r in rs)
        ntrue = st.mean(fnum(r, "n_true_both") for r in rs) or 1.0
        P = st.mean(fnum(r, "precision") for r in rs)
        R = st.mean(fnum(r, "recall") for r in rs)
        wr = 100.0 * wrong / ntrue
        mr = 100.0 * missed / ntrue
        F1 = 2 * P * R / (P + R) if (P + R) > 0 else 0.0
        C = weighted_cost(wrong, missed, alpha)
        out[m] = dict(pairs=pairs, corr=corr, wrong=wrong, missed=missed, P=P, R=R,
                      wr=wr, mr=mr, F1=F1, C=C, n=n)
        print(f"{m:<12} {n:>4} {pairs:>6.1f} {corr:>6.1f} {wrong:>6.2f} {missed:>6.2f} "
              f"{P:>7.4f} {R:>7.4f} {wr:>8.2f} {mr:>8.2f} {F1:>7.4f} {C:>9.2f}")
    return out


def dominance(res):
    """P-R 空间支配关系：A 支配 B 当且仅当 P_A>=P_B 且 R_A>=R_B（至少一项严格）"""
    print("\n" + "-" * 92)
    print("P-R 支配关系（谁在「查准」和「查全」两个方向都不劣于对方）")
    print("-" * 92)
    ms = list(res.keys())
    for i, a in enumerate(ms):
        for b in ms[i + 1:]:
            ra, rb = res[a], res[b]
            if ra["P"] >= rb["P"] - 1e-9 and ra["R"] >= rb["R"] - 1e-9:
                tag = "严格" if (ra["P"] > rb["P"] + 1e-9 or ra["R"] > rb["R"] + 1e-9) else "等价"
                print(f"  {a} {tag}支配 {b}   "
                      f"(P {ra['P']:.4f} vs {rb['P']:.4f} ; R {ra['R']:.4f} vs {rb['R']:.4f})")
            elif rb["P"] >= ra["P"] - 1e-9 and rb["R"] >= ra["R"] - 1e-9:
                tag = "严格" if (rb["P"] > ra["P"] + 1e-9 or rb["R"] > ra["R"] + 1e-9) else "等价"
                print(f"  {b} {tag}支配 {a}   "
                      f"(P {rb['P']:.4f} vs {ra['P']:.4f} ; R {rb['R']:.4f} vs {ra['R']:.4f})")
            else:
                print(f"  {a} 与 {b} 互不支配（各有取舍：{a} P={ra['P']:.4f}/R={ra['R']:.4f}，"
                      f"{b} P={rb['P']:.4f}/R={rb['R']:.4f}）")


def main():
    # ---------- 1. 稀疏 vs 密集（baseline_v2，含 NN / Seq / 三判据 / 双门限消融）----------
    v2 = read_csv("pairing_baseline_v2.csv")
    sparse = [r for r in v2 if r["scene"] not in DENSE]
    dense = [r for r in v2 if r["scene"] in DENSE]

    r_sp = summarize(sparse, "【A】稀疏场景（adsb_0002 / adsb_span）")
    r_de = summarize(dense, "【B】密集场景（adsb_0608 / adsb_0911）")
    print("\n>>> 稀疏场景：", "；".join(f"{k} 误配 {v['wrong']:.1f}" for k, v in r_sp.items()))
    print(">>> 密集场景：", "；".join(f"{k} 误配 {v['wrong']:.1f}" for k, v in r_de.items()))
    dominance(r_de)

    # ---------- 2. alpha 敏感性：误配代价倍数扫描 ----------
    print("\n" + "=" * 92)
    print("【C】误配代价倍数 alpha 敏感性（密集场景，加权代价 = alpha×误配 + 漏配，越小越好）")
    print("=" * 92)
    print(f"{'alpha':>6} " + " ".join(f"{m:>12}" for m in r_de.keys()) + "   最优")
    for alpha in (1, 2, 3, 5, 10, 20):
        vals = {m: alpha * r_de[m]["wrong"] + r_de[m]["missed"] for m in r_de}
        best = min(vals, key=vals.get)
        print(f"{alpha:>6} " + " ".join(f"{vals[m]:>12.2f}" for m in vals) + f"   {best}")

    # ---------- 3. M 扫描：三判据自身的最优工作点 ----------
    ms = read_csv("m_sweep.csv")
    print("\n" + "=" * 92)
    print("【D】三判据 M 扫描（密集 5 组，continuous 计数）")
    print("=" * 92)
    byM = {}
    for r in ms:
        byM.setdefault(int(fnum(r, "M")), []).append(r)
    print(f"{'M':>4} {'误配':>7} {'漏配':>7} {'P':>7} {'R':>7} {'F1':>7} {'加权代价':>9}")
    bestM = None
    for M in sorted(byM):
        rs = byM[M]
        wrong = st.mean(fnum(r, "n_wrong") for r in rs)
        missed = st.mean(fnum(r, "n_missed") for r in rs)
        P = st.mean(fnum(r, "precision") for r in rs)
        R = st.mean(fnum(r, "recall") for r in rs)
        F1 = 2 * P * R / (P + R) if (P + R) > 0 else 0
        C = weighted_cost(wrong, missed, 5.0)
        print(f"{M:>4} {wrong:>7.2f} {missed:>7.2f} {P:>7.4f} {R:>7.4f} {F1:>7.4f} {C:>9.2f}")
        if bestM is None or C < bestM[1]:
            bestM = (M, C)
    print(f"  → 加权代价最优 M = {bestM[0]}")

    # ---------- 4. 计数模式（continuous / cumulative / leaky）× M ----------
    print("\n" + "=" * 92)
    print("【E】计数模式对比（密集 5 组）：continuous / cumulative / leaky")
    print("=" * 92)
    gc = read_csv("gate_compare.csv")
    gl = read_csv("gate_leaky.csv")
    # cumulative 也在 gate_compare 里？先看有哪些 mode
    modes = sorted({r["mode"] for r in gc} | {r["mode"] for r in gl})
    print("  已有 mode：", modes)
    allrows = gc + gl
    byMC = {}
    for r in allrows:
        key = (r["mode"], int(fnum(r, "M")))
        byMC.setdefault(key, []).append(r)
    print(f"{'mode':<12} {'M':>3} {'误配':>7} {'漏配':>7} {'P':>7} {'R':>7} {'F1':>7} {'加权代价':>9}")
    for (mode, M) in sorted(byMC):
        rs = byMC[(mode, M)]
        if len(rs) < 3:   # 只保留完整 5 场景的组合
            continue
        wrong = st.mean(fnum(r, "n_wrong") for r in rs)
        missed = st.mean(fnum(r, "n_missed") for r in rs)
        P = st.mean(fnum(r, "precision") for r in rs)
        R = st.mean(fnum(r, "recall") for r in rs)
        F1 = 2 * P * R / (P + R) if (P + R) > 0 else 0
        C = weighted_cost(wrong, missed, 5.0)
        print(f"{mode:<12} {M:>3} {wrong:>7.2f} {missed:>7.2f} {P:>7.4f} {R:>7.4f} "
              f"{F1:>7.4f} {C:>9.2f}")


if __name__ == "__main__":
    main()
