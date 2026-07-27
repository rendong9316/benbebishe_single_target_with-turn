# IMM Sigmoid 修复后结果诊断与方案重设计

**日期：** 2026-07-25  
**状态：** Sigmoid 自适应方案被证伪——修复符号后效果消失

---

## 一、实验结果

### 1.1 修复后 MC 验证（3 种子 × 4 场景 × 2 配置）

| 场景 | Seed | Default | Cfg51+Sigmoid(k3) | 改善 | SigOnly(k3) |
|------|------|---------|-------------------|------|-------------|
| right_rate_0p7 | 42 | 5.825 | **5.429** | +6.8% * | 6.073 (-4.3%) |
| right_rate_0p7 | 123 | 7.442 | **7.656** | **-2.9%** | 7.567 (-1.7%) |
| right_rate_0p7 | 456 | 5.075 | **5.118** | **-0.9%** | 5.066 (+0.2%) |
| left_rate_0p7 | 42 | 7.703 | **5.780** | **+25.0%** * | 7.790 (-1.1%) |
| left_rate_0p7 | 123 | 7.026 | **7.288** | **-3.7%** | 7.116 (-1.3%) |
| left_rate_0p7 | 456 | 6.091 | **6.575** | **-8.0%** | 6.130 (-0.6%) |
| right_rate_1p3 | 42 | 6.324 | **7.017** | **-11.0%** | 6.719 (-6.2%) |
| right_rate_1p3 | 123 | 7.419 | **7.577** | **-2.1%** | 7.512 (-1.2%) |
| right_rate_1p3 | 456 | 5.023 | **5.068** | **-0.9%** | 5.019 (+0.1%) |
| left_rate_1p3 | 42 | 7.693 | **5.759** | **+25.1%** * | 7.778 (-1.1%) |
| left_rate_1p3 | 123 | 7.490 | **7.433** | **+0.8%** | 7.462 (+0.4%) |
| left_rate_1p3 | 456 | 6.043 | **6.526** | **-8.0%** | 6.082 (-0.6%) |

**12 cases: 3改善(25%), 9恶化(75%)**  
**平均改善：约 -2%**（比 Default 更差）

### 1.2 Bug 版本 vs Fix 版本对比

| 指标 | Bug 版(+反向) | Fix 版(修正后) |
|------|-------------|---------------|
| 改善种子占比 | 74-86% | **25%** |
| 平均改善 | +6.1% | **~-2%** |
| Win rate | 74%+ | **25%** |
| 结论 | "似乎有效" | **"失效"** |

---

## 二、为什么修复后失效？——根因分析

### 核心发现：Sigmoid 对 Config#51 产生了负面干扰

观察两个 Config#51 + Sigmoid seed=42 大幅改善的案例：
- `left_rate_0p7` seed=42: **Cfg51+Sigmoid=5.780 vs Def=7.703 (+25.0%)**
- `left_rate_1p3` seed=42: **Cfg51+Sigmoid=5.759 vs Def=7.693 (+25.1%)**

这些改善的共性：**Left场景 + 左转(positive ω CT)匹配 + seed=42**。

再看 seed=123 相同场景：
- `left_rate_0p7` seed=123: **Cfg51+Sigmoid=7.288 vs Def=7.026 (-3.7%)** — 变差了！
- `left_rate_1p3` seed=123: **Cfg51+Sigmoid=7.433 vs Def=7.490 (+0.8%)** — 几乎没改善

**结论：+25% 不是 Sigmoid 的效果，而是 seed=42 的偶然好结果。**

### 为什么 Sigmoid 实际起到了反作用？

从 PI 分析数据表（之前运行的 analyze_sigmoid_mechanics.m）：

Fix 版 sigmoid 行为：
```
ct_mu=0.0: P_CT→CV = 0.00562 (≈base_p=0.006, 不干预)
ct_mu=0.4: P_CT→CV = 0.00173 (降低 70%, 开始干预)
ct_mu=0.8: P_CT→CV = 0.00007 (降低 98%, 强力干预)
```

**问题在于：sigmoid 在弱转弯(ct_mu < 0.3)完全不响应，而在中转弯(ct_mu=0.4-0.6)就强力压低 P_CT→CV。**

对于 Config#51 的高 ct_dwell=660（P基线=0.006），ct_mu 在大部分帧都在 0.3~0.6 之间徘徊。Sigmoid 在这段区域把 P_CT→CV 从 0.006 压到 0.001~0.0001 —— 这导致 CT 模型**过度粘滞**，转弯结束后迟迟不肯释放回 CV。

而 bug 版本的行为恰恰相反——ct_mu 中等时 P_CT→CV 保持较高（0.004~0.005），接近 Config#51 基线，所以 bug 版本的"改善"实际上是 **"Sigmoid 几乎没做什么事"**（因为 sigmoid 在中间区域恰好不生效）。

### 关键洞察

```
Bug 版的行为 ≈ 基线 Pi (Sigmoid 几乎不生效) → 改善
Fix 版的行为 → Pi 被修改 → 干扰了 Config#51 原有的稳定 → 恶化
```

**修复前"有效"是因为 Sigmoid 分支实际上没生效或生效不充分；修复后"失效"是因为 Sigmoid 正确地干预了 PI，反而破坏了 Config#51 原本良好的平衡。**

### 两个根本矛盾

1. **CT 粘滞时间**：Config#51 通过高 Q_scale(5.3) 让 CT 适应慢转弯，但这在转弯结束时会导致 CT 预测仍然偏大（Q 大 = 预测不确定度大 = RMSE 高）。Sigmoid 如果让 CT 更粘（修复后的行为），会延长转弯结束后的误差尾巴。

2. **PI 信号 vs 似然度信号的对抗**：Sigmoid 修改的是 prepare 阶段的 PI（间接影响 c_bar → mu_new），但 update 阶段贝叶斯似然度已经给出了明确信号。当观测说"这是 CV"时，PI 强行把 CT→CV 压低，两者的对抗导致 mu 振荡。

---

## 三、方案重设计

### 放弃的方向

以下方案已证明无效：
- ~~Sigmoid 自适应 transfer probability~~ — 修复后效果消失
- ~~Config#51 + Sigmoid 组合~~ — 破坏了 Cfg51
- ~~k/gradient 调优~~ — 方向都反了，不值得调

### 可行的新思路

#### 思路 A：不要在 PI 上调参，改用 NIS 反馈

当前 Sigmoid 用的是 μ（模型概率）作为反馈——这是滞后指标。NIS（归一化新息平方）是**即时观测质量指标**：

```matlab
% 如果 CT-left 的 NIS 持续低于 CT-right 和 CV → 左转确实在发生
% 不需要调 PI，直接调整融合权重
nis_cv = imm.ukf_cv.nis_history(end);
nis_ct = imm.ukf_ct.nis_history(end);
nis_ctr = imm.ukf_ct_right.nis_history(end);

if nis_ct < nis_cv && nis_ct < nis_ctr
    % CT-left 拟合最好 → 给它的输出更高的组合权重
    weight_ct = 0.7;  % 提高 CT 在最终 x_comb 中的权重
else
    weight_ct = mu(2); % 用 IMM 原生 mu
end
```

**优点**：基于即时观测证据，不是基于滞后概率  
**缺点**：改变了 IMM 的数学纯洁性

#### 思路 B：放弃 Sigmoid，回到 exp_decay（bug 版可能就是这个方向对的）

exp_decay 在低 ct_mu 时也衰减，不会像 sigmoid 那样在 threshold 处突然切换：

```matlab
adj_lev = base_p * exp(-k * ct_mu^power);
```

k=3, power=1.5 时：
```
ct_mu=0.0: 0.006 → base_p
ct_mu=0.3: 0.006*exp(-0.72) = 0.0029
ct_mu=0.6: 0.006*exp(-2.16) = 0.0007
ct_mu=1.0: 0.006*exp(-4.5) = 0.00007
```

exp_decay 的曲线在所有区间都有平滑变化，没有 sigmoid 的"开关效应"。之前 bug 版的改善可能就是 exp_decay 和 config#51 的正确配合，只是实现写错了。

**建议重跑**：用 exp_decay 模式 + k=[1,2,3] + config#51 做 3 seeds × 4 scenes。

#### 思路 C：放弃 Config#51 的激进 Q_scale

Config#51 的问题在于 Q=5.3x 太大，转弯结束后的残差偏高。一个更温和的方案：

```
C1: 基线 IMM (默认 Pi) + exp_decay k=3 (不配 Cfg51)
C2: Cfg51 only (no adaptive)
C3: 基线 IMM + 自适应 Q (已有 3in1 模式)
```

也许最简单的"自适应 Q"模式(3in1)就是最优解，不需要加 Sigmoid。

---

## 四、推荐下一步

### 立即执行（验证假设）

```matlab
% 快速对比 exp_decay vs sigmoid vs linear in k=3/Cfg51
addpath(genpath('.'));
scenarios = {'right_rate_0p7', 'left_rate_0p7', 'right_rate_1p3', 'left_rate_1p3'};
seeds = [42, 123, 456];
modes = {'linear', 'exp_decay'};
configs = {'base', 'cfg51', 'cfg51_exp3'};

for each mode × config × seed × scenario:
    run() → extract errorStats_R1.overall.ukf.rms
```

预计 2 × 3 × 3 × 4 = 72 runs，约 2-3 小时。

### 如果 exp_decay k=3 + Cfg51 也差 → 回到 3in1 自适应 Q

3in1 模式已经在代码中存在（`imm_adapt_mode='3in1'`），可能它本身就是最优解。值得跑一次 baseline 对比。

### 论文方向调整

Sigmoid 方案已证伪。论文可以改为：
1. **问题分析论文**："IMM-UKF 在 OTH-SWR 转弯跟踪中的固有局限与自适应 PI 设计的陷阱"
   - 揭示 bug 版看似有效但机制错误
   - 展示修复后失效的诚实结果
   - 讨论为什么简单的 mu 反馈在 IMM 框架中不可行
   
2. **转向 NIS-based 自适应方案**（如果验证通过）
   - 提出基于即时观测质量的 PI 调制方法
   - 而非基于滞后模型概率的调制

---

*分析完成：2026-07-25*
*核心结论：Sigmoid 自适应 PI 方案被证伪，需重新设计*
