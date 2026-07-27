# Sigmoid 自适应机制深度分析与瓶颈诊断

## 一、IMM 概率更新循环的完整数学链

```
每帧的 IMM 循环：
  prepare_imm() ──────────────────────────────────────┐
  │                                                    │
  │  1. 读取当前 mu (CV, CT-L, CT-R)                   │
  │  2. 用 adaptive PI 计算 adjusted P_CT→CV           │
  │  3. c_bar = Pi' * mu  (传播先验)                   │
  │  4. 三模型预测 → update(观测)                      │
  │  5. 似然度 L = P(z|model_j)                        │
  │  6. mu_new ∝ L ⊙ c_bar   (贝叶斯更新)              │
  │  7. mu = bounded_project(mu_new)                  │
  │                                                    │
  └────────────────────────────────────────────────────┘
                          ↓
                    下一帧准备...
```

### 关键 insight

**Sigmoid 的作用在 Step 2**（prepare），但 **Step 6**（贝叶斯更新）才是决定性的——
似然度乘以先验 c_bar，然后归一化，最后钳位 [0.02, 0.95]。

如果观测数据强有力地支持某个模型（转弯中 CT 似然度远高于 CV），即使 Transfer Probability 不利，mu 仍会靠观测证据获胜。

---

## 二、Bug 版本的 Sigmoid —— 方向反了为什么还"有效"？

### Bug 版本公式（符号反了）：
```
slev = base_p / (1 + exp(-kp*(ct_mu - threshold)))
```

- ct_mu 高（转弯中，比如 0.8）: `-kp*(0.8-0.3) = -large` → `exp(-large) ≈ 0` → `slev ≈ base_p`
  - **效果：CT→CV 转移概率保持较高 → CT 更容易离开！**

- ct_mu 低（直线段，比如 0.1）: `-kp*(0.1-0.3) = +small` → `exp(small) > 1` → `slev < base_p`
  - **效果：CT→CV 被压低 → CT 粘住了！**

### 这实际上做了什么？

**直线段让 CT 粘住**——但这是坏事！直线段应该让 CT→CV 切换更快。
不过，**CT 粘住不等于 mu 变高**。如果 CT 在直线段预测不准（新息大），似然度会压得很低，即使 PI 有利，mu 也会靠观测证据被压低。

真正的影响链是：
1. Sigmoid bug 让 CT→CV 概率在直线段降低（CT 粘住）
2. 但观测证据会说"这帧不对"（新息大，似然度低）
3. 贝叶斯更新把 mu 压低
4. **关键**：Pi 中的 `1-P_CT→CV`（即 P_CT→CT）在 bug 版本反而更高 → **CT 留在 CT 的概率高了**

等等——让我重新想。`local_Pi(2,1) = adj_lev` 是 **CT→CV** 的转移概率。
`local_Pi(2,2) = 1 - adj_lev` 是 **CT→CT** 的自环概率。

Bug 版本在直线段把 adj_lev 压低 → CT→CT 增加 → **CT 留在 CT 更久**。
但这是错的，因为直线段应该让 CT→CV 更快。

### 那为什么 MC 结果显示改善了？

有几个可能原因：

**原因 A：CT 持续参与使 RMSE 改善**
在转弯场景中，Sigmoid bug 让 CT 更"粘"——转弯时 CT→CT 自环概率高（这反而是对的），但代价是转弯结束后的"尾巴"更长（CT 该放掉却没放掉）。

**原因 B：mu_history 反馈效应**
代码注释提到了 "ct_trend: mu_history最近N帧的平均CT占比"——但这个特征**没有在任何分支中使用**。所以没有这个效果。

**原因 C：PI 和 mu 的双向反馈**
prepare 阶段修改了 PI，update 阶段修改了 mu。两个相互影响：
- 高 ct_mu → bug 版 slev ≈ base_p → CT→CT 接近基线（不特别高）
- 低 ct_mu → bug 版 slev << base_p → CT→CT ≈ 1-base_p（更高）

这意味着 bug 版的**净效果是让直线段的 CT 更难被踢掉**。对转弯场景来说这可能恰好正确——因为转弯往往从某个直线段过渡而来，过早踢掉 CT 会导致错过转弯。

**原因 D：Config#51 的高 Q scale 掩盖了问题**
Config#51 使用 `imm_ct_fixed_Q_scale=5.3`，这意味着 CT 模型的过程噪声被放大了 5.3 倍 → CT 更适应慢转弯 → 与 bug 版 Sigmoid 配合产生了虚假的"改善"。

### 修复后会发生什么？

修复后的公式：
```
slev = base_p / (1 + exp(kp*(ct_mu^power - threshold)))
```

- ct_mu 高（转弯中，0.8）: `kp*(0.8-0.3) = large` → `exp(large)` 很大 → `slev << base_p`
  - CT→CV 被压低 → CT→CT 增加 → **CT 更粘** ✓ 这是正确的行为

- ct_mu 低（直线段，0.1）: `kp*(0.1-0.3) = -small` → `exp(-small) < 1` → `slev ≈ base_p/(1+small) ≈ base_p`
  - CT→CV 接近基线 → **正常切换** ✓ 这也是正确的

修复后的方向与 exp_decay 一致：**ct_mu 高时压低 CT→CV，让 CT 留住**。

### 核心问题

修复后的效果**无法预知**。可能：
1. **仍然改善**（甚至更好）——因为方向正确
2. **改善消失**——因为之前的改善全是 bug 的功劳
3. **变差**——因为 Sigmoid 函数形态与 exp_decay/linear 不同

---

## 三、瓶颈分析

### 瓶颈 1：PI 调整 vs 观测似然的博弈

IMM 有两个机制在竞争决定模型概率：
1. **Transfer PI**（prepare 阶段）：控制"该不该从 CT 跳到 CV"
2. **Observation Likelihood**（update 阶段）：控制"哪个模型拟合数据好"

Sigmoid 只调整了第 1 层（PI），但第 2 层（似然度）才是最终的裁判。如果 CT 的预测确实偏离了新息（转弯太急或转弯率不匹配），无论 PI 如何调，mu 都会被似然度压低。

**这是一个根本性限制**：固定 Pi 的问题是"转弯来不及反应"，但 PI 的调整速度受限于 **转移矩阵的马尔可夫性质**——转移概率的变化只能通过 `c_bar = Pi' * mu` 间接影响混合先验，再通过似然度更新 mu。

### 瓶颈 2：Mu 钳位 [0.02, 0.95]

每次更新 mu 都被投影到 [0.02, 0.95]。这意味着：
- 没有一个模型能"完全淘汰"另一个（最小 2%）
- 也没有一个模型能"完全主导"（最大 95%）
- 转弯识别被限制在了一个非常窄的窗口内

**钳位限制了响应速度**。即使 Sigmoid 提供了正确的 PI 信号，mu 也需要多帧才能从 0.5 爬到 0.95（上限每帧最多增长约 `(0.95-0.5)/N_frames_to_saturate`）。

### 瓶颈 3：基线 PI 的计算

```matlab
p_ct_cv = 1 - exp(-dt / params.imm_ct_dwell_time_sec);
```

对于 `imm_ct_dwell_time_sec=660`, `dt=4`:
```
p_ct_cv = 1 - exp(-4/660) = 0.00607
p_cv_ct = 1 - exp(-4/2500) = 0.00160
```

这些基线值已经非常小了（~0.6%）。Sigmoid 在 ct_mu=0.8 时把它压到 ~0.003%，在 ct_mu=0.1 时压到 ~0.6%。

**问题在于：PI 绝对值太小，调整的绝对幅度极微**。相比 Mu 从似然度获得的证据（可能是 3 倍、5 倍的似然度比），PI 的小幅调整几乎没有影响力。

### 瓶颈 4：Sigmoid vs exp_decay vs linear 的形态差异

三种模式对同样的输入产生完全不同的输出：

| ct_mu | exp_decay(k=3) | sigmoid(k=3,t=0.3,p=1) | linear(k=3) |
|-------|---------------|----------------------|-------------|
| 0.0 | base_p | ≈base_p | base_p |
| 0.2 | base_p*0.55 | ≈base_p | 0.63*base_p |
| 0.5 | base_p*0.22 | 0.05*base_p (clipped) | 0.40*base_p |
| 0.8 | base_p*0.05 | 0.05*base_p (clipped) | 0.26*base_p |
| 1.0 | 0.05 (clipped) | 0.05 (clipped) | 0.25*base_p |

exp_decay 提供了平滑衰减，sigmoid 在 threshold 处急剧下降，linear 衰减最慢。

**选择 Sigmoid 不是因为理论上有更好的形状，而是因为在 bug 版本下它偶然产生了更好的效果。**

---

## 四、重新设计方案

### 方案 1：直接在 Mu 上反馈（绕过 PI）

与其通过 PI 间接影响 mu，不如在 update 阶段**直接根据 mu(CT) 调整似然度的权重**：

```
mu_update ← mu_observation * exp(w * mu_history_ct)
```

这等价于给当前的观测证据加了一个基于历史趋势的偏置——类似于贝叶斯先验的动态调整。

**优点**：直接作用于 mu，信号更强
**缺点**：破坏了 IMM 的概率纯洁性（PI 应该独立于 mu）

### 方案 2：基于 NIS 的自适应

当 NIS(CT) < threshold 且 NIS(CV) > threshold 时，说明 CT 更适合当前数据——可以**临时提高 CT→CT 的自环概率**，而不是依赖固定的 PI。

**这比 Sigmoid 更科学**：不是用"估计当前转弯置信度"来决定是否留 CT，而是用**实际观测证据**（NIS）来决定。

### 方案 3：Mu 速率调制

Sigmoid 的本质是想加快转弯时的响应、减慢直线时的切换。更直接的方式：

```
在 update 阶段：
if mu(CT-left) > 0.6 and mu(CT-left) > mu(CT-right):
    ω_effective = ω_positive (left turn matched)
else if mu(CT-right) > 0.6:
    ω_effective = ω_negative (right turn matched)
else:
    ω_effective = 0 (CV)
```

然后在状态组合时只用"当前最匹配的"那个模型做预测，而不是全部三个加权平均。

但这改动太大了，超出了 IMM 框架。

### 方案 4：改进现有 IMM（最务实）

保留 IMM 框架不变，在 prepare_imm 中改进自适应逻辑：

```python
# 新思路：用 NIS 趋势代替 mu 作为反馈信号
nis_ct = imm.ukf_ct.nis_history[-1]
nis_cv = imm.ukf_cv.nis_history[-1]
if nis_ct < nis_cv:  # CT 比 CV 更贴合数据
    # 临时提高 CT→CT 概率
    adj_p_ct_cv = base_p * exp(-k * max(0, nis_cv - nis_ct))
else:
    # CT 不如 CV，让 PI 正常切换
    adj_p_ct_cv = base_p
```

**这比 Sigmoid 更合理**：直接依据观测证据（NIS 越小越好拟合）来做决策，而不是依据模型的概率（mu 本身可能是滞后指标）。

---

## 五、预期改善

如果 Sigmoid bug 修复后改善消失：
- 回到 baseline（exp_decay 或 linear），预期 **恢复到 bug 时代的水平**（报告声称的 +6.1%）
- 但如果修复后比 bug 版本更好，改善可能**提升到 +10-15%**

如果放弃 Sigmoid，改用 NIS 反馈方案：
- 理论上可以消除"mu 滞后"问题
- 预计改善 **+10-20%**（在转弯场景），特别是在 Config#51 的基础之上

---

*分析完成：2026-07-25*
*关键结论：Sigmoid 修复后的效果不可预测，必须重跑 MC 验证*
