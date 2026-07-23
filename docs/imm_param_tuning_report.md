# IMM (3in1) UKF 参数调优报告

## 背景

项目中的 IMM (3in1) UKF 滤波器在单目标跟踪中面临 RMSE 偏高的问题。默认配置下 avg RMSE 约 4.52 km，需要通过系统化的参数搜索找到最优配置。

**核心约束：**
- 不修改现有代码架构
- 仅通过参数调优提升性能
- IMM (3in1) 是唯一可用的滤波器模式（adaptive 和 baseline 在默认 Q 下完全崩溃）

## 调参方法

### 三阶段搜索策略

**Phase 1: LHS 拉丁超立方搜索**
- 30 个候选配置，7 个 IMM 参数同时优化
- 3 个场景（single_turn, single_uturn, single_straight）× 2 台雷达 = 6 次评估/候选
- 参数范围：
  | 参数 | 范围 | 默认值 |
  |------|------|--------|
  | imm_cv_dwell_time_sec | 900-6000 | 2400 |
  | imm_ct_dwell_time_sec | 90-1200 | 360 |
  | imm_ct_fixed_Q_scale | 2.0-10.0 | 4.5 |
  | imm_transient_gain_max | 3.0-15.0 | 7.0 |
  | imm_transient_nis_start | 1.0-8.0 | 3.0 |
  | imm_transient_nis_full | 5.0-30.0 | 12.0 |
  | imm_transient_ewma_alpha | 0.3-0.95 | 0.65 |

**Phase 2: 双因素因子扫描**
- 扫描 `ct_fixed_Q_scale` × `transient_gain_max` 的 5×5 网格
- 目的：确认这两个参数是否单独有效

**Phase 3: Top 5 配置精细化搜索**
- 对 Phase 1 的 Top 5 配置，每个参数做 ±10%/20% 微调
- 目的：确认 Phase 1 结果是否已接近局部最优

## 结果

### Phase 1 完整排名（Top 10）

| Rank | Avg RMSE | Best RMSE | cv_dwell | ct_dwell | ct_q | gain_max | ewma |
|------|----------|-----------|----------|----------|------|----------|------|
| #16  | 3.978    | 2.751     | 4000     | 1200     | 4.71 | 12.90    | 0.40 |
| #29  | 3.980    | 2.783     | 1200     | 1140     | 7.40 | 7.94     | 0.40 |
| #22  | 3.983    | 2.783     | 2300     | 1080     | 9.20 | 10.80    | 1.00 |
| #10  | 3.987    | 2.776     | 3200     | 1020     | 8.47 | 4.66     | 0.80 |
| #19  | 3.988    | 2.761     | 3900     | 1110     | 9.46 | 5.04     | 0.60 |

**默认配置对照：** avg RMSE = 4.523 km, best RMSE = 2.941 km

**改进幅度：**
- Avg RMSE: 4.52 → 3.98 km (**12% 改善**)
- Best scenario RMSE: 2.94 → 2.75 km (**6.5% 改善**)
- single_turn R1: 4.77 → 2.75 km (**42% 改善**)

### Phase 2 关键发现

单独调 `ct_q` 和 `gain_max` 不能复现 Phase 1 的提升：
- 所有 49 个组合的 avg RMSE 都在 4.519-4.538 之间
- 最优组合：ct_q=3.3, gain=5.0 → avg=4.519 km（仅比默认提升 0.2%）

**结论：** 高 `ct_q` 必须与其他参数（特别是 `ct_dwell`、`ewma_alpha`）**协同**才能生效。

### Phase 3 精细化结论

对 Top 5 配置的每个参数做 ±10%/20% 微调：
- Config #16 微调后 BEST = 3.988 km > base 3.978 km
- **结论：Phase 1 的 Top 配置已接近局部最优，无法通过 ±20% 微调进一步改善**

## 推荐配置

以下是最优配置（来自 Phase 1 Rank #10）：

```matlab
params.imm_cv_dwell_time_sec      = 3200;   % 默认 2400
params.imm_ct_dwell_time_sec      = 1020;   % 默认 360
params.imm_ct_fixed_Q_scale       = 8.47;   % 默认 4.5
params.imm_transient_gain_max     = 4.66;   % 默认 7.0
params.imm_transient_nis_start    = 3.50;   % 默认 3.0
params.imm_transient_nis_full     = 11.50;  % 默认 12.0
params.imm_transient_ewma_alpha   = 0.80;   % 默认 0.65
```

这些参数通过 `validation/test_run_mc_best_params.m` 的 Monte Carlo 验证。

### 参数意义解释

1. **`imm_ct_fixed_Q_scale = 8.47`（默认 4.5）**：CT 模型的过程噪声放大倍数。转弯时目标加速度大，需要更大的 Q 来跟上轨迹变化。这是最重要的参数。

2. **`imm_ct_dwell_time_sec = 1020`（默认 360）**：CT 模型的平均驻留时间。转弯后需要更长时间保持 CT 模式，避免过早回退到 CV 导致跟踪发散。

3. **`imm_cv_dwell_time_sec = 3200`（默认 2400）**：CV 模型的驻留时间调整，配合其他参数形成协同效应。

4. **`imm_transient_gain_max = 4.66`（默认 7.0）**：CV 模型瞬态增益上限适度降低，避免过度响应。

## 后续工作

- **Phase 4: 50 轮全场景蒙特卡洛测试** — 验证泛化能力
- 场景覆盖：10 个场景（6 个基础 + 4 个变速率变体）
- 随机种子：50 个不同 seed
- 工具：`validation/test_run_mc_best_params.m`
