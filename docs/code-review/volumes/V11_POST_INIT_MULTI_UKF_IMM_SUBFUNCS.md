# V11 `tracker/post_init_multi.m` + `ukf/ukf_imm.m` 子函数 逐语句块代码审查

## 1. 审查元数据

| 字段 | 值 |
|---|---|
| 审查对象 | `tracker/post_init_multi.m`、`ukf/ukf_imm.m`（`apply_fuzzy_adapt_imm`、`keep_prediction`） |
| 覆盖范围 | post_init_multi: 1—12 行；ukf_imm: 301—379 行 |
| 源码基线 | `HEAD 7c166d41541ccd74f23fd6c3ea0b871d8603950e` |
| 工作树文件哈希 | `db31971086172d2e` / `24c9753c053acbeb` |
| 审查方式 | 只读静态源码核对、公式推导、仓库调用点交叉核对 |
| 运行状态 | `post_init_multi` 被 `run_simulation_multi.m` 调用；`apply_fuzzy_adapt_imm` 被 `ukf_imm.m` 主函数调用 |
| 修改范围 | 仅新增本审查文档 |

---

## Part A: `tracker/post_init_multi.m`

### 2. 文件职责与接口契约

**职责。** 多目标跟踪器初始化后辅助函数：设置时间步长、标记初始化完成、初始化 EMA 参数。[S][P]

**输入输出。**

| 输入 | 类型 | 含义 |
|---|---|---|
| `ukf` | struct | UKF 状态结构体 |
| `params` | struct | 仿真参数，需含 `.dt_sec` |

| 输出 | 类型 | 含义 |
|---|---|---|
| `ukf` | struct | 更新后的 UKF 结构体（按值返回） |

### 3. 逐语句块审查

#### 3.1 时间步长和初始化标记（第 2 行）

```matlab
ukf.dt = params.dt_sec; ukf.initialized = true;
```

**语句职责。** 将仿真时间步长写入 UKF 结构体，标记跟踪器已初始化。[S]

**代码质量。** 单行两条赋值语句，用空格分隔而非分号，输出到命令行（无 `;` 结尾）。在函数内部这不是问题，但 `ukf.initialized` 的值被打印到命令行（因为无分号），可能产生不必要的输出。[S]

**问题记录。** `V11A-P2-01`：`ukf.dt` 的写入覆盖了 UKF 结构中可能已有的 `dt` 值。若调用者期望保留原有 `dt`，此行为是破坏性的。[S]

#### 3.2 IMM 子滤波器同步（第 3—6 行）

```matlab
if isfield(ukf, 'ukf_cv')
    ukf.ukf_cv.dt = params.dt_sec; ukf.ukf_cv.initialized = true;
    ukf.ukf_ct.dt = params.dt_sec; ukf.ukf_ct.initialized = true;
end
```

**语句职责。** 若 UKF 结构包含 IMM 子滤波器（CV 和 CT），同步设置它们的 `dt` 和 `initialized` 标记。[S]

**数学正确性。** IMM 的两个子滤波器需要独立维护状态，此同步逻辑正确。[M]

**问题记录。** `V11A-P2-02`：`ukf.ukf_ct` 的存在性未通过 `isfield` 检查，直接访问可能导致字段不存在时的错误。虽然逻辑上若存在 `ukf_cv` 则应存在 `ukf_ct`（IMM 配对设计），但代码未 enforce 此假设。[S]

#### 3.3 NIS 历史清零（第 7 行）

```matlab
ukf.nis_history = [];
```

**语句职责。** 清空新息平方历史，为新一轮跟踪准备。[S]

**问题记录。** `V11A-P3-01`：`nis_history` 在 `ukf_jichu.m` 和 `ukf_imm.m` 中都被使用（用于模糊自适应 Q 调整和 NIS 统计）。清空操作确保了初始状态干净，但若此前已有 NIS 数据（如从冷启动过渡到跟踪模式），这些数据被丢弃且无警告。[S]

#### 3.4 Q 基线和 EMA 初始化（第 8—11 行）

```matlab
if ~isfield(ukf, 'Q_base') || isempty(ukf.Q_base)
    if isfield(ukf, 'Q'), ukf.Q_base = ukf.Q; end
end
if ~isfield(ukf, 'Q_ema') || isempty(ukf.Q_ema), ukf.Q_ema = 1.0; end
```

**语句职责。** 初始化过程噪声协方差的基准值 `Q_base` 和指数移动平均因子 `Q_ema`。[S]

**数学正确性。** `Q_ema = 1.0` 表示初始缩放因子为 1（不使用自适应），后续由 `apply_fuzzy_adapt_imm` 根据 NIS 统计调整。[M]

**问题记录。** `V11A-P2-03`：`Q_base = Q` 是浅拷贝。若 `Q` 是矩阵且在后续被修改（如 `Q = Q * factor`），`Q_base` 也会受到影响（MATLAB 的 struct 字段是深拷贝还是浅拷贝取决于赋值时机）。在纯函数式编程约束下，此处的浅拷贝风险可忽略，但应知晓。[S]

**问题记录。** `V11A-P3-02`：第 11 行 `ukf.Q_ema = 1.0` 无条件设置（只要字段不存在或为空）。但若调用者已设置了 `Q_ema`（如从上一轮跟踪继承），此操作会覆盖。[S]

### 4. Part A 问题汇总

| Issue ID | 位置 | 严重度 | 类别 | 证据 | 状态 |
|---|---:|---|---|---|---|
| V11A-P2-01 | 第 2 行 | P2 | dt 覆盖无警告 | S | OPEN |
| V11A-P2-02 | 第 5 行 | P2 | ukf_ct 未检查存在性 | S | OPEN |
| V11A-P3-01 | 第 7 行 | P3 | nis_history 静默清空 | S | OPEN |
| V11A-P3-02 | 第 11 行 | P3 | Q_ema 覆盖已有值 | S | OPEN |

### 5. Part A 结论

`post_init_multi` 是一个简短的辅助函数（12 行），职责单一：初始化多目标跟踪器的状态标记。代码简洁，但缺少防御性编程（字段存在性检查、覆盖警告）。在纯函数式约束下，按值返回的 `ukf` 结构体确保了无副作用。[S]

---

## Part B: `ukf/ukf_imm.m` 子函数

### 6. 文件职责与接口契约

本节审查 `ukf_imm.m` 中的两个子函数：`keep_prediction`（第 301—310 行）和 `apply_fuzzy_adapt_imm`（第 316—365 行），以及 `trimf_val_imm`（第 371—379 行）。[S]

### 7. `keep_prediction` 子函数（第 301—310 行）

```matlab
function ukf = keep_prediction(ukf, cache, model)
switch model
    case 'cv'
        ukf.x = cache.x_pred_cv;
        ukf.P = cache.P_pred_cv;
    case 'ct'
        ukf.x = cache.x_pred_ct;
        ukf.P = cache.P_pred_ct;
end
end
```

**语句职责。** 根据模型类型从缓存中恢复预测状态和协方差。[S]

**数学正确性。** IMM 算法中，每个模型的预测在 `prepare_imm` 中计算并缓存，此函数在模型切换时恢复对应的预测结果。[M]

**问题记录。** `V11B-P2-01`：`switch` 缺少 `otherwise` 分支。若 `model` 为其他值（如 `'cv_ct'` 或空字符串），函数返回未修改的 `ukf`，可能导致后续代码使用过期状态。[S]

**问题记录。** `V11B-P2-02`：`cache.x_pred_cv` 等字段的存在性未验证。若 `prepare_imm` 未正确填充缓存，此处会引发运行时错误。[S]

### 8. `apply_fuzzy_adapt_imm` 子函数（第 316—365 行）

**语句职责。** 基于 NIS 统计的模糊自适应过程噪声协方差调整。[S]

#### 8.1 NIS 历史检查（第 317—319 行）

```matlab
if ~isfield(ukf, 'nis_history') || isempty(ukf.nis_history)
    return;
end
```

**语句职责。** 无 NIS 历史时跳过自适应。[S]

**问题记录。** `V11B-P3-03`：直接 `return` 不修改 `ukf.Q`。若调用者期望 Q 被调整，此静默跳过可能导致后续跟踪使用不合适的 Q 值。[S]

#### 8.2 Q_ema 和 Q_base 初始化（第 323—328 行）

```matlab
if ~isfield(ukf, 'Q_ema') || isempty(ukf.Q_ema)
    ukf.Q_ema = 1.0;
end
if ~isfield(ukf, 'Q_base') || isempty(ukf.Q_base)
    ukf.Q_base = ukf.Q;
end
```

**语句职责。** 确保 EMA 因子和基准 Q 存在。[S]

**问题记录。** `V11B-P2-03`：与 V11A-P3-02 相同的问题——`Q_base = ukf.Q` 是浅拷贝，且 `Q_ema` 的默认值 1.0 可能覆盖调用者设置的值。[S]

#### 8.3 NIS 比率计算（第 330—331 行）

```matlab
nis_avg = mean(nis_history);
nis_ratio = nis_avg / 2.0;
```

**语句职责。** 计算 NIS 平均值与理论期望值（2 维卡方分布的期望 = 自由度 = 2）的比值。[S]

**数学正确性。** 对于正确设定的 UKF，NIS（新息平方）的期望值等于量测维度。在 2D（位置）情况下期望为 2。`nis_ratio = 1.0` 表示 NIS 符合理论期望，`> 1.0` 表示预测过于自信（Q 太小），`< 1.0` 表示预测过于保守（Q 太大）。[M]

**问题记录。** `V11B-P1-01`：`nis_history` 中可能包含 2D NIS（位置）和 3D NIS（含径向速度），两者的期望值不同（2 vs 3）。`mean(nis_history)` 混合了不同维度的 NIS，导致 `nis_ratio` 的统计意义不明确。[M]

**问题记录。** `V11B-P2-04`：`nis_history` 可能包含异常值（如目标机动时的瞬时大 NIS）。`mean` 对异常值敏感，建议使用 `median` 或截尾均值。[S]

#### 8.4 模糊隶属度计算（第 333—337 行）

```matlab
mu_VS = trimf_val_imm(nis_ratio, 0.0, 0.0, 0.4);
mu_S  = trimf_val_imm(nis_ratio, 0.2, 0.5, 0.8);
mu_M  = trimf_val_imm(nis_ratio, 0.6, 1.0, 1.5);
mu_L  = trimf_val_imm(nis_ratio, 1.3, 2.0, 3.0);
mu_VL = trimf_val_imm(nis_ratio, 2.5, 4.0, 4.0);
```

**语句职责。** 计算五个模糊集（Very Small, Small, Medium, Large, Very Large）的隶属度。[S]

**数学正确性。** 三角形隶属函数 `trimf(x; a, b, c)` 在 `[a, b]` 上线性上升到 1，在 `[b, c]` 上线性下降到 0。[M]

**问题记录。** `V11B-P2-05`：模糊集的划分存在重叠和不重叠区域：
- VS: [0, 0.4]，S: [0.2, 0.8]，M: [0.6, 1.5]，L: [1.3, 3.0]，VL: [2.5, 4.0]
- 在 nis_ratio ∈ [0.4, 0.2] = 空集（VS 和 S 在 [0.2, 0.4] 重叠，OK）
- 在 nis_ratio ∈ [0.8, 0.6] = 空集（S 和 M 在 [0.6, 0.8] 重叠，OK）
- 但 nis_ratio > 4.0 时所有隶属度为 0，`total_mu < 1e-10` 触发 `factor_fuzzy = 1.0`。这意味着极端大的 NIS 被当作"正常"处理，与意图相反。[M]

**问题记录。** `V11B-P2-06`：模糊集的峰值（b 值）分布不均匀：VS=0.0, S=0.5, M=1.0, L=2.0, VL=4.0。这表示"Medium"对应 NIS 比率为 1.0（理论期望），是合理的。但"Very Large"的峰值在 4.0，意味着 NIS 比率为 4 时才认为 Q 需要"快速增加"。对于 OTH-SWR 场景，NIS 比率达到 4 表示滤波器严重失配，此时再调整 Q 可能为时已晚。[M]

#### 8.5 模糊推理输出（第 339—352 行）

```matlab
out_Decrease       = 0.6;
out_SlightDecrease = 0.8;
out_Maintain       = 1.0;
out_Increase       = 1.8;
out_RapidIncrease  = 3.0;
...
factor_fuzzy = (mu_VS * 0.6 + mu_S * 0.8 + mu_M * 1.0 + ...
               mu_L * 1.8 + mu_VL * 3.0) / total_mu;
```

**语句职责。** 重心法（centroid defuzzification）将模糊输出聚合为单一缩放因子。[S]

**数学正确性。** 加权平均：`Σ μ_i * o_i / Σ μ_i`。当只有一个模糊集激活时，输出等于该集的输出值。[M]

**问题记录。** `V11B-P2-07`：输出值的设计——"Decrease"对应 0.6（缩小 Q），"RapidIncrease"对应 3.0（放大 Q 三倍）。放大范围（1.0 → 3.0）大于缩小范围（0.6 → 1.0），这符合直觉（机动目标需要更快地增大 Q，而平稳目标可以缓慢减小 Q）。[M]

#### 8.6 EMA 平滑（第 354—364 行）

```matlab
factor_raw = max(0.5, min(4.0, factor_fuzzy));
ema_eta = 0.20;
ukf.Q_ema = ema_eta * factor_raw + (1 - ema_eta) * ukf.Q_ema;
if abs(ukf.Q_ema - 1.0) < 0.05
    ukf.Q = ukf.Q_base;
else
    ukf.Q = ukf.Q_base * ukf.Q_ema;
end
```

**语句职责。** 对模糊输出做钳位和 EMA 平滑，更新 Q。[S]

**数学正确性。** EMA: `Q_ema[k] = α * factor + (1-α) * Q_ema[k-1]`，其中 α = 0.2。平滑因子限制了突变。[M]

**问题记录。** `V11B-P1-02`：`abs(ukf.Q_ema - 1.0) < 0.05` 是一个**魔法阈值**。当 Q_ema 在 [0.95, 1.05] 范围内时，Q 被重置为 Q_base（即不使用自适应）。这意味着小幅度的 NIS 波动不会触发 Q 调整，这是合理的去抖设计。但 0.05 的选择缺乏理论依据，且与模糊集的输出范围（0.6-3.0）相比，0.05 的窗口非常窄。[M]

**问题记录。** `V11B-P2-08`：`factor_raw` 被钳位到 [0.5, 4.0]，但模糊输出已经在 [0.6, 3.0] 范围内（因为隶属度的加权平均不可能超出最小/最大输出值）。`max(0.5, ...)` 和 `min(4.0, ...)` 的钳位在理论上永远不会生效（除非 `total_mu ≈ 0` 导致除零，但此时代码已处理为 `factor_fuzzy = 1.0`）。[S]

### 9. `trimf_val_imm` 子函数（第 371—379 行）

```matlab
function mu = trimf_val_imm(x, a, b, c)
if x <= a || x >= c
    mu = 0;
elseif x < b
    mu = (x - a) / (b - a);
else
    mu = (c - x) / (c - b);
end
end
```

**语句职责。** 三角形隶属函数求值。[S]

**数学正确性。** 标准三角隶属函数：在 `[a, b]` 上线性上升，在 `[b, c]` 上线性下降，在 `[a, c]` 外为 0。当 `x = b` 时 `mu = 1`。[M]

**问题记录。** `V11B-P3-04`：与 `ukf_zishiying.m:261` 的 `trimf_val_maneuver` 完全相同（已在 CR-ISSUE-023 中登记）。重复实现违反 DRY 原则。[S]

**问题记录。** `V11B-P2-09`：当 `b == a` 或 `b == c` 时，分母为零，MATLAB 返回 `NaN` 或 `Inf`。虽然当前调用中 `a < b < c` 始终成立，但函数缺少防御性检查。[S]

### 10. Part B 问题汇总

| Issue ID | 位置 | 严重度 | 类别 | 证据 | 状态 |
|---|---:|---|---|---|---|
| V11B-P1-01 | 第 330 行 | P1 | 2D/3D NIS 混合平均 | M | OPEN |
| V11B-P1-02 | 第 360 行 | P1 | Q_ema 重置阈值 0.05 缺乏依据 | M | OPEN |
| V11B-P2-01 | 第 301—310 行 | P2 | keep_prediction 缺 otherwise | S | OPEN |
| V11B-P2-02 | 第 304 行 | P2 | cache 字段存在性未验证 | S | OPEN |
| V11B-P2-03 | 第 323—328 行 | P2 | Q_base 浅拷贝 + Q_ema 覆盖 | S | OPEN |
| V11B-P2-04 | 第 330 行 | P2 | mean 对异常值敏感 | S | OPEN |
| V11B-P2-05 | 第 346—347 行 | P2 | nis_ratio > 4 时 total_mu ≈ 0 | M | OPEN |
| V11B-P2-06 | 第 333—337 行 | P2 | VL 峰值 4.0 响应滞后 | M | OPEN |
| V11B-P2-07 | 第 354 行 | P3 | factor_raw 钳位理论上不生效 | S | OPEN |
| V11B-P2-09 | 第 371—379 行 | P2 | trimf 分母为零风险 | S | OPEN |
| V11B-P3-03 | 第 317—319 行 | P3 | nis_history 为空时静默 return | S | OPEN |
| V11B-P3-04 | 第 371—379 行 | P3 | 与 ukf_zishiying.m 重复实现 | S | OPEN |

### 11. Part B 结论

`apply_fuzzy_adapt_imm` 实现了基于 NIS 统计的模糊自适应 Q 调整，核心思路（NIS 比率 → 模糊隶属度 → 重心法 → EMA 平滑）在自适应滤波领域是标准的。主要缺陷是：2D/3D NIS 混合平均导致 nis_ratio 统计意义不明确；Q_ema 重置阈值 0.05 缺乏依据；nis_ratio > 4 时所有隶属度为 0 导致退化处理。[M][S]

## 12. 下一审查游标

- 文件：`fusion/track_fusion_algorithms.m` 的 `fuse_ci`、`fuse_fci` 子函数
- 重点：协方差交叉算法的正确性、迹权重量纲
- 稳定指纹：`fuse_ci` 第 250—320 行
