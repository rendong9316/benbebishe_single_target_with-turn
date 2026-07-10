# V13 `ukf/ukf_imm.m` 主函数核心 逐语句块代码审查（Part 1：create_imm、init_imm、prepare_imm）

## 1. 审查元数据

| 字段 | 值 |
|---|---|
| 审查对象 | `ukf/ukf_imm.m`（`create_imm`、`init_imm`、`prepare_imm` 主函数核心） |
| 覆盖范围 | 第 37—215 行 |
| 源码基线 | `HEAD 7c166d41541ccd74f23fd6c3ea0b871d8603950e` |
| 工作树文件哈希 | `24c9753c053acbeb` |
| 审查方式 | 只读静态源码核对、公式推导、仓库调用点交叉核对 |
| 运行状态 | 被 `imm/imm_tracker.m` 和 `tracker/single_track_runner.m` 调用 |
| 修改范围 | 仅新增本审查文档 |

> 注：`update_imm`、`apply_fuzzy_adapt_imm`、`keep_prediction`、`trimf_val_imm` 已在 V11 中审查。本文档聚焦 `create_imm`、`init_imm`、`prepare_imm`。

## 2. 文件职责与接口契约

### 2.1 职责

IMM（交互多模型）UKF 滤波器封装：维护 CV（匀速）和 CT（协调转弯）双模型的独立 UKF 实例 + 模型概率 + Markov 转移矩阵。对外暴露 `create/init/prepare/update` 四步 action 接口，与 `ukf_jichu` 和 `ukf_zishiying` 完全兼容。[S][P]

### 2.2 输入输出（create）

| 输入 | 类型 | 含义 |
|---|---|---|
| `params` | struct | 仿真参数 |
| `radar_lon, radar_lat` | deg | 接收站经纬度 |
| `tx_lon, tx_lat` | deg | 发射站经纬度 |
| `dt` | sec | 时间步长 |

| 输出 | 类型 | 含义 |
|---|---|---|
| `imm` | struct | IMM 状态结构体 |

## 3. 逐语句块审查

### 3.1 Action Dispatcher（第 37—51 行）

```matlab
function varargout = ukf_imm(action, varargin)
switch action
    case 'create'
        varargout{1} = create_imm(varargin{:});
    case 'init'
        varargout{1} = init_imm(varargin{:});
    case 'prepare'
        [varargout{1}, varargout{2}, varargout{3}, varargout{4}, ...
         varargout{5}, varargout{6}, varargout{7}] = prepare_imm(varargin{:});
    case 'update'
        [varargout{1}, varargout{2}, varargout{3}] = update_imm(varargin{:});
    otherwise
        error('ukf_imm: unknown action ''%s''', action);
end
end
```

**语句职责。** 统一入口分发器，根据 `action` 字符串调用对应子函数。[S]

**代码质量。** 纯函数式 dispatcher 模式，符合项目约束（禁止 OOP）。`varargout` 支持可变数量输出。[S]

**问题记录。** `V13-P3-01`：`varargout` 的下标赋值在 MATLAB 中要求所有输出数量在编译期确定。此处 `prepare` 返回 7 个输出，`update` 返回 3 个，`create`/`init` 返回 1 个。若调用者期望固定数量的输出（如 `[a,b,c,d,e,f,g] = ukf_imm('create', ...)`），`create` 只会返回 1 个值，多余输出位为错误。但当前调用者（`imm_tracker.m`、`single_track_runner.m`）都只请求 1 个输出，行为正确。[S]

### 3.2 基础参数保存（第 58—65 行）

```matlab
imm.params = params;
imm.radar_lon = radar_lon;
imm.radar_lat = radar_lat;
imm.tx_lon = tx_lon;
imm.tx_lat = tx_lat;
imm.dt = dt;
```

**语句职责。** 保存雷达位置、发射站位置和時間步长。[S]

**代码质量。** 字段命名与传入参数名一致，便于追溯。[S]

### 3.3 双 UKF 创建（第 67—80 行）

```matlab
ukf_cv = ukf_jichu('create', params, radar_lon, radar_lat, tx_lon, tx_lat, dt);
ukf_ct = ukf_jichu('create', params, radar_lon, radar_lat, tx_lon, tx_lat, dt);
ukf_ct.model_type = 'CT';
if isfield(params, 'imm_turn_rate_rad_per_sec')
    ukf_ct.turn_rate_rad_per_sec = params.imm_turn_rate_rad_per_sec;
else
    ukf_ct.turn_rate_rad_per_sec = 1.0 * pi / 180;  % 默认 1°/s
end
```

**语句职责。** 创建 CV 和 CT 两个独立 UKF 实例，CT 设置转弯率。[S]

**数学正确性。** CT（Constant Turn）模型的转向率 `ω` 出现在状态转移矩阵中：

```
F_CT = [1, sin(ωΔt)/ω, 0, -(1-cos(ωΔt))/ω;
        0, cos(ωΔt), 0, -sin(ωΔt);
        0, (1-cos(ωΔt))/ω, sin(ωΔt)/ω, 0;
        0, sin(ωΔt), 0, cos(ωΔt)]
```

当 `ω → 0` 时，`F_CT → F_CV`，模型连续退化。[M]

**问题记录。** `V13-P2-01`：`ukf_ct.turn_rate_rad_per_sec = 1°/s` 是经验值。对于民航客机（典型转弯率 ≤ 3°/s = 0.052 rad/s），1°/s = 0.017 rad/s 是合理的。但对于高机动目标（如战斗机，转弯率可达 10°/s = 0.17 rad/s），此默认值偏低，CT 模型的预测精度会下降。[M]

**问题记录。** `V13-P2-02`：`ukf_ct.model_type = 'CT'` 的设置仅在 `ukf_jichu` 内部使用（如选择不同的 sigma 点方案）。若 `ukf_jichu` 不读取此字段，则为死代码。[S]

### 3.4 Markov 转移矩阵（第 82—91 行）

```matlab
if isfield(params, 'imm_Pi_CV_to_CT') && isfield(params, 'imm_Pi_CT_to_CV')
    p_cv_ct = params.imm_Pi_CV_to_CT;
    p_ct_cv = params.imm_Pi_CT_to_CV;
    imm.Pi = [1-p_cv_ct, p_cv_ct;
              p_ct_cv, 1-p_ct_cv];
else
    imm.Pi = [0.90, 0.10;
              0.10, 0.90];
end
```

**语句职责。** 构建 2×2 Markov 转移概率矩阵。[S]

**数学正确性。** Markov 矩阵每行和为 1：`Σ_j P(i→j) = 1`。此处：

```
Pi = [P(CV→CV), P(CV→CT);
      P(CT→CV), P(CT→CT)]
   = [1-p_cv_ct, p_cv_ct;
      p_ct_cv, 1-p_ct_cv]
```

每行和 = `(1-p_cv_ct) + p_cv_ct = 1`，`p_ct_cv + (1-p_ct_cv) = 1`。[M]

**问题记录。** `V13-P2-03`：默认值 `[0.90, 0.10; 0.10, 0.90]` 意味着模型平均每 10 帧切换一次（几何分布的期望 = 1/(1-0.9) = 10 帧）。对于转弯频率较低的场景（如 90% 时间直线飞行），这是合理的。但对于频繁转弯的场景（如每 5 帧转弯一次），此矩阵响应太慢。[M]

**问题记录。** `V13-P1-01`：已在 V01（`simulation_params.m`）中登记 CR-ISSUE-001，`imm_Pi_CV_to_CT` 和 `imm_Pi_CT_to_CV` 被重复定义 6 组（每组两条），值均为 `0.005`。这意味着实际转移概率为：

```
Pi = [0.995, 0.005;
      0.005, 0.995]
```

模型平均切换周期 = 1/0.005 = 200 帧。即 IMM 几乎不切换模型，CT 模型的贡献微乎其微。若目标确实有转弯，IMM 的优势无法发挥。[S+M]

### 3.5 初始模型概率（第 93—99 行）

```matlab
if isfield(params, 'imm_mu_init_CV')
    mu_cv = params.imm_mu_init_CV;
    imm.mu = [mu_cv; 1 - mu_cv];
else
    imm.mu = [0.5; 0.5];
end
```

**语句职责。** 设置初始模型概率。[S]

**数学正确性。** `mu(1) + mu(2) = 1`，构成合法概率分布。[M]

### 3.6 IMM-IPDA 检测参数（第 101—105 行）

```matlab
imm.Pd = params.detection_probability;
imm.Pg = params.pda_pd_gate;
imm.Pd_Pg = imm.Pd * imm.Pg;
imm.L_no_det = 1.0 - imm.Pd_Pg;
```

**语句职责。** 计算 IPDA（集成概率数据关联）所需的检测参数。[S]

**数学正确性。** 在 IPDA 框架中，无检测似然度为 `L_no_det = 1 - Pd × Pg`，其中 `Pd` 是检测概率，`Pg` 是门内概率。此公式假设检测和门内命中独立。[M]

**问题记录。** `V13-P1-02`：`imm.Pd = params.detection_probability` 当前值为 `1.0`（CR-ISSUE-010）。因此 `imm.Pd_Pg = 1.0 × Pg = Pg`，`L_no_det = 1 - Pg`。若 `Pg = 0.8647`（见 CR-ISSUE-002），则 `L_no_det = 0.1353`。这意味着无检测的似然度固定为 13.53%，与目标是否存在无关。当 `Pd < 1.0` 时，此值会随 `Pd` 变化，当前 `Pd=1.0` 是一种退化情况。[M]

### 3.7 概率钳位（第 107—109 行）

```matlab
imm.mu_min = 0.02;
imm.mu_max = 0.95;
```

**语句职责。** 模型概率的上下界钳位，防止概率过早收敛到 0 或 1（导致模型灭绝或独占）。[S]

**数学正确性。** 钳位后需重新归一化（在 `update_imm` 第 277—278 行完成）。[M]

**问题记录。** `V13-P2-04`：`mu_min = 0.02` 意味着即使某个模型完全不适合当前数据，它仍保留 2% 的概率权重。这保证了模型的"探索"能力，但也意味着不适合的模型不会被完全丢弃。`mu_max = 0.95` 同理，保证另一个模型至少有 5% 的权重。[M]

### 3.8 `init_imm` 初始化（第 123—145 行）

```matlab
imm.ukf_cv = ukf_jichu('init', imm.ukf_cv, meas1, meas2);
imm.ukf_cv.dt = imm.dt;
imm.ukf_cv.initialized = true;
imm.ukf_cv.Q_base = imm.ukf_cv.Q;
imm.ukf_cv.Q_ema = 1.0;
imm.ukf_cv.nis_history = [];
```

**语句职责。** 初始化两个 UKF 实例的两点差分、时间步长、初始化标记、Q 基准和 EMA。[S]

**问题记录。** `V13-P2-05`：`init_imm` 中 `Q_ema = 1.0` 和 `nis_history = []` 的初始化与 `post_init_multi.m` 中的逻辑重复。两者都被调用时，后执行的会覆盖先执行的。执行顺序取决于调用者。[S]

**问题记录。** `V13-P2-06`：`imm.mu = [0.5; 0.5]` 在 `init_imm` 中无条件重置为均匀分布。若 `create_imm` 中设置了非均匀的初始概率（如 `imm_mu_init_CV = 0.8`），`init` 会将其覆盖为 `[0.5; 0.5]`。[S]

**问题记录。** `V13-P2-07`：`imm.mu_history = zeros(0, 2)` 初始化为 0×2 的空矩阵。后续在 `update_imm` 第 288 行通过 `end+1` 追加。空矩阵的 `end+1` 在 MATLAB 中返回索引 1，行为正确。[M]

### 3.9 `prepare_imm` 模型混合（第 153—182 行）

```matlab
c_bar = Pi' * mu;
mu_mix = zeros(M, M);
for i = 1:M
    for j = 1:M
        mu_mix(i, j) = Pi(i, j) * mu(i) / max(c_bar(j), 1e-12);
    end
end
```

**语句职责。** 计算混合概率 `μ_{i|j} = P(i→j) × μ(i) / Σ_k P(k→j) × μ(k)`。[S]

**数学正确性。** 这是 IMM 的标准混合公式：

```
c_bar(j) = Σ_i π_{ij} × μ_i    （归一化常数）
μ_{i|j} = π_{ij} × μ_i / c_bar(j)    （模型 j 的混合初始概率）
```

`max(c_bar(j), 1e-12)` 防止除零。[M]

**验证。** 混合概率应满足 `Σ_i μ_{i|j} = 1`（对每个 j）。代码未显式验证，但数学上成立（只要 `c_bar` 计算正确）。[M]

**问题记录。** `V13-P2-08`：`c_bar = Pi' * mu` 是矩阵-向量乘法。`Pi'` 是 2×2 转置矩阵，`mu` 是 2×1 向量，结果 `c_bar` 是 2×1。`c_bar(j)` 表示进入模型 j 的总概率流。[M]

### 3.10 混合状态和协方差（第 169—182 行）

```matlab
for j = 1:M
    for i = 1:M
        x_mix{j} = x_mix{j} + mu_mix(i, j) * ukf_models{i}.x;
    end
    for i = 1:M
        dx = ukf_models{i}.x - x_mix{j};
        P_mix{j} = P_mix{j} + mu_mix(i, j) * (ukf_models{i}.P + dx * dx');
    end
end
imm.ukf_cv.x = x_mix{1};  imm.ukf_cv.P = P_mix{1};
imm.ukf_ct.x = x_mix{2};  imm.ukf_ct.P = P_mix{2};
```

**语句职责。** 对每个模型 j，计算混合状态均值和协方差。[S]

**数学正确性。** 混合协方差公式：

```
x_{j} = Σ_i μ_{i|j} × x_i
P_{j} = Σ_i μ_{i|j} × [P_i + (x_i - x_j)(x_i - x_j)']
```

第二项 `dx * dx'` 是模型均值离差的外积，确保混合协方差大于等于各模型协方差的加权和（凸性）。这是 IMM 的关键公式，保证模型分歧被正确传播。[M]

**问题记录。** `V13-P1-03`：此处 `prepare_imm` 正确计算了混合协方差（含离差项），但 `update_imm` 中返回的组合协方差 `imm.P = mu(1)*P_cv + mu(2)*P_ct`（第 287 行）**缺少离差项**。这是一个独立的 P1 问题（已在 V03c 中登记为 CR-ISSUE-019），但在此处需要强调：`prepare_imm` 的混合协方差是正确的，而顶层组合协方差是不完整的。[S+M]

### 3.11 双模型独立预测（第 184—188 行）

```matlab
[x_pred_cv, P_pred_cv, X_pred_cv, z_pred_cv, Z_pred_cv, P_zz_cv, imm.ukf_cv] = ...
    ukf_jichu('prepare', imm.ukf_cv);
[x_pred_ct, P_pred_ct, X_pred_ct, z_pred_ct, Z_pred_ct, P_zz_ct, imm.ukf_ct] = ...
    ukf_jichu('prepare', imm.ukf_ct);
```

**语句职责。** 对混合后的两个 UKF 实例分别执行 UKF 预测步。[S]

**代码质量。** 两个模型独立预测，无数据依赖，可并行化。但当前顺序执行。[S]

**问题记录。** `V13-P3-02`：`ukf_jichu('prepare', ...)` 返回 7 个输出，其中最后一个 `imm.ukf_cv` 是更新后的 UKF 结构体（包含预测后的 x、P 等）。直接赋值给 `imm.ukf_cv` 字段，修改了 IMM 结构的内部状态。这种"副作用式"返回值设计增加了理解难度。[S]

### 3.12 组合预测和 CV 优先输出（第 190—203 行）

```matlab
x_pred_comb = mu(1) * x_pred_cv + mu(2) * x_pred_ct;
z_pred_comb = mu(1) * z_pred_cv + mu(2) * z_pred_ct;
P_zz_comb = 0.5 * P_zz_cv + 0.5 * P_zz_ct;
...
x_pred = x_pred_cv;
z_pred = z_pred_cv;
P_zz = P_zz_cv;
```

**语句职责。** 计算组合预测（内部缓存用），但返回给调用者的是 CV 模型的预测。[S]

**问题记录。** `V13-P1-04`：组合测量协方差 `P_zz_comb` 固定按 0.5/0.5 加权，**不使用当前模型概率 `mu`**。这与 `update_imm` 中的相同问题（CR-ISSUE-017 / V03c-P1-02）一致。[S]

**问题记录。** `V13-P1-05`：返回给 tracker 的门中心固定为 CV 模型的预测（`x_pred_cv`、`z_pred_cv`、`P_zz_cv`），即使当前模型概率显示 CT 占优（如 `mu = [0.1; 0.9]`）。这是 CR-ISSUE-009 / V03c-P1-03 的核心问题：转弯时 IMM 的 CT 优势无法传递到关联层。[S]

**问题记录。** `V13-P2-09`：`x_pred_comb`、`z_pred_comb`、`P_zz_comb` 计算后仅被缓存到 `imm.cache`，从未被 `update_imm` 使用。这些计算是死代码，除非后续有人修改代码使用它们。[S]

### 3.13 缓存（第 205—214 行）

```matlab
imm.cache = struct(...
    'x_pred_cv', x_pred_cv, 'x_pred_ct', x_pred_ct, ...
    'P_pred_cv', P_pred_cv, 'P_pred_ct', P_pred_ct, ...
    ...
    'x_pred_comb', x_pred_comb, 'z_pred_comb', z_pred_comb, 'P_zz_comb', P_zz_comb, ...
    'c_bar', c_bar);
```

**语句职责。** 缓存所有中间结果供 `update_imm` 使用。[S]

**问题记录。** `V13-P3-03`：缓存中包含 `x_pred_comb` 等无用字段（见 V13-P2-09），增加了内存占用。每个缓存条目是 4×1 或 3×3 矩阵，7 个模型 × 2 个状态/协方差 × 2 个预测 + 3 个组合 + 1 个 c_bar ≈ 70 个 double = 560 bytes。在 MC 循环中可忽略，但设计不精简。[S]

## 4. 文件级问题汇总

| Issue ID | 位置 | 严重度 | 类别 | 证据 | 状态 |
|---|---:|---|---|---|---|
| V13-P1-01 | 第 82—91 行 | P1 | Markov 转移概率 0.005 导致模型切换周期 200 帧 | S+M | OPEN |
| V13-P1-02 | 第 101—105 行 | P1 | Pd=1.0 时 IPDA 似然度退化 | M | OPEN |
| V13-P1-03 | 第 287 行 vs 172—179 行 | P1 | 顶层组合协方差缺离差项 | S+M | OPEN |
| V13-P1-04 | 第 193 行 | P1 | P_zz_comb 固定 0.5/0.5 不用 mu | S | OPEN |
| V13-P1-05 | 第 196—198 行 | P1 | 门中心固定 CV，CT 优势不传递 | S | OPEN |
| V13-P2-01 | 第 76 行 | P2 | CT 转弯率默认值 1°/s 对高机动目标偏低 | M | OPEN |
| V13-P2-02 | 第 72 行 | P2 | model_type 字段可能未使用 | S | OPEN |
| V13-P2-03 | 第 89—91 行 | P2 | 默认 Markov 矩阵切换周期 10 帧可能不匹配场景 | M | OPEN |
| V13-P2-04 | 第 107—109 行 | P2 | 概率钳位 0.02/0.95 的合理性 | M | OPEN |
| V13-P2-05 | 第 123—145 行 | P2 | init_imm 与 post_init_multi 初始化逻辑重复 | S | OPEN |
| V13-P2-06 | 第 138 行 | P2 | init_imm 覆盖 create_imm 的初始概率 | S | OPEN |
| V13-P2-07 | 第 141 行 | P2 | mu_history 空矩阵初始化 | S | OPEN |
| V13-P2-08 | 第 161 行 | P3 | c_bar 计算注释不足 | S | OPEN |
| V13-P2-09 | 第 191—193 行 | P2 | 组合预测计算后未使用 | S | OPEN |
| V13-P3-01 | 第 37—51 行 | P3 | varargout 输出数量依赖 action | S | OPEN |
| V13-P3-02 | 第 185—188 行 | P3 | ukf_jichu 返回值含副作用 | S | OPEN |
| V13-P3-03 | 第 206—214 行 | P3 | 缓存包含无用字段 | S | OPEN |

## 5. 文件级结论

- **数学可信度**：IMM 核心算法（混合概率、混合协方差含离差项）在 `prepare_imm` 中正确实现。但顶层组合协方差（`update_imm` 第 287 行）缺少离差项，与混合协方差的计算不一致。门中心固定 CV 模型导致 IMM 的转弯适应性无法发挥。[S+M]
- **代码质量**：Action dispatcher 模式清晰，但 `varargout` 输出数量不固定增加了调用复杂度。缓存设计包含死代码。[S]
- **性能**：双 UKF 独立预测可并行化但当前顺序执行。缓存占用可忽略。[S]
- **测试充分性**：IMM 在不同转弯率、不同 Markov 矩阵下的性能未量化。[S]
- **剩余未验证项**：Markov 转移概率 0.005 对转弯检测的实际影响；CT 转弯率默认值 1°/s 对不同机动目标的适用性。

## 6. 下一审查游标

- 文件：`imm/imm_tracker.m`（IMM 跟踪器包装器）
- 重点：`imm_tracker` 如何调用 `ukf_imm`、life_count 管理、重起始逻辑
- 稳定指纹：`imm_tracker` 主循环
