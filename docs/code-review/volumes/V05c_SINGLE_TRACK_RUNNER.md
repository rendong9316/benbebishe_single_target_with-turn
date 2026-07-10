# V05c `tracker/single_track_runner.m` 逐语句块代码审查

## 1. 审查元数据

| 字段 | 值 |
|---|---|
| 审查对象 | `tracker/single_track_runner.m` |
| 覆盖范围 | 第 1—329 行 |
| 源码基线 | `HEAD 7c166d41541ccd74f23fd6c3ea0b871d8603950e` |
| 工作树文件哈希 | `4575f9c0668d48a0` |
| 审查方式 | 只读静态源码核对、公式推导、仓库调用点交叉核对 |
| 运行状态 | 随 `run_simulation` 端到端冒烟 |
| 修改范围 | 仅新增本审查文档 |

## 2. 文件职责与接口契约

### 2.1 职责

单目标逐帧航迹跟踪主循环。状态机：INITIATING → TRACKING → LOST → INITIATING。支持三种滤波器后端（基础 UKF、自适应 UKF、IMM UKF），通过 `ukf_dispatch` 多态路由。[S][P]

### 2.2 输入输出

| 输入 | 类型 | 含义 |
|---|---|---|
| `detList` | cell[n_frames] | 每帧检测点迹 |
| `ukf_tpl` | 结构体 | UKF 模板（由 `ukf_xxx('create',...)` 产生） |
| `params` | 结构体 | 全局参数 |
| `n_frames` | int | 总帧数 |
| `varargin` | 可选 | `true_track`, `t_grid`（真值辅助起始） |

| 输出 | 类型 | 含义 |
|---|---|---|
| `trackSnapshots` | cell[n_frames] | 每帧航迹快照 |
| `finalTrack` | 结构体 | 最终航迹摘要 |

## 3. 逐语句块审查

### 3.1 可选参数解析（第 32—37 行）

**语句职责。** 从 `varargin` 提取真值辅助起始数据。[S]

**代码质量。** `length(varargin) >= 2` 检查合理，但 `varargin` 语义不明确——调用者必须知道第二个参数是 `t_grid`。建议改为命名参数或结构体输入。[S]

### 3.2 真值辅助首次起始（第 74—104 行）

**语句职责。** 若 `params.use_truth_init=true` 且提供了真值轨迹，从真值插值得到初始量测，直接跳过 M/N 起始。[S]

**数学正确性。**

```matlab
tl = interp1(true_track(:,5), true_track(:,1), t_grid(k), 'linear', 'extrap');
tb = interp1(true_track(:,5), true_track(:,2), t_grid(k), 'linear', 'extrap');
Rg = skywave_geometry('group_range', ..., tl, tb);
az = sphere_utils_azimuth(..., tl, tb);
```

两帧真值辅助量测构造初始 UKF 状态。[M]

**问题记录。** `V05c-P0-01`：这是 CR-ISSUE-015 的核心。`use_truth_init=true` 默认开启意味着 M/N 起始器从未被测试。所有 RMSE 指标建立在"完美开局"假设上，起始延迟、失败率、虚警起始等关键性能完全未评估。[S]

**问题记录。** `V05c-P1-03`：真值辅助起始中 `init_det1` 和 `init_det2` 通过 `interp1` 从真值轨迹插值得到。若 `t_grid(k)` 超出 `true_track(:,5)` 的时间范围，`'extrap'` 模式会产生外推值，可能严重失真。[M]

### 3.3 重新起始超时兜底（第 106—151 行）

**语句职责。** M/N 起始超时后，切换到真值辅助重新起始。[S]

**数学正确性。** `reinit_timeout_frames = max(4, params.tracker_N - 2) = max(4, 8-2) = 6`。M/N 起始 6 帧未成功后触发真值兜底。[M]

**问题记录。** `V05c-P1-04`：重新起始的超时兜底同样使用真值辅助，延续了 CR-ISSUE-015 的问题——即使 M/N 起始失败，最终也通过真值兜底"成功"。这掩盖了 M/N 起始器的真实失败率。[S]

### 3.4 纯 M/N 滑窗逻辑（第 153—169 行）

**语句职责。** 调用 `track_initiation('process', ...)` 执行 M/N 起始。[S]

**数学正确性。** 若 `track_initiation` 返回 `success=true`，切换到 TRACKING 状态。[M]

### 3.5 TRACKING 状态 — 杂波预筛（第 178—184 行）

**语句职责。** 过滤 `is_clutter=true` 的点迹。[S]

**代码质量。** `clean_dets = [clean_dets, dets(d)]` 动态数组增长。每帧最多 1.5 个期望杂波 + 1 个目标，影响可忽略。[S]

### 3.6 TRACKING 状态 — 滤波器预测（第 187 行）

**语句职责。** 调用 `ukf_dispatch('prepare', ukf)`。[S]

**性能。** 每次调用内部执行 9 次 `skywave_geometry`（见 V03 审查）。[S]

### 3.7 TRACKING 状态 — NN 关联（第 189—193 行）

**语句职责。** 调用 `nn_associate`，临时禁用 Vr 门（`gate_vr_ms=9999`）。[S]

**问题记录。** `V05c-P2-05`：`params.gate_vr_ms = 9999` 临时覆盖全局参数。若 `nn_associate` 在多线程或回调中访问 `params`，可能导致竞态条件。当前 MATLAB 单线程无此风险，但设计不优雅。[S]

**问题记录。** `V05c-P1-05`：Vr 门在正式运行中被硬编码禁用（`9999 m/s`）。注释称"临时禁用Vr门"，但为什么禁用？文档和代码均未说明。若 Vr 门有效（20/40 m/s），可能过滤掉机动目标的合法量测。[S]

### 3.8 TRACKING 状态 — 连续丢点防劫持（第 195—203 行）

**语句职责。** 连续丢点 ≥2 且地理距离 > 50 km 时拒绝关联。[S]

**数学正确性。** 50 km 门限合理：目标在 2 帧（60s）内飞行约 230×60 = 13.8 km，50 km 远大于此值。[M]

### 3.9 TRACKING 状态 — PDA 加权（第 207 行）

**语句职责。** 调用 `pda_weight` 计算加权新息。[S]

### 3.10 TRACKING 状态 — Probation 期保护（第 209—214 行）

**语句职责。** 生命期 ≤5 帧且 NIS > 50 时拒绝更新。[S]

**数学正确性。** `NIS=50` 对应 3D 卡方 `chi2cdf(50,3)≈1`，几乎不拒绝。若 NIS 是 2D（`chi2cdf(50,2)≈1`），同样几乎不拒绝。probation 期保护实际形同虚设。[M]

**问题记录。** `V05c-P2-06`：`probate_nis_limit = 50` 过高。2D 卡方 `chi2cdf(50,2) > 0.99999`，几乎所有新息都会被接受。若意图是保护初期航迹免受异常量测影响，阈值应设为 10-20。[M]

### 3.11 TRACKING 状态 — 滤波器更新（第 216—238 行）

**语句职责。** 有匹配量测时 PDA 更新，无匹配时纯预测。[S]

**数学正确性。** `ukf.life_count = life + 1` 补偿生命周期计数，使自适应 Q 在 `life>12` 后生效。[S]

**问题记录。** `V05c-P2-07`：`nis_history` 无限增长（第 226 行 `end+1` 追加）。在长时间运行场景（120 帧），`nis_history` 最多 120 个样本。自适应 Q 的 EMA 平滑窗口（`ema_eta=0.1`）等效记忆长度约 10 帧，长历史影响可忽略。但 `ukf_zishiying.m:180` 中 `innov_history` 也无限增长到 10 帧后截断，两者行为不一致。[S]

### 3.12 LOST 状态（第 256—264 行）

**语句职责。** 航迹丢失后重置为 INITIATING，触发重新起始。[S]

**数学正确性。** `reinit_attempt_frame = k` 记录丢失帧，后续超时检查基于此。[M]

### 3.13 `post_init` 辅助函数（第 281—300 行）

**语句职责。** UKF 初始化后设置通用字段。[S]

**代码质量。** 对 IMM 类型特殊处理（设置 `ukf_cv` 和 `ukf_ct` 的 `dt` 和 `initialized`）。[S]

### 3.14 `make_track_snap` 辅助函数（第 306—320 行）

**语句职责。** 构造航迹快照结构体。[S]

**代码质量。** 始终设置 `trk.x_pred = []` 和 `trk.P_pred = []`，即使 `det` 非空。这些字段在后续处理中可能未被使用。[S]

### 3.15 `iif` 辅助函数（第 326—328 行）

**语句职责。** 条件表达式简化。[S]

**代码质量。** 重复实现 MATLAB 内置 `if-else` 的功能，不如直接使用三元表达式。[S]

## 4. 文件级问题汇总

| Issue ID | 位置 | 严重度 | 类别 | 证据 | 状态 |
|---|---:|---|---|---|---|
| V05c-P0-01 | 第 74 行 | P0 | 真值辅助起始绕过 M/N | S | OPEN |
| V05c-P1-03 | 第 76-77 行 | P1 | 真值外推风险 | M | OPEN |
| V05c-P1-04 | 第 106-151 行 | P1 | 重新起始真值兜底掩盖 M/N 失败 | S | OPEN |
| V05c-P1-05 | 第 191 行 | P1 | Vr 门硬编码禁用 | S | OPEN |
| V05c-P2-05 | 第 190-193 行 | P2 | 全局参数临时覆盖 | S | OPEN |
| V05c-P2-06 | 第 210 行 | P2 | probation NIS 门槛过高 | M | OPEN |
| V05c-P2-07 | 第 226 行 | P2 | nis_history 无限增长 | S | OPEN |

## 5. 文件级结论

- **数学可信度**：状态机逻辑完整（INITIATING→TRACKING→LOST→INITIATING），但真值辅助起始默认开启使 M/N 起始器从未被真实测试。Vr 门被硬编码禁用，probation NIS 门槛过高形同虚设。
- **代码质量**：`varargin` 语义不明确；全局参数临时覆盖设计不优雅；`iif` 函数多余。
- **性能**：每帧 9 次 `skywave_geometry` 调用（prepare）+ 1 次（真值辅助起始），是主要热点。
- **测试充分性**：端到端冒烟通过，但 M/N 起始器在无真值辅助下的性能未获独立验证。
- **剩余未验证项**：关闭真值辅助后 M/N 起始的成功率和延迟；Vr 门启用/禁用对关联率的影响。

## 6. 下一审查游标

- 文件：`ukf/ukf_dispatch.m`（路由调度器）
- 重点：create/init/prepare/update 路由逻辑
- 稳定指纹：`exist('ukf_cv', 'var')` 判断 IMM
