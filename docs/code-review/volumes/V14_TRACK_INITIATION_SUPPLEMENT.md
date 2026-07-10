# V14 `initiation/track_initiation.m` 逐语句块代码审查

## 1. 审查元数据

| 字段 | 值 |
|---|---|
| 审查对象 | `initiation/track_initiation.m` |
| 覆盖范围 | 第 1—147 行 |
| 源码基线 | `HEAD 7c166d41541ccd74f23fd6c3ea0b871d8603950e` |
| 工作树文件哈希 | `7c92f315fb34ea4b` |
| 审查方式 | 只读静态源码核对、公式推导、仓库调用点交叉核对 |
| 运行状态 | 被 `single_track_runner.m` 和 `run_simulation_turn.m` 调用 |
| 修改范围 | 仅新增本审查文档 |

> 注：V05b 已审查 `track_initiation.m` 的核心 M/N 逻辑。本文档补充 V05b 未覆盖的细节（action dispatcher、边界条件、南阳变体差异）。

## 2. 文件职责与接口契约

### 2.1 职责

纯过程化 M/N 滑窗航迹起始器：维护长度为 N 的滑窗，记录每帧点迹；当滑窗内 ≥ M 帧有点迹且当前帧有点迹时，尝试多假设配对 → 速度检验 → 共识评分 → 最佳起始。[S][P]

### 2.2 输入输出（init）

| 输入 | 类型 | 含义 |
|---|---|---|
| `params` | struct | 需含 `.tracker_N`、`.tracker_M`、`.dt_sec` |

| 输出 | 类型 | 含义 |
|---|---|---|
| `state` | struct | 滑窗状态 |

### 2.3 输入输出（process）

| 输入 | 类型 | 含义 |
|---|---|---|
| `state` | struct | 滑窗状态 |
| `dets` | struct array | 当前帧点迹 |
| `params` | struct | 参数 |
| `frame_id` | scalar | 帧编号（未使用） |

| 输出 | 类型 | 含义 |
|---|---|---|
| `state` | struct | 更新后的滑窗状态 |
| `det1` | struct | 配对中的早期点迹 |
| `det2` | struct | 配对中的当前帧点迹 |
| `success` | logical | 是否成功起始 |

## 3. 逐语句块审查

### 3.1 Action Dispatcher（第 27—41 行）

```matlab
function varargout = track_initiation(action, varargin)
switch action
    case 'init'
        [varargout{1}] = init_state(varargin{1});
    case 'process'
        [varargout{1:nargout}] = process_frame(varargin{:});
    case 'reset'
        [varargout{1}] = init_state(varargin{1});
    otherwise
        error('track_initiation: unknown action "%s"', action);
end
end
```

**语句职责。** 三种 action：`init` 初始化、`process` 逐帧处理、`reset` 重置（等价于 init）。[S]

**问题记录。** `V14-P2-01`：`reset` 等价于 `init`，但语义不同。`init` 是首次初始化（滑窗为空），`reset` 是跟踪失败后的重置（可能需保留历史统计信息如失败次数）。当前实现中两者完全等价，丢失了"重置"的上下文。[S]

**问题记录。** `V14-P3-01`：`process` 分支使用 `[varargout{1:nargout}] = process_frame(...)` 返回可变数量输出。若调用者只请求 1 个输出（`state = track_initiation('process', ...)`），`process_frame` 也只返回 1 个值；若请求 4 个输出，则全部返回。MATLAB 的 `nargout` 在函数内部可查，但在 `varargout` 赋值中需小心——此处 `varargout{1:nargout}` 在 dispatcher 层面，`nargout` 是调用 `track_initiation` 时的输出数，正确。[S]

### 3.2 `init_state`（第 47—52 行）

```matlab
function state = init_state(params)
    state.window = {};
    state.has_det = [];
    state.N = params.tracker_N;
    state.M = params.tracker_M;
end
```

**语句职责。** 初始化空滑窗和检测标记数组。[S]

**数学正确性。** `window = {}` 是空 cell 数组，`has_det = []` 是空 double 数组。两者在 `end+1` 追加时类型兼容。[M]

**问题记录。** `V14-P2-02`：`state.window` 是 cell 数组（每个元素是 struct array 的点迹），`state.has_det` 是 logical 数组（每个元素是对应帧是否有检测）。两者的长度应始终一致（`length(state.window) == length(state.has_det)`）。但代码在滑动窗口淘汰时（第 82—84 行）同步删除两者的第一个元素，若不同步会导致不一致。[S]

### 3.3 滑窗管理（第 77—84 行）

```matlab
state.window{end+1} = dets;
state.has_det(end+1) = ~isempty(dets);
if length(state.window) > state.N
    state.window(1) = [];
    state.has_det(1) = [];
end
```

**语句职责。** 追加当前帧点迹到滑窗尾部，淘汰最老帧以保持窗长 N。[S]

**性能。** `state.window(1) = []` 删除 cell 数组的第一个元素，需要移动剩余 N-1 个元素。当 N 较小时（典型值 5-10）开销可忽略。[S]

**问题记录。** `V14-P2-03`：`state.window(1) = []` 删除单个 cell 元素，但 `state.has_det(1) = []` 删除单个 logical 元素。若 `state.has_det` 是 logical 数组，删除元素后数组缩短；若 `state.has_det` 是 cell 数组（类型不一致），MATLAB 会报错。此处 `has_det` 是 logical 数组，行为正确。[M]

### 3.4 M/N 条件检查（第 86—90 行）

```matlab
n_with_det = sum(state.has_det);
if n_with_det < state.M || isempty(dets)
    return;
end
```

**语句职责。** 滑窗内有检测的帧数 < M 或当前帧无检测时，跳起始尝试。[S]

**数学正确性。** M/N 起始条件：N 帧窗内至少 M 帧有检测，且当前帧必须有检测（否则无新点迹可配对）。[M]

**问题记录。** `V14-P1-01`：已在 V05b 中登记 CR-ISSUE-033，共识评分假设直线运动。在转弯场景中，历史帧的点迹与当前帧点迹的 Haversine 距离可能超出 80 km 门限（第 126 行），导致共识评分为 0，起始失败。[S+M]

### 3.5 多假设配对（第 92—139 行）

```matlab
for curr_idx = 1:length(dets)
    for i = 1:(length(state.window)-1)
        prev_dets = state.window{i};
        for p = 1:length(prev_dets)
            dp = prev_dets(p);
            dc = dets(curr_idx);
```

**语句职责。** 三重循环：当前帧点迹 × 历史帧索引 × 历史帧点迹。[S]

**性能。** 时间复杂度 `O(|dets_current| × N × |dets_history|)`。若每帧有 K 个点迹，总配对尝试数为 `K² × N`。当 K = 10、N = 10 时，每帧 1000 次配对尝试。[S]

**问题记录。** `V14-P2-04`：三重循环无法向量化（因为每对点迹的距离和速度检验是独立的）。在 MC 循环中，若每帧都执行此逻辑，累积开销可感知。[S]

### 3.6 速度检验（第 108—113 行）

```matlab
dist = sphere_utils_haversine_distance(dp.lon, dp.lat, dc.lon, dc.lat);
dt_frames = length(state.window) - i;
est_speed = dist / (dt_frames * params.dt_sec);
if est_speed < 30 || est_speed > 600
    continue;
end
```

**语句职责。** 通过两点间距离和时间差估计速度，过滤不合理值（30-600 m/s）。[S]

**数学正确性。** `est_speed = Δdistance / Δtime` 是平均速度。对于 OTH-SWR 目标（民航客机典型速度 200-250 m/s，战斗机 300-500 m/s），30-600 m/s 的范围覆盖了大部分场景。[M]

**问题记录。** `V14-P1-02`：两点间的 Haversine 距离是**地表大圆距离**，但 OTH-SWR 的目标位置是通过天波量测（群距离 + 方位角）反解得到的。反解误差（见 V02b-P1-01，~16% 的系统偏差）会直接影响 `dist` 的计算，进而影响速度估计。在转弯场景中，此误差可能导致速度估计偏离真实值 16% 以上，使有效速度范围缩小到约 25-700 m/s（而非 30-600 m/s）。[M]

**问题记录。** `V14-P2-05`：`dt_frames = length(state.window) - i` 计算的是当前帧与历史帧之间的**帧数差**。若滑窗中某些帧被跳过（无检测），`dt_frames` 仍然基于滑窗索引而非实际帧号。例如，滑窗长度为 5，当前帧索引为 5，历史帧索引为 2，则 `dt_frames = 5-2 = 3` 帧。但如果第 3 帧无检测被跳过，实际时间差仍然是 `3 × dt_sec`（因为滑窗是连续的帧索引）。此逻辑正确的前提是"滑窗索引 = 帧序号"，即每帧都调用 `process_frame`。若某帧完全跳过（如传感器故障），滑窗索引会断裂。[M]

### 3.7 共识评分（第 115—130 行）

```matlab
support = 0;
for jj = 1:(length(state.window)-1)
    if jj == i, continue; end
    other = state.window{jj};
    for oo = 1:length(other)
        do = other(oo);
        d1 = sphere_utils_haversine_distance(dp.lon, dp.lat, do.lon, do.lat);
        d2 = sphere_utils_haversine_distance(dc.lon, dc.lat, do.lon, do.lat);
        if d1 < 80000 && d2 < 80000
            support = support + 1;
        end
    end
end
```

**语句职责。** 对每对候选点迹 (dp, dc)，统计其他历史帧中有多少点迹同时靠近 dp 和 dc 的延伸轨迹。[S]

**数学正确性。** 共识评分的本质是"三角验证"：若 dp 和 dc 属于同一目标的连续轨迹，则其他帧中该目标的位置应同时靠近 dp 和 dc 的外推位置。[M]

**问题记录。** `V14-P1-03`：已在 V05b 中登记 CR-ISSUE-033，共识评分使用**直线假设**（两点间 Haversine 距离 < 80 km）。在转弯场景中，目标实际沿弧线运动，直线外推会偏离真实位置。转弯半径 R = v / ω，对于 v = 250 m/s、ω = 3°/s = 0.052 rad/s，R ≈ 4800 m = 4.8 km。在 80 km 的门限下，4.8 km 的偏差（10%）仍可接受。但当转弯更剧烈（ω = 10°/s）时，R ≈ 1.4 km，偏差更大，直线假设失效。[M]

**问题记录。** `V14-P2-06`：`d1 < 80000 && d2 < 80000` 中的 `80000` 是硬编码的 80 km 门限（米）。此值未在注释中说明单位，也不从 `params` 读取。若场景尺度变化（如近距 vs 远距 OTH-SWR），此门限可能需要调整。[S]

**问题记录。** `V14-P2-07`：共识评分的双重循环（`jj` 遍历历史帧，`oo` 遍历帧内点迹）的时间复杂度为 `O(N × K)`，其中 N 是窗长，K 是平均每帧点迹数。在最坏情况下（每帧都有大量杂波点迹），此评分计算可能成为性能瓶颈。[S]

### 3.8 最佳配对选择（第 132—146 行）

```matlab
if support > best_support
    best_support = support;
    best_prev = dp;
    best_curr_idx = curr_idx;
end
...
if best_support >= 1
    det1 = best_prev;
    det2 = dets(best_curr_idx);
    success = true;
end
```

**语句职责。** 选择共识评分最高的配对，评分 ≥ 1 即成功起始。[S]

**数学正确性。** 共识评分 ≥ 1 意味着至少有一帧的其他点迹同时靠近两条配对轨迹，提供了额外的位置一致性验证。[M]

**问题记录。** `V14-P2-08`：`best_support` 初始化为 -1，`support > best_support` 意味着即使 `support = 0` 也能触发更新（因为 0 > -1）。但最终的判定条件是 `best_support >= 1`（第 142 行），所以 `support = 0` 的配对不会被接受。逻辑一致，但 `best_support = -1` 的初始值可能引起混淆（为什么不是 0？）。[S]

## 4. 文件级问题汇总

| Issue ID | 位置 | 严重度 | 类别 | 证据 | 状态 |
|---|---:|---|---|---|---|
| V14-P1-01 | 第 86—90 行 | P1 | 直线假设转弯场景共识评分失效 | M | OPEN |
| V14-P1-02 | 第 108—113 行 | P1 | 反解偏差影响速度估计 | M | OPEN |
| V14-P1-03 | 第 115—130 行 | P1 | 共识评分直线假设转弯失效 | M | OPEN |
| V14-P2-01 | 第 35—36 行 | P2 | reset 与 init 等价 | S | OPEN |
| V14-P2-02 | 第 47—52 行 | P2 | window/has_det 长度一致性假设 | S | OPEN |
| V14-P2-03 | 第 82—84 行 | P2 | cell 数组删除元素的性能 | S | OPEN |
| V14-P2-04 | 第 97—139 行 | P2 | 三重循环无法向量化 | S | OPEN |
| V14-P2-05 | 第 109 行 | P2 | dt_frames 基于滑窗索引非实际帧号 | M | OPEN |
| V14-P2-06 | 第 126 行 | P2 | 80000 硬编码无注释 | S | OPEN |
| V14-P2-07 | 第 117—130 行 | P2 | 共识评分 O(N×K) 复杂度 | S | OPEN |
| V14-P2-08 | 第 132 行 | P3 | best_support=-1 初始值语义不清 | S | OPEN |
| V14-P3-01 | 第 27—41 行 | P3 | reset 语义丢失 | S | OPEN |

## 5. 文件级结论

- **数学可信度**：M/N 起始算法在数学上是标准的滑窗检测 + 共识评分框架。核心缺陷是直线假设在转弯场景下失效（已在 V05b 中详细分析）。反解偏差（~16%）通过 Haversine 距离传播到速度估计和共识评分。[M]
- **代码质量**：action dispatcher 模式清晰，三重循环可读性好但性能不佳。硬编码参数（80 km、30-600 m/s）缺乏参数化。[S]
- **性能**：三重循环 `O(K² × N)` + 共识评分 `O(N × K)`，每帧总复杂度 `O(K²N + NK) = O(KN(K+1))`。在 K=10、N=10 时约 1100 次操作/帧。[S]
- **测试充分性**：转弯场景下的起始成功率未量化。[S]
- **剩余未验证项**：不同转弯率下的起始失败率；80 km 门限对共识评分选择性的影响。

## 6. 下一审查游标

- 文件：`ukf/ukf_jichu.m` 的 `prepare` 子函数（Sigma 点传播）
- 重点：UKF 预测步的数学细节、信息矩阵
- 稳定指纹：`ukf_jichu` prepare 分支
