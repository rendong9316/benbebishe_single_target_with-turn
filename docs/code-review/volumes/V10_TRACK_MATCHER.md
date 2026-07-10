# V10 `fusion/track_matcher.m` 逐语句块代码审查

## 1. 审查元数据

| 字段 | 值 |
|---|---|
| 审查对象 | `fusion/track_matcher.m` |
| 覆盖范围 | 第 1—436 行 |
| 源码基线 | `HEAD 7c166d41541ccd74f23fd6c3ea0b871d8603950e` |
| 工作树文件哈希 | `ddf78a62718ef539` |
| 审查方式 | 只读静态源码核对、公式推导、仓库调用点交叉核对 |
| 运行状态 | 被 `run_simulation_multi.m` Phase 7 调用 |
| 修改范围 | 仅新增本审查文档 |

## 2. 文件职责与接口契约

### 2.1 职责

跨雷达航迹时空配对模块：将 R1 和 R2 两部雷达的航迹列表按时空接近度进行配对，输出 `matched_pairs` 数组，供 `run_track_fusion` 使用。采用多维特征（距离 + 速度 + 航向）和全局最优分配（匈牙利算法的枚举/贪心近似）。[S][P]

### 2.2 输入输出

| 输入 | 类型 | 含义 |
|---|---|---|
| `trackSnapshots_R1` | [n_frames x 1] cell | R1 各帧航迹快照 |
| `trackSnapshots_R2` | [n_frames x 1] cell | R2 各帧航迹快照（已时间对齐） |
| `params` | struct | 参数结构体（当前未被使用） |

| 输出 | 类型 | 含义 |
|---|---|---|
| `matched_pairs` | struct array | 配对结果，含 ID、共现帧数、平均距离、质量评分等 |

### 2.3 调用关系

被 `run_simulation_multi.m` 的 Phase 7 调用，在融合前确定 R1/R2 航迹的一对一映射。[S]

## 3. 逐语句块审查

### 3.1 主函数签名与参数初始化（第 37—45 行）

```matlab
function matched_pairs = track_matcher(trackSnapshots_R1, trackSnapshots_R2, params)
n_frames = length(trackSnapshots_R1);
coexist_thresh = 5;  % 最少共现帧数
dist_thresh_km = 50; % 距离门限（km）
w_dist = 0.6;
w_speed = 0.25;
w_heading = 0.15;
```

**语句职责。** 设置配对的全局阈值和权重配置。[S]

**问题记录。** `V10-P2-01`：`params` 参数传入但未被使用。权重和门限硬编码在函数体内，而非从 `params` 读取。这违反了参数化设计原则，使得同一模块在不同场景下无法调整阈值。若后续需要敏感性分析，必须修改源码。[S]

**问题记录。** `V10-P2-02`：`coexist_thresh = 5` 表示航迹必须在至少 5 帧中共现才被视为有效配对。对于 OTH-SWR 场景（典型帧间隔 10-60 秒，总帧数可能数百），5 帧仅表示 50-300 秒的共现。但若某条真实航迹在早期帧被丢失（如转弯时），此阈值会过滤掉真实配对。[M]

**问题记录。** `V10-P2-03`：`dist_thresh_km = 50` 表示距离门限 50 km。对于 OTH-SWR（目标距离 1000-2000 km），50 km 的门限相对宽松，但考虑到两部雷达的位置分离（基线可能数百 km），同一目标的两个独立估计之间的偏差可能超过 50 km。[M]

### 3.2 逐帧特征提取（第 47—54 行）

```matlab
r1_active = cell(n_frames, 1);
r2_active = cell(n_frames, 1);
for k = 1:n_frames
    r1_active{k} = extract_track_features(trackSnapshots_R1{k});
    r2_active{k} = extract_track_features(trackSnapshots_R2{k});
end
```

**语句职责。** 从每帧快照中提取活跃航迹的特征（位置、速度、航向）。[S]

**性能。** 两次循环：一次提取特征，一次做配对。可合并为单次循环。[S]

**问题记录。** `V10-P2-04`：`extract_track_features` 返回的是 cell 数组（见第 191—225 行），每个元素是一个 struct。`r1_active{k}` 是 cell 而非 struct array。后续 `length(r1_tracks)` 和 `r1_tracks{i}` 的使用是一致的，但类型注释（第 47-48 行）说"活跃航迹ID"，实际返回的是包含多个 struct 的 cell。[S]

### 3.3 逐帧配对与统计累积（第 56—120 行）

```matlab
pair_stats = containers.Map('KeyType', 'char', 'ValueType', 'any');
for k = 1:n_frames
    ...
    cost_matrix = zeros(n_r1, n_r2);
    for i = 1:n_r1
        for j = 1:n_r2
            cost_matrix(i, j) = compute_match_cost(...);
        end
    end
    ...
```

**语句职责。** 对每帧计算代价矩阵，然后用全局最优分配（枚举或贪心）得到配对，累积配对统计信息。[S]

**数学正确性。** 代价函数 `compute_match_cost` 是距离、速度、航向的加权和，权重和为 1.0（0.6 + 0.25 + 0.15 = 1.0）。[M]

**问题记录。** `V10-P1-01`：`perms(cols)` 的时间复杂度为 `O(n_col!)`。当 `n_col = 10` 时，`10! = 3,628,800`，枚举法完全不可行。代码在第 78 行用 `n_r1 <= 4 && n_r2 <= 4` 限制了枚举规模（`4! = 24`），这是合理的。但当航迹数超过 4 时，切换到贪心算法，贪心算法不是全局最优，可能在密集场景中产生次优配对。[S]

**问题记录。** `V10-P1-02`：`containers.Map` 的 key 是 `sprintf('%d_%d', r1_id, r2_id)` 生成的字符串。若航迹 ID 是浮点数（如 `1.0` 和 `1` 的字符串表示不同），会导致 key 不一致。需确认 `trk.id` 的类型始终是整数。[S]

**问题记录。** `V10-P2-05`：第 117 行 `s.frames = [s.frames, k]` 是动态数组增长。在 MATLAB 中，cell 内的 struct 字段更新（`s = pair_stats(key); ... pair_stats(key) = s;`）涉及拷贝，效率较低。对于 N_frames = 200 的场景，累积开销可感知。[S]

**问题记录。** `V10-P2-06`：`compute_match_cost` 内部再次调用 `sphere_utils_haversine_distance`（第 232 行），而主函数在第 94 行也调用了相同的函数计算 `dist_km`。同一距离被计算两次。[S]

### 3.4 距离门限过滤（第 93—96 行）

```matlab
dist_km = sphere_utils_haversine_distance(r1_tracks{i}.lon, r1_tracks{i}.lat, ...
    r2_tracks{j}.lon, r2_tracks{j}.lat) / 1000;
if dist_km > dist_thresh_km, continue; end
```

**语句职责。** 在分配结果出来后，用 50 km 距离门限过滤配对。[S]

**问题记录。** `V10-P1-03`：`sphere_utils_haversine_distance` 返回米，除以 1000 得到 km。但 `dist_thresh_km = 50` 是硬编码的。在 OTH-SWR 场景下，两部雷达对同一目标的独立估计之间的偏差可能达到数十 km（取决于雷达位置分离和量测噪声），50 km 是否合理需场景验证。[M]

**问题记录。** `V10-P2-07`：距离门限过滤在分配**之后**进行，意味着匈牙利分配可能已经将一个远距离配对选为"最优"（因为所有候选都参与了代价计算）。正确的做法应该是在代价矩阵中将超出门限的配对设为 `inf`，让分配算法自然排除。[S]

### 3.5 质量评分计算（第 154—164 行）

```matlab
dist_score = max(0, 100 - mean_dist * 2);
speed_score = max(0, 100 - mean_speed_diff * 2);
heading_score = max(0, 100 - mean_heading_diff * 1.1);
coexist_score = min(100, max_coexist * 5);
quality = 0.5 * dist_score + 0.2 * speed_score + 0.15 * heading_score + 0.15 * coexist_score;
```

**语句职责。** 综合距离、速度、航向、共现帧数四个维度给出配对质量评分（0-100）。[S]

**数学正确性。** 各评分分量线性衰减，权重和为 1.0（0.5 + 0.2 + 0.15 + 0.15 = 1.0）。[M]

**问题记录。** `V10-P2-08`：质量评分中，`dist_score` 的斜率为 2（每 km 降 2 分），`speed_score` 的斜率也为 2（每 m/s 降 2 分），`heading_score` 的斜率为 1.1（每度降 1.1 分）。这些斜率的量纲不同（km vs m/s vs deg），直接比较数值大小无物理意义。例如，mean_dist = 10 km 时 dist_score = 80，mean_speed_diff = 10 m/s 时 speed_score = 80，但 10 km 的距离偏差和 10 m/s 的速度偏差在航迹配对中的重要性不同。[M]

**问题记录。** `V10-P2-09`：`coexist_score = min(100, max_coexist * 5)` 意味着共现 20 帧即达到满分 100。对于典型的 MC 场景（数百帧），20 帧的共现并不罕见，此评分可能区分度不足。[S]

### 3.6 全局最优配对选择（第 178—180 行）

```matlab
matched_pairs = select_optimal_pairs(candidates);
```

**语句职责。** 从候选配对中选择一一对应的最优集合（每对 R1 ID 最多匹配一个 R2 ID）。[S]

**算法正确性。** `select_optimal_pairs` 采用贪心策略：按质量降序排序，依次选择不冲突的配对。这不是全局最优（可能存在局部高质量配对互相冲突的情况），但计算效率高 `O(n log n)`。[M]

**问题记录。** `V10-P2-10`：贪心选择的顺序依赖质量评分的精度。若质量评分计算有偏差（见 V10-P2-08），贪心结果可能不是最优的。[S]

### 3.7 兜底启发式匹配（第 182—185 行）

```matlab
if isempty(matched_pairs)
    matched_pairs = heuristic_match(r1_active, r2_active, n_frames, coexist_thresh);
end
```

**语句职责。** 若主流程未找到有效配对，使用基于共现帧数的启发式方法兜底。[S]

**问题记录。** `V10-P3-04`：`heuristic_match` 完全不使用位置信息，仅基于共现帧的重叠度进行配对。在复杂场景（多目标、航迹交叉）中，此方法的准确率极低。但作为兜底策略，其价值在于"总比没有好"。[S]

### 3.8 `extract_track_features` 子函数（第 191—225 行）

```matlab
v_lon_ms = v_lon * 111000 * cos(lat_rad);
v_lat_ms = v_lat * 111000;
speed = sqrt(v_lon_ms^2 + v_lat_ms^2);
heading = atan2(v_lon_ms, v_lat_ms) * 180 / pi;
```

**语句职责。** 从 UKF 状态中提取经纬度、速度（度/秒 → m/s）、航向。[S]

**数学正确性。** 速度转换：1 度纬度 ≈ 111 km，1 度经度 ≈ 111 × cos(lat) km。公式正确。[M]

**问题记录。** `V10-P1-04`：速度单位是**度/秒**（来自 UKF 状态 `x = [lon; lon_dot; lat; lat_dot]`），乘以 `111000 * cos(lat_rad)` 转换为 m/s。但 UKF 的状态定义在 `ukf_jichu.m:116-143` 中是 `[lon; lon_dot; lat; lat_dot]`，其中 `lon_dot` 和 `lat_dot` 的单位是**弧度/秒**还是**度/秒**？若 UKF 内部使用弧度，则此处的转换系数 `111000` 是错误的（应先用 `180/pi` 将弧度转为度）。[M]

**问题记录。** `V10-P1-05`：航向计算 `atan2(v_lon_ms, v_lat_ms)` 返回的是数学角度（从 x 轴正向逆时针），但航空/航海惯例中航向是从正北（y 轴正向）顺时针。代码中 `atan2(v_lon, v_lat)` 实际上是将 v_lat 作为"北"分量、v_lon 作为"东"分量，这与 `atan2(East, North)` 的航空惯例一致。结果再转为度并归一化到 [0°, 360°)。[M]

**问题记录。** `V10-P2-11`：`trk.type ~= 7` 的含义不明。若 `type == 7` 表示某种特殊类型的航迹（如虚航迹、测试航迹），应添加注释说明。[S]

**问题记录。** `V10-P2-12`：速度转换系数 `111000` 是近似值（地球半径取 6371 km 时，1 度纬度 = 2π × 6371 / 360 ≈ 111.195 km）。使用 111000 引入约 0.18% 的误差，在航迹配对的 50 km 门限内可忽略。[M]

### 3.9 `compute_match_cost` 子函数（第 230—246 行）

```matlab
dist_cost = min(100, dist_km * 2);
speed_cost = min(100, speed_diff * 2);
heading_cost = min(100, heading_diff * 1.1);
cost = w_dist * dist_cost + w_speed * speed_cost + w_heading * heading_cost;
```

**语句职责。** 计算两个航迹的匹配代价（越小越相似）。[S]

**数学正确性。** 各分量线性归一化到 [0, 100]，加权求和。[M]

**问题记录。** `V10-P2-13`：`dist_cost` 在 `dist_km = 50` 时达到 100（饱和），`speed_cost` 在 `speed_diff = 50 m/s` 时饱和，`heading_cost` 在 `heading_diff ≈ 90.9°` 时饱和。这些饱和点的选择缺乏理论依据，似乎是经验值。[S]

### 3.10 `optimal_assignment_enum` 子函数（第 266—307 行）

```matlab
perms_list = perms(cols);
for p = 1:n_perms
    perm = perms_list(p, 1:n_row);
    total_cost = 0;
    for i = 1:n_row
        total_cost = total_cost + cost_matrix(i, perm(i));
    end
```

**语句职责。** 枚举所有排列，寻找总代价最小的分配。[S]

**性能。** 时间复杂度 `O(n_row! × n_row)`。当 `n_row = 4` 时，`4! × 4 = 96` 次操作，可接受。当 `n_row = 10` 时，`10! × 10 ≈ 3.6 × 10^7`，不可接受。代码通过 `n_r1 <= 4 && n_r2 <= 4` 的限制规避了此问题。[S]

**问题记录。** `V10-P3-05`：`perms(cols)` 在 MATLAB R2023a 中对 `cols = 1:10` 会产生 `3,628,800` 行矩阵，占用约 290 MB 内存。即使 `n_row <= 4` 的限制存在，若未来有人移除此限制，此函数会导致 OOM。建议使用 `assignement` 算法（如 KM 算法或 `matchpairs` 内置函数）。[S]

### 3.11 `greedy_assignment` 子函数（第 312—339 行）

```matlab
[~, order] = sort(costs);
for k = 1:length(order)
    i = rows(order(k));
    j = cols(order(k));
    if assignment(i) == 0 && ~used_col(j)
        assignment(i) = j;
        used_col(j) = true;
    end
end
```

**语句职责。** 贪心分配：按代价从小到大排序所有配对，依次选择不冲突的配对。[S]

**算法正确性。** 贪心策略不是全局最优，但对于大多数航迹配对场景（航迹数少、距离差异明显）表现良好。[M]

**问题记录。** `V10-P2-14`：贪心算法在"代价相近"的场景中可能产生次优解。例如，R1-A 与 R2-B 的代价为 10，R1-B 与 R2-A 的代价也为 10，贪心可能选择前者而放弃后者（因为排序顺序不确定）。[M]

### 3.12 `select_optimal_pairs` 子函数（第 344—374 行）

```matlab
qualities = [candidates.quality];
[~, order] = sort(qualities, 'descend');
for k = 1:n
    c = candidates(order(k));
    if ~ismember(c.R1_track_id, used_r1) && ~ismember(c.R2_track_id, used_r2)
```

**语句职责。** 按质量降序选择配对，排除已使用的 ID。[S]

**性能。** `ismember` 在循环内调用，时间复杂度 `O(n × m)`，其中 `n` 是候选数，`m` 是已用 ID 数。可用 `containers.Map` 或 logical 数组优化到 `O(n)`。[S]

**问题记录。** `V10-P3-06`：`ismember` 的返回值是 logical 标量，在 ID 为整数的情况下正确。但若 ID 为浮点数，`ismember` 的相等比较可能因精度问题失败。[S]

### 3.13 `heuristic_match` 子函数（第 379—435 行）

```matlab
overlap = sum(ismember(frames1, frames2));
```

**语句职责。** 基于共现帧重叠度的启发式配对。[S]

**问题记录。** `V10-P3-07`：`ismember(frames1, frames2)` 假设帧号是整数且无重复。在正常情况下帧号是 `1, 2, ..., n_frames`，此假设成立。但若某些帧被跳过（如检测失败），帧号可能不连续，`ismember` 仍然正确（因为比较的是值而非索引）。[S]

## 4. 文件级问题汇总

| Issue ID | 位置 | 严重度 | 类别 | 证据 | 状态 |
|---|---:|---|---|---|---|
| V10-P1-01 | 第 78 行 | P1 | 枚举法阶乘复杂度 | S | OPEN |
| V10-P1-02 | 第 98 行 | P1 | containers.Map key 类型假设 | S | OPEN |
| V10-P1-03 | 第 93—96 行 | P1 | 距离门限在分配后过滤 | S | OPEN |
| V10-P1-04 | 第 207—208 行 | P1 | UKF 速度单位假设（弧度 vs 度） | M | OPEN |
| V10-P1-05 | 第 212 行 | P1 | 航向计算与航空惯例一致性 | M | OPEN |
| V10-P2-01 | 第 37—45 行 | P2 | params 未使用，阈值硬编码 | S | OPEN |
| V10-P2-02 | 第 39 行 | P2 | coexist_thresh = 5 的场景依赖性 | M | OPEN |
| V10-P2-03 | 第 40 行 | P2 | dist_thresh_km = 50 的场景依赖性 | M | OPEN |
| V10-P2-04 | 第 51—53 行 | P2 | 返回值类型注释不清 | S | OPEN |
| V10-P2-05 | 第 117 行 | P2 | 动态数组增长 + struct 拷贝 | S | OPEN |
| V10-P2-06 | 第 94 行 vs 232 行 | P2 | 距离重复计算 | S | OPEN |
| V10-P2-07 | 第 93—96 行 | P2 | 距离门限应在代价矩阵中施加 | S | OPEN |
| V10-P2-08 | 第 154—164 行 | P2 | 质量评分量纲不一致 | M | OPEN |
| V10-P2-09 | 第 162 行 | P2 | coexist_score 区分度不足 | S | OPEN |
| V10-P2-10 | 第 344—374 行 | P2 | 贪心选择依赖质量评分精度 | S | OPEN |
| V10-P2-11 | 第 196 行 | P2 | trk.type ~= 7 含义不明 | S | OPEN |
| V10-P2-12 | 第 207—208 行 | P2 | 111000 近似误差 | M | OPEN |
| V10-P2-13 | 第 233—235 行 | P2 | 饱和点选择无理论依据 | S | OPEN |
| V10-P2-14 | 第 312—339 行 | P2 | 贪心在代价相近时次优 | M | OPEN |
| V10-P2-15 | 第 354 行 | P3 | ismember 浮点精度风险 | S | OPEN |
| V10-P3-01 | 第 266—307 行 | P3 | perms OOM 风险 | S | OPEN |
| V10-P3-02 | 第 1—35 行 | P3 | 注释中"匈牙利算法"不准确 | S | OPEN |
| V10-P3-03 | 第 379—435 行 | P3 | 启发式兜底完全不使用位置信息 | S | OPEN |
| V10-P3-04 | 第 182—185 行 | P3 | heuristic_match 兜底策略质量低 | S | OPEN |
| V10-P3-05 | 第 357—373 行 | P3 | ismember 在循环内性能低 | S | OPEN |

## 5. 文件级结论

- **数学可信度**：核心算法（多维代价 + 全局最优分配）在数学上是合理的。但枚举法仅限于小规模（≤ 4），大规模退化为贪心，不能保证全局最优。质量评分的量纲混合是主要数学缺陷。[M]
- **代码质量**：模块化良好，子函数职责清晰。但 params 未使用、阈值硬编码、注释中"匈牙利算法"表述不准确（实际是枚举/贪心，非真正的匈牙利/KM 算法）等问题降低了代码质量。[S]
- **性能**：`perms` 在 `n > 10` 时 OOM；`ismember` 在循环内 `O(nm)`；动态数组增长。在航迹数少的场景下影响可忽略，但代码缺乏可扩展性。[S]
- **测试充分性**：未找到针对 `track_matcher` 的单元测试。多目标场景下的配对准确率、ID switch 率未量化。[S]
- **剩余未验证项**：UKF 速度槽位的单位（弧度/秒 vs 度/秒）；质量评分在不同场景下的区分度；50 km 距离门限对配对准确率的影响。

## 6. 下一审查游标

- 文件：`tracker/post_init_multi.m`（多目标初始化辅助）
- 重点：初始化逻辑、参数传递
- 稳定指纹：`post_init_multi` 函数签名
