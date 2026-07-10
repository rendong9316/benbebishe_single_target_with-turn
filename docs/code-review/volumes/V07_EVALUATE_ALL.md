# V07 `evaluation/evaluate_all.m` 逐语句块代码审查

## 1. 审查元数据

| 字段 | 值 |
|---|---|
| 审查对象 | `evaluation/evaluate_all.m` |
| 覆盖范围 | 第 1—362 行 |
| 源码基线 | `HEAD 7c166d41541ccd74f23fd6c3ea0b871d8603950e` |
| 工作树文件哈希 | `526880ca08561f4a` |
| 审查方式 | 只读静态源码核对、公式推导、仓库调用点交叉核对 |
| 运行状态 | 随 `run_simulation` 端到端冒烟 |
| 修改范围 | 仅新增本审查文档 |

## 2. 文件职责与接口契约

### 2.1 职责

评估模块调度器。`tracking_errors` 计算单站跟踪误差（UKF/标亮点迹/原始点迹），`fusion` 计算融合误差和单站基线。[S][P]

### 2.2 输入输出与单位契约

| action | 输入 | 输出 |
|---|---|---|
| `tracking_errors` | 航迹快照、检测列表、真值、帧数 | `errorStats` 结构体 |
| `fusion` | 融合快照、配对信息、真值 | `fusion_eval` 结构体 |

输出中所有距离单位为 km（通过 `haversine_km_eval`）。[S]

## 3. 逐语句块审查

### 3.1 调度函数（第 10—19 行）

**语句职责。** `switch action` 分发到两个局部函数。[S]

**代码质量。** 错误消息格式与项目其他 dispatcher 一致（无 identifier），遗留 `P3-01`。[S]

### 3.2 `compute_tracking_errors`（第 25—118 行）

#### 3.2.1 真值插值（第 37—44 行）

**语句职责。** 对每架飞机、每帧，从真值轨迹插值得到 `t_true_lat/lon`。[S]

**数学正确性。** `interp1(..., 'linear', 'extrap')` 线性插值并外推。外推在轨迹边界可能产生不合理值（如经纬度超出范围）。[M]

#### 3.2.2 航迹-真值匹配（第 47—63 行）

**语句职责。** 对每帧的每条航迹，计算到真值的 Haversine 距离，取最小者（若 < 200 km）。[S]

**数学正确性。**

```matlab
d = haversine_km_eval(trk.lon, trk.lat, t_true_lon, t_true_lat);
if d < best_ukf_dist && d < 200
```

`d < 200` 是 200 km 门限（`haversine_km_eval` 使用 `R=6371`，返回 km）。[M]

**问题记录。** `V07-P1-01`：200 km 门限对单目标场景过宽。典型 OTH-SWR 量测误差 7-14 km，航迹 RMSE 通常 3-6 km。200 km 门限不会排除任何合理航迹，但可能错误匹配发散航迹或身份错误的航迹。在多目标场景中（通过 `trackSnapshots` 传入），此门限完全失效——没有身份约束，最近航迹可能属于错误飞机。[S][M]

**问题记录。** `V07-P1-02`：`best_ukf_dist = inf` 初始值，若帧内无有效航迹则 `ukf_errs{a}(k)` 不赋值，导致 `ukf_errs{a}` 长度小于 `n_frames`。后续 `compute_summary_eval` 对不等长数组使用 `[combined, errs]` 拼接，可能遗漏帧间时间顺序信息。[S]

#### 3.2.3 检测点迹误差（第 65—80 行）

**语句职责。** 遍历检测列表，计算标校后和原始点迹到真值的距离。[S]

**数学正确性。** 使用 `aircraft_id` 字段匹配真值飞机，正确。[M]

**代码质量。** `dp.is_clutter` 过滤杂波，`dp.aircraft_id ~= a` 过滤非目标飞机检测。[S]

#### 3.2.4 汇总统计（第 84—98 行）

**语句职责。** 对每架飞机的三类误差计算汇总统计。[S]

**数学正确性。** 使用 `median`、`mean`、`std`、`rms`、`prctile(95)`。[M]

**问题记录。** `V07-P2-01`：`ukf_vs_det_pct = (1 - median_ukf / max(median_det, 0.01)) * 100` 使用中位数比值衡量 UKF 相对标校检测的改善百分比。但中位数对异常值不敏感，而 RMSE 对异常值敏感。若使用 `median` 而非 `mean` 或 `rms`，改善百分比可能与可视化图中的 RMSE 改善不一致。[M]

#### 3.2.5 全局汇总（第 100—108 行）

**语句职责。** 跨飞机拼接误差数组。[S]

**性能问题。** `all_ukf = [all_ukf, ukf_errs{a}]` 动态数组增长。在单帧场景中 `ukf_errs{a}` 长度约 50，影响可忽略；但在 MC 场景中拼接 500 次 × 50 帧的数据，动态增长效率低。[S]

### 3.3 `evaluate_fusion`（第 124—278 行）

#### 3.3.1 Pair-to-Aircraft 映射（第 131—152 行）

**语句职责。** 用 R1 航迹平均大地线距离匹配真值飞机。[S]

**数学正确性。** `nanmean(haversine_km_vec_eval(r1_lons, r1_lats, t_lon, t_lat))` 计算整条 R1 航迹到真值的平均距离。[M]

**问题记录。** `V07-P2-02`：此映射假设 R1 航迹在整个飞行过程中都接近真值。若航迹在转弯段发散，平均距离可能被直线段的低误差稀释，导致错误匹配。建议使用加权平均（靠近真值的帧权重更高）。[M]

#### 3.3.2 融合误差逐帧计算（第 162—190 行）

**语句职责。** 对每种融合方法、每架飞机、每帧，计算融合航迹到真值的距离。[S]

**数学正确性。** 通过 `ftrk.id` 找到 pair 索引，再通过 `pair_to_aircraft` 找到真值飞机。[M]

**问题记录。** `V07-P1-03`：融合误差计算中 `pair_to_aircraft(p_idx)` 直接通过 `ftrk.id` 索引。`ftrk.id = p`（pair 索引），但 `pair_to_aircraft` 的长度是 `n_pairs`。若 `ftrk.id` 超出范围（如 ID 从 1 开始但 pair 索引从 0 开始），会跳过或越界。当前代码 `p_idx < 1 || p_idx > length(pair_to_aircraft)` 保护了越界。[S]

**性能问题。** 每帧对每架飞机调用 `interp1` 两次（经度和纬度），共 `n_frames × n_ac × 2` 次插值。可预先计算 `truth_interp` 矩阵避免重复调用。[S]

#### 3.3.3 单站基线误差（第 192—236 行）

**语句职责。** 对 R1 和 R2 分别计算到真值的距离（仅使用匹配对的航迹）。[S]

**数学正确性。** 通过 `pair_to_aircraft` 找到匹配对的 R1/R2 ID，再查找对应航迹。[M]

**问题记录。** `V07-P2-03`：`find(matched_pairs(p).R1_track_id, 1)` 在循环中重复调用 `find`，每次 `O(n_r1_tracks)`。可用 `r1_to_pair` 映射（如 `run_track_fusion.m` 中所示）替代。[S]

### 3.4 `compute_summary_eval`（第 284—298 行）

**语句职责。** 计算误差数组的统计摘要。[S]

**数学正确性。** 标准统计量。[M]

### 3.5 `haversine_km_eval`（第 304—311 行）

**语句职责。** Haversine 距离计算，返回 km。[S]

**数学正确性。** 与 `utils/skywave_geometry.m` 中的 `geocentric_angle_impl` 使用相同公式（`R=6371` vs `R_e=6371000`）。单位不同但数学一致。[M]

**问题记录。** `V07-P3-01`：`haversine_km_eval` 是 `utils/sphere_utils_haversine_distance.m` 的重复实现。仓库中 Haversine 至少 4 份独立实现（见历史 issue），应统一。[S]

### 3.6 `compute_err_stats_eval`（第 317—331 行）

**语句职责。** 与 `compute_summary_eval` 功能相同，但命名为 `*_eval` 后缀。[S]

**问题记录。** `V07-P3-02`：`compute_summary_eval` 和 `compute_err_stats_eval` 功能完全相同，重复实现。[S]

### 3.7 `find_track_by_id_eval`（第 337—346 行）

**语句职责。** 按 ID 查找航迹。[S]

**代码质量。** 与 `run_track_fusion.m:find_track` 功能相同，重复实现。[S]

### 3.8 `haversine_km_vec_eval`（第 352—361 行）

**语句职责。** 向量化 Haversine 距离。[S]

**问题记录。** `V07-P2-04`：名义上是"向量化"，实际是逐元素循环调用 `haversine_km_eval`。真正的向量化应使用逐元素运算 `.^`、`.*` 直接处理数组输入。[S]

## 4. 文件级问题汇总

| Issue ID | 位置 | 严重度 | 类别 | 证据 | 状态 |
|---|---:|---|---|---|---|
| V07-P1-01 | 第 54 行 | P1 | 200 km 门限过宽 | S+M | OPEN |
| V07-P1-02 | 第 59-61 行 | P1 | 不等长误差数组 | S | OPEN |
| V07-P1-03 | 第 182 行 | P1 | pair_id 索引越界风险 | S | OPEN |
| V07-P2-01 | 第 94 行 | P2 | 中位数 vs RMSE 口径 | M | OPEN |
| V07-P2-02 | 第 133-152 行 | P2 | pair-to-aircraft 映射 | M | OPEN |
| V07-P2-03 | 第 209-233 行 | P2 | find 性能 | S | OPEN |
| V07-P2-04 | 第 352-361 行 | P2 | 伪向量化 | S | OPEN |
| V07-P3-01 | 第 304-311 行 | P3 | Haversine 重复 | S | OPEN |
| V07-P3-02 | 第 284-298 行 | P3 | 统计函数重复 | S | OPEN |
| V07-P3-03 | 第 337-346 行 | P3 | find_track 重复 | S | OPEN |

## 5. 文件级结论

- **数学可信度**：评估公式（Haversine 距离、统计摘要）正确。但 200 km 门限在单目标场景可能掩盖身份错误，在多目标场景完全失效。不等长误差数组可能导致帧间时间顺序信息丢失。
- **代码质量**：Haversine、统计函数、find_track 均有重复实现，应统一到 `utils/`。伪向量化 `haversine_km_vec_eval` 实际是循环。
- **性能**：逐帧 `interp1` 调用可预计算；`find` 线性搜索可替换为 `containers.Map`。
- **测试充分性**：端到端冒烟通过，但 200 km 门限对不同场景的影响、不等长数组的统计行为未获独立验证。
- **剩余未验证项**：不同门限（5/10/50/200 km）对 RMSE 的影响；pair-to-aircraft 映射在航迹交叉场景的正确性。

## 6. 下一审查游标

- 文件：`fusion/time_align_tracks.m`（时间对齐）
- 重点：回退 dt 符号、Q 缩放、协方差传播
- 稳定指纹：`dt = -dt_offset` + `Q_dt = Q_base * (dt_abs / params.dt_sec)`
