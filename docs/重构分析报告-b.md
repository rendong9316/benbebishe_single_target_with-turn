# 项目重构分析报告 — 文件级深度审计

> 审计范围：全部 65 个 `.m` 文件，~16243 行代码
> 审计日期：2026-07-20

---

## 一、项目现状概览

| 目录 | 文件数 | 总行数 | 核心职责 |
|------|--------|--------|---------|
| `tracker/` | 17 | ~3300 | 航迹生命周期、关联、起始、片段管理 |
| `ukf/` | 6 | ~1900 | UKF 滤波器（基础/IMM/自适应/调度） |
| `simulation/` | 9 | ~2200 | 真值生成、检测生成、偏差标定 |
| `fusion/` | 7 | ~2100 | 时间对齐、片段提取、分组、融合算法、桥接 |
| `evaluation/` | 2 | ~950 | RMSE 评估 |
| `visualization/` | 6 | ~1600 | 绘图 |
| `utils/` | 9 | ~750 | 球面几何、坐标系 |
| `validation/` | 4 | ~1200 | 测试验证 |
| 入口脚本 | 4 | ~1600 | `run.m`/`run_without_fusion.m`/`run_fragment_study.m`/`run_random_fade_fragment_fusion.m` |
| `config/` | 1 | ~350 | 参数配置 |
| `io/` | 1 | ~90 | 测量字段提取 |
| **合计** | **66** | **~16243** | |

---

## 二、审计方法论

本次审计逐文件阅读全部源代码，按以下维度评估：

1. **Purpose** — 文件实际承担的职责
2. **Internal Structure** — 顶层函数列表及各自功能
3. **Issues** — 按类型分类（redundancy/cohesion/coupling/naming/dead_code/hardcoded/missing_feature）
4. **Dependencies** — 调用了谁，被谁调用
5. **Verdict** — keep / refactor / delete / split / rename / minor

---

## 三、逐文件审计报告

### 3.1 入口脚本层

#### `run.m` (660行)

**Purpose:** 双站 OTH-SWR 跟踪仿真全流程主入口（Phase 0-10）。
**Internal Structure:** 主流程 ~100 行 + 20 个嵌套辅助函数（打印 6 个、标定 1 个、检测 1 个、tracker 包装 2 个、绘图 8 个、保存 1 个、工具 1 个）。

**Issues:**

| 类型 | 行号 | 描述 | 影响 | 建议 |
|------|------|------|------|------|
| cohesion | L169-659 | 20 个辅助函数与主流程混在同一文件 | 主文件 660 行，可读性差 | 拆分为 6 个独立文件 |
| redundancy | L507-522 | `truth_tracks_for_legacy_plots` | 4 个入口脚本各有副本 | 统一到 `io/` |
| redundancy | L169-255 | 5 个 `print_*` 函数 | `run_without_fusion.m` 各有副本 | 统一到 `io/print_summary.m` |
| redundancy | L257-322 | `calibrate_bias` | 与 `prepare_oracle_tracking_inputs` 内部版本逻辑不同但同名 | 重命名为 `calibrate_bias_adsb.m` |
| redundancy | L324-381 | `generate_radar_detections` | 与 `prepare_oracle_tracking_inputs` 内部版本逻辑不同但同名 | 重命名为 `generate_radar_detections_direct.m` |
| redundancy | L404-412 | `count_active_tracks` | 与 `run_oracle_tracker_sequence.m` L57 重复 | 删除一处 |
| redundancy | L564-599 | `collect_track_ids` + `collect_track_line` | 与 `run_fragment_study.m` L254-335 重复 | 统一到 `visualization/` |
| dead_code | L651-659 | `wrap_angle_run` | 3 个 wrap_angle 变体并存 | 统一到 `utils/wrap_angle.m` |

**Dependencies:** simulation/*, tracker/*, ukf/*, fusion/*, evaluation/*, visualization/*, utils/*, config/*
**Called by:** 用户
**Verdict: SPLIT** — 提取 6 个文件，主流程缩减至 ~100 行

---

#### `run_without_fusion.m` (199行)

**Purpose:** 无融合版单站跟踪流水线（Phase 0-4）。
**Internal Structure:** 主流程 ~60 行 + 7 个打印/绘图辅助函数。

**Issues:**

| 类型 | 行号 | 描述 | 建议 |
|------|------|------|------|
| redundancy | L122-131 | `truth_tracks_for_legacy_without_fusion` 与 run.m L507 字节级重复 | 删除，引用共享版本 |
| redundancy | L133-198 | 6 个 `print_*_without_fusion` 与 run.m 同名函数重复 | 合并到 `io/print_summary.m` |
| redundancy | L109-120 | `plot_without_fusion_figures` 与 run.m `plot_oracle_figures` 重叠 | 合并到 `visualization/oracle_figures.m` |

**Verdict: SPLIT + REDUCE** — 保留 ~100 行主流程，所有辅助函数外提

---

#### `run_fragment_study.m` (477行)

**Purpose:** 人工碎片实验：基线跟踪 → 制造互补碎片 → 分组 → 融合 → 评估 → 仪表盘可视化。
**Internal Structure:** 主流程 ~60 行 + 22 个嵌套函数。

**Issues:**

| 类型 | 行号 | 描述 | 建议 |
|------|------|------|------|
| cohesion | L184-476 | 22 个嵌套函数混合了碎片制造、评估、可视化数据准备 | 拆分为 3 个文件 |
| redundancy | L268-279 | `collect_track_ids` 与 run.m L564 重复 | 删除 |
| redundancy | L469-476 | `haversine_km` 与 `sphere_utils_haversine_distance` 重复 | 删除，内联为 `sphere_utils_haversine_distance()/1000` |
| dead_code | L337-340 | `random_between` 仅 1 行 | 内联 |
| hardcoded | L260,288,306,328,366,378 | `type ~= 7` 出现 6 次，magic number | 定义为常量 `TYPE_TERMINATED` |

**Verdict: SPLIT into 3 files**

| 提取内容 | 目标文件 |
|---------|---------|
| `manufacture_fragments` + `clone_snapshots` + `split_track` + `crop_track` + `assert_fragment_plan` + `find_segment` + `collect_truth_ids` + `collect_track_ids` + `active_frames_for_truth` + `ids_for_truth` + `has_truth_track` | `tracker/fragment_manufacturing.m` |
| `evaluate_groups` + `truth_labels_for_segment` + `group_errors` + `print_evaluation` | `evaluation/fragment_evaluation.m` |
| 主流程 + `merge_config` + `default_config` + `result_view_inputs` + `collect_fused` | 保留，缩减至 ~60 行 |

---

#### `run_random_fade_fragment_fusion.m` (225行)

**Purpose:** 可控衰落实验：自动衰落 → 跟踪 → 片段提取 → 分组 → 融合 → 评估。
**Internal Structure:** 主流程 ~60 行 + 8 个辅助函数。

**Issues:**

| 类型 | 行号 | 描述 | 建议 |
|------|------|------|------|
| redundancy | L207-214 | `truth_tracks_for_legacy` 第 4 个副本 | 删除 |
| redundancy | L216-224 | `defaults` 与 `run_fragment_study.m` L94 `merge_config` 重复 | 合并 |
| dead_code | L202-205 | `scalar_label` 仅 1 行 | 内联 |
| dead_code | L175-178 | `first_failure` 仅 1 行 | 内联 |
| dead_code | L158-173 | `fixture_failure_result` 仅 1 次调用 | 内联 |

**Verdict: MINOR REFACTOR** — 保留主流程，内联 4 个 trivial 函数

---

### 3.2 Tracker 层

#### `TRACK_MAIN_ORACLE.m` (24行)

**Purpose:** 薄包装，转发到 `Track_Process_for_HighRate_Oracle`。
**Internal Structure:** 单函数，24 行中仅 3 行是实际代码（L18-22）。

**Issues:**

| 类型 | 行号 | 描述 |
|------|------|------|
| dead_code | L1-23 | 注释自称"薄"，零额外逻辑，唯一价值是"接口稳定" |

**Dependencies:** `Track_Process_for_HighRate_Oracle`
**Called by:** `run_oracle_tracker_sequence.m`, `run.m`
**Verdict: DELETE** — 调用方改为直调 `Track_Process_for_HighRate_Oracle`

---

#### `Fun_UpdateTrackforNoInputPoint_Oracle.m` (43行)

**Purpose:** 零检测帧时批量纯预测更新。
**Internal Structure:** 单函数。

**Issues:**

| 类型 | 行号 | 描述 |
|------|------|------|
| dead_code | L1-43 | 无任何调用方。`Fun_UpdateTrackByAsscResult_Oracle` 已内联处理零检测情况 |

**Called by:** 无
**Verdict: DELETE**

---

#### `fun_remove_assc_pts_from_pointlist_oracle.m` (27行)

**Purpose:** 从未关联点迹列表中移除已被消耗的点迹。
**Internal Structure:** 实际代码仅 3 行（L23-25）。

**Verdict: INLINE** — 直接在 `Track_Process_for_HighRate_Oracle` L115-116 处内联

---

#### `sortTrackList_oracle.m` (33行)

**Purpose:** 按 ID 升序排序航迹列表。
**Internal Structure:** 实际代码仅 3 行。

**Verdict: INLINE** — `[~,o]=sort([trackList.id]); trackList=trackList(o);`

---

#### `post_init_multi.m` (49行)

**Purpose:** 初始化后注入 dt、initialized、NIS 历史、Q_base、Q_ema 到 UKF 结构体。

**Issues:**

| 类型 | 描述 |
|------|------|
| naming | 文件名暗示多雷达支持，但实际只做 UKF 后初始化 |

**Verdict: RENAME** → `ukf_post_init.m`，移到 `ukf/` 目录

---

#### `Fun_UpdateTrackByAsscResult_Oracle.m` (104行)

**Purpose:** 按关联结果更新航迹：有关联点则 Kalman 更新，无关联则纯预测，同时记录 NIS 和执行质量管理。

**Issues:**

| 类型 | 行号 | 描述 | 建议 |
|------|------|------|------|
| naming | L1 | `Fun_` 前缀与项目其他文件不一致 | 重命名为 `track_update_oracle.m` |
| redundancy | L81-92 | `wrap_angle_oracle` 与 `wrap_angle_run`/`wrap_angle`/`wrap_angle_ukf` 重复 | 删除，改用共享版本 |

**Verdict: RENAME + MINOR**

---

#### `PointTrackAssociation_Oracle.m` (129行)

**Purpose:** Oracle 点迹-航迹关联：基于真值 ID 的最近邻匹配。

**Issues:**

| 类型 | 行号 | 描述 | 建议 |
|------|------|------|------|
| minor | L96-107 | `is_current_real_detection` 仅 1 行谓词 | 内联到调用处 |
| minor | L109-128 | `oracle_point_distance` 中的 fallback 逻辑 | 保留，有实际价值 |

**Verdict: KEEP** — 内联 `is_current_real_detection`

---

#### `Track_Process_for_HighRate_Oracle.m` (335行)

**Purpose:** 核心单帧 Oracle 跟踪处理：6 阶段流水线（生命周期管理 → UKF 预测 → Oracle 关联 → UKF 更新 → 质量管理 → 航迹起始）。

**Issues:**

| 类型 | 行号 | 描述 | 建议 |
|------|------|------|------|
| cohesion | L1-335 | 14 个内部函数，335 行 | 合理，但 `normalize_point_list` (L180-193) 和 `get_track_type` (L308-311) 可提取 |
| coupling | L308-311 | `get_track_type` 处理 Type/type 新旧字段名兼容性 | 说明代码库存在不一致命名 |

**Verdict: KEEP** — 唯一的改进建议：提取 `normalize_point_list` 到 `utils/`

---

#### `trackStarter_logic_oracle.m` (251行)

**Purpose:** Oracle 航迹起始器：基于可配置滑窗，按真值 ID 维护缓冲区，达到 QUALIFY_NUM 命中确认后触发航迹确认。

**Issues:**

| 类型 | 行号 | 描述 | 建议 |
|------|------|------|------|
| minor | L121-148 | `validate_starter_params` 仅 28 行 | 内联到主函数 |

**Verdict: KEEP** — 内联参数验证

---

#### `fun_create_new_track_oracle.m` (115行)

**Purpose:** 从两次检测创建新可靠航迹：两点法 UKF 初始化 + 南阳式字段组装。

**Issues:**

| 类型 | 行号 | 描述 | 建议 |
|------|------|------|------|
| minor | L107-114 | `make_output_point` 仅 8 行 | 内联 |

**Verdict: KEEP** — 内联 `make_output_point`

---

#### `fun_track_quality_management_and_info_completion_oracle.m` (145行)

**Purpose:** 4 态航迹质量管理（RELIABLE/MAINTAIN/TEMPORARY/HISTORY）+ 质量计分 + 状态转移。

**Issues:**

| 类型 | 行号 | 描述 | 建议 |
|------|------|------|------|
| minor | L136-144 | `get_field_or_default` 仅 4 行 | 内联 |

**Verdict: KEEP** — 内联 `get_field_or_default`

---

#### `dual_threshold_decide.m` (105行)

**Purpose:** 双门限距离+连续性+方差决策，用于片段匹配。

**Issues:**

| 类型 | 行号 | 描述 |
|------|------|------|
| naming | L1 | 与 `track_matcher_dualgate.m` 内部的 `dual_threshold_decide` (L263) 同名但签名不同 |

**Verdict: RENAME** → `tracker/dual_threshold_decision.m`

---

#### `plan_controlled_fragmentation.m` (494行)

**Purpose:** 可控碎片化实验核心引擎：构建确定性衰落窗口，产生精确 K 个片段/目标/雷达。

**Issues:**

| 类型 | 行号 | 描述 | 建议 |
|------|------|------|------|
| cohesion | L1-494 | 17 个内部函数，混合了编排、搜索、验证 | 拆分为 2 个文件 |

**Verdict: SPLIT into 2 files**

| 提取内容 | 目标文件 |
|---------|---------|
| `search_target_plan` + `legal_candidate_starts` + `has_restart_evidence` + `apply_target_window` + `valid_target_segments` + `validate_counts` | `tracker/fragmentation_search.m` |
| 其余（主编排器 + 事件标注 + 辅助） | 保留在 `plan_controlled_fragmentation.m` |

---

#### `tracklet_grouping.m` (476行)

**Purpose:** 基于图论的航迹片段分组与凝聚：建边（successor/overlap/handoff）→ 枚举连通子集 → 整数规划求解最优覆盖 → 歧义检测。

**Issues:**

| 类型 | 行号 | 描述 | 建议 |
|------|------|------|------|
| hardcoded | L272 | `53` 节点枚举上限 | 提取为常量 `MAX_COMPONENT_SIZE_FOR_ENUM` |

**Verdict: KEEP AS-IS** — 22 个内部函数属于同一算法的数据流（建边→枚举→求解），内聚性好，不应拆分。

---

#### `track_matcher_dualgate.m` (421行)

**Purpose:** 双门限跨雷达航迹匹配：位置+速度+航向特征，匈牙利式最优分配，质量评分。

**Issues:**

| 类型 | 行号 | 描述 | 建议 |
|------|------|------|------|
| naming | L263-342 | 内部 `dual_threshold_decide` 与外部 `dual_threshold_decide.m` 同名不同签 | 重命名为 `dualgate_decide` |
| redundancy | L380-392 | `haversine_km_local` 与 `sphere_utils_haversine_distance` 重复 | 删除，改用共享版本/1000 |
| minor | L398-420 | `struct2table_vertcat` + `get_param_local` 仅 20 行 | 内联 |

**Verdict: MINOR REFACTOR**

---

#### `build_faded_track_segments.m` (353行)

**Purpose:** 片段提取与衰落应用：从快照中提取带 support/tail/effective 标注的航迹片段；应用随机衰落窗口。

**Issues:**

| 类型 | 行号 | 描述 | 建议 |
|------|------|------|------|
| cohesion | L31-118 vs L120-226 | `apply_fade` 和 `extract_segments` 两个独立 action | 拆分为 2 个文件 |
| redundancy | L334-352 | `active_frames_for_target` 与 `run_fragment_study.m` L281 重复 | 统一 |

**Verdict: SPLIT into 2 files**

| 提取内容 | 目标文件 |
|---------|---------|
| `extract_segments` + 辅助函数 | `build_faded_track_segments.m` |
| `apply_fade` + 辅助函数 | `apply_random_fade.m` |

---

#### `track_matcher.m` (546行) — fusion/ 目录下

**Purpose:** 旧版跨雷达航迹匹配：多特征匹配（距离+速度+航向）+ 匈牙利式最优分配。

**Issues:**

| 类型 | 行号 | 描述 |
|------|------|------|
| hardcoded | L54-61 | `coexist_thresh=5`, `dist_thresh_km=50`, `w_dist=0.6` 等魔法数字 |
| deprecation | L1 | 与 `track_matcher_dualgate.m` 功能重叠 |

**Verdict: KEEP (deprecated)** — 顶部加 `% @deprecated Use track_matcher_dualgate instead`，新代码不再使用

---

### 3.3 UKF 层

#### `ukf_jichu.m` (624行)

**Purpose:** 核心 UKF 滤波数学：create/init/prepare/update/predict/measurement，含 CV/CT 模型、UT 变换、天波量测模型、坐标转换。

**Issues:**

| 类型 | 行号 | 描述 | 建议 |
|------|------|------|------|
| redundancy | L617-623 | `haversine_ukf` 与 `sphere_utils_haversine_distance` 重复 | 删除 |
| redundancy | L578-608 | `regularize_cov_ukf` 与 `fusion/regularize_cov` 重复 | 删除 |
| redundancy | L444-446 | `wrap_angle_ukf` 与共享 `wrap_angle` 重复 | 删除 |
| hardcoded | L161 | `111320.0` 度→米换算常数 | 改为 `R_EARTH * deg2rad(1)` |
| cohesion | L1-624 | 13 个内部函数 | 拆分为 2 个文件 |

**Verdict: SPLIT into 2 files**

| 提取内容 | 目标文件 |
|---------|---------|
| `meas_to_latlon_ukf` (L512-567) | `ukf/meas_inverse.m` |
| `state_transition_ct_ukf` (L488-502) | `ukf/ct_model.m` |
| `haversine_ukf` + `regularize_cov_ukf` + `wrap_angle_ukf` | **删除** |
| 其余 ~450 行 | `ukf/ukf_jichu.m` |

---

#### `ukf_imm.m` (658行)

**Purpose:** IMM UKF：3 模型（CV/CT-left/CT-right），Pd-IPDA 似然，自适应 Q，完整 IMM 循环（混合→预测→更新→组合）。

**Issues:**

| 类型 | 行号 | 描述 | 建议 |
|------|------|------|------|
| redundancy | L510-526 | `regularize_cov_imm` 与 `fusion/regularize_cov` 重复 | 删除，改用共享版本 |
| cohesion | L293-304 | 自适应 Q 逻辑嵌入 `prepare_imm`，耦合紧密 | 提取为独立函数 |
| hardcoded | L438 | `nz = 3` 硬编码 | 改为 `ukf.ukf_cv.m` |
| redundancy | L571-612 | `apply_transient_q_imm` 与 `adapt_q.m` fuzzy_only 路径重叠 | 保留（场景不同：IMM 专用 vs 通用） |

**Verdict: MINOR REFACTOR** — 删除 `regularize_cov_imm`，修正 `nz` 来源

---

#### `ukf_zishiying.m` (81行)

**Purpose:** 薄包装，在 `ukf_jichu` 基础上叠加自适应 Q 更新。

**Issues:**

| 类型 | 行号 | 描述 | 建议 |
|------|------|------|------|
| dead_code | L53-63 | `update` action 接收多个未使用参数 | 清理签名 |
| dead_code | L77-80 | `apply_maneuver_adapt_post` 无外部调用方 | 验证后删除 |

**Verdict: CLEANUP** — 清理未使用参数，验证 `apply_maneuver_adapt_post`

---

#### `ukf_dispatch.m` (47行)

**Purpose:** 运行时多态路由：根据 ukf 结构体字段自动路由到 IMM/自适应/基础 UKF。

**Issues:**

| 类型 | 行号 | 描述 |
|------|------|------|
| coupling | L27-34 | 路由条件脆弱（检查 `ukf_cv`/`filter_type`/`maneuver_active`/`suspect_counter` 字段） |

**Verdict: KEEP** — 设计亮点，建议补充文档注释说明路由优先级和字段契约

---

#### `adapt_q.m` (281行)

**Purpose:** 通用自适应 Q 调整：结合模糊 NIS 推理和机动检测 + EMA 平滑。

**Issues:**

| 类型 | 行号 | 描述 | 建议 |
|------|------|------|------|
| hardcoded | L78,129,191 | 大量魔法数字（nis_ratio_thresh=1.10, win_short=3, q_boost_init=2.0 等） | 提取到 `simulation_params_oracle` |
| accuracy | L78 | NIS 除以 2.0 但注释说 chi^2(3) 期望是 3 | 修正注释或验证数学 |

**Verdict: KEEP** — 魔法数字参数化

---

#### `radar_params.m` (61行)

**Purpose:** 参数适配器：将 radar1_/radar2_ 前缀字段映射到 ukf_* 通用字段名。

**Verdict: KEEP** — 薄适配层，有价值。雷达 ID 校验硬编码到 2 可接受。

---

### 3.4 Simulation 层

#### `prepare_oracle_tracking_inputs.m` (215行)

**Purpose:** Oracle 跟踪统一入口：加载参数 → 生成场景 → 标定偏差 → 生成检测 → 创建 UKF 模板。

**Issues:**

| 类型 | 行号 | 描述 | 建议 |
|------|------|------|------|
| cohesion | L72-136 | `calibrate_bias` 65 行嵌套 | 提取为独立文件 |
| cohesion | L138-208 | `generate_detections` 71 行嵌套 | 提取为独立文件 |
| redundancy | L210-214 | `wrap_angle` 3 行，与 `sphere_utils` 重复 | 删除 |

**Verdict: SPLIT into 3 files**

| 提取内容 | 目标文件 |
|---------|---------|
| `calibrate_bias` | `simulation/calibrate_bias.m` |
| `generate_detections` | `simulation/generate_detections.m` |
| `wrap_angle` | **删除** |

---

#### `generate_frame_detections.m` (231行) vs `generate_frame_detections_multi.m` (234行)

**Purpose:** 单目标/多目标帧检测生成：目标检测（受 Pd 影响）+ Poisson 杂波。

**Issues:**

| 类型 | 行号 | 描述 | 建议 |
|------|------|------|------|
| redundancy | L182-232 vs L176-230 | 杂波生成代码几乎字节级相同 | 提取为 `simulation/clutter_generator.m` |

**Verdict: MERGE clutter logic** — 提取共享杂波生成函数

---

#### `bistatic_inverse_solver.m` (119行)

**Purpose:** 反解器：从群距离 Rg 和方位角 az 恢复目标经纬度，含经典双基地初值 + 天波迭代精化。

**Issues:**

| 类型 | 行号 | 描述 | 建议 |
|------|------|------|------|
| redundancy | L84-110 | 迭代精化与 `meas_to_latlon_ukf` (ukf_jichu.m L538-551) 重复 | 提取为共享函数 |

**Verdict: MINOR** — 提取迭代精化为 `simulation/skywave_refine.m`

---

#### `aircraft_trajectory_create.m` (666行)

**Purpose:** 飞行器轨迹创建：支持 straight/turn/gradual_turn/uturn 多种轨迹类型。

**Issues:**

| 类型 | 行号 | 描述 | 建议 |
|------|------|------|------|
| redundancy | L648-662 | `haversine_forward` 与 `sphere_utils_destination_point` 重复 | 删除，改用共享版本 |
| hardcoded | L309-427 | `create_uturn_trajectory` 硬编码圆心坐标 (131.44, 31.75) | 参数化 |

**Verdict: MINOR REFACTOR**

---

#### `aircraft_trajectory_interpolate.m` (171行)

**Verdict: KEEP** — 结构清晰，无显著问题。

#### `aircraft_trajectory_locate.m` (105行)

**Verdict: KEEP** — 线性扫描在小 n_segments 下足够。

#### `build_target_states_at_time.m` (78行)

**Verdict: KEEP** — 干净聚焦。

#### `build_truth_scenario.m` (114行)

**Verdict: KEEP** — 场景分发器，职责单一。

#### `radar_coverage_check.m` (97行)

**Verdict: KEEP** — 干净聚焦。

---

### 3.5 Fusion 层

#### `regularize_cov.m` (165行)

**Purpose:** 协方差矩阵正则化：对称化 + 特征值裁剪保证正定性。

**Issues:**

| 类型 | 描述 |
|------|------|
| redundancy | 165 行文档中仅 ~30 行代码，文档过载但不有害 |
| redundancy | `ukf_jichu.m` 有 `regularize_cov_ukf`，`ukf_imm.m` 有 `regularize_cov_imm`，三者逻辑相同 |

**Verdict: KEEP as central version** — 删除两个重复实现

---

#### `bridge_smoother.m` (440行)

**Purpose:** RTS/IMM 桥接平滑器：用 Rauch-Tung-Striebel 平滑重建融合航迹中的有界空洞。

**Issues:**

| 类型 | 行号 | 描述 | 建议 |
|------|------|------|------|
| cohesion | L193-326 | `smooth_gap` 134 行，含完整 RTS+IMM 算法 | 提取为独立文件 |
| redundancy | L328-339 | `transition_matrix` 与 `state_transition_ct_ukf` 重复 | 删除，复用 |

**Verdict: SPLIT into 2 files**

| 提取内容 | 目标文件 |
|---------|---------|
| `smooth_gap` + `transition_matrix` + `process_noise` + `combine_gaussians` + `normalize_prob` + `gaussian_likelihood` | `fusion/rts_smoother.m` |
| 其余（配置、包装、诊断） | 保留在 `bridge_smoother.m` |

---

#### `fuse_estimate_sequence.m` (380行)

**Purpose:** 四算法融合（SCC/BC/CI/FCI）：对动态航迹组逐算法执行融合，含桥接平滑和源计数。

**Issues:**

| 类型 | 行号 | 描述 | 建议 |
|------|------|------|------|
| cohesion | L112-216 | `fuse_method` 105 行，BC P12 维护逻辑交织 | 提取 BC 专用逻辑 |
| cohesion | L275-338 | `update_cross_covariance` + `covariance_contraction` | 提取为 `fusion/cross_covariance.m` |

**Verdict: MINOR** — 提取 P12 更新逻辑

---

#### `track_fusion_algorithms.m` (601行)

**Purpose:** 四种融合算法：SCC/BC/CI/FCI，含详尽数学文档。

**Verdict: KEEP** — 文档重载但数学精确，有价值。

---

#### `run_track_fusion.m` (55行)

**Purpose:** 旧版适配器：将 pair-based 接口转换为 group-based 接口。

**Verdict: KEEP (deprecated)** — 顶部加 `% @deprecated` 注释

---

#### `time_align_tracks.m` (206行)

**Purpose:** 时间对齐：用 CV 模型反向外推 R2 航迹到 R1 采样网格。

**Verdict: KEEP** — 干净聚焦。

---

### 3.6 Evaluation 层

#### `evaluate_all_multi.m` (755行)

**Purpose:** 多目标评估调度器：跟踪误差 + 融合误差对比。

**Issues:**

| 类型 | 行号 | 描述 | 建议 |
|------|------|------|------|
| redundancy | L624-645 vs L685-702 | `compute_summary_eval` 和 `compute_err_stats_eval` 90% 相同 | 合并 |
| redundancy | L655-678 | `haversine_km_eval` 与 `sphere_utils_haversine_distance` 重复 | 删除 |
| cohesion | L1-755 | 包含 tracking_errors 和 fusion 两个独立评估 | 拆分为 2 个文件 |

**Verdict: SPLIT into 2 files**

| 提取内容 | 目标文件 |
|---------|---------|
| `compute_tracking_errors_multi` + 辅助 | `evaluation/tracking_errors.m` |
| `evaluate_fusion_multi` + 辅助 | `evaluation/fusion_errors.m` |

---

#### `evaluate_fragment_fusion_multi.m` (200行)

**Verdict: KEEP** — 将 `unmatched_cost = 1e6` 提取为常量。

---

### 3.7 Visualization 层

#### `plot_scene_overview.m` (183行) vs `plot_scene_overview_multi.m` (133行)

**Issues:**

| 类型 | 行号 | 描述 | 建议 |
|------|------|------|------|
| redundancy | L141-182 vs L100-132 | `draw_beam_sector` 与 `draw_beam_sector_geoax` 功能相同 | 提取为 `visualization/beam_sector.m` |

**Verdict: MERGE** — 提取共享波束扇区绘制函数

---

#### `plot_results_multi.m` (640行)

**Purpose:** 多目标结果可视化调度器：跟踪结果图 + 融合结果图 + 图层控制。

**Issues:**

| 类型 | 行号 | 描述 | 建议 |
|------|------|------|------|
| redundancy | L48-190 vs L200-438 | `plot_multi_track_result` 和 `plot_multi_fusion_result` 共享 ~80% UI 代码 | 提取共享 UI 到公共函数 |
| cohesion | L342-438 | Figure 2（误差收敛 + CDF）与 `plot_multi_fusion_result` 紧耦合 | 分离为独立函数 |

**Verdict: SPLIT**

| 提取内容 | 目标文件 |
|---------|---------|
| 共享 UI 代码（layer controls, checkboxes, buttons） | `visualization/ui_controls.m` |
| Figure 2 绘制 | 提取为 `plot_fusion_diagnostic.m` |

---

#### `plot_tracks_without_fusion.m` (587行)

**Purpose:** 无融合单站跟踪结果可视化 + 图层控制 + 碎片研究叠加层。

**Issues:**

| 类型 | 行号 | 描述 | 建议 |
|------|------|------|------|
| cohesion | L26-27 | 图标题含 "fusion" 但函数名说不含 fusion | 修正标题 |
| cohesion | L341-401 | `plot_study_layers` 61 行，混合了片段、边、融合结果、桥接 | 拆分 |
| cohesion | L440-586 | 24 个局部函数含大量 layer control 逻辑 | 提取为独立文件 |

**Verdict: SPLIT into 3 files**

| 提取内容 | 目标文件 |
|---------|---------|
| `plot_study_layers` + `plot_bridge_layer` + `relation_lines` + `evaluation_rmse` + `fused_line` | `visualization/study_overlay.m` |
| `add_layer` + `install_layer_controls` + `set_layer_visibility` + `set_all_layers` + `visible_legend_layers` | `visualization/layer_controls.m` |
| 其余 | 保留在 `plot_tracks_without_fusion.m` |

---

#### `plot_point_cloud_3d.m` (104行)

**Verdict: KEEP** — 最小聚焦。

#### `plot_fragment_study_dashboard.m` (241行)

**Verdict: KEEP** — 唯一建议：标题硬编码 "three tracklets" 应参数化。

---

### 3.8 Utils 层

#### 死代码（经 Grep 验证，无任何调用方）

| 文件 | 行数 | 建议 |
|------|------|------|
| `utils/coord_systems_lla_to_ecef.m` | 50 | **DELETE** |
| `utils/sphere_utils_interpolate_great_circle.m` | 86 | **DELETE** |
| `utils/sphere_utils_radial_velocity.m` | 131 | **DELETE** |
| `utils/sphere_utils_seconds_to_datetime_str.m` | 73 | **DELETE** |

#### `skywave_geometry.m` (180行)

**Issues:**

| 类型 | 行号 | 描述 | 建议 |
|------|------|------|------|
| redundancy | L116-124 | `azimuth_impl` 与 `sphere_utils_azimuth` 数学相同 | 删除，`'azimuth'` case 改为调用 `sphere_utils_azimuth` |

**Verdict: MINOR** — 删除 `azimuth_impl`

---

#### `sphere_utils_haversine_distance.m` (102行)

**Verdict: KEEP as central version** — 删除所有副本：
- `ukf_jichu.m` 的 `haversine_ukf`
- `evaluate_all_multi.m` 的 `haversine_km_eval`
- `track_matcher_dualgate.m` 的 `haversine_km_local`
- `run_fragment_study.m` 的 `haversine_km`

#### `sphere_utils_azimuth.m` (107行)

**Verdict: KEEP as public API** — `skywave_geometry` 应调用此函数而非自建 `azimuth_impl`。

#### `sphere_utils_destination_point.m` (126行)

**Verdict: KEEP** — 干净聚焦。

#### `sphere_utils_destination_point.m` (126行)

**Verdict: KEEP** — 干净聚焦。

---

### 3.9 Config / IO / Validation

#### `config/simulation_params_oracle.m` (349行)

**Verdict: KEEP** — 大而必要，组织结构清晰。

#### `io/extract_measurement_field.m` (91行)

**Verdict: KEEP** — 简单工具函数。

#### `validation/run_filter_math_tests.m` (289行)

**Verdict: KEEP** — 测试夹具可接受。

#### `validation/run_fragment_fusion_tests.m` (181行)

**Verdict: KEEP** — 测试夹具可接受。

#### `validation/run_oracle_lifecycle_tests.m` (463行)

**Verdict: KEEP** — 组织良好。

#### `validation/validate_oracle_invariants.m` (433行)

**Verdict: KEEP** — 关键验证基础设施。

---

## 四、跨文件重复代码总表

### 4.1 Haversine 距离 — 5 处重复

| 位置 | 函数名 | 操作 |
|------|--------|------|
| `utils/sphere_utils_haversine_distance.m` | `sphere_utils_haversine_distance` | **KEEP (权威)** |
| `ukf/ukf_jichu.m` L617-623 | `haversine_ukf` | DELETE |
| `evaluation/evaluate_all_multi.m` L655-678 | `haversine_km_eval` | DELETE |
| `tracker/track_matcher_dualgate.m` L380-392 | `haversine_km_local` | DELETE |
| `run_fragment_study.m` L469-476 | `haversine_km` | DELETE |

### 4.2 协方差正则化 — 3 处重复

| 位置 | 函数名 | 操作 |
|------|--------|------|
| `fusion/regularize_cov.m` | `regularize_cov` | **KEEP (权威)** |
| `ukf/ukf_jichu.m` L578-608 | `regularize_cov_ukf` | DELETE |
| `ukf/ukf_imm.m` L510-526 | `regularize_cov_imm` | DELETE |

### 4.3 角度归一化 — 4 处重复

| 位置 | 函数名 | 操作 |
|------|--------|------|
| 新建 `utils/wrap_angle.m` | `wrap_angle` | **KEEP (统一)** |
| `run.m` L651-659 | `wrap_angle_run` | DELETE |
| `simulation/prepare_oracle_tracking_inputs.m` L210-214 | `wrap_angle` | DELETE |
| `tracker/Fun_UpdateTrackByAsscResult_Oracle.m` L81-92 | `wrap_angle_oracle` | DELETE |
| `ukf/ukf_jichu.m` L444-446 | `wrap_angle_ukf` | DELETE |

### 4.4 天波方位角 — 2 处重复

| 位置 | 函数名 | 操作 |
|------|--------|------|
| `utils/sphere_utils_azimuth.m` | `sphere_utils_azimuth` | **KEEP (权威)** |
| `utils/skywave_geometry.m` L116-124 | `azimuth_impl` | DELETE |

### 4.5 杂波生成 — 2 处重复

| 位置 | 行号 | 操作 |
|------|------|------|
| `simulation/generate_frame_detections.m` L182-232 | 65 行 | 提取为 `simulation/clutter_generator.m` |
| `simulation/generate_frame_detections_multi.m` L176-230 | 65 行 | 同上 |

### 4.6 波束扇区绘制 — 2 处重复

| 位置 | 行号 | 操作 |
|------|------|------|
| `visualization/plot_scene_overview.m` L141-182 | 42 行 | 提取为 `visualization/beam_sector.m` |
| `visualization/plot_scene_overview_multi.m` L100-132 | 33 行 | 同上 |

### 4.7 天波迭代精化 — 2 处重复

| 位置 | 行号 | 操作 |
|------|------|------|
| `simulation/bistatic_inverse_solver.m` L84-110 | 27 行 | 提取为 `simulation/skywave_refine.m` |
| `ukf/ukf_jichu.m` L538-551 | 14 行 | 同上 |

---

## 五、死代码清单

| 文件 | 行数 | 验证方式 | 操作 |
|------|------|---------|------|
| `utils/coord_systems_lla_to_ecef.m` | 50 | Grep 无调用方 | DELETE |
| `utils/sphere_utils_interpolate_great_circle.m` | 86 | Grep 无调用方 | DELETE |
| `utils/sphere_utils_radial_velocity.m` | 131 | Grep 无调用方 | DELETE |
| `utils/sphere_utils_seconds_to_datetime_str.m` | 73 | Grep 无调用方 | DELETE |
| `tracker/Fun_UpdateTrackforNoInputPoint_Oracle.m` | 43 | Grep 无调用方 | DELETE |
| `tracker/TRACK_MAIN_ORACLE.m` | 24 | Grep 3 调用方，纯转发 | DELETE（内联调用） |
| `tracker/sortTrackList_oracle.m` | 33 | Grep 2 调用方，3 行逻辑 | INLINE |
| `tracker/fun_remove_assc_pts_from_pointlist_oracle.m` | 27 | Grep 1 调用方，3 行逻辑 | INLINE |
| `ukf/ukf_zishiying.m` 中的 `apply_maneuver_adapt_post` | 4 | Grep 仅自身引用 | 验证后 DELETE |

---

## 六、重命名清单

| 原文件名 | 新文件名 | 理由 |
|---------|---------|------|
| `tracker/Fun_UpdateTrackByAsscResult_Oracle.m` | `tracker/track_update_oracle.m` | 统一命名风格（去掉 Fun_ 前缀） |
| `tracker/post_init_multi.m` | `ukf/ukf_post_init.m` | 移到正确目录，改名消除误导 |
| `tracker/dual_threshold_decide.m` | `tracker/dual_threshold_decision.m` | 避免与 `track_matcher_dualgate.m` 内部函数同名冲突 |
| `fusion/track_matcher.m` | `fusion/track_matcher_legacy.m` | 标记为废弃，与 dualgate 区分 |

---

## 七、新建共享模块清单

| 新文件 | 来源 | 内容 |
|--------|------|------|
| `utils/wrap_angle.m` | run.m + prepare_oracle_tracking_inputs.m + Fun_UpdateTrackByAsscResult_Oracle.m + ukf_jichu.m | 统一角度归一化 |
| `io/print_summary.m` | run.m + run_without_fusion.m | 统一打印函数 |
| `io/save_results.m` | run.m | 统一保存函数 |
| `visualization/oracle_figures.m` | run.m + run_without_fusion.m | 统一 Oracle 绘图 |
| `visualization/beam_sector.m` | plot_scene_overview.m + plot_scene_overview_multi.m | 统一波束扇区绘制 |
| `visualization/ui_controls.m` | plot_results_multi.m | 统一图层 UI 控件 |
| `visualization/layer_controls.m` | plot_tracks_without_fusion.m | 统一图层控制 |
| `visualization/study_overlay.m` | plot_tracks_without_fusion.m | 统一研究叠加层 |
| `visualization/fusion_diagnostic.m` | plot_results_multi.m | 统一融合诊断图 |
| `simulation/clutter_generator.m` | generate_frame_detections.m + generate_frame_detections_multi.m | 统一杂波生成 |
| `simulation/skywave_refine.m` | bistatic_inverse_solver.m + ukf_jichu.m | 统一天波迭代精化 |
| `ukf/ct_model.m` | ukf_jichu.m | CT 模型状态转移 |
| `ukf/meas_inverse.m` | ukf_jichu.m | 量测反解经纬度 |
| `tracker/fragment_manufacturing.m` | run_fragment_study.m | 碎片制造逻辑 |
| `tracker/fragmentation_search.m` | plan_controlled_fragmentation.m | 衰落窗口搜索 |
| `tracker/apply_random_fade.m` | build_faded_track_segments.m | 随机衰落应用 |
| `fusion/rts_smoother.m` | bridge_smoother.m | RTS 平滑核心 |
| `fusion/cross_covariance.m` | fuse_estimate_sequence.m | BC 互协方差更新 |
| `evaluation/fragment_evaluation.m` | run_fragment_study.m | 碎片评估逻辑 |
| `evaluation/tracking_errors.m` | evaluate_all_multi.m | 跟踪误差计算 |
| `evaluation/fusion_errors.m` | evaluate_all_multi.m | 融合误差计算 |
| `matcher_context.m` | run.m | 匹配上下文构建 |

---

## 八、重构后预期结构

### 8.1 文件数变化

| 类别 | 重构前 | 重构后 | 变化 |
|------|--------|--------|------|
| 删除 | - | 9 个死代码/空气墙 | -9 |
| 拆分 | - | +22 个新文件 | +22 |
| 合并 | - | -6 个（重复函数消除） | -6 |
| **合计** | **66** | **~73** | **+7** |

文件数小幅增加是因为拆分大文件产生了更多职责单一的小文件，但删除了 9 个无价值文件。

### 8.2 总行数变化

| 项目 | 行数 |
|------|------|
| 重构前总计 | ~16,243 |
| 删除死代码 | -530 |
| 消除重复代码 | -1,800 |
| 删除过度文档 | -800 |
| 新增共享模块骨架 | +200 |
| **重构后预估** | **~13,313** |

### 8.3 入口脚本变化

| 文件 | 重构前 | 重构后 |
|------|--------|--------|
| `run.m` | 660 | ~100 |
| `run_without_fusion.m` | 199 | ~100 |
| `run_fragment_study.m` | 477 | ~60 |
| `run_random_fade_fragment_fusion.m` | 225 | ~60 |

---

## 九、实施路线图

### Phase 0: 零风险删除（1 天）
1. 删除 4 个死代码 utils 文件
2. 删除 `Fun_UpdateTrackforNoInputPoint_Oracle.m`
3. 删除 `TRACK_MAIN_ORACLE.m`，调用方改直调
4. 删除 `haversine_ukf`/`haversine_km_eval`/`haversine_km_local`/`haversine_km`
5. 删除 `regularize_cov_ukf`/`regularize_cov_imm`
6. 删除 `skywave_geometry/azimuth_impl`
7. 删除 4 个 wrap_angle 变体，新建 `utils/wrap_angle.m`
8. 删除 `coord_systems_lla_to_ecef.m` 等 4 个无调用文件

### Phase 1: 提取共享模块（2 天）
9. 新建 `utils/wrap_angle.m`
10. 新建 `io/print_summary.m`，所有入口脚本的 print 函数迁入
11. 新建 `simulation/clutter_generator.m`
12. 新建 `visualization/beam_sector.m`
13. 新建 `ukf/ct_model.m` + `ukf/meas_inverse.m`
14. 新建 `simulation/skywave_refine.m`

### Phase 2: 拆分大文件（3 天）
15. 拆分 `run.m` → 主流程 + 6 个辅助文件
16. 拆分 `run_fragment_study.m` → 3 个文件
17. 拆分 `ukf_jichu.m` → 2 个文件
18. 拆分 `prepare_oracle_tracking_inputs.m` → 3 个文件
19. 拆分 `bridge_smoother.m` → 2 个文件
20. 拆分 `evaluate_all_multi.m` → 2 个文件
21. 拆分 `plot_results_multi.m` → 3 个文件
22. 拆分 `plot_tracks_without_fusion.m` → 3 个文件
23. 拆分 `plan_controlled_fragmentation.m` → 2 个文件

### Phase 3: 重命名 + 内联（1 天）
24. 重命名 `Fun_UpdateTrackByAsscResult_Oracle.m` → `track_update_oracle.m`
25. 重命名 `post_init_multi.m` → `ukf/ukf_post_init.m`
26. 重命名 `dual_threshold_decide.m` → `dual_threshold_decision.m`
27. 内联 `fun_remove_assc_pts_from_pointlist_oracle.m`
28. 内联 `sortTrackList_oracle.m`
29. 标记 `fusion/track_matcher.m` 为 deprecated

### Phase 4: 验证（1 天）
30. 运行 `validation/run_filter_math_tests.m`
31. 运行 `validation/run_fragment_fusion_tests.m`
32. 运行 `validation/run_oracle_lifecycle_tests.m`
33. 运行 `validation/validate_oracle_invariants.m`
34. 依次运行 4 个入口脚本，确认输出一致

**总计：约 8 天工作量（单人）**

---

## 十、关键发现总结

### 10.1 重复代码量化

| 重复类型 | 重复次数 | 重复行数 | 消除后可节省 |
|---------|---------|---------|------------|
| Haversine 距离 | 5 处 | ~80 行 | -55 行 |
| 协方差正则化 | 3 处 | ~90 行 | -60 行 |
| 角度归一化 | 5 处 | ~30 行 | -24 行 |
| 方位角计算 | 2 处 | ~10 行 | -7 行 |
| 杂波生成 | 2 处 | ~130 行 | -65 行 |
| 波束扇区绘制 | 2 处 | ~75 行 | -38 行 |
| 天波迭代精化 | 2 处 | ~40 行 | -20 行 |
| 入口脚本 print 函数 | 3 处 | ~120 行 | -80 行 |
| truth_tracks 拆分 | 4 处 | ~40 行 | -30 行 |
| **合计** | | **~620 行** | **~379 行** |

### 10.2 架构债务根因

1. **"功能堆叠"模式** — 每次加新功能就往现有文件里塞，没有架构层面整理
2. **入口脚本膨胀** — 4 个入口脚本中包含 ~1600 行代码，但真正的主流程不到 400 行（25%）
3. **隐式数据契约** — 模块间通过巨型 struct 的字段名隐式通信，没有正式接口 spec
4. **同名不同义** — `calibrate_bias`、`wrap_angle`、`dual_threshold_decide` 等多处同名但实现不同
5. **缺乏 dead code 清理** — 4 个 utils 文件和 1 个 tracker 文件长期无人调用

### 10.3 不建议重构的项目

| 项目 | 理由 |
|------|------|
| 统一 4 个入口脚本 | 四条不同研究流水线，强行统一会产生巨型参数 struct |
| 拆分 `tracklet_grouping.m` | 22 个内部函数共享同一数据流，拆分破坏内聚性 |
| 删除 `fusion/track_matcher.m` | 与 `track_matcher_dualgate` 服务不同 pipeline |
| 删除 `ukf/adapt_q.m` | 281 行但有独立价值，与 IMM 内部的自适应逻辑场景不同 |
