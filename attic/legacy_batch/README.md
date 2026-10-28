# attic/legacy_batch —— 旧批处理链路归档

**归档日期**：2026-10-28
**归档原因**：主链路已改为全在线（`run.m` 九阶段逐帧推进），
旧代码里"整段匹配 + 整段融合"的实现不再被主链路调用。

对应文档：`docs/主链路在线化重构契约_20261028.md`。

---

## 这里面是什么

| 文件 | 原作用 | 为什么不再用 |
|---|---|---|
| `run_without_fusion.m` | 无融合基线入口（整段跑单站跟踪） | 新主链路用 `run.m` + `params.online_fusion_methods` 控制 |
| `track_matcher.m` | 最早的跨站匹配器 | 被三判据 / MEWF 取代 |
| `track_matcher_dualgate.m` | 三判据双门限匹配（整段） | 在线版本在 `tracker/+online_assoc/` |
| `track_matcher_dualgate_fast.m` | 上面那个的索引化加速版 | 同上 |
| `track_matcher_mewf.m` | 多证据加权融合匹配（整段） | 在线版本在 `tracker/+online_assoc/` |
| `track_matcher_nn.m` | 最近邻基线（整段） | 在线版本在 `tracker/+online_assoc/` |
| `track_matcher_sequential.m` | 序贯最近邻基线（整段） | 在线版本在 `tracker/+online_assoc/` |
| `dual_threshold_decide.m` | 双门限判决内核 | 被在线 M/N 序贯确认取代 |
| `fuse_estimate_sequence.m` | 动态航迹组四算法融合引擎 | 在线版本在 `fusion/+online_fuse/` |
| `run_track_fusion.m` | 旧 pair→group 融合适配器 | 同上 |
| `tracklet_grouping.m` | 片段整数规划分组 | 碎片研究线整体废弃 |
| `plan_controlled_fragmentation.m` | 人为制造航迹碎片 | 同上 |
| `build_faded_track_segments.m` | 抽取航迹片段 | 同上 |
| `run_fragment_study.m` / `run_random_fade_fragment_fusion.m` | 碎片研究入口 | 同上 |
| `evaluate_fragment_fusion_multi.m` / `evaluate_fusion_track_length.m` | 碎片/长度评估 | 长度评估改用 `evaluate_online_track_length.m` |
| `compare_pairing_methods.m` | 四配对器离线对比 | 改为在线链路上的对比（见 `experiments/`） |
| `plot_fragment_study_dashboard.m` | 碎片看板 | 随碎片线废弃 |
| `run_fragment_fusion_tests.m` | 碎片测试 | 同上 |

## 重要说明

**归档 ≠ 数值可比。** 旧批处理匹配使用了整段序列的信息
（候选回溯、区间前伸、全局 quality 排序），在线版本的判决只看当前帧及以前。
因此**同一场景同一配置下，新旧链路的配对结果与 RMSE 会有差异**，
这不是回归，而是把非因果判决改成因果判决的必然结果。

历史实验数据（`results/paper/`）是旧链路产出的，已在
`experiments/run_experiments.m` 的注释里标注。用在线链路重跑时请使用
新的 exp_id 前缀（如 `E9_baseline_online`），不要覆盖旧结果。

## 还能用吗

能用，但要自己把路径加上 `attic/legacy_batch`。例如复现论文里
"MEWF 整段匹配 vs 三判据"的对比表时：

```matlab
addpath('attic/legacy_batch');
p_dg  = track_matcher_dualgate_fast(R1, R2, params);
p_mewf = track_matcher_mewf(R1, R2, params);
```
