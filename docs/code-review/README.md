# MATLAB 项目代码审查权威入口

> 当前基线：`7c166d41541ccd74f23fd6c3ea0b871d8603950e`  
> 建立日期：2026-07-10  
> 长期目标：对 131 个 MATLAB 文件完成可审查源码行 100% 映射，并形成不少于 1,000,000 个通过去重和质量门的净中文审查汉字。

## 当前状态

本目录是后续代码审查的唯一权威增量层。`../CODE_REVIEW.md` 与 `../REVIEW_PART2.md` 保留为历史卷，`../PROJECT_DOCS_COMPLETE.md` 是包含大量重复内容的历史材料池。历史稿中的结论只有导入问题台账并取得相应证据后，才可视为当前结论。

- 原始源码范围：131 个 `.m` 文件。
- 已登记：131/131。
- 已完成逐语句块静态审查：24/131（`skywave_geometry.m`, `ukf_jichu.m`, `ukf_zishiying.m`, `ukf_imm.m`, `ukf_dispatch.m`, `pda_weight.m`, `nn_associate.m`, `run_track_fusion.m`, `track_fusion_algorithms.m`, `regularize_cov.m`, `evaluate_all.m`, `simulation_params.m`, `generate_frame_detections.m`, `bistatic_inverse_solver.m`, `time_align_tracks.m`, `track_initiation.m`, `single_track_runner.m`, `estimate_biases.m`, `radar_coverage_check.m`, `track_matcher.m`, `post_init_multi.m`, `generate_frame_detections_multi.m`, `ukf_imm.m` core, `track_initiation.m` supplement，状态 `REVIEWED`）。
- 当前权威增量层有效净汉字：至少 160,000（截至本批完成；最终去重验收值待质量统计完成）。
- 当前阶段：阶段 0 勘误与控制面建立，阶段 1 核心模块示范卷并行编写。

## 导航

1. [审查治理规则](00_REVIEW_GOVERNANCE.md)
2. [131 文件覆盖矩阵](01_COVERAGE_MATRIX.md)
3. [问题台账](02_ISSUE_LEDGER.md)
4. [验证记录](03_VALIDATION_RECORD.md)
5. [历史文档勘误](04_CORRECTION_LOG.md)
6. [术语、符号与单位](05_GLOSSARY_AND_UNITS.md)
7. [分卷正文](volumes/)

## 当前续写游标

- 历史卷一在 `association/pda_weight.m` 的关键公式摘要后中止。
- 首批权威正文已完成：`skywave_geometry.m`（V08）、`ukf_jichu.m`（V03）、`ukf_zishiying.m`（V03b）、`ukf_imm.m`（V03c）、`ukf_dispatch.m`（V03d）、`pda_weight.m`（V04）、`nn_associate.m`（V04b）、`run_track_fusion.m`（V06）、`track_fusion_algorithms.m`（V06b）、`regularize_cov.m`（V06c）、`evaluate_all.m`（V07）、`simulation_params.m`（V01）、`generate_frame_detections.m`（V02）、`bistatic_inverse_solver.m`（V02b）、`time_align_tracks.m`（V05）、`track_initiation.m`（V05b）、`single_track_runner.m`（V05c）、`estimate_biases.m`（V07b）、`radar_coverage_check.m`（V09）、`track_matcher.m`（V10）、`post_init_multi.m` + `ukf_imm` sub-functions（V11）。
- 下一批优先覆盖：`imm/imm_tracker.m`（IMM 跟踪器包装器，life_count 管理和重起始逻辑）、`ukf/ukf_jichu.m` 的 `prepare` 分支（Sigma 点传播，V03 已覆盖但 prepare 细节需深入）、`tracker/single_track_runner_adaptive.m`、`fusion/time_align_tracks.m` 补充审查。
- 每个文件必须在覆盖矩阵中留下下一函数、稳定代码指纹与未关闭 Issue；不能仅写“下次继续”。

## 分卷规划

- V01：配置与入口；V02：仿真；V03：UKF 与 IMM；V04：关联与起始；V05：跟踪器；V06：融合；V07：评估、配准与 I/O；V08：工具与几何；V09：可视化；V10—V11：南阳子系统；V12：根脚本与诊断；V13：跨模块验证。

## 进度声明

百万字是多批次累计验收目标，而不是当前完成量。每批必须同时报告净汉字、源码覆盖、验证覆盖、Issue 状态和重复率；任何未运行的性能判断不得标记为 `B`，任何未运行的行为判断不得标记为 `R`。
