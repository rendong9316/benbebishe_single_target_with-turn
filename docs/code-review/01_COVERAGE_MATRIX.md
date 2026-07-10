# MATLAB 源码审查覆盖矩阵

> 基线提交：`7c166d41541ccd74f23fd6c3ea0b871d8603950e`  
> 生成日期：2026-07-10  
> 原始范围：131 个 `.m` 文件。`可审查行` 当前按非空物理行统计；随着逐语句块映射建立，注释块和结构分隔符会进一步分类，但分母调整必须留痕。

## 状态说明

- `NOT_STARTED`：尚无权威逐语句块正文。
- `IN_PROGRESS`：已覆盖部分代码，并记录下一游标。
- `REVIEWED`：全部可审查行均映射到审查块，但仍可能存在待运行结论。
- `VERIFIED`：覆盖完成且关键结论达到规定证据等级。
- `STALE_NEEDS_REREVIEW`：文件哈希或稳定指纹变化，需要重新定位和复核。

## 文件清单

| 文件 ID | 路径 | 模块 | SHA-256 前16位 | 物理行 | 可审查行 | 已覆盖行 | 覆盖率 | 状态 | Issue | 权威正文 |
|---|---|---|---|---:|---:|---:|---:|---|---|---|
| CR-FILE-001 | `_extract_data.m` | root | 23029d36c32ba932 | 57 | 57 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-002 | `_get_precise.m` | root | 176c6995769ef911 | 28 | 25 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-003 | `analyze_cov_simple.m` | root | 55acaebdb561cc03 | 44 | 35 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-004 | `analyze_covariance.m` | root | 9368b0cbba3a8430 | 112 | 96 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-005 | `association/jpda_multi.m` | association | 4329a937f8f8a490 | 139 | 123 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-006 | `association/nn_associate.m` | association | 1466c60fbc12e321 | 110 | 96 | 96 | 100.00% | REVIEWED | V04b-P2-01… | `volumes/V04b_NN_ASSOCIATE.md` |
| CR-FILE-007 | `association/pda_weight.m` | association | f8dda69c04f1a10e | 97 | 87 | 87 | 100.00% | REVIEWED | V04-P1-01… | `volumes/V04_PDA_WEIGHT.md` |
| CR-FILE-008 | `config/simulation_params.m` | config | 1e37d325e0e6c25a | 545 | 476 | 476 | 100.00% | REVIEWED | V01-P0-01… | `volumes/V01_SIMULATION_PARAMS.md` |
| CR-FILE-009 | `config/simulation_params_multi.m` | config | b602db07016724f0 | 36 | 32 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-010 | `evaluation/evaluate_all.m` | evaluation | 526880ca08561f4a | 361 | 331 | 331 | 100.00% | REVIEWED | V07-P1-01… | `volumes/V07_EVALUATE_ALL.md` |
| CR-FILE-011 | `evaluation/evaluate_all_multi.m` | evaluation | 6d097638876ba084 | 380 | 348 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-012 | `fusion/regularize_cov.m` | fusion | f4ab48d34baef10a | 154 | 145 | 145 | 100.00% | REVIEWED | V06c-P2-01 | `volumes/V06c_REGULARIZE_COV.md` |
| CR-FILE-013 | `fusion/run_track_fusion.m` | fusion | 90174ecb5ddacf33 | 305 | 273 | 273 | 100.00% | REVIEWED | V06-P1-01… | `volumes/V06_RUN_TRACK_FUSION.md` |
| CR-FILE-014 | `fusion/time_align_tracks.m` | fusion | d7a96ed1585a5db5 | 135 | 123 | 123 | 100.00% | REVIEWED | V05-P1-01… | `volumes/V05_TIME_ALIGN_TRACKS.md` |
| CR-FILE-015 | `fusion/track_fusion_algorithms.m` | fusion | afbabea20861bddd | 421 | 387 | 387 | 100.00% | REVIEWED | V06b-P1-01… | `volumes/V06b_TRACK_FUSION_ALGORITHMS.md` |
| CR-FILE-016 | `fusion/track_matcher.m` | fusion | ddf78a62718ef539 | 435 | 380 | 380 | 100.00% | REVIEWED | V10-P1-01… | `volumes/V10_TRACK_MATCHER.md` |
| CR-FILE-017 | `imm/imm_tracker.m` | imm | 393d5f4a18e2c621 | 545 | 483 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-018 | `initiation/track_initiation.m` | initiation | 7c92f315fb34ea4b | 147 | 130 | 130 | 100.00% | REVIEWED | V05b-P1-01…V14-P1-01… | `volumes/V05b_TRACK_INITIATION.md`,`volumes/V14_TRACK_INITIATION_SUPPLEMENT.md` |
| CR-FILE-019 | `io/extract_measurement_field.m` | io | 89a3f31b8bee1e89 | 90 | 85 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-020 | `io/load_adsb.m` | io | 7157e2a08b3c88ec | 321 | 291 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-021 | `io/save_all.m` | io | 33c313a1a30d52dc | 331 | 295 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-022 | `nanyang/cleanTrackList.m` | nanyang | a4c593766023f3a6 | 46 | 39 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-023 | `nanyang/det2nanyang_point.m` | nanyang | aa6923848b56fd8f | 114 | 105 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-024 | `nanyang/det2trackDataConverter.m` | nanyang | ede4a6865f952d39 | 334 | 305 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-025 | `nanyang/distance.m` | nanyang | 17f5ccd668f33791 | 38 | 35 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-026 | `nanyang/fun_assign_batchNo_to_new_track.m` | nanyang | 4192f0812922cdde | 25 | 23 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-027 | `nanyang/fun_calculate_track_travelLen.m` | nanyang | e1b89f22ff805456 | 7 | 7 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-028 | `nanyang/fun_check_35logic_points_improved.m` | nanyang | 7c77e9d3d2206214 | 172 | 138 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-029 | `nanyang/fun_check_colinear_points.m` | nanyang | 97c955c159a3ea7d | 162 | 145 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-030 | `nanyang/fun_check_track_validation.m` | nanyang | 2ac134ee50268659 | 145 | 132 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-031 | `nanyang/fun_create_new_track.m` | nanyang | aeca9f627a4bb59c | 292 | 264 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-032 | `nanyang/fun_fill_smooth_list_by_alpha_beta_filter.m` | nanyang | 497be61a63cb600c | 59 | 50 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-033 | `nanyang/fun_fill_smooth_list_by_predict_result.m` | nanyang | a0916f10e83ce3ee | 38 | 31 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-034 | `nanyang/fun_find_tracks_to_report.m` | nanyang | 6f149acbbeb892c5 | 68 | 60 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-035 | `nanyang/Fun_PredictNextStep_CV.m` | nanyang | f84fcd0c6e5cc084 | 10 | 8 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-036 | `nanyang/fun_remove_assc_pts_from_pointlist.m` | nanyang | 0e81b870c7b2cbdd | 42 | 36 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-037 | `nanyang/fun_select_point_by_rd.m` | nanyang | 76abf3b1a40fe39e | 15 | 12 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-038 | `nanyang/fun_select_track_by_rd.m` | nanyang | 49a557cc651d7c23 | 20 | 17 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-039 | `nanyang/fun_set_tracking_parameter.m` | nanyang | cbdc5fc68fd33d45 | 11 | 9 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-040 | `nanyang/fun_track_quality_management_and_info_completion.m` | nanyang | 76229170c533e3a9 | 148 | 133 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-041 | `nanyang/fun_trackfilter_AlphaBeta.m` | nanyang | c9718884951759c9 | 272 | 230 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-042 | `nanyang/Fun_UpdateTrackByAsscResult.m` | nanyang | cb0d5aa0aac902ce | 75 | 64 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-043 | `nanyang/Fun_UpdateTrackforNoInputPoint.m` | nanyang | 286c43243d874b8d | 26 | 22 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-044 | `nanyang/header.m` | nanyang | 82702ce49f0ee4e9 | 188 | 166 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-045 | `nanyang/is_duplicate_track.m` | nanyang | ac000b934d47f83a | 24 | 18 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-046 | `nanyang/pdCoefInterprator.m` | nanyang | 6ab8ab7b8f0e6a80 | 61 | 53 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-047 | `nanyang/PointTrackAssociation_JNN.m` | nanyang | 1f99894ae0655253 | 161 | 147 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-048 | `nanyang/predictNextStep_cv.m` | nanyang | 1f8b3ce32e2e9149 | 137 | 121 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-049 | `nanyang/reckon.m` | nanyang | 9b1e1cf3d486c0ad | 19 | 18 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-050 | `nanyang/resetAllTracks.m` | nanyang | 17efeac212697e9a | 32 | 28 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-051 | `nanyang/robustMinSquareErr.m` | nanyang | 5ee34230b1f7643c | 49 | 33 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-052 | `nanyang/sortTrackList.m` | nanyang | eb6161a718bcd053 | 114 | 106 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-053 | `nanyang/sub_func_for_AsscJNN/calculate_cost_of_point_track_pair.m` | nanyang | e0c2154deb79c8dd | 47 | 43 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-054 | `nanyang/sub_func_for_AsscJNN/candidate_matrix_selection.m` | nanyang | d4a583edeb64f65b | 36 | 35 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-055 | `nanyang/sub_func_for_AsscJNN/convert_bigraph_into_matrix.m` | nanyang | c9376834349e8181 | 40 | 38 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-056 | `nanyang/sub_func_for_AsscJNN/determine_if_point_within_the_scope_of_track.m` | nanyang | 3e5ed740ec7ba3c8 | 37 | 33 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-057 | `nanyang/sub_func_for_AsscJNN/extract_sub_bigraph.m` | nanyang | 58bc26550eb01cdd | 99 | 96 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-058 | `nanyang/sub_func_for_AsscJNN/get_list_index_by_matrix_index_for_vertex_point.m` | nanyang | bfa03f31bdddc344 | 27 | 25 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-059 | `nanyang/sub_func_for_AsscJNN/get_list_index_by_matrix_index_for_vertex_track.m` | nanyang | 9e584f470700531d | 27 | 25 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-060 | `nanyang/sub_func_for_AsscJNN/get_matrix_index_by_list_index_for_vertex_point.m` | nanyang | ff19275d6fca25b0 | 26 | 24 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-061 | `nanyang/sub_func_for_AsscJNN/get_the_cost_of_match_plan.m` | nanyang | 3e9d7e620d912684 | 49 | 44 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-062 | `nanyang/sub_func_for_AsscJNN/get_tracking_gate.m` | nanyang | 6035ed61e05c65a3 | 32 | 31 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-063 | `nanyang/sub_func_for_AsscJNN/mat_division.m` | nanyang | bbee137991247ce9 | 64 | 62 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-064 | `nanyang/tool_calculate_distance.m` | nanyang | ddf9ec514a90121f | 19 | 13 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-065 | `nanyang/tool_get_time_difference.m` | nanyang | ab23961798d13b61 | 11 | 8 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-066 | `nanyang/tool_header.m` | nanyang | c588794f2121cb5b | 9 | 8 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-067 | `nanyang/tool_radar2blh_fake_monostatic.m` | nanyang | 0dcf3203157ff609 | 29 | 25 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-068 | `nanyang/tool_radar2xoy_pd.m` | nanyang | a07060c4372259a6 | 53 | 43 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-069 | `nanyang/track2reportDataConverter.m` | nanyang | 8de752ca2efeedbd | 216 | 206 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-070 | `nanyang/trackStarter_logic.m` | nanyang | f3dd411fc54551f9 | 313 | 264 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-071 | `registration/align_radar_to_grid.m` | registration | e7fdd160c5849336 | 229 | 211 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-072 | `registration/cost_fcn_with_params.m` | registration | 0faafa1d18214f43 | 196 | 174 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-073 | `registration/estimate_biases.m` | registration | c740a6d60afc976d | 528 | 472 | 472 | 100.00% | REVIEWED | V07b-P0-01… | `volumes/V07b_ESTIMATE_BIASES.md` |
| CR-FILE-074 | `run_mc_multi.m` | root | 1cb36d7f598c2c69 | 485 | 433 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-075 | `run_mc_straight.m` | root | d030bc98b965db95 | 562 | 507 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-076 | `run_mc_turn.m` | root | c923b79fbad0a95d | 675 | 609 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-077 | `run_mc_turn_180deg.m` | root | e9fa1671c5ad1635 | 664 | 598 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-078 | `run_mc_turn_180deg_compare.m` | root | 2cd4c1b5178d127b | 748 | 673 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-079 | `run_mc_turn_compare.m` | root | 9b9241e60a93582f | 792 | 715 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-080 | `run_simulation.m` | root | 37115cad3205f3aa | 1367 | 1278 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-081 | `run_simulation_multi.m` | root | d4f4713aee24e2d8 | 924 | 837 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-082 | `run_simulation_turn.m` | root | af10e1a82aa19b27 | 709 | 626 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-083 | `run_simulation_turn_180deg.m` | root | f93d42c3a1ccbe00 | 693 | 611 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-084 | `scan_Pi.m` | root | 82fdcfe87c856365 | 859 | 786 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-085 | `scan_Q_scale.m` | root | 64c59ee2b8aa50ff | 863 | 789 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-086 | `simulation/aircraft_trajectory_create.m` | simulation | 7e96a0ffd290509c | 665 | 603 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-087 | `simulation/aircraft_trajectory_interpolate.m` | simulation | 64ef2ae330250fe2 | 170 | 158 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-088 | `simulation/aircraft_trajectory_locate.m` | simulation | 9ea13fb4ab9157d2 | 104 | 98 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-089 | `simulation/bistatic_inverse_solver.m` | simulation | 48e41779d50f1f56 | 43 | 39 | 39 | 100.00% | REVIEWED | V02b-P1-01… | `volumes/V02b_BISTATIC_INVERSE_SOLVER.md` |
| CR-FILE-090 | `simulation/generate_frame_detections.m` | simulation | 68a5b90d94fc0078 | 230 | 211 | 211 | 100.00% | REVIEWED | V02-P1-01… | `volumes/V02_GENERATE_FRAME_DETECTIONS.md` |
| CR-FILE-091 | `simulation/generate_frame_detections_multi.m` | simulation | ee66ed28d5ff2e6a | 104 | 90 | 90 | 100.00% | REVIEWED | V12-P1-01… | `volumes/V12_GENERATE_FRAME_DETECTIONS_MULTI.md` |
| CR-FILE-092 | `simulation/measurement_simulator.m` | simulation | c53af23da1d13287 | 124 | 111 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-093 | `simulation/radar_coverage_check.m` | simulation | c4d6170f8cbb61eb | 96 | 90 | 90 | 100.00% | REVIEWED | V09-P1-01… | `volumes/V09_RADAR_COVERAGE_CHECK.md` |
| CR-FILE-094 | `simulation/radar_station_true_polar.m` | simulation | 6c5bfaaf31925deb | 57 | 54 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-095 | `simulation/tracker_utils.m` | simulation | 47e1d13b07aa6256 | 240 | 225 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-096 | `tmp_check_coverage.m` | root | c7f8192599a14b56 | 50 | 44 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-097 | `tmp_check_fix.m` | root | 1f7dbbc2b9bcf5ad | 41 | 38 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-098 | `tmp_debug_r2.m` | root | d160594bd9859295 | 59 | 53 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-099 | `tmp_debug_rand.m` | root | e9ab0c3970d2e5d9 | 11 | 10 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-100 | `tmp_fusion_detail.m` | root | bb81fb2594919148 | 17 | 16 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-101 | `tmp_load_turn.m` | root | 08dff28e6dd9150d | 60 | 54 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-102 | `tmp_multi_eval.m` | root | 195b1d7bdadd655a | 27 | 24 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-103 | `tracker/inject_truth_velocity.m` | tracker | 23da7e61a6b20822 | 12 | 12 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-104 | `tracker/multi_track_manager.m` | tracker | c4c8ef7d3607eb34 | 184 | 155 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-105 | `tracker/multi_track_runner_kf.m` | tracker | bcc801ed8220b50c | 196 | 176 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-106 | `tracker/multi_track_start.m` | tracker | d41d7628e27fd6ff | 37 | 35 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-107 | `tracker/post_init_multi.m` | tracker | db31971086172d2e | 12 | 12 | 12 | 100.00% | REVIEWED | V11A-P2-01… | `volumes/V11_POST_INIT_MULTI_UKF_IMM_SUBFUNCS.md` |
| CR-FILE-108 | `tracker/single_track_runner.m` | tracker | 4575f9c0668d48a0 | 328 | 293 | 293 | 100.00% | REVIEWED | V05c-P0-01… | `volumes/V05c_SINGLE_TRACK_RUNNER.md` |
| CR-FILE-109 | `tracker/single_track_runner_adaptive.m` | tracker | 66ec31d21bb7452b | 210 | 180 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-110 | `tracker/single_track_runner_nanyang.m` | tracker | 550473d991afa05e | 371 | 336 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-111 | `tracker/single_track_runner_nanyang_adaptive.m` | tracker | 3bd39cc64782aa99 | 315 | 285 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-112 | `tracker/track_management.m` | tracker | 3f524b7590b855e7 | 155 | 141 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-113 | `ukf/ukf_dispatch.m` | ukf | 5be9c4d6df1ac346 | 38 | 36 | 36 | 100.00% | REVIEWED | V03d-P2-01 | `volumes/V03d_UKF_DISPATCH.md` |
| CR-FILE-114 | `ukf/ukf_imm.m` | ukf | 24c9753c053acbeb | 379 | 328 | 328 | 100.00% | REVIEWED | V03c-P1-01…V13-P1-01… | `volumes/V03c_UKF_IMM.md`,`volumes/V13_UKF_IMM_CORE.md` |
| CR-FILE-115 | `ukf/ukf_jichu.m` | ukf | 6d170bce1fc5c35e | 479 | 401 | 401 | 100.00% | REVIEWED | V03-P1-01… | `volumes/V03_UKF_JICHU.md` |
| CR-FILE-116 | `ukf/ukf_zishiying.m` | ukf | ca9174bd791bdbe7 | 269 | 242 | 242 | 100.00% | REVIEWED | V03b-P1-01… | `volumes/V03b_UKF_ZISHIYING.md` |
| CR-FILE-117 | `utils/coord_systems_lla_to_ecef.m` | utils | 5febd686cb36d371 | 33 | 32 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-118 | `utils/skywave_geometry.m` | utils | fbe3ab47a995e7cb | 179 | 156 | 156 | 100.00% | REVIEWED | V08-P1-01, V08-P2-01… | `volumes/V08_UTILS_AND_GEOMETRY.md` |
| CR-FILE-119 | `utils/sphere_utils_azimuth.m` | utils | ee3c5720b406a6ba | 107 | 101 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-120 | `utils/sphere_utils_destination_point.m` | utils | ba119c2370391451 | 126 | 121 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-121 | `utils/sphere_utils_haversine_distance.m` | utils | d66f056a6187285d | 102 | 97 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-122 | `utils/sphere_utils_interpolate_great_circle.m` | utils | bca0df445452fc2f | 86 | 82 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-123 | `utils/sphere_utils_radial_velocity.m` | utils | 70ab93ba001c5f2e | 131 | 126 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-124 | `utils/sphere_utils_seconds_to_datetime_str.m` | utils | cf7a5b7ae73be7b7 | 73 | 71 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-125 | `visualization/plot_point_cloud_3d.m` | visualization | c2f099cfbcf6df4c | 99 | 90 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-126 | `visualization/plot_results.m` | visualization | cf11099ff0f449c0 | 1269 | 1122 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-127 | `visualization/plot_results_multi.m` | visualization | fc212e8ced0e36ea | 510 | 463 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-128 | `visualization/plot_scene_overview.m` | visualization | 0a6755178aef9128 | 163 | 147 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-129 | `visualization/plot_scene_overview_multi.m` | visualization | 50bff827fd726741 | 75 | 70 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-130 | `visualization/plot_turn_spatial.m` | visualization | 7b3dc96098feb982 | 571 | 496 | 0 | 0.00% | NOT_STARTED | — | — |
| CR-FILE-131 | `visualization/plot_turn_stats.m` | visualization | be381c386553a443 | 765 | 665 | 0 | 0.00% | NOT_STARTED | — | — |

## 当前汇总

- 文件登记率：131/131（100%，仅表示已纳入矩阵）。
- 文件审查完成率：24/131（18.32%）；`skywave_geometry.m`、`ukf_jichu.m`、`ukf_zishiying.m`、`ukf_imm.m`、`ukf_dispatch.m`、`pda_weight.m`、`nn_associate.m`、`run_track_fusion.m`、`track_fusion_algorithms.m`、`regularize_cov.m`、`evaluate_all.m`、`simulation_params.m`、`generate_frame_detections.m`、`bistatic_inverse_solver.m`、`time_align_tracks.m`、`track_initiation.m`、`single_track_runner.m`、`estimate_biases.m`、`radar_coverage_check.m`、`track_matcher.m`、`post_init_multi.m`、`generate_frame_detections_multi.m`、`ukf_imm.m` core（create/init/prepare）、`track_initiation.m` supplement 已完成静态逐语句块覆盖，尚未标记 `VERIFIED`。
- 已覆盖非空源码行：156 + 401 + 242 + 328 + 36 + 87 + 96 + 273 + 387 + 145 + 331 + 476 + 211 + 39 + 123 + 130 + 293 + 472 + 90 + 380 + 12 + 90 + 328 + 130 = 5,221；其余历史稿中仅”提到文件名”的内容不折算为逐行覆盖。
- 快照说明：哈希针对当前工作树内容；源码变化后按治理规则重新定位稳定指纹。
