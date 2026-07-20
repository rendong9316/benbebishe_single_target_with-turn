function result = run_random_fade_fragment_fusion(config)
% RUN_RANDOM_FADE_FRAGMENT_FUSION 可控多目标碎片实验。
%
% 【实验流程】
%   1. 运行基线跟踪（无衰落，每个目标每雷达 1 个完整航迹）
%   2. 对每个雷达独立施加可控衰落（移除窗口内的检测）
%   3. 重新运行跟踪 → 航迹被切成多个片段
%   4. 提取片段 → 构建边 → 枚举假设 → 整数规划分组
%   5. 对每个 group 执行四种融合算法
%   6. 外部评估：RMSE、覆盖率、纯度
%
% 【config 参数】
%   scenario_name — 场景名（默认 multi_cross）
%   show_figures  — 是否显示图窗
%   save_result   — 是否保存 .mat
%   verbose       — 是否打印详细日志

if nargin < 1, config = struct(); end  % 无配置时使用空结构体
config = defaults(config);  % 填充默认值
addpath(genpath('.'));  % 加入搜索路径
inputs = prepare_oracle_tracking_inputs(config.scenario_name);  % 准备输入数据
params = inputs.params;

% ===== 步骤 1: 基线跟踪（无衰落） =====
[base_r1, ~, base_snap_r1] = run_oracle_tracker_sequence( ...
    inputs.detList_R1, inputs.ukf1_tpl, inputs.params_r1, ...
    inputs.truth_all, inputs.t1_grid, false);
[base_r2, ~, base_snap_r2] = run_oracle_tracker_sequence( ...
    inputs.detList_R2, inputs.ukf2_tpl, inputs.params_r2, ...
    inputs.truth_all, inputs.t2_grid, false);

% ===== 步骤 2: 可控衰落 =====
% 对每个雷达独立施加衰落，生成片段化检测列表和衰落方案
[det_r1, plan_r1, validation_r1, status_r1] = plan_controlled_fragmentation( ...
    inputs.detList_R1, base_r1, base_snap_r1, inputs.ukf1_tpl, ...
    inputs.params_r1, inputs.truth_all, inputs.t1_grid, 1, params);
[det_r2, plan_r2, validation_r2, status_r2] = plan_controlled_fragmentation( ...
    inputs.detList_R2, base_r2, base_snap_r2, inputs.ukf2_tpl, ...
    inputs.params_r2, inputs.truth_all, inputs.t2_grid, 2, params);

fragment_plan = struct('R1', plan_r1, 'R2', plan_r2);  % 衰落方案
fragment_validation = struct('R1', validation_r1, 'R2', validation_r2);  % 验证结果
fixture_status = first_failure(status_r1, status_r2);  % 检查是否有失败
if ~strcmp(fixture_status, 'SUCCESS')  % 衰落方案构建失败
    result = fixture_failure_result(fixture_status, config, params, inputs, ...
        det_r1, det_r2, fragment_plan, fragment_validation);
    print_summary(result);
    return;
end

% ===== 步骤 3: 衰落后的跟踪 =====
[tracks_r1, temp_r1, snapshots_r1, diag_r1] = run_oracle_tracker_sequence( ...
    det_r1, inputs.ukf1_tpl, inputs.params_r1, inputs.truth_all, ...
    inputs.t1_grid, config.verbose);
[tracks_r2, temp_r2, snapshots_r2, diag_r2] = run_oracle_tracker_sequence( ...
    det_r2, inputs.ukf2_tpl, inputs.params_r2, inputs.truth_all, ...
    inputs.t2_grid, config.verbose);
% 验证 Oracle 不变量
validate_oracle_invariants(snapshots_r1, det_r1, diag_r1, inputs.params_r1, tracks_r1);
validate_oracle_invariants(snapshots_r2, det_r2, diag_r2, inputs.params_r2, tracks_r2);

% ===== 步骤 4: 片段提取 + 时间对齐 + 分组 =====
segments_r1 = build_faded_track_segments('extract', snapshots_r1, tracks_r1, 1);  % R1 片段
segments_r2 = build_faded_track_segments('extract', snapshots_r2, tracks_r2, 2);  % R2 片段
aligned_r2 = time_align_tracks(snapshots_r2, params);  % R2 时间对齐
aligned_segments_r2 = build_faded_track_segments('extract', aligned_r2, tracks_r2, 2);  % 对齐后片段
segments = [segments_r1, aligned_segments_r2];  % 合并
for i = 1:numel(segments), segments(i).segment_id = i; end  % 重新编号

segment_truth_labels = build_segment_truth_sidecar(segments, tracks_r1, tracks_r2);  % 片段真值标签
matching_params = params;
if isfield(matching_params, 'fragmentation')  % 分组不应使用碎片化参数
    matching_params = rmfield(matching_params, 'fragmentation');
end
grouping = tracklet_grouping('segments', segments, matching_params);  % 片段分组

% ===== 步骤 5: 融合 =====
fusion_results = struct([]);
if strcmp(grouping.status, 'SUCCESS')  % 分组成功则执行融合
    for g = 1:numel(grouping.groups)
        item = fuse_estimate_sequence(grouping.groups(g), segments, matching_params);  % 四算法融合
        if g == 1
            fusion_results = item;
        else
            fusion_results(g) = item;
        end
    end
end

% ===== 步骤 6: 评估 =====
evaluation = evaluate_fragment_fusion_multi(fusion_results, grouping.groups, ...
    segments, segment_truth_labels, inputs.truthTrajs, inputs.t1_grid);
status = derive_status(segments, grouping, fusion_results);  % 推导状态
study = struct('segments', segments, 'groups', grouping.groups, ...
    'edges', grouping.edges, 'fusion_results', fusion_results, ...
    'evaluation', evaluation);

result = struct('status', status, 'config', config, 'params', params, ...
    'scenario', inputs.scenario, 'truth_all', {inputs.truth_all}, ...
    'truthTrajs', {inputs.truthTrajs}, 'detList_R1', {det_r1}, ...
    'detList_R2', {det_r2}, 'fragment_plan', fragment_plan, ...
    'fragment_validation', fragment_validation, ...
    'trackList_R1', {tracks_r1}, 'trackList_R2', {tracks_r2}, ...
    'tempTrackList_R1', temp_r1, 'tempTrackList_R2', temp_r2, ...
    'trackSnapshots_R1', {snapshots_r1}, 'trackSnapshots_R2', {snapshots_r2}, ...
    'aligned_R2', {aligned_r2}, 'diag_R1', {diag_r1}, 'diag_R2', {diag_r2}, ...
    'segments_R1', segments_r1, 'segments_R2', segments_r2, ...
    'segments', segments, 'segment_truth_labels', segment_truth_labels, ...
    'grouping', grouping, 'fusion_results', fusion_results, ...
    'evaluation', evaluation);

print_summary(result);  % 打印摘要
if config.show_figures  % 可视化
    [track_a, track_b, track_c] = truth_tracks_for_legacy(inputs.truth_all);
    plot_scene_overview_multi(track_a, track_b, track_c, params, 'results');
    plot_point_cloud_3d(det_r1, 'R1', '');
    plot_point_cloud_3d(det_r2, 'R2', '');
    plot_tracks_without_fusion(inputs.truth_all, det_r1, det_r2, ...
        snapshots_r1, snapshots_r2, tracks_r1, tracks_r2, params, study);
end
if config.save_result  % 保存结果
    if ~exist('results', 'dir'), mkdir('results'); end
    save(fullfile('results', 'random_fade_fragment_fusion.mat'), '-struct', 'result');
end
end

function labels = build_segment_truth_sidecar(segments, tracks_r1, tracks_r2)
% build_segment_truth_sidecar 为每个片段标注其所属的真实目标。
% 通过 track_id 反查航迹列表中的 truth_idx。
labels = nan(1, numel(segments));  % 初始化标签数组
for i = 1:numel(segments)
    if segments(i).radar_id == 1, tracks = tracks_r1; else, tracks = tracks_r2; end  % 选择对应雷达的航迹
    for j = 1:numel(tracks)
        if double(tracks{j}.id) == segments(i).track_id  % 找到匹配的航迹
            labels(i) = double(tracks{j}.truth_idx);  % 提取真值索引
            break;
        end
    end
end
end

function status = derive_status(segments, grouping, fusion_results)
% derive_status 推导实验的最终状态。
% 按优先级检查：片段 → 分组 → 融合
if isempty(segments)
    status = 'NO_EFFECTIVE_SEGMENTS';  % 没有有效片段
elseif ~strcmp(grouping.status, 'SUCCESS')
    status = grouping.status;  % 分组失败
elseif isempty(grouping.groups)
    status = 'NO_GROUPS';  % 分组成功但没有产生任何组
elseif numel(fusion_results) ~= numel(grouping.groups)
    status = 'FUSION_FAILED';  % 融合结果数量与组数不匹配
else
    status = 'SUCCESS';  % 全部成功
end
end

function result = fixture_failure_result(status, config, params, inputs, ...
    det_r1, det_r2, fragment_plan, fragment_validation)
% fixture_failure_result 构建失败状态的完整结果结构体。
% 即使衰落方案构建失败，也需要返回完整的结构以便调试。
empty_grouping = struct('status', 'NOT_RUN', 'segments', struct([]), ...
    'edges', struct([]), 'candidate_diagnostics', struct([]), ...
    'hypotheses', struct([]), 'groups', struct([]), ...
    'solver', struct('status', 'NOT_RUN'));
result = struct('status', status, 'config', config, 'params', params, ...
    'scenario', inputs.scenario, 'truth_all', {inputs.truth_all}, ...
    'truthTrajs', {inputs.truthTrajs}, 'detList_R1', {det_r1}, ...
    'detList_R2', {det_r2}, 'fragment_plan', fragment_plan, ...
    'fragment_validation', fragment_validation, 'segments', struct([]), ...
    'grouping', empty_grouping, 'fusion_results', struct([]), ...
    'evaluation', struct([]));
end

function status = first_failure(a, b)
% first_failure 返回两个状态中第一个非 SUCCESS 的（优先级：a > b）
if ~strcmp(a, 'SUCCESS'), status = a; else, status = b; end
end

function print_summary(result)
% print_summary 打印可控碎片实验的摘要信息。
fprintf('可控碎片实验: %s%s', result.status, newline);
if isfield(result, 'fragment_plan')  % 打印衰落事件数
    fprintf('R1事件=%d, R2事件=%d, 目标数=%d, 每目标每站K=%d%s', ...
        numel(result.fragment_plan.R1.events), numel(result.fragment_plan.R2.events), ...
        result.scenario.n_targets, ...
        result.params.fragmentation.segments_per_target_per_radar, newline);
end
if ~strcmp(result.status, 'SUCCESS'), return; end  % 失败时不打印详细结果
% 打印片段/边/假设/组的统计
fprintf('片段=%d, 接受边=%d, hypotheses=%d, groups=%d%s', ...
    numel(result.segments), numel(result.grouping.edges), ...
    numel(result.grouping.hypotheses), numel(result.grouping.groups), newline);
for g = 1:numel(result.evaluation.groups)  % 逐组打印评估结果
    e = result.evaluation.groups(g);
    fprintf('Group%d -> Truth%s, purity=%.2f, best=%s %.2fkm%s', ...
        e.group_id, scalar_label(e.truth_idx), e.purity, e.best_method, ...
        e.best_rmse_km, newline);
end
end

function label = scalar_label(value)
% scalar_label 将值转为字符串：有限值 → 数字字符串，NaN → 'unmatched'
if isfinite(value), label = sprintf('%d', value); else, label = 'unmatched'; end
end

function [track_a, track_b, track_c] = truth_tracks_for_legacy(truth_all)
% truth_tracks_for_legacy 将 truth_all 拆分为 A/B/C 三个变量（最多 3 个目标）
empty_track = nan(1, 5);
track_a = empty_track; track_b = empty_track; track_c = empty_track;
if numel(truth_all) >= 1, track_a = truth_all{1}; end
if numel(truth_all) >= 2, track_b = truth_all{2}; end
if numel(truth_all) >= 3, track_c = truth_all{3}; end
end

function config = defaults(config)
% defaults 填充配置默认值：用户传入的字段覆盖默认值
defaults_local = struct('scenario_name', 'multi_cross', ...
    'show_figures', true, 'save_result', true, 'verbose', true);
names = fieldnames(defaults_local);
for i = 1:numel(names)
    if ~isfield(config, names{i}), config.(names{i}) = defaults_local.(names{i}); end
end
end
