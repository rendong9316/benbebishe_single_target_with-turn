function result = run_random_fade_fragment_fusion(config)
% RUN_RANDOM_FADE_FRAGMENT_FUSION 随机传播衰落驱动的动态片段融合实验。
%
% 【功能概述】
%   模拟雷达信号随机衰落（在指定时间窗口内移除目标检测），观察航迹断裂后
%   能否通过片段重组和融合恢复跟踪。完整流程：
%     1. 生成基线检测点迹（无衰落）
%     2. 对指定目标在随机窗口内施加衰落，得到衰落检测点迹
%     3. 分别对衰落前后执行 Oracle 跟踪
%     4. 提取航迹片段，构建片段图（节点=片段，边=重叠/关联）
%     5. 对多雷达片段组执行四算法融合（SCC/BC/CI/FCI）
%     6. 评估融合效果，输出 RMSE 和状态诊断
%
% 【输入参数】
%   config — 可选结构体，含以下字段：
%     target_id      : 施加衰落的目标编号（默认 1）
%     seed_r1/seeds_r2 : R1/R2 随机衰落种子（默认 1101/2202）
%     show_figures   : 是否显示可视化（默认 true）
%     save_result    : 是否保存结果到 results/（默认 true）
%     verbose        : 是否打印详细信息（默认 true）
%
% 【返回值】
%   result — 结构体，包含所有中间结果、片段信息、融合结果和状态诊断
%
if nargin < 1, config = struct(); end
config = defaults(config);                           % 合并默认配置参数
addpath(genpath('.'));                                % 添加所有子目录到 MATLAB 路径
% 使用 multi_cross 场景（多目标交叉）
inputs = prepare_oracle_tracking_inputs('multi_cross');
params = inputs.params;                               % 提取全局参数

% 对 R1/R2 分别执行 Oracle 跟踪，得到基线航迹和快照
[base_r1, ~, base_snap_r1] = run_oracle_tracker_sequence(inputs.detList_R1, inputs.ukf1_tpl, inputs.params_r1, inputs.truth_all, inputs.t1_grid, false);
[base_r2, ~, base_snap_r2] = run_oracle_tracker_sequence(inputs.detList_R2, inputs.ukf2_tpl, inputs.params_r2, inputs.truth_all, inputs.t2_grid, false);
% 对 R1/R2 分别施加随机衰落：在指定窗口内移除 target_id 的检测
[det_r1, fade_r1, status_r1] = build_faded_track_segments('apply_fade', inputs.detList_R1, base_snap_r1, base_r1, config.target_id, params.tracker_K_loss, config.seed_r1, 1);
[det_r2, fade_r2, status_r2] = build_faded_track_segments('apply_fade', inputs.detList_R2, base_snap_r2, base_r2, config.target_id, params.tracker_K_loss, config.seed_r2, 2);
% 衰落窗口非法则提前退出
if ~strcmp(status_r1, 'SUCCESS') || ~strcmp(status_r2, 'SUCCESS')
    result = struct('status', first_failure(status_r1, status_r2), 'fade_R1', fade_r1, 'fade_R2', fade_r2); return;
end

% 对衰落后的检测点迹执行 Oracle 跟踪
[tracks_r1, temp_r1, snapshots_r1, diag_r1] = run_oracle_tracker_sequence(det_r1, inputs.ukf1_tpl, inputs.params_r1, inputs.truth_all, inputs.t1_grid, config.verbose);
[tracks_r2, temp_r2, snapshots_r2, diag_r2] = run_oracle_tracker_sequence(det_r2, inputs.ukf2_tpl, inputs.params_r2, inputs.truth_all, inputs.t2_grid, config.verbose);
% 从快照中提取航迹片段（segment）
segments_r1 = build_faded_track_segments('extract', snapshots_r1, tracks_r1, 1);
segments_r2 = build_faded_track_segments('extract', snapshots_r2, tracks_r2, 2);
% 将 R2 航迹对齐到 R1 时间基准后再提取片段
aligned_r2 = time_align_tracks(snapshots_r2, params);
aligned_segments_r2 = build_faded_track_segments('extract', aligned_r2, tracks_r2, 2);
% 合并 R1/R2 片段
segments = [segments_r1, aligned_segments_r2];
% 对片段进行聚类分组（基于重叠和关联关系）
grouping = tracklet_grouping('segments', segments, params);
groups = grouping.groups;

fusion_results = cell(0, 1);                       % 初始化融合结果列表
published = cell(0, 1);                            % 初始化发布结果列表（最佳方法）
for g = 1:numel(groups)                            % 遍历每个片段组
    % 检查是否为可融合组：需要同时包含两雷达且有跨雷达关联边
    if ~is_fusion_group(groups(g), segments, grouping.edges), continue; end
    % 对该组执行四算法融合
    fr = fuse_estimate_sequence(groups(g), segments, params);
    % 用真值评估每种融合方法的 RMSE
    fr = evaluate_methods(fr, inputs.truthTrajs{config.target_id}, inputs.t1_grid);
    fusion_results{end+1} = fr; %#ok<AGROW>       % 保存融合结果
    % 记录最佳方法的结果
    if ~isempty(fr.best_method)
        idx = find(strcmp({fr.methods.method}, fr.best_method), 1);
        published{end+1} = struct('group_id', fr.group_id, 'method', fr.best_method, ...
            'rmse_km', fr.best_rmse_km, 'snapshots', {fr.methods(idx).snapshots}); %#ok<AGROW>
    end
end

published = [published{:}];                        % 合并发布结果结构体数组
% 推导实验状态：检查片段、边、融合等各环节是否成功
status = derive_status(segments, grouping.edges, groups, fusion_results, ...
    fade_r1, fade_r2, tracks_r1, tracks_r2);
study = struct('segments', segments, 'published', published);
% 组装返回结果结构体，包含所有中间数据
result = struct('status', status, 'config', config, 'params', params, 'scenario', inputs.scenario, ...
    'truth_all', {inputs.truth_all}, 'truthTrajs', {inputs.truthTrajs}, ...
    'detList_R1', {det_r1}, 'detList_R2', {det_r2}, 'fade_R1', fade_r1, 'fade_R2', fade_r2, ...
    'trackList_R1', {tracks_r1}, 'trackList_R2', {tracks_r2}, 'tempTrackList_R1', temp_r1, 'tempTrackList_R2', temp_r2, ...
    'trackSnapshots_R1', {snapshots_r1}, 'trackSnapshots_R2', {snapshots_r2}, 'aligned_R2', {aligned_r2}, ...
    'diag_R1', {diag_r1}, 'diag_R2', {diag_r2}, 'segments_R1', segments_r1, 'segments_R2', segments_r2, ...
    'grouping', grouping, 'fusion_results', {fusion_results}, 'published', published);
print_summary(result);                           % 打印实验摘要信息
% 可选可视化
if config.show_figures
    [track_a, track_b, track_c] = truth_tracks_for_legacy(inputs.truth_all);
    plot_scene_overview_multi(track_a, track_b, track_c, params, 'results');
    plot_point_cloud_3d(det_r1, 'R1', '');
    plot_point_cloud_3d(det_r2, 'R2', '');
    plot_tracks_without_fusion(inputs.truth_all, det_r1, det_r2, snapshots_r1, snapshots_r2, tracks_r1, tracks_r2, params, study);
end
% 可选保存结果
if config.save_result
    if ~exist('results', 'dir'), mkdir('results'); end
    save(fullfile('results', 'random_fade_fragment_fusion.mat'), '-struct', 'result');
end
end

function [track_a, track_b, track_c] = truth_tracks_for_legacy(truth_all)
    % 将 truth_all 拆分为 A/B/C 三个变量（最多 3 个目标），用于 legacy 绘图接口
empty_track = nan(1, 5);
track_a = empty_track; track_b = empty_track; track_c = empty_track;
if numel(truth_all) >= 1, track_a = truth_all{1}; end
if numel(truth_all) >= 2, track_b = truth_all{2}; end
if numel(truth_all) >= 3, track_c = truth_all{3}; end
end

function tf = is_fusion_group(group, segments, edges)
    % 判断一个分组是否可以融合：
    %   1. 包含两个雷达的片段（has_both_radars）
    %   2. 包含跨雷达的 overlap 边（has_cross_edge）
    members = group.segment_indices;                    % 获取当前组包含的片段索引
    % 检查这些片段是否来自两个不同的雷达
    has_both_radars = numel(unique([segments(members).radar_id])) == 2;
    % 检查是否存在跨雷达的重叠关联边
    has_cross_edge = any(arrayfun(@(e) strcmp(e.edge_type, 'overlap') && ...
        ismember(e.a, members) && ismember(e.b, members), edges));
    tf = has_both_radars && has_cross_edge;             % 两个条件都满足才可融合
end

function fr = evaluate_methods(fr, truth, t_grid)
    % 对融合结果中的每种方法，计算与真值的逐帧 RMSE
    for m = 1:numel(fr.methods)                        % 遍历每种融合算法
        errors = [];
        snaps = fr.methods(m).snapshots;               % 获取该方法的所有帧快照
        for k = 1:min(numel(snaps), numel(t_grid))     % 逐帧比较
            if isempty(snaps{k}) || isempty(snaps{k}.trackList), continue; end
            trk = snaps{k}.trackList{1};               % 取融合后的第一条航迹
            % 插值得到真值在当前帧的经纬度
            lon = interp1(truth.time_sec, truth.lon, t_grid(k), 'linear', NaN);
            lat = interp1(truth.time_sec, truth.lat, t_grid(k), 'linear', NaN);
            if isfinite(lat) && isfinite(lon)
                % 计算融合航迹与真值的球面距离（km）
                errors(end+1) = sphere_utils_haversine_distance(trk.lon, trk.lat, lon, lat) / 1000; %#ok<AGROW>
            end
        end
        % 计算 RMSE（无误差时设为 NaN）
        if isempty(errors), fr.methods(m).rmse_km=NaN; else, fr.methods(m).rmse_km=sqrt(mean(errors.^2)); end
    end
    values = [fr.methods.rmse_km]; valid = find(isfinite(values));
    % 找出 RMSE 最小的方法和对应索引
    if ~isempty(valid), [fr.best_rmse_km, idx] = min(values(valid)); fr.best_method = fr.methods(valid(idx)).method; end
end

function status = derive_status(segments, edges, groups, fusion, fade1, fade2, tracks1, tracks2)
    % 推导实验的整体状态（SUCCESS 或各种失败原因）
    % 按优先级检查：片段 → 边 → 组数 → 融合 → RMSE → 衰落重启
    if isempty(segments)
        status = 'NO_EFFECTIVE_SEGMENTS';              % 没有有效片段
    elseif isempty(edges)
        status = 'NO_ACCEPTED_EDGES';                   % 没有接受的边
    elseif sum(arrayfun(@(g) is_fusion_group(g, segments, edges), groups)) > 1
        status = 'MULTIPLE_GROUPS_AMBIGUOUS';           % 多个融合组，结果歧义
    elseif isempty(fusion)
        status = 'FUSION_FAILED';                       % 融合失败
    elseif ~any(cellfun(@(f) isfinite(f.best_rmse_km), fusion))
        status = 'NO_VALID_FUSION_RMSE';                % 没有有效的融合 RMSE
    elseif ~has_fade_restart(tracks1, fade1.window)
        status = restart_status(tracks1, fade1.window, 1);  % R1 衰落窗口后未重启
    elseif ~has_fade_restart(tracks2, fade2.window)
        status = restart_status(tracks2, fade2.window, 2);  % R2 衰落窗口后未重启
    else
        status = 'SUCCESS';                             % 一切正常
    end
end

function tf = has_fade_restart(tracks, window)
    % 判断衰落窗口后是否有航迹重启：
    %   1. 原航迹在窗口结束帧死亡（type==7, death_reason=='k_loss'）
    %   2. 有新航迹在窗口结束后起始
death_ids = [];
for i = 1:numel(tracks)
    trk = tracks{i};
        % 查找在衰落窗口结束帧死亡的航迹
    if trk.type == 7 && isfield(trk, 'death_reason') && strcmp(trk.death_reason, 'k_loss') && ...
            isfield(trk, 'death_frame') && isscalar(trk.death_frame) && ...
            trk.death_frame == window(2)
        death_ids(end+1) = double(trk.id); %#ok<AGROW>
    end
end
if isempty(death_ids), tf = false; return; end    % 没有找到死亡航迹则无重启
tf = false;
for i = 1:numel(tracks)
    trk = tracks{i};
    frames = association_frames(trk);               % 获取该航迹关联的帧号
        % 新航迹：起始帧在窗口之后，且不是已死亡的航迹
    if ~isempty(frames) && min(frames) > window(2) && ~ismember(double(trk.id), death_ids)
        tf = true;
        return;
    end
end
end

function status = restart_status(tracks, window, radar_id)
    % 生成 R1/R2 衰落重启状态的详细错误信息
has_death = false;
for i = 1:numel(tracks)
    trk = tracks{i};
        % 检查是否有航迹在衰落窗口结束帧因 k_loss 死亡
    if trk.type == 7 && isfield(trk, 'death_reason') && strcmp(trk.death_reason, 'k_loss') && ...
            isfield(trk, 'death_frame') && isscalar(trk.death_frame) && trk.death_frame == window(2)
        has_death = true;
        break;
    end
end
if has_death
    status = sprintf('NO_RESTART_R%d', radar_id);   % 有死亡但无重启
else
    status = sprintf('TRACK_DID_NOT_DIE_R%d', radar_id);  % 航迹根本没死
end
end

function frames = association_frames(trk)
    % 从航迹的 asscPointList 中提取所有关联帧号
frames = [];
if ~isfield(trk, 'asscPointList'), return; end
for j = 1:numel(trk.asscPointList)
    dp = trk.asscPointList{j};
    if ~isempty(dp) && isfield(dp, 'frameID'), frames(end+1) = double(dp.frameID); end %#ok<AGROW>
end
end
function status = first_failure(a,b)
    % 返回第一个失败的错误码（优先返回非 SUCCESS 的值）
if ~strcmp(a,'SUCCESS'), status=a; else, status=b; end
end
function print_summary(r)
    % 打印实验摘要：状态、衰落窗口信息、片段/边/group 数量、各组的最佳融合方法和 RMSE
fprintf('随机衰落实验: %s%s',r.status,newline); fprintf('R1窗口=[%d,%d] 删除%d点；R2窗口=[%d,%d] 删除%d点%s',r.fade_R1.window,r.fade_R1.removed_target_detections,r.fade_R2.window,r.fade_R2.removed_target_detections,newline);
fprintf('片段=%d, 接受边=%d, groups=%d%s',numel(r.grouping.segments),numel(r.grouping.edges),numel(r.grouping.groups),newline);
if isempty(r.fusion_results)
    return;
end
for i = 1:numel(r.fusion_results)
    f = r.fusion_results{i};
    fprintf('Group%d 最佳=%s RMSE=%.2fkm%s', f.group_id, char(f.best_method), f.best_rmse_km, newline);
end
end
function c = defaults(c)
    % 填充默认配置：target_id=1, seed_r1=1101, seed_r2=2202, show/save/verbose=true
def=struct('target_id',1,'seed_r1',1101,'seed_r2',2202,'show_figures',true,'save_result',true,'verbose',true); f=fieldnames(def); for i=1:numel(f), if ~isfield(c,f{i}), c.(f{i})=def.(f{i}); end, end
end
