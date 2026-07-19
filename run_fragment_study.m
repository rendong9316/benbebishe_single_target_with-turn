function result = run_fragment_study(scenario_name, config)
% RUN_FRAGMENT_STUDY 人工制造双站互补碎片并研究 M:N 航迹凝聚。
% 不修改现有模块；truth_idx 仅用于测试夹具和外部评估。

if nargin < 1 || isempty(scenario_name)
    scenario_name = 'single_turn';
end
if nargin < 2 || isempty(config)
    config = default_config();
end

config = merge_config(default_config(), config);
addpath(genpath('.'));
rng(config.seed);

fprintf('========== 基线单站跟踪 ==========%s', newline);
baseline = run_without_fusion(scenario_name);

fprintf('%s========== 人工制造互补碎片 ==========%s', newline, newline);
[fractured_R1, fractured_R2, fracture_plan] = manufacture_fragments( ...
    baseline.trackSnapshots_R1, baseline.trackSnapshots_R2, config);
assert_fragment_plan(fractured_R1, fractured_R2, fracture_plan);
print_fragment_plan(fracture_plan);

fprintf('%s========== R2 时间对齐 ==========%s', newline, newline);
aligned_R2 = time_align_tracks(fractured_R2, baseline.params);

fprintf('%s========== M:N 航迹段凝聚与融合 ==========%s', newline, newline);
grouping = tracklet_grouping(fractured_R1, aligned_R2, baseline.params);

fprintf('%s========== 真值外部评估 ==========%s', newline, newline);
evaluation = evaluate_groups(grouping, fractured_R1, fractured_R2, ...
    baseline.truthTrajs, baseline.scenario.t1_grid, baseline.params);
print_evaluation(evaluation);

figure_paths = {};
if config.show_figures || config.save_figures
    fprintf('%s========== 碎片凝聚过程可视化 ==========%s', newline, newline);
    figure_paths = plot_fragment_study_dashboard(result_view_inputs(...
        baseline, fractured_R1, aligned_R2, fracture_plan, grouping, evaluation), config);
end

result = struct();
result.config = config;
result.baseline = baseline;
result.fractured_R1 = fractured_R1;
result.fractured_R2 = fractured_R2;
result.aligned_R2 = aligned_R2;
result.fracture_plan = fracture_plan;
result.grouping = grouping;
result.evaluation = evaluation;
result.figure_paths = figure_paths;

if config.save_result
    if ~exist('results', 'dir'), mkdir('results'); end
    output_path = fullfile('results', sprintf('fragment_study_%s_%s.mat', ...
        scenario_name, datestr(now, 'yyyymmdd_HHMMSS')));
    save(output_path, '-struct', 'result');
    fprintf('结果已保存: %s%s', output_path, newline);
end
end

function config = merge_config(defaults, overrides)
config = defaults;
if isempty(overrides), return; end
names = fieldnames(overrides);
for i = 1:numel(names)
    config.(names{i}) = overrides.(names{i});
end
end

function views = result_view_inputs(baseline, fractured_R1, aligned_R2, plan, grouping, evaluation)
views = struct('truth_idx', {}, 'truth', {}, 'plan', {}, 'r1_before', {}, ...
    'r1_after', {}, 'r2_middle', {}, 'fused', {}, 'group', {}, ...
    'group_segments', {}, 'group_edges', {}, 'overlap1', {}, 'overlap2', {}, ...
    'evaluation', {});
for q = 1:numel(plan)
    p = plan(q);
    candidates = find([evaluation.truth_idx] == p.truth_idx);
    if isempty(candidates), continue; end
    [~, best] = max([evaluation(candidates).coverage_frames]);
    ev = evaluation(candidates(best));
    group_idx = find([grouping.groups.group_id] == ev.group_id, 1);
    if isempty(group_idx), continue; end
    group = grouping.groups(group_idx);
    segs = grouping.segments(group.segment_indices);
    before = find_segment(segs, 1, p.r1_original_id);
    after = find_segment(segs, 1, p.r1_new_id);
    middle = find_segment(segs, 2, p.r2_original_id);
    edges = grouping.edges(arrayfun(@(e) ismember(e.a, group.segment_indices) && ...
        ismember(e.b, group.segment_indices), grouping.edges));
    common1 = intersect(before.frames, middle.frames);
    common2 = intersect(after.frames, middle.frames);
    fused = collect_fused(grouping.fused_snapshots, group.group_id);
    views(end+1) = struct('truth_idx', p.truth_idx, ... %#ok<AGROW>
        'truth', baseline.truthTrajs{p.truth_idx}, 'plan', p, ...
        'r1_before', before, 'r1_after', after, 'r2_middle', middle, ...
        'fused', fused, 'group', group, 'group_segments', segs, ...
        'group_edges', edges, 'overlap1', common1, 'overlap2', common2, ...
        'evaluation', ev);
end
end

function seg = find_segment(segs, radar_id, track_id)
seg = struct('frames', [], 'lats', [], 'lons', [], 'start_frame', [], 'end_frame', [], 'radar_id', radar_id, 'track_id', track_id);
idx = find([segs.radar_id] == radar_id & [segs.track_id] == track_id, 1);
if ~isempty(idx), seg = segs(idx); end
end

function fused = collect_fused(snapshots, group_id)
fused = struct('frames', [], 'lat', [], 'lon', [], 'source', {{}});
for k = 1:numel(snapshots)
    for t = 1:numel(snapshots{k}.trackList)
        trk = snapshots{k}.trackList{t};
        if trk.group_id == group_id
            fused.frames(end+1) = k; %#ok<AGROW>
            fused.lat(end+1) = trk.lat; %#ok<AGROW>
            fused.lon(end+1) = trk.lon; %#ok<AGROW>
            fused.source{end+1} = trk.source; %#ok<AGROW>
        end
    end
end
end

function config = default_config()
config = struct();
config.seed = 42;
config.r1_gap_range = [2, 5];
config.r2_start_fraction_range = [0.10, 0.25];
config.r2_end_fraction_range = [0.75, 0.90];
config.min_segment_frames = 8;
config.new_id_offset = 1000;
config.save_result = true;
config.show_figures = true;
config.save_figures = true;
config.figure_visible = 'on';
config.figure_dpi = 180;
config.output_root = fullfile('results', 'fragment_study');
config.min_overlap_frames = 3;
end

function [out_R1, out_R2, plan] = manufacture_fragments(in_R1, in_R2, config)
out_R1 = clone_snapshots(in_R1);
out_R2 = clone_snapshots(in_R2);
truth_ids = unique([collect_truth_ids(in_R1), collect_truth_ids(in_R2)]);
max_id = max([collect_track_ids(in_R1), collect_track_ids(in_R2), 0]);
next_id = max_id + config.new_id_offset;
plan = struct('truth_idx', {}, 'r1_original_id', {}, 'r1_new_id', {}, ...
    'r1_gap_start', {}, 'r1_gap_end', {}, 'r2_keep_start', {}, ...
    'r2_keep_end', {}, 'r2_original_id', {});

for q = 1:numel(truth_ids)
    truth_idx = truth_ids(q);
    [r1_frames, r1_id] = active_frames_for_truth(in_R1, truth_idx);
    [r2_frames, r2_id] = active_frames_for_truth(in_R2, truth_idx);
    if numel(r1_frames) < 2 * config.min_segment_frames + config.r1_gap_range(2) || ...
            numel(r2_frames) < 2 * config.min_segment_frames
        continue;
    end

    gap = randi(config.r1_gap_range);
    lower = r1_frames(1) + config.min_segment_frames;
    upper = r1_frames(end) - config.min_segment_frames - gap + 1;
    gap_start = randi([lower, upper]);
    gap_end = gap_start + gap - 1;
    new_id = next_id;
    next_id = next_id + 1;

    r2_span = r2_frames(end) - r2_frames(1) + 1;
    start_fraction = random_between(config.r2_start_fraction_range);
    end_fraction = random_between(config.r2_end_fraction_range);
    keep_start = max(r2_frames(1), floor(r2_frames(1) + start_fraction * r2_span));
    keep_end = min(r2_frames(end), ceil(r2_frames(1) + end_fraction * r2_span));
    keep_start = min(keep_start, gap_start - 1);
    keep_end = max(keep_end, gap_end + 1);

    out_R1 = split_track(out_R1, truth_idx, r1_id, gap_start, gap_end, new_id);
    out_R2 = crop_track(out_R2, truth_idx, r2_id, keep_start, keep_end);

    plan(end+1) = struct('truth_idx', truth_idx, ... %#ok<AGROW>
        'r1_original_id', r1_id, 'r1_new_id', new_id, ...
        'r1_gap_start', gap_start, 'r1_gap_end', gap_end, ...
        'r2_keep_start', keep_start, 'r2_keep_end', keep_end, ...
        'r2_original_id', r2_id);
end
end

function snapshots = clone_snapshots(input)
snapshots = cell(size(input));
for k = 1:numel(input)
    snap = input{k};
    tracks = {};
    if ~isempty(snap) && isfield(snap, 'trackList')
        tracks = snap.trackList;
    end
    snapshots{k} = struct('trackList', {tracks}, 'frameID', snap.frameID);
end
end

function ids = collect_truth_ids(snapshots)
ids = [];
for k = 1:numel(snapshots)
    for t = 1:numel(snapshots{k}.trackList)
        trk = snapshots{k}.trackList{t};
        if ~isempty(trk) && trk.type ~= 7 && isfield(trk, 'truth_idx')
            ids(end+1) = double(trk.truth_idx); %#ok<AGROW>
        end
    end
end
ids = unique(ids);
end

function ids = collect_track_ids(snapshots)
ids = [];
for k = 1:numel(snapshots)
    for t = 1:numel(snapshots{k}.trackList)
        trk = snapshots{k}.trackList{t};
        if ~isempty(trk) && isfield(trk, 'id')
            ids(end+1) = double(trk.id); %#ok<AGROW>
        end
    end
end
end

function [frames, track_id] = active_frames_for_truth(snapshots, truth_idx)
frames = [];
track_id = NaN;
for k = 1:numel(snapshots)
    for t = 1:numel(snapshots{k}.trackList)
        trk = snapshots{k}.trackList{t};
        if ~isempty(trk) && trk.type ~= 7 && isfield(trk, 'truth_idx') && ...
                double(trk.truth_idx) == truth_idx
            frames(end+1) = k; %#ok<AGROW>
            if isnan(track_id), track_id = double(trk.id); end
            break;
        end
    end
end
end

function snapshots = split_track(snapshots, truth_idx, original_id, gap_start, gap_end, new_id)
for k = 1:numel(snapshots)
    tracks = snapshots{k}.trackList;
    keep = true(1, numel(tracks));
    for t = 1:numel(tracks)
        trk = tracks{t};
        if ~isempty(trk) && trk.type ~= 7 && isfield(trk, 'truth_idx') && ...
                double(trk.truth_idx) == truth_idx && double(trk.id) == original_id
            if k >= gap_start && k <= gap_end
                keep(t) = false;
            elseif k > gap_end
                trk.id = new_id;
                tracks{t} = trk;
            end
        end
    end
    snapshots{k}.trackList = tracks(keep);
end
end

function snapshots = crop_track(snapshots, truth_idx, original_id, keep_start, keep_end)
for k = 1:numel(snapshots)
    tracks = snapshots{k}.trackList;
    keep = true(1, numel(tracks));
    for t = 1:numel(tracks)
        trk = tracks{t};
        if ~isempty(trk) && trk.type ~= 7 && isfield(trk, 'truth_idx') && ...
                double(trk.truth_idx) == truth_idx && double(trk.id) == original_id && ...
                (k < keep_start || k > keep_end)
            keep(t) = false;
        end
    end
    snapshots{k}.trackList = tracks(keep);
end
end

function value = random_between(range)
value = range(1) + rand() * (range(2) - range(1));
end

function assert_fragment_plan(r1, r2, plan)
assert(~isempty(plan), '未生成任何碎片计划');
for q = 1:numel(plan)
    p = plan(q);
    ids_r1 = ids_for_truth(r1, p.truth_idx);
    assert(ismember(p.r1_original_id, ids_r1) && ismember(p.r1_new_id, ids_r1), ...
        'R1 未形成两个独立航迹 ID');
    for k = p.r1_gap_start:p.r1_gap_end
        assert(~has_truth_track(r1{k}, p.truth_idx), 'R1 gap 内仍有目标航迹');
        assert(has_truth_track(r2{k}, p.truth_idx), 'R2 未覆盖 R1 gap，空间分集条件不成立');
    end
end
end

function ids = ids_for_truth(snapshots, truth_idx)
ids = [];
for k = 1:numel(snapshots)
    for t = 1:numel(snapshots{k}.trackList)
        trk = snapshots{k}.trackList{t};
        if ~isempty(trk) && isfield(trk, 'truth_idx') && double(trk.truth_idx) == truth_idx
            ids(end+1) = double(trk.id); %#ok<AGROW>
        end
    end
end
ids = unique(ids);
end

function tf = has_truth_track(snap, truth_idx)
tf = false;
for t = 1:numel(snap.trackList)
    trk = snap.trackList{t};
    if ~isempty(trk) && isfield(trk, 'truth_idx') && double(trk.truth_idx) == truth_idx
        tf = true;
        return;
    end
end
end

function print_fragment_plan(plan)
for q = 1:numel(plan)
    p = plan(q);
    fprintf('目标%d: R1 #%d -> gap[%d,%d] -> #%d；R2 #%d 保留[%d,%d]%s', ...
        p.truth_idx, p.r1_original_id, p.r1_gap_start, p.r1_gap_end, ...
        p.r1_new_id, p.r2_original_id, p.r2_keep_start, p.r2_keep_end, newline);
end
end

function evaluation = evaluate_groups(grouping, r1, r2, truthTrajs, t_grid, params)
evaluation = struct('group_id', {}, 'truth_idx', {}, 'segment_count', {}, ...
    'coverage_frames', {}, 'truth_frames', {}, 'coverage_ratio', {}, ...
    'best_single_frames', {}, 'extension_frames', {}, 'rmse_km', {});
for g = 1:numel(grouping.groups)
    group = grouping.groups(g);
    segment_indices = group.segment_indices;
    truth_votes = [];
    single_lengths = [];
    for s = segment_indices
        seg = grouping.segments(s);
        truth_votes = [truth_votes, truth_labels_for_segment(seg, r1, r2)]; %#ok<AGROW>
        single_lengths(end+1) = numel(seg.frames); %#ok<AGROW>
    end
    truth_votes = truth_votes(truth_votes > 0);
    if isempty(truth_votes), continue; end
    truth_idx = mode(truth_votes);
    [errors, covered] = group_errors(grouping.fused_snapshots, g, truthTrajs{truth_idx}, t_grid);
    evaluation(end+1) = struct('group_id', group.group_id, 'truth_idx', truth_idx, ... %#ok<AGROW>
        'segment_count', numel(segment_indices), 'coverage_frames', covered, ...
        'truth_frames', numel(t_grid), 'coverage_ratio', covered / numel(t_grid), ...
        'best_single_frames', max(single_lengths), ...
        'extension_frames', covered - max(single_lengths), ...
        'rmse_km', sqrt(mean(errors.^2)));
end
end

function labels = truth_labels_for_segment(seg, r1, r2)
labels = [];
if seg.radar_id == 1, snapshots = r1; else, snapshots = r2; end
for k = seg.frames
    for t = 1:numel(snapshots{k}.trackList)
        trk = snapshots{k}.trackList{t};
        if double(trk.id) == seg.track_id && isfield(trk, 'truth_idx')
            labels(end+1) = double(trk.truth_idx); %#ok<AGROW>
        end
    end
end
end

function [errors, covered] = group_errors(fused, group_id, truth, t_grid)
errors = [];
covered = 0;
for k = 1:min(numel(fused), numel(t_grid))
    tracks = fused{k}.trackList;
    for t = 1:numel(tracks)
        trk = tracks{t};
        if trk.group_id ~= group_id, continue; end
        truth_lon = interp1(truth.time_sec, truth.lon, t_grid(k), 'linear', NaN);
        truth_lat = interp1(truth.time_sec, truth.lat, t_grid(k), 'linear', NaN);
        if isfinite(truth_lon) && isfinite(truth_lat)
            errors(end+1) = haversine_km(trk.lat, trk.lon, truth_lat, truth_lon); %#ok<AGROW>
            covered = covered + 1;
        end
    end
end
end

function print_evaluation(evaluation)
for i = 1:numel(evaluation)
    e = evaluation(i);
    fprintf('Group%d -> 目标%d: %d段, 覆盖=%d帧(%.1f%%), 比最长单段延长=%d帧, RMSE=%.2fkm%s', ...
        e.group_id, e.truth_idx, e.segment_count, e.coverage_frames, ...
        100*e.coverage_ratio, e.extension_frames, e.rmse_km, newline);
end
end

function d = haversine_km(lat1, lon1, lat2, lon2)
R = 6371.0088;
dlat = deg2rad(lat2-lat1);
dlon = deg2rad(lon2-lon1);
a = sin(dlat/2)^2 + cos(deg2rad(lat1))*cos(deg2rad(lat2))*sin(dlon/2)^2;
d = 2*R*atan2(sqrt(a), sqrt(max(0, 1-a)));
end
