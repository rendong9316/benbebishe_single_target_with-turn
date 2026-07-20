function result = run_random_fade_fragment_fusion(config)
% RUN_RANDOM_FADE_FRAGMENT_FUSION Controlled multi-target fragment study.

if nargin < 1, config = struct(); end
config = defaults(config);
addpath(genpath('.'));
inputs = prepare_oracle_tracking_inputs(config.scenario_name);
params = inputs.params;

[base_r1, ~, base_snap_r1] = run_oracle_tracker_sequence( ...
    inputs.detList_R1, inputs.ukf1_tpl, inputs.params_r1, ...
    inputs.truth_all, inputs.t1_grid, false);
[base_r2, ~, base_snap_r2] = run_oracle_tracker_sequence( ...
    inputs.detList_R2, inputs.ukf2_tpl, inputs.params_r2, ...
    inputs.truth_all, inputs.t2_grid, false);

[det_r1, plan_r1, validation_r1, status_r1] = plan_controlled_fragmentation( ...
    inputs.detList_R1, base_r1, base_snap_r1, inputs.ukf1_tpl, ...
    inputs.params_r1, inputs.truth_all, inputs.t1_grid, 1, params);
[det_r2, plan_r2, validation_r2, status_r2] = plan_controlled_fragmentation( ...
    inputs.detList_R2, base_r2, base_snap_r2, inputs.ukf2_tpl, ...
    inputs.params_r2, inputs.truth_all, inputs.t2_grid, 2, params);

fragment_plan = struct('R1', plan_r1, 'R2', plan_r2);
fragment_validation = struct('R1', validation_r1, 'R2', validation_r2);
fixture_status = first_failure(status_r1, status_r2);
if ~strcmp(fixture_status, 'SUCCESS')
    result = fixture_failure_result(fixture_status, config, params, inputs, ...
        det_r1, det_r2, fragment_plan, fragment_validation);
    print_summary(result);
    return;
end

[tracks_r1, temp_r1, snapshots_r1, diag_r1] = run_oracle_tracker_sequence( ...
    det_r1, inputs.ukf1_tpl, inputs.params_r1, inputs.truth_all, ...
    inputs.t1_grid, config.verbose);
[tracks_r2, temp_r2, snapshots_r2, diag_r2] = run_oracle_tracker_sequence( ...
    det_r2, inputs.ukf2_tpl, inputs.params_r2, inputs.truth_all, ...
    inputs.t2_grid, config.verbose);
validate_oracle_invariants(snapshots_r1, det_r1, diag_r1, inputs.params_r1, tracks_r1);
validate_oracle_invariants(snapshots_r2, det_r2, diag_r2, inputs.params_r2, tracks_r2);

segments_r1 = build_faded_track_segments('extract', snapshots_r1, tracks_r1, 1);
segments_r2 = build_faded_track_segments('extract', snapshots_r2, tracks_r2, 2);
aligned_r2 = time_align_tracks(snapshots_r2, params);
aligned_segments_r2 = build_faded_track_segments('extract', aligned_r2, tracks_r2, 2);
segments = [segments_r1, aligned_segments_r2];
for i = 1:numel(segments), segments(i).segment_id = i; end

segment_truth_labels = build_segment_truth_sidecar(segments, tracks_r1, tracks_r2);
matching_params = params;
if isfield(matching_params, 'fragmentation')
    matching_params = rmfield(matching_params, 'fragmentation');
end
grouping = tracklet_grouping('segments', segments, matching_params);

fusion_results = struct([]);
if strcmp(grouping.status, 'SUCCESS')
    for g = 1:numel(grouping.groups)
        item = fuse_estimate_sequence(grouping.groups(g), segments, matching_params);
        if g == 1
            fusion_results = item;
        else
            fusion_results(g) = item;
        end
    end
end

evaluation = evaluate_fragment_fusion_multi(fusion_results, grouping.groups, ...
    segments, segment_truth_labels, inputs.truthTrajs, inputs.t1_grid);
status = derive_status(segments, grouping, fusion_results);
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

print_summary(result);
if config.show_figures
    [track_a, track_b, track_c] = truth_tracks_for_legacy(inputs.truth_all);
    plot_scene_overview_multi(track_a, track_b, track_c, params, 'results');
    plot_point_cloud_3d(det_r1, 'R1', '');
    plot_point_cloud_3d(det_r2, 'R2', '');
    plot_tracks_without_fusion(inputs.truth_all, det_r1, det_r2, ...
        snapshots_r1, snapshots_r2, tracks_r1, tracks_r2, params, study);
end
if config.save_result
    if ~exist('results', 'dir'), mkdir('results'); end
    save(fullfile('results', 'random_fade_fragment_fusion.mat'), '-struct', 'result');
end
end

function labels = build_segment_truth_sidecar(segments, tracks_r1, tracks_r2)
labels = nan(1, numel(segments));
for i = 1:numel(segments)
    if segments(i).radar_id == 1, tracks = tracks_r1; else, tracks = tracks_r2; end
    for j = 1:numel(tracks)
        if double(tracks{j}.id) == segments(i).track_id
            labels(i) = double(tracks{j}.truth_idx);
            break;
        end
    end
end
end

function status = derive_status(segments, grouping, fusion_results)
if isempty(segments)
    status = 'NO_EFFECTIVE_SEGMENTS';
elseif ~strcmp(grouping.status, 'SUCCESS')
    status = grouping.status;
elseif isempty(grouping.groups)
    status = 'NO_GROUPS';
elseif numel(fusion_results) ~= numel(grouping.groups)
    status = 'FUSION_FAILED';
else
    status = 'SUCCESS';
end
end

function result = fixture_failure_result(status, config, params, inputs, ...
    det_r1, det_r2, fragment_plan, fragment_validation)
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
if ~strcmp(a, 'SUCCESS'), status = a; else, status = b; end
end

function print_summary(result)
fprintf('可控碎片实验: %s%s', result.status, newline);
if isfield(result, 'fragment_plan')
    fprintf('R1事件=%d, R2事件=%d, 目标数=%d, 每目标每站K=%d%s', ...
        numel(result.fragment_plan.R1.events), numel(result.fragment_plan.R2.events), ...
        result.scenario.n_targets, ...
        result.params.fragmentation.segments_per_target_per_radar, newline);
end
if ~strcmp(result.status, 'SUCCESS'), return; end
fprintf('片段=%d, 接受边=%d, hypotheses=%d, groups=%d%s', ...
    numel(result.segments), numel(result.grouping.edges), ...
    numel(result.grouping.hypotheses), numel(result.grouping.groups), newline);
for g = 1:numel(result.evaluation.groups)
    e = result.evaluation.groups(g);
    fprintf('Group%d -> Truth%s, purity=%.2f, best=%s %.2fkm%s', ...
        e.group_id, scalar_label(e.truth_idx), e.purity, e.best_method, ...
        e.best_rmse_km, newline);
end
end

function label = scalar_label(value)
if isfinite(value), label = sprintf('%d', value); else, label = 'unmatched'; end
end

function [track_a, track_b, track_c] = truth_tracks_for_legacy(truth_all)
empty_track = nan(1, 5);
track_a = empty_track; track_b = empty_track; track_c = empty_track;
if numel(truth_all) >= 1, track_a = truth_all{1}; end
if numel(truth_all) >= 2, track_b = truth_all{2}; end
if numel(truth_all) >= 3, track_c = truth_all{3}; end
end

function config = defaults(config)
defaults_local = struct('scenario_name', 'multi_cross', ...
    'show_figures', true, 'save_result', true, 'verbose', true);
names = fieldnames(defaults_local);
for i = 1:numel(names)
    if ~isfield(config, names{i}), config.(names{i}) = defaults_local.(names{i}); end
end
end
