function evaluation = evaluate_fragment_fusion_multi( ...
    fusion_results, groups, segments, segment_truth_labels, truthTrajs, t_grid)
% EVALUATE_FRAGMENT_FUSION_MULTI Truth-only evaluation after grouping/fusion.

n_groups = numel(groups);
n_truth = numel(truthTrajs);
cost = inf(n_groups, n_truth);
for g = 1:n_groups
    [frames, lat, lon] = group_reference(groups(g), segments);
    for target = 1:n_truth
        errors = trajectory_errors(frames, lat, lon, truthTrajs{target}, t_grid);
        if ~isempty(errors), cost(g, target) = sqrt(mean(errors.^2)); end
    end
end

unmatched_cost = 1e6;
solver_cost = cost;
solver_cost(~isfinite(solver_cost)) = unmatched_cost;
if isempty(solver_cost)
    assignments = zeros(0, 2);
else
    assignments = matchpairs(solver_cost, unmatched_cost);
    keep = arrayfun(@(q) isfinite(cost(assignments(q,1), assignments(q,2))), ...
        1:size(assignments,1));
    assignments = assignments(keep, :);
end

group_eval = struct('group_id', {}, 'truth_idx', {}, 'purity', {}, ...
    'mixed_target', {}, 'methods', {}, 'best_method', {}, 'best_rmse_km', {});
method_names = {'SCC','BC','CI','FCI'};
all_errors = cell(1, numel(method_names));
all_bridge_errors = cell(1, numel(method_names));
all_reconstructed_errors = cell(1, numel(method_names));

for g = 1:n_groups
    row = find(assignments(:,1) == g, 1);
    truth_idx = NaN;
    if ~isempty(row), truth_idx = assignments(row,2); end
    labels = segment_truth_labels(groups(g).segment_indices);
    labels = labels(isfinite(labels) & labels > 0);
    purity = NaN;
    if ~isempty(labels)
        counts = arrayfun(@(x) sum(labels == x), unique(labels));
        purity = max(counts) / numel(labels);
    end

    methods = empty_method_evaluation();
    best_method = '';
    best_rmse = inf;
    for m = 1:numel(method_names)
        errors = [];
        bridge_errors = [];
        reconstructed_errors = [];
        low_confidence_count = 0;
        if isfinite(truth_idx) && g <= numel(fusion_results)
            method_idx = find(strcmp({fusion_results(g).methods.method}, method_names{m}), 1);
            if ~isempty(method_idx)
                method_result = fusion_results(g).methods(method_idx);
                errors = snapshot_errors(method_result.snapshots, truthTrajs{truth_idx}, t_grid);
                if isfield(method_result, 'bridge_snapshots')
                    bridge_errors = snapshot_errors(method_result.bridge_snapshots, ...
                        truthTrajs{truth_idx}, t_grid);
                end
                if isfield(method_result, 'reconstructed_snapshots')
                    reconstructed_errors = snapshot_errors( ...
                        method_result.reconstructed_snapshots, truthTrajs{truth_idx}, t_grid);
                end
                if isfield(method_result, 'low_confidence_bridge_count')
                    low_confidence_count = method_result.low_confidence_bridge_count;
                end
            end
        end

        rmse = rms_or_nan(errors);
        bridge_rmse = rms_or_nan(bridge_errors);
        reconstructed_rmse = rms_or_nan(reconstructed_errors);
        coverage = numel(errors);
        bridge_coverage = numel(bridge_errors);
        reconstructed_coverage = numel(reconstructed_errors);
        ratio = coverage / max(1, numel(t_grid));

        methods(end+1) = struct('method', method_names{m}, ...
            'rmse_km', rmse, 'coverage_frames', coverage, ...
            'coverage_ratio', ratio, 'errors_km', errors, ...
            'supported_rmse_km', rmse, 'bridge_rmse_km', bridge_rmse, ...
            'reconstructed_rmse_km', reconstructed_rmse, ...
            'bridge_coverage_frames', bridge_coverage, ...
            'reconstructed_coverage_frames', reconstructed_coverage, ...
            'bridge_errors_km', bridge_errors, ...
            'reconstructed_errors_km', reconstructed_errors, ...
            'low_confidence_bridge_count', low_confidence_count); %#ok<AGROW>

        all_errors{m} = [all_errors{m}, errors]; %#ok<AGROW>
        all_bridge_errors{m} = [all_bridge_errors{m}, bridge_errors]; %#ok<AGROW>
        all_reconstructed_errors{m} = ...
            [all_reconstructed_errors{m}, reconstructed_errors]; %#ok<AGROW>
        if isfinite(rmse) && rmse < best_rmse
            best_rmse = rmse;
            best_method = method_names{m};
        end
    end

    group_eval(end+1) = struct('group_id', groups(g).group_id, ...
        'truth_idx', truth_idx, 'purity', purity, ...
        'mixed_target', isfinite(purity) && purity < 1, 'methods', methods, ...
        'best_method', best_method, 'best_rmse_km', best_rmse); %#ok<AGROW>
end

overall = struct('method', {}, 'rmse_km', {}, 'sample_count', {}, ...
    'bridge_rmse_km', {}, 'bridge_sample_count', {}, ...
    'reconstructed_rmse_km', {}, 'reconstructed_sample_count', {});
for m = 1:numel(method_names)
    overall(end+1) = struct('method', method_names{m}, ...
        'rmse_km', rms_or_nan(all_errors{m}), ...
        'sample_count', numel(all_errors{m}), ...
        'bridge_rmse_km', rms_or_nan(all_bridge_errors{m}), ...
        'bridge_sample_count', numel(all_bridge_errors{m}), ...
        'reconstructed_rmse_km', rms_or_nan(all_reconstructed_errors{m}), ...
        'reconstructed_sample_count', numel(all_reconstructed_errors{m})); %#ok<AGROW>
end

assigned_groups = assignments(:,1)';
assigned_truth = assignments(:,2)';
evaluation = struct('assignments', assignments, 'assignment_cost_km', cost, ...
    'groups', group_eval, 'overall', overall, ...
    'unmatched_groups', setdiff(1:n_groups, assigned_groups), ...
    'unmatched_truth', setdiff(1:n_truth, assigned_truth), ...
    'mixed_group_count', sum([group_eval.mixed_target]));
end

function methods = empty_method_evaluation()
methods = struct('method', {}, 'rmse_km', {}, 'coverage_frames', {}, ...
    'coverage_ratio', {}, 'errors_km', {}, 'supported_rmse_km', {}, ...
    'bridge_rmse_km', {}, 'reconstructed_rmse_km', {}, ...
    'bridge_coverage_frames', {}, 'reconstructed_coverage_frames', {}, ...
    'bridge_errors_km', {}, 'reconstructed_errors_km', {}, ...
    'low_confidence_bridge_count', {});
end

function errors = snapshot_errors(snapshots, truth, t_grid)
[frames, lat, lon] = snapshots_line(snapshots);
errors = trajectory_errors(frames, lat, lon, truth, t_grid);
end

function value = rms_or_nan(errors)
value = NaN;
if ~isempty(errors), value = sqrt(mean(errors.^2)); end
end

function [frames, lat, lon] = group_reference(group, segments)
% This reference is used only to assign groups to truth during evaluation.
frames = unique([segments(group.segment_indices).effective_frames]);
lat = nan(size(frames));
lon = nan(size(frames));
for q = 1:numel(frames)
    lats = [];
    lons = [];
    for idx = group.segment_indices
        if ~ismember(frames(q), segments(idx).effective_frames), continue; end
        raw = find(segments(idx).raw_frames == frames(q), 1);
        lats(end+1) = segments(idx).lats(raw); %#ok<AGROW>
        lons(end+1) = segments(idx).lons(raw); %#ok<AGROW>
    end
    if ~isempty(lats)
        lat(q) = mean(lats);
        lon(q) = mean(lons);
    end
end
valid = isfinite(lat) & isfinite(lon);
frames = frames(valid);
lat = lat(valid);
lon = lon(valid);
end

function [frames, lat, lon] = snapshots_line(snapshots)
frames = [];
lat = [];
lon = [];
for k = 1:numel(snapshots)
    if isempty(snapshots{k}) || isempty(snapshots{k}.trackList), continue; end
    trk = snapshots{k}.trackList{1};
    if ~isfinite(trk.lat) || ~isfinite(trk.lon), continue; end
    frames(end+1) = k; %#ok<AGROW>
    lat(end+1) = trk.lat; %#ok<AGROW>
    lon(end+1) = trk.lon; %#ok<AGROW>
end
end

function errors = trajectory_errors(frames, lat, lon, truth, t_grid)
errors = [];
for q = 1:numel(frames)
    if frames(q) < 1 || frames(q) > numel(t_grid), continue; end
    true_lon = interp1(truth.time_sec, truth.lon, t_grid(frames(q)), 'linear', NaN);
    true_lat = interp1(truth.time_sec, truth.lat, t_grid(frames(q)), 'linear', NaN);
    if ~isfinite(true_lat) || ~isfinite(true_lon), continue; end
    errors(end+1) = sphere_utils_haversine_distance( ...
        lon(q), lat(q), true_lon, true_lat) / 1000; %#ok<AGROW>
end
end
