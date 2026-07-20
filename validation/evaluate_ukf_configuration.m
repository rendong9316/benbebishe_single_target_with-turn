function report = evaluate_ukf_configuration(overrides, prepared_inputs, verbose)
%EVALUATE_UKF_CONFIGURATION Evaluate one filter-only parameter set.
% Detection data are prepared once and reused so candidates see identical data.

if nargin < 1 || isempty(overrides), overrides = struct(); end
if nargin < 2 || isempty(prepared_inputs)
    names = {'single_turn', 'single_uturn', 'multi_cross'};
    prepared_inputs = cellfun(@prepare_oracle_tracking_inputs, names, ...
        'UniformOutput', false);
end
if nargin < 3, verbose = false; end

cases = struct('scenario', {}, 'radar_id', {}, 'position_rmse_km', {}, ...
    'speed_rmse_ms', {}, 'nis_mean', {}, 'nees_mean', {}, ...
    'track_count', {}, 'sample_count', {});
all_position_sq = [];
all_speed_sq = [];
all_nis = [];
all_nees = [];

for scenario_index = 1:numel(prepared_inputs)
    inputs = prepared_inputs{scenario_index};
    params = apply_overrides_local(inputs.params, overrides);
    for radar_id = 1:2
        params_radar = radar_params(params, radar_id);
        if radar_id == 1
            detections = inputs.detList_R1;
            time_grid = inputs.t1_grid;
            radar_lon = params.radar1_lon;
            radar_lat = params.radar1_lat;
            tx_lon = params.radar1_tx_lon;
            tx_lat = params.radar1_tx_lat;
        else
            detections = inputs.detList_R2;
            time_grid = inputs.t2_grid;
            radar_lon = params.radar2_lon;
            radar_lat = params.radar2_lat;
            tx_lon = params.radar2_tx_lon;
            tx_lat = params.radar2_tx_lat;
        end
        template = ukf_imm('create', params_radar, radar_lon, radar_lat, ...
            tx_lon, tx_lat, params.dt_sec);
        [tracks, ~, snapshots] = run_oracle_tracker_sequence( ...
            detections, template, params_radar, inputs.truth_all, ...
            time_grid, false);

        position_sq = [];
        speed_sq = [];
        nees = [];
        nis = [];
        for track_index = 1:numel(tracks)
            filter = tracks{track_index}.ukf;
            if isfield(filter, 'ukf_cv')
                histories = [filter.ukf_cv.nis_history(:), ...
                    filter.ukf_ct.nis_history(:), ...
                    filter.ukf_ct_right.nis_history(:)];
                if ~isempty(histories)
                    % The best-supported model represents the active IMM branch.
                    nis = [nis; min(histories, [], 2)]; %#ok<AGROW>
                end
            end
        end
        for frame = 1:numel(snapshots)
            if isempty(snapshots{frame}) || ...
                    ~isfield(snapshots{frame}, 'trackList')
                continue;
            end
            for track_index = 1:numel(snapshots{frame}.trackList)
                track = snapshots{frame}.trackList{track_index};
                truth = inputs.truthTrajs{track.truth_idx};
                time_sec = time_grid(frame);
                truth_state = [interp1(truth.time_sec, truth.lon, time_sec, 'linear', NaN); ...
                    interp1(truth.time_sec, truth.lon_rate, time_sec, 'linear', NaN); ...
                    interp1(truth.time_sec, truth.lat, time_sec, 'linear', NaN); ...
                    interp1(truth.time_sec, truth.lat_rate, time_sec, 'linear', NaN)];
                if ~all(isfinite(truth_state)), continue; end
                estimate = track.ukf.x;
                error_state = estimate - truth_state;
                position_error = geographic_distance_local( ...
                    estimate(1), estimate(3), truth_state(1), truth_state(3));
                position_sq(end+1, 1) = position_error^2; %#ok<AGROW>
                estimated_speed = geographic_speed_local(estimate);
                truth_speed = geographic_speed_local(truth_state);
                speed_sq(end+1, 1) = (estimated_speed - truth_speed)^2; %#ok<AGROW>
                covariance = (track.ukf.P + track.ukf.P') / 2;
                if rcond(covariance) > 1e-14
                    value = error_state' * (covariance \ error_state);
                    if isfinite(value), nees(end+1, 1) = value; end %#ok<AGROW>
                end
            end
        end

        entry = struct('scenario', inputs.scenario.name, 'radar_id', radar_id, ...
            'position_rmse_km', sqrt(mean(position_sq)) / 1000, ...
            'speed_rmse_ms', sqrt(mean(speed_sq)), 'nis_mean', mean(nis), ...
            'nees_mean', mean(nees), 'track_count', numel(tracks), ...
            'sample_count', numel(position_sq));
        cases(end+1) = entry; %#ok<AGROW>
        all_position_sq = [all_position_sq; position_sq]; %#ok<AGROW>
        all_speed_sq = [all_speed_sq; speed_sq]; %#ok<AGROW>
        all_nis = [all_nis; nis]; %#ok<AGROW>
        all_nees = [all_nees; nees]; %#ok<AGROW>
        if verbose
            fprintf('%s R%d: pos=%.3f km speed=%.2f m/s NIS=%.2f NEES=%.2f\n', ...
                entry.scenario, radar_id, entry.position_rmse_km, ...
                entry.speed_rmse_ms, entry.nis_mean, entry.nees_mean);
        end
    end
end

report = struct();
report.overrides = overrides;
report.cases = cases;
report.position_rmse_km = sqrt(mean(all_position_sq)) / 1000;
report.speed_rmse_ms = sqrt(mean(all_speed_sq));
report.nis_mean = mean(all_nis);
report.nees_mean = mean(all_nees);
consistency_penalty = 0.10 * abs(report.nis_mean - 3) + ...
    0.20 * abs(report.nees_mean - 4);
report.score = report.position_rmse_km + ...
    0.01 * report.speed_rmse_ms + consistency_penalty;
end


function params = apply_overrides_local(params, overrides)
names = fieldnames(overrides);
for i = 1:numel(names)
    params.(names{i}) = overrides.(names{i});
end
end


function distance = geographic_distance_local(lon1, lat1, lon2, lat2)
delta_lon = deg2rad(lon2 - lon1);
delta_lat = deg2rad(lat2 - lat1);
a = sin(delta_lat / 2)^2 + cosd(lat1) * cosd(lat2) * sin(delta_lon / 2)^2;
a = max(0, min(1, a));
distance = 6371000 * 2 * atan2(sqrt(a), sqrt(1 - a));
end


function speed = geographic_speed_local(state)
speed = hypot(state(2) * 6371000 * pi / 180 * cosd(state(3)), ...
    state(4) * 6371000 * pi / 180);
end
