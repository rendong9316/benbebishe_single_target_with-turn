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
    'nis_count', {}, 'nees_count', {}, 'nis_coverage95', {}, ...
    'nees_coverage95', {}, 'turn_direction_accuracy', {}, ...
    'straight_false_ct_rate', {}, 'turn_detection_delay_frames', {}, ...
    'track_count', {}, 'sample_count', {}, 'random_seed', {});
all_position_sq = [];
all_speed_sq = [];
all_nis = [];
all_nees = [];

for scenario_index = 1:numel(prepared_inputs)
    input_item = prepared_inputs{scenario_index};
    if ischar(input_item) || isstring(input_item)
        loaded_input = load(char(input_item), 'inputs');
        inputs = loaded_input.inputs;
    else
        inputs = input_item;
    end
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
        truth_modes = [];
        predicted_modes = [];
        mode_frames = [];
        for frame = 1:numel(snapshots)
            if isempty(snapshots{frame}) || ...
                    ~isfield(snapshots{frame}, 'trackList')
                continue;
            end
            for track_index = 1:numel(snapshots{frame}.trackList)
                track = snapshots{frame}.trackList{track_index};
                if track.updated && isfinite(track.combined_nis)
                    nis(end+1, 1) = track.combined_nis; %#ok<AGROW>
                end
                truth = inputs.truthTrajs{track.truth_idx};
                time_sec = time_grid(frame);
                truth_state = [interp1(truth.time_sec, truth.lon, time_sec, 'linear', NaN); ...
                    interp1(truth.time_sec, truth.lon_rate, time_sec, 'linear', NaN); ...
                    interp1(truth.time_sec, truth.lat, time_sec, 'linear', NaN); ...
                    interp1(truth.time_sec, truth.lat_rate, time_sec, 'linear', NaN)];
                if ~all(isfinite(truth_state)), continue; end
                truth_mode = truth_mode_local(truth, time_sec, params.dt_sec);
                [~, predicted_mode] = max(track.ukf.mu);
                truth_modes(end+1, 1) = truth_mode; %#ok<AGROW>
                predicted_modes(end+1, 1) = predicted_mode; %#ok<AGROW>
                mode_frames(end+1, 1) = frame; %#ok<AGROW>
                estimate = track.ukf.x;
                [error_state, covariance] = physical_error_covariance_local( ...
                    estimate, truth_state, track.ukf.P);
                position_error = geographic_distance_local( ...
                    estimate(1), estimate(3), truth_state(1), truth_state(3));
                position_sq(end+1, 1) = position_error^2; %#ok<AGROW>
                estimated_speed = geographic_speed_local(estimate);
                truth_speed = geographic_speed_local(truth_state);
                speed_sq(end+1, 1) = (estimated_speed - truth_speed)^2; %#ok<AGROW>
                if rcond(covariance) > 1e-14
                    value = error_state' * (covariance \ error_state);
                    if isfinite(value), nees(end+1, 1) = value; end %#ok<AGROW>
                end
            end
        end

        turn_mask = truth_modes ~= 1;
        straight_mask = truth_modes == 1;
        turn_accuracy = mean_or_nan_local(predicted_modes(turn_mask) == truth_modes(turn_mask));
        false_ct_rate = mean_or_nan_local(predicted_modes(straight_mask) ~= 1);
        detection_delay = detection_delay_local( ...
            truth_modes, predicted_modes, mode_frames);
        entry = struct('scenario', inputs.scenario.name, 'radar_id', radar_id, ...
            'position_rmse_km', sqrt(mean(position_sq)) / 1000, ...
            'speed_rmse_ms', sqrt(mean(speed_sq)), 'nis_mean', mean(nis), ...
            'nees_mean', mean(nees), 'nis_count', numel(nis), ...
            'nees_count', numel(nees), ...
            'nis_coverage95', mean_or_nan_local(nis >= 0.2158 & nis <= 9.3484), ...
            'nees_coverage95', mean_or_nan_local(nees >= 0.4844 & nees <= 11.1433), ...
            'turn_direction_accuracy', turn_accuracy, ...
            'straight_false_ct_rate', false_ct_rate, ...
            'turn_detection_delay_frames', detection_delay, ...
            'track_count', numel(tracks), 'sample_count', numel(position_sq), ...
            'random_seed', params.random_seed);
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
        clear tracks snapshots detections template params_radar;
    end
    clear inputs loaded_input;
end

report = struct();
report.overrides = overrides;
report.cases = cases;
report.position_rmse_km = sqrt(mean(all_position_sq)) / 1000;
report.speed_rmse_ms = sqrt(mean(all_speed_sq));
report.nis_mean = mean(all_nis);
report.nees_mean = mean(all_nees);
report.nis_coverage95 = mean(all_nis >= 0.2158 & all_nis <= 9.3484);
report.nees_coverage95 = mean(all_nees >= 0.4844 & all_nees <= 11.1433);
consistency_penalty = 0.10 * abs(report.nis_mean - 3) + ...
    0.20 * abs(report.nees_mean - 4);
report.score = report.position_rmse_km + ...
    0.01 * report.speed_rmse_ms + consistency_penalty;
end


function mode = truth_mode_local(truth, time_sec, dt)
t0 = max(truth.time_sec(1), time_sec - dt);
t1 = min(truth.time_sec(end), time_sec + dt);
if t1 <= t0
    mode = 1;
    return;
end
lon_rate0 = interp1(truth.time_sec, truth.lon_rate, t0, 'linear');
lat_rate0 = interp1(truth.time_sec, truth.lat_rate, t0, 'linear');
lat0 = interp1(truth.time_sec, truth.lat, t0, 'linear');
lon_rate1 = interp1(truth.time_sec, truth.lon_rate, t1, 'linear');
lat_rate1 = interp1(truth.time_sec, truth.lat_rate, t1, 'linear');
lat1 = interp1(truth.time_sec, truth.lat, t1, 'linear');
heading0 = atan2d(lon_rate0 * cosd(lat0), lat_rate0);
heading1 = atan2d(lon_rate1 * cosd(lat1), lat_rate1);
heading_rate = (mod(heading1 - heading0 + 180, 360) - 180) / (t1 - t0);
if heading_rate < -0.2
    mode = 2;
elseif heading_rate > 0.2
    mode = 3;
else
    mode = 1;
end
end


function delay = detection_delay_local(truth_modes, predicted_modes, frames)
turn_index = find(truth_modes ~= 1, 1, 'first');
if isempty(turn_index)
    delay = NaN;
    return;
end
correct = find(predicted_modes(turn_index:end) == truth_modes(turn_index:end), 1, 'first');
if isempty(correct)
    delay = NaN;
else
    detection_index = turn_index + correct - 1;
    delay = frames(detection_index) - frames(turn_index);
end
end


function value = mean_or_nan_local(values)
if isempty(values), value = NaN; else, value = mean(values); end
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


function [error_metric, covariance_metric] = physical_error_covariance_local( ...
        estimate, truth_state, covariance_geo)
earth_radius = 6371000.0;
meters_per_lon_degree = earth_radius * pi / 180 * cosd(truth_state(3));
meters_per_lat_degree = earth_radius * pi / 180;
delta_lon = mod(estimate(1) - truth_state(1) + 180, 360) - 180;
v_east_est = estimate(2) * earth_radius * pi / 180 * cosd(estimate(3));
v_east_truth = truth_state(2) * earth_radius * pi / 180 * cosd(truth_state(3));
v_north_est = estimate(4) * meters_per_lat_degree;
v_north_truth = truth_state(4) * meters_per_lat_degree;
error_metric = [delta_lon * meters_per_lon_degree; ...
    v_east_est - v_east_truth; ...
    (estimate(3) - truth_state(3)) * meters_per_lat_degree; ...
    v_north_est - v_north_truth];
jacobian = zeros(4);
jacobian(1, 1) = meters_per_lon_degree;
jacobian(2, 2) = earth_radius * pi / 180 * cosd(estimate(3));
jacobian(2, 3) = -estimate(2) * earth_radius * (pi / 180)^2 * ...
    sind(estimate(3));
jacobian(3, 3) = meters_per_lat_degree;
jacobian(4, 4) = meters_per_lat_degree;
covariance_metric = jacobian * covariance_geo * jacobian';
covariance_metric = (covariance_metric + covariance_metric') / 2;
end
