function scenario = build_truth_scenario(params)
% build_truth_scenario — 统一构建单/多目标真值场景

    rng(params.random_seed);
    traj_list = {};

    switch lower(params.scenario)
        case 'straight'
            traj_list{1} = aircraft_trajectory_create(params.aircraft_waypoints, ...
                params.aircraft_speed_ms, params.dt_sec);

        case 'gradual_turn'
            [traj_list{1}, ~] = aircraft_trajectory_create('gradual_turn', params);

        case 'uturn'
            [traj_list{1}, ~] = aircraft_trajectory_create('uturn', params);

        case 'multi'
            waypoints = get_multi_waypoints(params);
            traj_list = cell(1, size(waypoints, 1));
            for ac = 1:size(waypoints, 1)
                traj_list{ac} = aircraft_trajectory_create(squeeze(waypoints(ac, :, :)), ...
                    params.aircraft_speed_ms, params.dt_sec);
            end

        otherwise
            error('build_truth_scenario: unknown scenario "%s"', params.scenario);
    end

    n_targets = length(traj_list);
    truth_all_cell = cell(1, n_targets);
    truthTrajs = cell(n_targets, 1);
    labels = cell(n_targets, 1);
    max_dur = 0;

    for ac = 1:n_targets
        tt = aircraft_trajectory_interpolate('generate', traj_list{ac});
        truth_all_cell{ac} = tt;
        labels{ac} = char('A' + ac - 1);
        truthTrajs{ac} = struct('label', labels{ac}, ...
            'speed_ms', params.aircraft_speed_ms, ...
            'time_sec', tt(:,5), 'lat', tt(:,2), 'lon', tt(:,1), ...
            'lon_rate', tt(:,3), 'lat_rate', tt(:,4));
        max_dur = max(max_dur, traj_list{ac}.duration_sec);
    end

    scenario = struct();
    scenario.truth_all_cell = truth_all_cell;
    scenario.truthTrajs = truthTrajs;
    scenario.traj_list = traj_list;
    scenario.t1_grid = params.time_offset_radar1_sec : params.dt_sec : max_dur;
    scenario.t2_grid = params.time_offset_radar2_sec : params.dt_sec : max_dur;
    scenario.n_frames = min(length(scenario.t1_grid), length(scenario.t2_grid));
    scenario.labels = labels;
    scenario.n_targets = n_targets;
    scenario.max_duration_sec = max_dur;
end


function waypoints = get_multi_waypoints(params)
    if isfield(params, 'multi_waypoints') && ~isempty(params.multi_waypoints)
        waypoints = params.multi_waypoints;
        return;
    end

    if isfield(params, 'n_targets') && params.n_targets ~= 3
        waypoints = make_default_n_waypoints(params.n_targets);
        return;
    end

    waypoints = zeros(3, 2, 3);
    waypoints(1,:,:) = [128.8, 30.5, 0; 132.0, 32.5, 0];
    waypoints(2,:,:) = [128.8, 32.5, 0; 132.0, 30.5, 0];
    waypoints(3,:,:) = [128.8, 31.5, 0; 130.5, 32.9, 0];
end


function wp = make_default_n_waypoints(n)
    wp = zeros(n, 2, 3);
    lat_span = 2.0;
    lat_start = 31.0 - lat_span/2;
    for i = 1:n
        lat0 = lat_start + (i-1) * (lat_span / max(n-1, 1));
        lat1 = lat_start + lat_span - (i-1) * (lat_span / max(n-1, 1));
        wp(i,:,:) = [128.8, lat0, 0; 132.0, lat1, 0];
    end
end
