function scenario = build_truth_scenario(scenario_name, params)
    if nargin < 1 || isempty(scenario_name)
        scenario_name = 'multi_cross';
    end

    switch char(scenario_name)
        case 'multi_cross'
            waypoints = {
                [128.8, 30.5, 0; 132.0, 32.5, 0], ...
                [128.8, 32.5, 0; 132.0, 30.5, 0], ...
                [128.8, 31.5, 0; 130.5, 32.9, 0]};
            labels = {'A', 'B', 'C'};
            trajs = cell(3, 1);
            for i = 1:3
                trajs{i} = aircraft_trajectory_create(waypoints{i}, params.aircraft_speed_ms, params.dt_sec);
            end
        case 'single_straight'
            labels = {'A'};
            trajs = {aircraft_trajectory_create(params.aircraft_waypoints, params.aircraft_speed_ms, params.dt_sec)};
        case 'single_turn'
            labels = {'A'};
            trajs = cell(1, 1);
            trajs{1} = aircraft_trajectory_create('turn', params);
        case {'single_uturn', 'single_u_turn'}
            labels = {'A'};
            trajs = cell(1, 1);
            trajs{1} = aircraft_trajectory_create('uturn', params);
        otherwise
            error('build_truth_scenario: unknown scenario "%s"', scenario_name);
    end

    n_targets = length(trajs);
    truth_all = cell(n_targets, 1);
    truthTrajs = cell(n_targets, 1);
    max_duration = 0;
    for ac = 1:n_targets
        tt = aircraft_trajectory_interpolate('generate', trajs{ac});
        truth_all{ac} = tt;
        max_duration = max(max_duration, tt(end, 5));
        truthTrajs{ac} = struct('label', labels{ac}, ...
            'speed_ms', params.aircraft_speed_ms, ...
            'time_sec', tt(:, 5), ...
            'lat', tt(:, 2), ...
            'lon', tt(:, 1), ...
            'lon_rate', tt(:, 3), ...
            'lat_rate', tt(:, 4));
    end

    t1_grid = params.time_offset_radar1_sec : params.dt_sec : max_duration;
    t2_grid = params.time_offset_radar2_sec : params.dt_sec : max_duration;
    n_frames = min(length(t1_grid), length(t2_grid));
    t1_grid = t1_grid(1:n_frames);
    t2_grid = t2_grid(1:n_frames);

    scenario = struct();
    scenario.name = char(scenario_name);
    scenario.n_targets = n_targets;
    scenario.truth_all = truth_all;
    scenario.truthTrajs = truthTrajs;
    scenario.t1_grid = t1_grid;
    scenario.t2_grid = t2_grid;
    scenario.n_frames = n_frames;
end
