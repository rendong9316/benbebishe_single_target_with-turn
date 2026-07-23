% test_best_configs — Quick 20-seed validation of top LHS configs.
% Usage: matlab -batch "addpath(genpath('D:/Desktop/single_target_with-turn')); test_best_configs();"

function test_best_configs()
    addpath(genpath('D:/Desktop/single_target_with-turn'));
    params = simulation_params_oracle();
    seed_start = 10001;
    n_seeds = 20;

    specs = {
        {'single_straight',       'straight',        1.0}, ...
        {'single_turn_left_short', 'left_short',     1.0}, ...
        {'single_turn_right_short','right_short',    1.0}, ...
        {'single_turn_left_sustained','left_sustained',1.0}, ...
        {'single_turn_right_sustained','right_sustained',1.0}, ...
        {'multi_cross',            'multi_cross',    1.0}, ...
        {'single_turn_left_sustained','left_rate_0p7', 0.7}, ...
        {'single_turn_right_sustained','right_rate_0p7', 0.7}, ...
        {'single_turn_left_sustained','left_rate_1p3',1.3}, ...
        {'single_turn_right_sustained','right_rate_1p3',1.3}};

    weights = [0.15; 0.08; 0.08; 0.08; 0.08; 0.08; 0.06; 0.06; 0.07; 0.07];

    % Load baseline
    bl = load('D:\Desktop\single_target_with-turn\validation\mc_best_params_results.mat');
    default_rmse = squeeze(mean(bl.all_pos_rmse(1, :, :), 3)); % [10, 1] col vector

    fprintf('=== Validating Top 2 LHS configs (20 seeds, 10 scenes) ===\n\n');

    % Config #51 (best screening score)
    cfg51 = struct(...
        'imm_cv_dwell_time_sec', 2500, ...
        'imm_ct_dwell_time_sec', 660, ...
        'imm_ct_fixed_Q_scale', 5.3, ...
        'imm_transient_gain_max', 11.0, ...
        'random_seed', seed_start);

    % Config #28 (2nd best)
    cfg28 = struct(...
        'imm_cv_dwell_time_sec', 1000, ...
        'imm_ct_dwell_time_sec', 1500, ...
        'imm_ct_fixed_Q_scale', 5.6, ...
        'imm_transient_gain_max', 4.0, ...
        'random_seed', seed_start);

    all_cfgs = {struct('name', 'configA (#51)', 'params', cfg51), ...
                struct('name', 'configB (#28)', 'params', cfg28)};

    for ci = 1:2
        c = all_cfgs{ci};
        fprintf('=== %s ===\n', c.name);

        total_weighted = 0;
        all_passed = true;
        max_penalty = 0;
        scores_per_scenario = zeros(10, 1);

        for si = 1:10
            turn_rate = specs{si}{3};
            inp_st = prepare_oracle_tracking_inputs(specs{si}{1}, ...
                struct('random_seed', seed_start, ...
                       'truth_turn_rate_deg_per_sec', turn_rate));
            total_sq = [];

            for radar_id = 1:2
                for sk = 1:n_seeds
                    sv = seed_start + sk - 1;
                    ov = struct('imm_adapt_mode', '3in1', 'random_seed', sv);

                    fnames = fieldnames(c.params);
                    for fi = 1:numel(fnames)
                        fname = fnames{fi};
                        if ~strcmp(fname, 'random_seed')
                            ov.(fname) = c.params.(fname);
                        end
                    end

                    rmse_val = run_one_eval(params, inp_st, radar_id, ov);
                    total_sq(end+1) = rmse_val^2;
                end
            end

            rmse_km = sqrt(mean(total_sq));
            scores_per_scenario(si) = rmse_km;
            clear total_sq;
        end

        % Protection check & summary
        fprintf('%-25s  %-10s  %-10s\n', 'Scenario', 'Config RMSE', 'Delta%');
        for si = 1:10
            delta = (scores_per_scenario(si) - default_rmse(si)) / default_rmse(si) * 100;
            status = '';
            if scores_per_scenario(si) > 1.05 * default_rmse(si)
                status = ' *** FAILS ***';
                all_passed = false;
                violation = scores_per_scenario(si) - 1.05 * default_rmse(si);
                max_penalty = max(max_penalty, violation);
            end
            fprintf('%-25s  %-10.3f  %+.1f%%%s\n', specs{si}{2}, ...
                scores_per_scenario(si), delta, status);
        end

        overall = 0; total_w = 0;
        for si = 1:10
            overall = overall + weights(si) * scores_per_scenario(si);
            total_w = total_w + weights(si);
        end
        overall = overall / total_w;

        baseline_w = 0; total_w2 = 0;
        for si = 1:10
            baseline_w = baseline_w + weights(si) * default_rmse(si);
            total_w2 = total_w2 + weights(si);
        end
        baseline_w = baseline_w / total_w2;
        overall_delta = (overall - baseline_w)/baseline_w*100;
        fprintf('\n  Weighted avg RMSE: config=%.3fkm default=%.3fkm delta=%+.1f%%\n', ...
            overall, baseline_w, overall_delta);
        if all_passed
            fprintf('  STATUS: ALL SCENARIOS PROTECTED (within 5%% degradation)\n\n');
        else
            fprintf('  STATUS: FAILED protection on some scenarios\n\n');
        end
    end
end

function rmse_km = run_one_eval(params, inp, radar_id, overrides)
    if radar_id == 1
        det = inp.detList_R1; tg = inp.t1_grid;
        rl = params.radar1_lon; rlat = params.radar1_lat;
        tl = params.radar1_tx_lon; tlat = params.radar1_tx_lat;
    else
        det = inp.detList_R2; tg = inp.t2_grid;
        rl = params.radar2_lon; rlat = params.radar2_lat;
        tl = params.radar2_tx_lon; tlat = params.radar2_tx_lat;
    end

    prm = radar_params(params, radar_id);
    fnames = fieldnames(overrides);
    for fi = 1:numel(fnames)
        prm.(fnames{fi}) = overrides.(fnames{fi});
    end

    tpl = ukf_imm('create', prm, rl, rlat, tl, tlat, prm.dt_sec);
    [~, ~, snaps] = run_oracle_tracker_sequence(det, tpl, prm, inp.truth_all, tg, false);

    pos_sq_all = [];
    for f = 1:numel(snaps)
        if isempty(snaps{f}) || ~isfield(snaps{f}, 'trackList')
            continue;
        end
        for ti = 1:numel(snaps{f}.trackList)
            trk = snaps{f}.trackList{ti};
            if ~trk.updated || ~isfinite(trk.combined_nis)
                continue;
            end
            truth = inp.truthTrajs{trk.truth_idx};
            if isempty(truth) || ~isfield(truth, 'time_sec')
                continue;
            end
            t_now = tg(f);
            if t_now < truth.time_sec(1) || t_now > truth.time_sec(end)
                continue;
            end
            true_lon = interp1(truth.time_sec, truth.lon, t_now, 'linear');
            true_lat = interp1(truth.time_sec, truth.lat, t_now, 'linear');
            if ~all(isfinite([true_lon, true_lat]))
                continue;
            end
            dist_m = haversine_distance(trk.ukf.x(1), trk.ukf.x(3), true_lon, true_lat);
            pos_sq_all(end+1) = dist_m^2;
        end
    end
    clear tpl det snaps;
    rmse_km = sqrt(mean(pos_sq_all)) / 1000;
end

function dist = haversine_distance(lon1, lat1, lon2, lat2)
    dlat = deg2rad(lat2 - lat1);
    dlon = deg2rad(lon2 - lon1);
    a = sin(dlon/2)^2 + cosd(lat1)*cosd(lat2)*sin(dlat/2)^2;
    a = max(0, min(1, a));
    dist = 6371000 * 2 * atan2(sqrt(a), sqrt(1-a));
end
