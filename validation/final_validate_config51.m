% final_validate_config51 — 50-seed final validation of Config #51.
% Usage: matlab -batch "addpath(genpath('D:/Desktop/single_target_with-turn')); final_validate_config51();"

function final_validate_config51()
    addpath(genpath('D:/Desktop/single_target_with-turn'));

    fprintf('============================================================\n');
    fprintf('FINAL VALIDATION — Config #51 (50 seeds, 10 scenes)\n');
    fprintf('============================================================\n\n');

    params = simulation_params_oracle();
    seed_start = 10001;
    n_seeds = 50;
    root = 'D:\Desktop\single_target_with-turn\validation';

    % Config #51 parameters
    cfg51 = struct(...
        'imm_cv_dwell_time_sec', 2500, ...
        'imm_ct_dwell_time_sec', 660, ...
        'imm_ct_fixed_Q_scale', 5.3, ...
        'imm_transient_gain_max', 11.0, ...
        'imm_transient_nis_start', 3.0, ...
        'imm_transient_nis_full', 12.0, ...
        'imm_transient_ewma_alpha', 0.65, ...
        'imm_mu_init_CV', 0.5);

    specs = { ...
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
    bl = load(fullfile(root, 'mc_best_params_results.mat'));
    default_rmse = squeeze(mean(bl.all_pos_rmse(1, :, :), 3));

    % Prepare scenarios
    fprintf('Preparing detection data...\n');
    prepared = cell(10, 1);
    for s = 1:10
        prepared{s} = prepare_oracle_tracking_inputs(specs{s}{1}, ...
            struct('random_seed', seed_start, ...
                   'truth_turn_rate_deg_per_sec', specs{s}{3}));
    end
    fprintf('Done.\n\n');

    scores = zeros(10, 1);
    fprintf('%-22s  %-12s  %-12s  %-8s  %-12s\n', 'Scenario', 'Config#51', 'Default', 'Delta%', 'Status');
    fprintf('%-22s  %-12s  %-12s  %-8s  %-12s\n', ...
        repmat('-', 1, 22), repmat('-', 1, 12), repmat('-', 1, 12), ...
        repmat('-', 1, 8), repmat('-', 1, 12));

    all_passed = true;
    max_penalty = 0;
    total_weighted = 0;
    total_weight_sum = 0;

    for si = 1:10
        inp = prepared{si};
        rmse_vals = [];

        for radar_id = 1:2
            for sk = 1:n_seeds
                sv = seed_start + sk - 1;
                ov = struct('imm_adapt_mode', '3in1', 'random_seed', sv);

                fnames = fieldnames(cfg51);
                for fi = 1:numel(fnames)
                    if ~strcmp(fnames{fi}, 'random_seed')
                        ov.(fnames{fi}) = cfg51.(fnames{fi});
                    end
                end

                rmse_val = run_one_eval(params, inp, radar_id, ov);
                rmse_vals(end+1) = rmse_val;
            end
        end

        rmse_avg = mean(rmse_vals);
        scores(si) = rmse_avg;
        delta = (rmse_avg - default_rmse(si)) / default_rmse(si) * 100;

        status = 'PASS';
        if rmse_avg > 1.05 * default_rmse(si)
            status = 'FAIL';
            all_passed = false;
            violation = rmse_avg - 1.05 * default_rmse(si);
            max_penalty = max(max_penalty, violation);
        end

        w = weights(si);
        total_weighted = total_weighted + w * rmse_avg;
        total_weight_sum = total_weight_sum + w;

        fprintf('%-22s  %-12.3f  %-12.3f  %+7.1f%%  %-12s\n', ...
            specs{si}{2}, rmse_avg, default_rmse(si), delta, status);
    end

    overall_score = total_weighted / total_weight_sum;
    overall_default = 0;
    for si = 1:10
        w = weights(si);
        overall_default = overall_default + w * default_rmse(si);
    end
    overall_delta = (overall_score - overall_default) / overall_default * 100;

    fprintf('\n------------------------------------------------------------\n');
    fprintf('FINAL RESULTS:\n');
    fprintf('  Overall weighted RMSE: config=%.3fkm  default=%.3fkm\n', overall_score, overall_default);
    fprintf('  Improvement: %+.1f%%\n', overall_delta);
    fprintf('  Protection constraint: %s\n', num2str(all_passed));

    if ~all_passed
        fprintf('  Max penalty: %.3f km\n', max_penalty);
        raw_score = overall_score + 100.0 * max_penalty^2;
        fprintf('  Score with penalty: %.3f\n', raw_score);
    end

    save(fullfile(root, 'final_validate_config51.mat'), ...
        'scores', 'default_rmse', 'all_passed', 'overall_score', ...
        'overall_default', 'overall_delta', 'max_penalty', 'specs');

    fprintf('\nResults saved to %s/final_validate_config51.mat\n', root);
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
