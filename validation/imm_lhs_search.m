% imm_lhs_search.m — Minimal LHS parameter search for IMM UKF (3in1 mode).
%
% Usage in MATLAB:
%   addpath(genpath('D:/Desktop/single_target_with-turn'));
%   imm_lhs_search()
%
% Searches 8-parameter space with 60 LHS candidates.
% Each candidate evaluated on 3 screen scenarios x 2 radars x 5 seeds (30 runs).
% Fast screening -> Top 10 -> Full 10-scenario validation (10 seeds, protection constraint).

function imm_lhs_search()
    fprintf('============================================================\n');
    fprintf('IMM UKF Parameter Search - LHS Phase\n');
    fprintf('============================================================\n\n');

    params = simulation_params_oracle();
    root = 'D:\Desktop\single_target_with-turn\validation';

    % --- Screen scenarios (fast filter) ---
    specs_screen = {
        'single_straight',           % critical protection scenario
        'single_turn_right_short',   % short right turn
        'single_turn_right_sustained'}; % sustained turn

    n_screen = numel(specs_screen);
    n_seeds = 5;
    seed_start = 10001;

    fprintf('Screening with %d scenes x %d seeds per candidate\n\n', n_screen, n_seeds);

    prepared = cell(n_screen, 1);
    for s = 1:n_screen
        prepared{s} = prepare_oracle_tracking_inputs(specs_screen{s}, ...
            struct('random_seed', seed_start));
    end

    % --- Load baseline for protection check ---
    bl_path = fullfile(root, 'mc_best_params_results.mat');
    if exist(bl_path, 'file')
        bl_data = load(bl_path);
        default_rmse_all = bl_data.all_pos_rmse; % [4 configs, 10 scenes, 50 seeds]
        default_rmse_avg = mean(default_rmse_all(1, :, :), 3); % mean over radar+seeds for config 1
    else
        error('Cannot find mc_best_params_results.mat - run test_run_mc_best_params first');
    end

    % --- 8 IMM parameters to sweep ---
    param_names = {'imm_cv_dwell_time_sec', ...
                   'imm_ct_dwell_time_sec', ...
                   'imm_ct_fixed_Q_scale', ...
                   'imm_transient_gain_max', ...
                   'imm_transient_nis_start', ...
                   'imm_transient_nis_full', ...
                   'imm_transient_ewma_alpha', ...
                   'imm_mu_init_CV'};

    param_lo = [600, 120, 2.0, 2.0, 1.0, 4.0, 0.20, 0.30];
    param_hi = [6000, 1800, 6.0, 12.0, 6.0, 20.0, 0.90, 0.80];
    param_step = [100, 30, 0.1, 0.5, 0.5, 1.0, 0.05, 0.05];

    N = 60;
    lhs_mat = generate_lhs(N, 8, 42);
    scores = zeros(N, 1);
    candidates = cell(N, 1);

    fprintf('Evaluating %d candidates...\n\n', N);

    for i = 1:N
        if mod(i, 10) == 0
            fprintf('  [%d/%d]\n', i, N);
        end

        c = struct();
        for p = 1:8
            name = param_names{p};
            lo = param_lo(p); hi = param_hi(p); st = param_step(p);
            v = lo + lhs_mat(i, p) * (hi - lo);
            v = max(lo, min(hi, v));
            v = round(v / st) * st;
            v = max(lo, min(hi, v));
            c.(name) = v;
        end
        candidates{i} = c;

        score = eval_candidate_fast(params, prepared, c, ...
            n_screen, n_seeds, seed_start);
        scores(i) = score;
    end

    fprintf('\n--- Top 10 by screening score (avg RMSE in km) ---\n');
    [sorted_scores, order] = sort(scores);

    for k = 1:min(10, N)
        idx = order(k);
        c = candidates{idx};
        fprintf('#%d: %.3f km  cv_dwell=%g ct_dwell=%g ct_q=%.1f gain=%.1f\n', ...
            idx, sorted_scores(k), ...
            c.imm_cv_dwell_time_sec, c.imm_ct_dwell_time_sec, ...
            c.imm_ct_fixed_Q_scale, c.imm_transient_gain_max);
    end

    save(fullfile(root, 'lhs_screen.mat'), ...
        'scores', 'order', 'candidates', 'param_names', ...
        'default_rmse_avg', 'N');
    fprintf('\nSaved lhs_screen.mat\n\n');

    % === FULL VALIDATION: Top 10 on all 10 scenarios ===
    fprintf('=== Validating top 10 on ALL 10 scenarios (10 seeds) ===\n\n');

    specs_full = get_full_scenarios();
    n_full = numel(specs_full);
    n_full_seeds = 10;
    full_seed = 10001;

    prepared_full = cell(n_full, 1);
    for s = 1:n_full
        prepared_full{s} = prepare_oracle_tracking_inputs(specs_full{s}{1}, ...
            struct('random_seed', full_seed));
    end

    best_score = inf;
    best_config = struct();
    best_idx = -1;
    all_full_scores = zeros(10, 1);

    weights = [0.15, 0.08, 0.08, 0.08, 0.08, 0.08, 0.06, 0.06, 0.07, 0.07];

    for ti = 1:10
        idx = order(ti);
        c = candidates{idx};

        fprintf('Validating #%d...\n', idx);
        info = validate_protected(params, prepared_full, c, ...
            specs_full, weights, default_rmse_avg, ...
            n_full, n_full_seeds, full_seed);

        all_full_scores(ti) = info.score;
        fprintf('  Score: %.3f  Protected: %s\n\n', ...
            info.score, num2str(info.protected));

        if info.protected && info.score < best_score
            best_score = info.score;
            best_config = c;
            best_idx = idx;
        end
    end

    if ~exist('info', 'var')
        fprintf('WARNING: No config passed protection!\n');
        fprintf('Best unprotected score: %.3f\n', min(all_full_scores));
    end

    save(fullfile(root, 'lhs_final.mat'), ...
        'best_config', 'best_score', 'best_idx', ...
        'all_full_scores', 'specs_full', 'default_rmse_avg');

    fprintf('\n=== SEARCH COMPLETE ===\n');
    if best_idx > 0
        fprintf('Best protected config: #%d (score=%.3f km)\n\n', best_idx, best_score);
        fprintf('Params:\n');
        fnames = fieldnames(best_config);
        for fi = 1:numel(fnames)
            fprintf('  %-30s = %.3f\n', fnames{fi}, best_config.(fnames{fi}));
        end
    else
        fprintf('No config passed protection constraint.\n');
        fprintf('Best unprotected scores:\n');
        for k = 1:min(10, N)
            idx = order(k);
            c = candidates{idx};
            fprintf('  #%d: %.3f\n', idx, all_full_scores(k));
        end
    end
end


% =========================================================================
% HELPER FUNCTIONS
% =========================================================================

function specs = get_full_scenarios()
    specs = {...
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
end

function lhs_matrix = generate_lhs(n_samples, n_dims, seed_val)
    rng(seed_val, 'twister');
    unit = zeros(n_samples, n_dims);
    for col = 1:n_dims
        perm = randperm(n_samples);
        unit(:, col) = (perm' - rand(n_samples, 1)) / n_samples;
    end
    lhs_matrix = unit .^ 0.5;
end

function score_km = eval_candidate_fast(params, prepared, config, ...
        n_scenarios, n_seeds, seed_start)
    total_rmse = [];

    for si = 1:n_scenarios
        inp = prepared{si};

        for radar_id = 1:2
            for seed_idx = 1:n_seeds
                seed_val = seed_start + seed_idx - 1;
                ov = struct('imm_adapt_mode', '3in1', 'random_seed', seed_val);

                fnames = fieldnames(config);
                for fi = 1:numel(fnames)
                    ov.(fnames{fi}) = config.(fnames{fi});
                end

                rmse_val = run_one_eval(params, inp, radar_id, ov);
                total_rmse(end+1) = rmse_val;
            end
        end
    end

    score_km = mean(total_rmse);
end

function info = validate_protected(params, prepared, config, specs, ...
        weights, default_rmse, n_scenarios, n_seeds, seed_start)
    total_weighted = 0;
    total_weight_sum = 0;
    all_passed = true;
    max_penalty = 0;

    for si = 1:n_scenarios
        inp = prepared{si};
        rmse_vals = [];

        for radar_id = 1:2
            for seed_idx = 1:n_seeds
                seed_val = seed_start + seed_idx - 1;
                ov = struct('imm_adapt_mode', '3in1', 'random_seed', seed_val);

                fnames = fieldnames(config);
                for fi = 1:numel(fnames)
                    ov.(fnames{fi}) = config.(fnames{fi});
                end

                rmse_val = run_one_eval(params, inp, radar_id, ov);
                rmse_vals(end+1) = rmse_val;
            end
        end

        rmse_avg = mean(rmse_vals);
        w = weights(si);
        total_weighted = total_weighted + w * rmse_avg;
        total_weight_sum = total_weight_sum + w;

        if si <= length(default_rmse)
            baseline = mean(default_rmse(si, :));
            if rmse_avg > 1.05 * baseline
                all_passed = false;
                violation = rmse_avg - 1.05 * baseline;
                max_penalty = max(max_penalty, violation);
            end
        end
    end

    raw_score = total_weighted / total_weight_sum;
    if ~all_passed
        raw_score = raw_score + 100.0 * max_penalty^2;
    end

    info.score = raw_score;
    info.protected = all_passed;
    info.max_penalty = max_penalty;
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
