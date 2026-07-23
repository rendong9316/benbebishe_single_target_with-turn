function orchestrate_search()
%ORCHESTRATE_SEARCH Entry point for Phase 2 IMM parameter search.
% Runs full pipeline: quick fuzzy check -> LHS 3in1 scan -> validate top configs.

addpath(genpath('D:/Desktop/single_target_with-turn'));

fprintf('============================================================\n');
fprintf('IMM UKF Parameter Search - Phase 2\n');
fprintf('============================================================\n\n');

params = simulation_params_oracle();
root = 'D:\Desktop\single_target_with-turn\validation';

% =====================================================================
% Step 1: Quick fuzzy_only sanity check (3 scenarios, 3 seeds)
% =====================================================================
fprintf('=== STEP 1: Quick fuzzy_only sanity check ===\n\n');

specs_quick = {'single_straight', 'single_turn_right_short', 'single_turn_right_sustained'};
n_quick_scenes = numel(specs_quick);
n_quick_seeds = 3;
seed_q = 10001;

prepared_q = cell(n_quick_scenes, 1);
for s = 1:n_quick_scenes
    prepared_q{s} = prepare_oracle_tracking_inputs(specs_quick{s}, struct('random_seed', seed_q));
end

modes_quick = {'3in1', 'fuzzy_only'};
quick_rmse = zeros(2, n_quick_scenes);

for mi = 1:2
    mode = modes_quick{mi};
    fprintf('Mode: %s\n', mode);
    for si = 1:n_quick_scenes
        inp = prepared_q{si};
        total_rmse = [];
        for rid = 1:2
            for sk = 1:n_quick_seeds
                sv = seed_q + sk - 1;
                ov = struct('imm_adapt_mode', mode, 'random_seed', sv);
                rmse_val = run_one_eval(params, inp, rid, ov);
                total_rmse(end+1) = rmse_val;
            end
        end
        rmse_km = mean(total_rmse);
        quick_rmse(mi, si) = rmse_km;
        fprintf('  %-25s: %.3f km\n', specs_quick{si}, rmse_km);
        clear total_sq;
    end
    fprintf('\n');
end

avg_3in1_q = mean(quick_rmse(1,:));
avg_fuzz_q = mean(quick_rmse(2,:));
delta_q = (avg_fuzz_q - avg_3in1_q) / avg_3in1_q * 100;
fprintf('AVG: 3in1=%.3f fuzzy=%.3f delta=%+.1f%%\n\n', avg_3in1_q, avg_fuzz_q, delta_q);

if delta_q > 20
    fprintf('fuzzy_only is >20%% worse. Focus search on 3in1 mode only.\n\n');
    run_fuzzy_only = false;
elseif delta_q < -10
    fprintf('fuzzy_only is >10%% better! Search both tracks.\n\n');
    run_fuzzy_only = true;
else
    fprintf('fuzzy_only within +/-20%%. Search both tracks.\n\n');
    run_fuzzy_only = true;
end

clear prepared_q quick_rmse;

% =====================================================================
% Step 2: LHS search - Track A (3in1), 80 candidates, 5 scenes, 5 seeds
% =====================================================================
fprintf('=== STEP 2: Track A LHS - 3in1 mode ===\n');

specs_screen = get_screen_scenarios();
n_screen = numel(specs_screen);
n_screen_seeds = 5;
screen_seed = 10001;

fprintf('Screening with %d scenes x %d seeds...\n', n_screen, n_screen_seeds);

prepared_screen = cell(n_screen, 1);
for s = 1:n_screen
    prepared_screen{s} = prepare_oracle_tracking_inputs(specs_screen{s}, ...
        struct('random_seed', screen_seed));
end

N_lhs = 80;
lhs_3in1 = generate_lhs(N_lhs, 8, 42);
scores_a = zeros(N_lhs, 1);

for i = 1:N_lhs
    if mod(i, 20) == 0
        fprintf('  Candidate %d/80...\n', i);
    end

    c = struct();
    c.imm_cv_dwell_time_sec = decode_param(lhs_3in1(i,1), 600, 6000, 100);
    c.imm_ct_dwell_time_sec = decode_param(lhs_3in1(i,2), 120, 1800, 30);
    c.imm_ct_fixed_Q_scale  = decode_param(lhs_3in1(i,3), 2.0, 6.0, 0.1);
    c.imm_transient_gain_max = decode_param(lhs_3in1(i,4), 2.0, 12.0, 0.5);
    c.imm_transient_nis_start = decode_param(lhs_3in1(i,5), 1.0, 6.0, 0.5);
    c.imm_transient_nis_full = decode_param(lhs_3in1(i,6), 4.0, 20.0, 1.0);
    c.imm_transient_ewma_alpha = decode_param(lhs_3in1(i,7), 0.20, 0.90, 0.05);
    c.imm_mu_init_CV = decode_param(lhs_3in1(i,8), 0.30, 0.80, 0.05);

    score = eval_candidate_screen(params, prepared_screen, c, ...
        '3in1', n_screen, n_screen_seeds, screen_seed);
    scores_a(i) = score;
end

[sorted_scores_a, order_a] = sort(scores_a);
top_a_idx = order_a(1:min(20, N_lhs));
fprintf('Top 5 3in1 candidates:\n');
for k = 1:min(5, numel(top_a_idx))
    idx = top_a_idx(k);
    fprintf('  #%d: score=%.3f\n', idx, sorted_scores_a(k));
end

save(fullfile(root, 'lhs_screen_results.mat'), ...
    'scores_a', 'order_a', 'top_a_idx', 'lhs_3in1', 'N_lhs');
fprintf('Saved lhs_screen_results.mat\n\n');

% =====================================================================
% Step 3: If fuzzy_only worth it, do same LHS for Track B
% =====================================================================
if run_fuzzy_only
    fprintf('=== STEP 3: Track B LHS - fuzzy_only mode ===\n');

    N_lhs_b = 80;
    lhs_fuzzy = generate_lhs(N_lhs_b, 7, 123);
    scores_b = zeros(N_lhs_b, 1);

    for i = 1:N_lhs_b
        if mod(i, 20) == 0
            fprintf('  Candidate %d/80...\n', i);
        end

        c = struct();
        c.fuzzy_window_size = round(decode_param(lhs_fuzzy(i,1), 1, 7, 1));
        c.fuzzy_ema_eta = decode_param(lhs_fuzzy(i,2), 0.02, 0.50, 0.01);
        c.adaptive_Q_min = decode_param(lhs_fuzzy(i,3), 0.2, 0.8, 0.05);
        c.adaptive_Q_max = decode_param(lhs_fuzzy(i,4), 2.0, 6.0, 0.1);
        c.imm_cv_dwell_time_sec = decode_param(lhs_fuzzy(i,5), 600, 6000, 100);
        c.imm_ct_dwell_time_sec = decode_param(lhs_fuzzy(i,6), 120, 1800, 30);
        c.imm_mu_init_CV = decode_param(lhs_fuzzy(i,7), 0.30, 0.80, 0.05);

        score = eval_candidate_screen(params, prepared_screen, c, ...
            'fuzzy_only', n_screen, n_screen_seeds, screen_seed);
        scores_b(i) = score;
    end

    [sorted_scores_b, order_b] = sort(scores_b);
    top_b_idx = order_b(1:min(20, N_lhs_b));
    fprintf('Top 5 fuzzy_only candidates:\n');
    for k = 1:min(5, numel(top_b_idx))
        idx = top_b_idx(k);
        fprintf('  #%d: score=%.3f\n', idx, sorted_scores_b(k));
    end

    save(fullfile(root, 'lhs_fuzzy_screen_results.mat'), ...
        'scores_b', 'order_b', 'top_b_idx', 'lhs_fuzzy', 'N_lhs_b');
    fprintf('Saved lhs_fuzzy_screen_results.mat\n\n');
else
    fprintf('Skipping Track B (fuzzy_only). Focusing on 3in1 only.\n\n');
end

% =====================================================================
% Step 4: Full-scenario validation for top candidates
% =====================================================================
fprintf('=== STEP 4: Full validation of top %d 3in1 candidates ===\n', min(20, N_lhs));

specs_full = get_all_10_scenarios();
n_full = numel(specs_full);
n_full_seeds = 10;
full_seed = 10001;

fprintf('Validating top 20 with %d scenes x %d seeds...\n\n', n_full, n_full_seeds);

prepared_full = cell(n_full, 1);
for s = 1:n_full
    prepared_full{s} = prepare_oracle_tracking_inputs(specs_full{s}{1}, ...
        struct('random_seed', full_seed));
end

best_score = inf;
best_config = struct();
best_protected = false;

for ti = 1:min(20, N_lhs)
    idx = top_a_idx(ti);
    c = build_3in1_config_from_lhs(idx, lhs_3in1);

    val_score = eval_candidate_full(params, prepared_full, specs_full, c, ...
        '3in1', n_full, n_full_seeds, full_seed);

    fprintf('  #%d: score=%.3f protected=%s\n', ...
        idx, val_score.score, num2str(val_score.protected));

    if val_score.protected && val_score.score < best_score
        best_score = val_score.score;
        best_config = c;
        best_protected = true;
    end
end

if ~best_protected
    fprintf('\nWARNING: No config passed protection constraint!\n');
    fprintf('Best unprotected score: %.3f\n', min(sorted_scores_a(1:20)));
end

save(fullfile(root, 'validated_candidates.mat'), ...
    'best_config', 'best_score', 'best_protected', 'specs_full');
fprintf('\nSaved validated_candidates.mat\n');

fprintf('\n=== SEARCH COMPLETE ===\n');
fprintf('Results saved to:\n');
fprintf('  - lhs_screen_results.mat (Track A LHS)\n');
if run_fuzzy_only
    fprintf('  - lhs_fuzzy_screen_results.mat (Track B LHS)\n');
end
fprintf('  - validated_candidates.mat (top candidates)\n');
fprintf('\nBest protected config score: %.3f km\n', best_score);

end


% =========================================================================
% HELPER FUNCTIONS
% =========================================================================

function specs = get_screen_scenarios()
    specs = {
        'single_straight',
        'single_turn_right_short',
        'single_turn_left_short',
        'single_turn_right_sustained',
        'multi_cross'};
end

function specs = get_all_10_scenarios()
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

function v = decode_param(u, lo, hi, step)
    v = lo + u * (hi - lo);
    v = max(lo, min(hi, v));
    v = round(v / step) * step;
    v = max(lo, min(hi, v));
end

function c = build_3in1_config_from_lhs(lhs_idx, lhs_mat)
    c = struct();
    c.imm_cv_dwell_time_sec = decode_param(lhs_mat(lhs_idx,1), 600, 6000, 100);
    c.imm_ct_dwell_time_sec = decode_param(lhs_mat(lhs_idx,2), 120, 1800, 30);
    c.imm_ct_fixed_Q_scale  = decode_param(lhs_mat(lhs_idx,3), 2.0, 6.0, 0.1);
    c.imm_transient_gain_max = decode_param(lhs_mat(lhs_idx,4), 2.0, 12.0, 0.5);
    c.imm_transient_nis_start = decode_param(lhs_mat(lhs_idx,5), 1.0, 6.0, 0.5);
    c.imm_transient_nis_full = decode_param(lhs_mat(lhs_idx,6), 4.0, 20.0, 1.0);
    c.imm_transient_ewma_alpha = decode_param(lhs_mat(lhs_idx,7), 0.20, 0.90, 0.05);
    c.imm_mu_init_CV = decode_param(lhs_mat(lhs_idx,8), 0.30, 0.80, 0.05);
end

function score_km = eval_candidate_screen(params, prepared, config, mode, ...
        n_scenarios, n_seeds, seed_start)
    total_rmse = [];

    for si = 1:n_scenarios
        inp = prepared{si};

        for radar_id = 1:2
            for seed_idx = 1:n_seeds
                seed_val = seed_start + seed_idx - 1;
                ov = struct('imm_adapt_mode', mode, 'random_seed', seed_val);

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

function score_val = eval_candidate_full(params, prepared, specs, config, ...
        mode, n_scenarios, n_seeds, seed_start)

    weights = [0.15, 0.08, 0.08, 0.08, 0.08, 0.08, 0.06, 0.06, 0.07, 0.07];

    bl_path = 'D:\Desktop\single_target_with-turn\validation\mc_best_params_results.mat';
    if exist(bl_path, 'file')
        bl_data = load(bl_path);
        if isfield(bl_data, 'means_global')
            default_rmse = bl_data.means_global;
        else
            default_rmse = zeros(n_scenarios, 1);
        end
    else
        default_rmse = zeros(n_scenarios, 1);
    end

    total_weighted = 0;
    total_weight_sum = 0;
    all_protected = true;
    max_penalty = 0;

    for si = 1:n_scenarios
        inp = prepared{si};
        rmse_list = [];

        for radar_id = 1:2
            for seed_idx = 1:n_seeds
                seed_val = seed_start + seed_idx - 1;
                ov = struct('imm_adapt_mode', mode, 'random_seed', seed_val);

                fnames = fieldnames(config);
                for fi = 1:numel(fnames)
                    ov.(fnames{fi}) = config.(fnames{fi});
                end

                pos_sq = run_one_eval(params, inp, radar_id, ov);
                rmse_list(end+1) = pos_sq;
            end
        end

        rmse_avg_radar = mean(rmse_list);
        weight = weights(si);
        total_weighted = total_weighted + weight * rmse_avg_radar;
        total_weight_sum = total_weight_sum + weight;

        if si <= length(default_rmse)
            baseline_avg = mean(default_rmse(si, :));
            if rmse_avg_radar > 1.05 * baseline_avg
                all_protected = false;
                violation = rmse_avg_radar - 1.05 * baseline_avg;
                max_penalty = max(max_penalty, violation);
            end
        end
    end

    raw_score = total_weighted / total_weight_sum;
    if ~all_protected
        raw_score = raw_score + 100.0 * max_penalty^2;
    end

    score_val.score = raw_score;
    score_val.protected = all_protected;
    score_val.max_penalty = max_penalty;
    clear rmse_list;
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
