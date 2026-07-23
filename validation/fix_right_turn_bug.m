% fix_right_turn_bug — Targeted search to fix Config #51's right_rate_0p7 failure.
% Strategy: Lower gain AND lower ct_q to suppress right-turn resonance.
% Usage: matlab -batch "addpath(genpath('D:/Desktop/single_target_with-turn')); fix_right_turn_bug();"

function fix_right_turn_bug()
    addpath(genpath('D:/Desktop/single_target_with-turn'));

    fprintf('============================================================\n');
    fprintf('Fix right_rate_0p7 — Targeted Search Around Config #51\n');
    fprintf('============================================================\n\n');

    params = simulation_params_oracle();
    root = 'D:\Desktop\single_target_with-turn\validation';
    seed_start = 10001;
    n_seeds = 20;

    % Load baseline
    bl = load(fullfile(root, 'mc_best_params_results.mat'));
    default_rmse = squeeze(mean(bl.all_pos_rmse(1, :, :), 3)); % mean across radars & seeds -> [10,1]

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

    % Config #51 base values
    base_cv_dwell = 2500;
    base_ct_dwell = 660;
    base_ct_q = 5.3;

    % Search space: focused on fixing right_turn while preserving other gains
    % gain: [11→7] reduces over-response to right turns
    % ct_q: [5.3→4.0] reduces CT noise pollution
    gains = [11.0, 9.0, 7.0];
    ct_qs   = [5.3, 4.5, 4.0];

    fprintf('Scanning %d gains x %d ct_qs = %d combos\n', length(gains), length(ct_qs), length(gains)*length(ct_qs));
    fprintf('(Other params fixed at base: cv_dwell=%d, ct_dwell=%d)\n\n', base_cv_dwell, base_ct_dwell);

    prepared_full = cell(10, 1);
    for s = 1:10
        prepared_full{s} = prepare_oracle_tracking_inputs(specs{s}{1}, ...
            struct('random_seed', seed_start, ...
                   'truth_turn_rate_deg_per_sec', specs{s}{3}));
    end

    best_score = inf;
    best_protected = false;
    best_cfg = struct();
    all_results = struct();
    ri = 0;

    for gi = 1:length(gains)
        for cqi = 1:length(ct_qs)
            ri = ri + 1;
            g = gains(gi);
            q = ct_qs(cqi);

            c = struct(...
                'imm_cv_dwell_time_sec', base_cv_dwell, ...
                'imm_ct_dwell_time_sec', base_ct_dwell, ...
                'imm_ct_fixed_Q_scale', q, ...
                'imm_transient_gain_max', g, ...
                'imm_transient_nis_start', 3.0, ...
                'imm_transient_nis_full', 12.0, ...
                'imm_transient_ewma_alpha', 0.65, ...
                'imm_mu_init_CV', 0.5, ...
                'random_seed', seed_start);

            fprintf('[%d/%d] gain=%.1f ct_q=%.1f... ', ri, length(gains)*length(ct_qs), g, q);

            info = validate_protected(params, prepared_full, c, ...
                specs, weights, default_rmse, n_seeds, seed_start);

            fprintf('score=%.3f protected=%s\n', info.score, num2str(info.protected));

            all_results(ri).gain = g;
            all_results(ri).ct_q = q;
            all_results(ri).score = info.score;
            all_results(ri).protected = info.protected;
            all_results(ri).rmse_per_scenario = info.rmse_per_scenario;
            all_results(ri).max_penalty = info.max_penalty;

            if info.protected && info.score < best_score
                best_score = info.score;
                best_protected = true;
                best_cfg = c;
                fprintf('  --> NEW BEST PROTECTED!\n');
            end
        end
    end

    % Print all results sorted by score
    fprintf('\n=== ALL RESULTS SORTED BY SCORE ===\n');
    fprintf('%-6s  %-8s  %-6s  %-6s  %-10s  %-8s\n', 'Rank', 'Mode', 'Gain', 'Q', 'Score', 'Protected');
    fprintf('%s\n', repmat('-', 1, 50));

    scores_arr = [all_results.score];
    [sorted_scores, sort_idx] = sort(scores_arr);

    for k = 1:length(sorted_scores)
        idx = sort_idx(k);
        r = all_results(idx);
        flag = num2str(r.protected);
        fprintf('%-6d  %-8s  %-6.1f  %-6.1f  %-10.3f  %-8s\n', ...
            idx, '3in1', r.gain, r.ct_q, r.score, flag);

        % Show per-scenario for top 5
        if k <= 5
            for si = 1:10
                delta = (r.rmse_per_scenario(si) - default_rmse(si)) / default_rmse(si) * 100;
                status_str = '';
                if delta > 5, status_str = ' ***FAIL***'; end
                fprintf('    %-22s: %.3f (%+.1f%%%s)\n', specs{si}{2}, r.rmse_per_scenario(si), delta, status_str);
            end
        end
    end

    save(fullfile(root, 'fix_right_turn_results.mat'), ...
        'all_results', 'best_cfg', 'best_score', 'best_protected', ...
        'specs', 'default_rmse');

    fprintf('\nResults saved to fix_right_turn_results.mat\n');

    if best_protected
        fprintf('\n*** FOUND PROTECTED CONFIG! ***\n');
        fprintf('Best config params:\n');
        fnames = fieldnames(best_cfg);
        for fi = 1:numel(fnames)
            fprintf('  %-30s = %.1f\n', fnames{fi}, best_cfg.(fnames{fi}));
        end
    else
        fprintf('\nNo protected config found in this 3x3 scan.\n');
        fprintf('Top unprotected scores:\n');
        for k = 1:min(3, length(all_results))
            fprintf('  #%d: gain=%.1f ct_q=%.1f score=%.3f\n', ...
                k, all_results(k).gain, all_results(k).ct_q, all_results(k).score);
        end
    end
end

function info = validate_protected(params, prepared, config, specs, ...
        weights, default_rmse, n_seeds, seed_start)
    total_weighted = 0;
    total_weight_sum = 0;
    all_passed = true;
    max_penalty = 0;
    rmse_per_scenario = zeros(10, 1);

    for si = 1:10
        inp = prepared{si};
        rmse_vals = [];

        for radar_id = 1:2
            for seed_idx = 1:n_seeds
                seed_val = seed_start + seed_idx - 1;
                ov = struct('imm_adapt_mode', '3in1', 'random_seed', seed_val);

                fnames = fieldnames(config);
                for fi = 1:numel(fnames)
                    fname = fnames{fi};
                    if ~strcmp(fname, 'random_seed')
                        ov.(fname) = config.(fname);
                    end
                end

                rmse_val = run_one_eval(params, inp, radar_id, ov);
                rmse_vals(end+1) = rmse_val;
            end
        end

        rmse_avg = mean(rmse_vals);
        rmse_per_scenario(si) = rmse_avg;

        w = weights(si);
        total_weighted = total_weighted + w * rmse_avg;
        total_weight_sum = total_weight_sum + w;

        if si <= length(default_rmse)
            baseline = default_rmse(si);
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
    info.rmse_per_scenario = rmse_per_scenario;
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
