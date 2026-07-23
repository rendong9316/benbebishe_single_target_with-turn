function run_fuzzy_vs_3in1_baseline()
%RUN_FUZZY_VS_3IN1_BASELINE Quick benchmark: fuzzy_only default vs 3in1 default.
%
% Usage:
%   run_fuzzy_vs_3in1_baseline
%
% Compares default params in 3in1 mode vs fuzzy_only mode across all 10 scenarios.
% 50 seeds, R1 radar only. Purpose: determine if fuzzy_only is worth searching.

    root = fileparts(mfilename('fullpath'));
    params = simulation_params_oracle();

    % --- All 10 validation scenarios -------------------------------------
    specs = {...
        {'single_straight',       'single_straight',    1.0}, ...
        {'single_turn_left_short', 'left_short',         1.0}, ...
        {'single_turn_right_short','right_short',        1.0}, ...
        {'single_turn_left_sustained','left_sustained',  1.0}, ...
        {'single_turn_right_sustained','right_sustained',1.0}, ...
        {'multi_cross',            'multi_cross',        1.0}, ...
        {'single_turn_left_sustained','left_rate_0p7',   0.7}, ...
        {'single_turn_right_sustained','right_rate_0p7', 0.7}, ...
        {'single_turn_left_sustained','left_rate_1p3',  1.3}, ...
        {'single_turn_right_sustained','right_rate_1p3',1.3}};

    n_scenarios = numel(specs);
    n_seeds = 50;
    seed_start = 10001;

    fprintf('============================================================\n');
    fprintf('Fuzzy-Only vs 3in1 Baseline Comparison\n');
    fprintf('============================================================\n');
    fprintf('Seeds:        %d (%d-%d)\n', n_seeds, seed_start, seed_start+n_seeds-1);
    fprintf('Scenarios:    %d\n', n_scenarios);
    fprintf('Radar:        R1 (precision) only\n');
    fprintf('Total runs:   %d\n\n', 2 * n_scenarios * n_seeds);

    % --- Prepare detection data ONCE --------------------------------------
    fprintf('Preparing detection data...\n');
    prepared = cell(n_scenarios, 1);
    for s = 1:n_scenarios
        scen_name = specs{s}{1};
        lbl = specs{s}{2};
        rate = specs{s}{3};
        prepared{s} = prepare_oracle_tracking_inputs(...
            scen_name, ...
            struct('random_seed', seed_start, ...
                   'truth_turn_rate_deg_per_sec', rate));
        fprintf('  [%d/%d] %s\n', s, n_scenarios, lbl);
    end
    fprintf('Done.\n\n');

    % --- Storage ----------------------------------------------------------
    all_pos_rmse_3in1 = zeros(n_scenarios, n_seeds);
    all_pos_rmse_fuzzy = zeros(n_scenarios, n_seeds);

    % --- Run 3in1 (default) -----------------------------------------------
    fprintf('=== Running 3in1 mode (default) ===\n\n');
    for si = 1:n_scenarios
        inp = prepared{si};
        scenario_name = specs{si}{2};
        for seed_idx = 1:n_seeds
            seed_val = seed_start + seed_idx - 1;
            overrides_3in1 = struct('imm_adapt_mode', '3in1', 'random_seed', seed_val);
            pos_rmse = eval_single_r1(params, inp, 1, overrides_3in1);
            all_pos_rmse_3in1(si, seed_idx) = pos_rmse;
            if mod(seed_idx, 10) == 0 || seed_idx == 1
                fprintf('  [%s] seed=%d pos=%.3f km\n', scenario_name, seed_val, pos_rmse);
            end
        end
    end

    % --- Run fuzzy_only (default) -----------------------------------------
    fprintf('\n=== Running fuzzy_only mode (default) ===\n\n');
    for si = 1:n_scenarios
        inp = prepared{si};
        scenario_name = specs{si}{2};
        for seed_idx = 1:n_seeds
            seed_val = seed_start + seed_idx - 1;
            overrides_fuzzy = struct('imm_adapt_mode', 'fuzzy_only', 'random_seed', seed_val);
            pos_rmse = eval_single_r1(params, inp, 1, overrides_fuzzy);
            all_pos_rmse_fuzzy(si, seed_idx) = pos_rmse;
            if mod(seed_idx, 10) == 0 || seed_idx == 1
                fprintf('  [%s] seed=%d pos=%.3f km\n', scenario_name, seed_val, pos_rmse);
            end
        end
    end

    % --- Save results -----------------------------------------------------
    save(fullfile(root, 'fuzzy_3in1_baseline_results.mat'), ...
        'all_pos_rmse_3in1', 'all_pos_rmse_fuzzy', 'seed_start', 'n_seeds');
    fprintf('\nResults saved to %s/fuzzy_3in1_baseline_results.mat\n\n', root);

    % --- Print summary table ----------------------------------------------
    fprintf('============================================================\n');
    fprintf('SUMMARY TABLE: AVG RMSE (km) PER MODE x SCENARIO\n');
    fprintf('============================================================\n\n');

    fprintf('%-22s  %-12s  %-12s  %-8s\n', 'Scenario', '3in1', 'fuzzy_only', 'Delta%');
    fprintf('%-22s  %-12s  %-12s  %-8s\n', repmat('-', 1, 22), repmat('-', 1, 12), repmat('-', 1, 12), repmat('-', 1, 8));

    baseline_3in1 = zeros(n_scenarios, 1);
    for si = 1:n_scenarios
        m3in1 = mean(all_pos_rmse_3in1(si, :));
        mfuzz = mean(all_pos_rmse_fuzzy(si, :));
        baseline_3in1(si) = m3in1;
        delta = (mfuzz - m3in1) / m3in1 * 100;
        fprintf('%-22s  %-12.3f  %-12.3f  %+7.1f%%\n', ...
            specs{si}{2}, m3in1, mfuzz, delta);
    end

    mean_3in1 = mean(baseline_3in1);
    mean_fuzz = mean(mean(all_pos_rmse_fuzzy, 2));
    overall_delta = (mean_fuzz - mean_3in1) / mean_3in1 * 100;

    fprintf('\nMEAN                     %-12.3f  %-12.3f  %+7.1f%%\n\n', mean_3in1, mean_fuzz, overall_delta);

    if overall_delta > 5
        fprintf('*** WARNING: fuzzy_only default is %+.1f%% WORSE than 3in1 default.\n', overall_delta);
        fprintf('*** This suggests fuzzy_only may need significant tuning or is fundamentally worse.\n\n');
    elseif overall_delta < -5
        fprintf('*** INTERESTING: fuzzy_only default is %+.1f%% BETTER than 3in1 default.\n', -overall_delta);
        fprintf('*** fuzzy_only mode warrants dedicated search!\n\n');
    else
        fprintf('*** fuzzy_only default is within +/-5%% of 3in1 default. Worth searching.\n\n');
    end

    % Per-scenario best
    fprintf('Per-scenario winner:\n');
    win_3in1 = 0; win_fuzz = 0;
    for si = 1:n_scenarios
        if baseline_3in1(si) <= mean(all_pos_rmse_fuzzy(si, :))
            win_3in1 = win_3in1 + 1;
            fprintf('  %-22s: 3in1 (%.3f vs %.3f)\n', specs{si}{2}, baseline_3in1(si), mean(all_pos_rmse_fuzzy(si,:)));
        else
            win_fuzz = win_fuzz + 1;
            fprintf('  %-22s: fuzzy_only (%.3f vs %.3f)\n', specs{si}{2}, baseline_3in1(si), mean(all_pos_rmse_fuzzy(si,:)));
        end
    end
    fprintf('  3in1 wins: %d/%d, fuzzy_only wins: %d/%d\n\n', win_3in1, n_scenarios, win_fuzz, n_scenarios);
end


% =========================================================================
% Helper functions (copied from test_run_mc_best_params.m)
% =========================================================================

function rmse_km = eval_single_r1(params, inp, radar_id, overrides)
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
    [~, ~, snaps] = run_oracle_tracker_sequence(...
        det, tpl, prm, inp.truth_all, tg, false);

    pos_sq = [];
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
            pos_err = haversine_distance(...
                trk.ukf.x(1), trk.ukf.x(3), true_lon, true_lat);
            pos_sq(end+1) = pos_err^2;
        end
    end
    clear tpl det snaps;

    if isempty(pos_sq)
        rmse_km = NaN;
    else
        rmse_km = sqrt(mean(pos_sq)) / 1000;
    end
end


function dist = haversine_distance(lon1, lat1, lon2, lat2)
    dlat = deg2rad(lat2 - lat1);
    dlon = deg2rad(lon2 - lon1);
    a = sin(dlon/2)^2 + cosd(lat1)*cosd(lat2)*sin(dlat/2)^2;
    a = max(0, min(1, a));
    dist = 6371000 * 2 * atan2(sqrt(a), sqrt(1-a));
end
