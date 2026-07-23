function run_mc_best_params()
%RUN_MC_BEST_PARAMS 50-round Monte Carlo test for best IMM (3in1) configs.
%
% Usage:
%   run_mc_best_params  % runs full test (50 seeds x 4 configs x 10 scenarios)
%
% Compares: default vs 3 Phase-1 Top configs across ALL 10 validation scenarios.
% Uses radar 1 (precision) only for speed.

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

    % --- Configs to compare -----------------------------------------------
    cfg_default = struct();  % empty = use simulation_params_oracle defaults
    cfg_A = struct(...
        'imm_cv_dwell_time_sec', 3200, ...
        'imm_ct_dwell_time_sec', 1020, ...
        'imm_ct_fixed_Q_scale', 8.47, ...
        'imm_transient_gain_max', 4.66, ...
        'imm_transient_nis_start', 3.5, ...
        'imm_transient_nis_full', 11.5, ...
        'imm_transient_ewma_alpha', 0.80);
    cfg_B = struct(...
        'imm_cv_dwell_time_sec', 4000, ...
        'imm_ct_dwell_time_sec', 1200, ...
        'imm_ct_fixed_Q_scale', 4.71, ...
        'imm_transient_gain_max', 12.90, ...
        'imm_transient_nis_start', 5.0, ...
        'imm_transient_nis_full', 7.0, ...
        'imm_transient_ewma_alpha', 0.40);
    cfg_C = struct(...
        'imm_cv_dwell_time_sec', 1200, ...
        'imm_ct_dwell_time_sec', 1140, ...
        'imm_ct_fixed_Q_scale', 7.40, ...
        'imm_transient_gain_max', 7.94, ...
        'imm_transient_nis_start', 2.5, ...
        'imm_transient_nis_full', 16.0, ...
        'imm_transient_ewma_alpha', 0.40);

    % Store overrides in cell array instead of struct array (to avoid dimension mismatch)
    cfg_names = {'default'; '#10 (configA)'; '#16 (configB)'; '#29 (configC)'};
    cfg_overrides = {cfg_default; cfg_A; cfg_B; cfg_C};

    n_configs = numel(cfg_names);
    n_seeds = 50;
    seed_start = 10001;

    fprintf('============================================================\n');
    fprintf('Monte Carlo Full-Chain Test\n');
    fprintf('============================================================\n');
    fprintf('Seeds:        %d (%d-%d)\n', n_seeds, seed_start, seed_start+n_seeds-1);
    fprintf('Configs:      %d (default + 3 Phase-1 tops)\n', n_configs);
    fprintf('Scenarios:    %d (10 validation scenarios)\n', n_scenarios);
    fprintf('Radar:        R1 (precision) only\n');
    fprintf('Total runs:   %d\n\n', n_configs * n_scenarios * n_seeds);

    % --- Prepare detection data ONCE (cached) -----------------------------
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
    all_pos_rmse = zeros(n_configs, n_scenarios, n_seeds);

    % --- Main Monte Carlo loop --------------------------------------------
    for ci = 1:n_configs
        fprintf('=== Config #%d: %s ===\n', ci, cfg_names{ci});

        for si = 1:n_scenarios
            inp = prepared{si};
            scenario_name = specs{si}{2};

            for seed_idx = 1:n_seeds
                seed_val = seed_start + seed_idx - 1;

                % Apply overrides (merge with seed)
                overrides = cfg_overrides{ci};
                overrides.random_seed = seed_val;

                pos_rmse = eval_single_r1(params, inp, 1, overrides);
                all_pos_rmse(ci, si, seed_idx) = pos_rmse;

                if mod(seed_idx, 10) == 0 || seed_idx == 1
                    fprintf('  %s R1: seed=%d pos=%.3f km\n', ...
                        scenario_name, seed_val, pos_rmse);
                end
            end
        end
        fprintf('\n');
    end

    % --- Save results -----------------------------------------------------
    save(fullfile(root, 'mc_best_params_results.mat'), ...
        'all_pos_rmse', 'cfg_names', 'cfg_overrides', ...
        'seed_start', 'n_seeds');
    fprintf('Results saved to %s/mc_best_params_results.mat\n\n', root);

    % --- Print summary ----------------------------------------------------
    fprintf('============================================================\n');
    fprintf('SUMMARY TABLE: AVG RMSE (km) PER CONFIG x SCENARIO\n');
    fprintf('============================================================\n\n');

    % Header
    fprintf('%-14s  ', 'Config');
    for si = 1:n_scenarios
        lbl = specs{si}{2};
        if numel(lbl) > 12, lbl = lbl(1:12); end
        fprintf('%-12s', lbl);
    end
    fprintf('  MEAN\n');
    fprintf('%-14s  ', repmat('-', 1, 1));
    for si = 1:n_scenarios, fprintf('%-12s', repmat('-', 1, 12)); end
    fprintf('  ------\n');

    for ci = 1:n_configs
        fprintf('%-14s  ', cfg_names{ci});
        for si = 1:n_scenarios
            rmse_all = all_pos_rmse(ci, si, :);
            fprintf('%-12.3f', mean(rmse_all));
        end
        rmse_all_global = all_pos_rmse(ci, :, :);
        fprintf('  %-12.3f', mean(rmse_all_global));
        fprintf('\n');
    end

    % --- Best config ------------------------------------------------------
    fprintf('\n============================================================\n');
    fprintf('BEST CONFIGURATION\n');
    fprintf('============================================================\n\n');

    means_global = zeros(n_configs, 1);
    for ci = 1:n_configs
        means_global(ci) = mean(all_pos_rmse(ci, :, :));
    end
    [best_val, best_ci] = min(means_global);
    fprintf('Best overall: %s (avg RMSE = %.3f km)\n\n', cfg_names{best_ci}, best_val);

    % Per-scenario best
    fprintf('Best per scenario:\n');
    for si = 1:n_scenarios
        means_si = zeros(n_configs, 1);
        for ci = 1:n_configs
            means_si(ci) = mean(all_pos_rmse(ci, si, :));
        end
        [min_val, min_ci] = min(means_si);
        fprintf('  %-25s: %s (%.3f km)\n', ...
            specs{si}{2}, cfg_names{min_ci}, min_val);
    end
end


% =========================================================================
% Helper functions
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
    [tracks, ~, snaps] = run_oracle_tracker_sequence(...
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
    clear tracks tpl det snaps;

    rmse_km = sqrt(mean(pos_sq)) / 1000;
end


function dist = haversine_distance(lon1, lat1, lon2, lat2)
    dlat = deg2rad(lat2 - lat1);
    dlon = deg2rad(lon2 - lon1);
    a = sin(dlon/2)^2 + cosd(lat1)*cosd(lat2)*sin(dlat/2)^2;
    a = max(0, min(1, a));
    dist = 6371000 * 2 * atan2(sqrt(a), sqrt(1-a));
end
