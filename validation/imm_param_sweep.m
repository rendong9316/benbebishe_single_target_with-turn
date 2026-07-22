function imm_param_sweep(mode, mode_arg)
%IMM_PARAM_SWEEP Systematic parameter sweep for the IMM (3in1) UKF.
%
% Usage:
%   imm_param_sweep('scan')          % LHS search over 3 scenarios
%   imm_param_sweep('factorial', N)  % N-level factorial scan
%   imm_param_sweep('refine', C)     % refine top C configs
%   imm_param_sweep('report')        % show results table

    if nargin < 1 || isempty(mode)
        mode = 'scan';
    end
    if nargin < 2
        mode_arg = [];
    end

    root = fileparts(mfilename('fullpath'));
    params = simulation_params_oracle();
    seed = 42;

    baseline_scenarios = {'single_turn', 'single_uturn', 'single_straight'};

    switch mode
        case 'scan'
            run_lhs_search(root, params, baseline_scenarios, seed);
        case 'factorial'
            if isempty(mode_arg)
                N = 5;
            else
                N = mode_arg;
            end
            run_factorial_search(root, params, baseline_scenarios, seed, N);
        case 'refine'
            if isempty(mode_arg)
                C = 5;
            else
                C = mode_arg;
            end
            run_refinement_search(root, params, baseline_scenarios, seed, C);
        case 'report'
            run_report(root);
        otherwise
            error('imm_param_sweep:unknownMode', 'Unknown mode: %s', mode);
    end
end


% =========================================================================
% Phase 1: LHS Search
% =========================================================================

function run_lhs_search(root, params, scenarios, seed)
    fprintf('============================================================\n');
    fprintf('Phase 1: LHS Parameter Search\n');
    fprintf('============================================================\n\n');

    param_names = {'imm_cv_dwell_time_sec', 'imm_ct_dwell_time_sec', ...
                   'imm_ct_fixed_Q_scale', 'imm_transient_gain_max', ...
                   'imm_transient_nis_start', 'imm_transient_nis_full', ...
                   'imm_transient_ewma_alpha'};

    ranges = [900,      6000;
              90,       1200;
              2.0,      10.0;
              3.0,      15.0;
              1.0,      8.0;
              5.0,      30.0;
              0.3,      0.95];

    n_params = numel(param_names);
    n_candidates = 30;

    fprintf('Parameters: %d\nLHS candidates: %d\nScenarios: %d\n\n', ...
        n_params, n_candidates, numel(scenarios));

    rng(seed, 'twister');
    unit = zeros(n_candidates, n_params);
    for col = 1:n_params
        perm = randperm(n_candidates);
        unit(:, col) = (perm' - rand(n_candidates, 1)) / n_candidates;
    end

    candidates = cell(n_candidates, 1);
    for i = 1:n_candidates
        c = struct();
        for p = 1:n_params
            name = param_names{p};
            lo = ranges(p, 1); hi = ranges(p, 2);
            v = lo + unit(i, p) * (hi - lo);
            if strcmp(name, 'imm_cv_dwell_time_sec')
                v = max(round(v / 100) * 100, 900);
            elseif strcmp(name, 'imm_ct_dwell_time_sec')
                v = max(round(v / 30) * 30, 90);
            elseif strcmp(name, 'imm_transient_nis_start') || ...
                 strcmp(name, 'imm_transient_nis_full')
                v = round(v * 2) / 2;
            elseif strcmp(name, 'imm_transient_ewma_alpha')
                v = round(v * 5) / 5;
            end
            c.(name) = v;
        end
        candidates{i} = c;
    end

    candidates{1}.imm_cv_dwell_time_sec = params.imm_cv_dwell_time_sec;
    candidates{1}.imm_ct_dwell_time_sec = params.imm_ct_dwell_time_sec;
    candidates{1}.imm_ct_fixed_Q_scale = params.imm_ct_fixed_Q_scale;
    candidates{1}.imm_transient_gain_max = params.imm_transient_gain_max;
    candidates{1}.imm_transient_nis_start = params.imm_transient_nis_start;
    candidates{1}.imm_transient_nis_full = params.imm_transient_nis_full;
    candidates{1}.imm_transient_ewma_alpha = params.imm_transient_ewma_alpha;

    prepared = {};
    for s = 1:numel(scenarios)
        prepared{s} = prepare_oracle_tracking_inputs(scenarios{s}, ...
            struct('random_seed', seed));
    end

    n_evals = numel(prepared) * 2;
    pos_rmse_matrix = zeros(n_evals, n_candidates);
    avg_rmse_all = zeros(1, n_candidates);
    best_rmse_all = zeros(1, n_candidates);

    fprintf('Evaluating %d candidates...\n\n', n_candidates);

    for i = 1:n_candidates
        c = candidates{i};
        fprintf('Candidate %d/%d:\n', i, n_candidates);
        fprintf('  cv_dwell=%g ct_dwell=%g ct_q=%.1f gain=%.1f nis_s=%.1f nis_f=%.1f alpha=%.2f\n', ...
            c.imm_cv_dwell_time_sec, c.imm_ct_dwell_time_sec, ...
            c.imm_ct_fixed_Q_scale, c.imm_transient_gain_max, ...
            c.imm_transient_nis_start, c.imm_transient_nis_full, ...
            c.imm_transient_ewma_alpha);

        idx_col = 0;
        for si = 1:numel(prepared)
            inp = prepared{si};
            for r = 1:2
                [pos_sq, ~, ~] = eval_tracker_local(params, inp, r, c);
                pos_rmse_km = sqrt(mean(pos_sq)) / 1000;
                idx_col = idx_col + 1;
                pos_rmse_matrix(idx_col, i) = pos_rmse_km;
                fprintf('    %-20s R%d: %.3f km\n', scenarios{si}, r, pos_rmse_km);
                clear pos_sq;
            end
        end

        avg_rmse_all(i) = mean(pos_rmse_matrix(:, i));
        best_rmse_all(i) = min(pos_rmse_matrix(:, i));

        fprintf('  AVG RMSE: %.3f km | Best scenario RMSE: %.3f km\n\n', ...
            avg_rmse_all(i), best_rmse_all(i));
    end

    fprintf('\n============================================================\n');
    fprintf('TOP 10 CONFIGURATIONS BY AVERAGE RMSE\n');
    fprintf('============================================================\n\n');

    [~, order] = sort(avg_rmse_all);

    fprintf('%-6s  %-10s  %-10s  %-12s\n', ...
        'Rank', 'avg_rmse', 'best_rmse', 'config_idx');
    fprintf('%s\n', repmat('-', 1, 42));
    for k = 1:min(10, numel(order))
        idx = order(k);
        fprintf('%-6d  %-10.3f  %-10.3f  %-12d\n', ...
            k, avg_rmse_all(idx), best_rmse_all(idx), idx);
    end

    fprintf('\n\nTOP 5 DETAILS:\n');
    for k = 1:min(5, numel(order))
        idx = order(k);
        fprintf('\n  #%d: avg=%.3f km, best=%.3f km\n', k, avg_rmse_all(idx), best_rmse_all(idx));
        for col = 1:numel(pos_rmse_matrix(:, idx))
            fprintf('    scenario_%d x radar_%d: %.3f km\n', ...
                mod(col-1, numel(scenarios))+1, ceil(col/numel(scenarios)), pos_rmse_matrix(col, idx));
        end
        fprintf('    Params:\n');
        fnames = fieldnames(candidates{idx});
        for fi = 1:numel(fnames)
            fprintf('      %-30s = %.2f\n', fnames{fi}, candidates{idx}.(fnames{fi}));
        end
    end

    % Save all results (cell arrays must be saved separately from numeric arrays)
    save(fullfile(root, 'imm_sweep_results.mat'), ...
        'param_names', 'ranges', 'avg_rmse_all', 'best_rmse_all', 'pos_rmse_matrix');
    save(fullfile(root, 'imm_sweep_candidates.mat'), 'candidates', 'scenarios');
    fprintf('\nResults saved to %s/imm_sweep_results.mat and %s/imm_sweep_candidates.mat\n', root, root);
end


% =========================================================================
% Phase 2: Two-Factor Factorial Scan
% =========================================================================

function run_factorial_search(root, params, scenarios, seed, N)
    fprintf('============================================================\n');
    fprintf('Phase 2: Two-Factor Factorial Scan (N=%d levels)\n', N);
    fprintf('============================================================\n\n');

    q_levels = linspace(2, 10, N);
    gain_levels = linspace(3, 15, N);

    fprintf('Factor 1 (ct_q): '); disp(q_levels);
    fprintf('Factor 2 (gain): '); disp(gain_levels);

    prepared = {};
    for s = 1:numel(scenarios)
        prepared{s} = prepare_oracle_tracking_inputs(scenarios{s}, ...
            struct('random_seed', seed));
    end

    n_combos = numel(q_levels) * numel(gain_levels);
    n_evals = numel(prepared) * 2;
    avg_rmse_matrix = zeros(n_evals, n_combos);

    fprintf('Evaluating %d combos x %d scenarios x 2 radars\n\n', ...
        n_combos, numel(scenarios));

    for qi = 1:numel(q_levels)
        for gi = 1:numel(gain_levels)
            ci = (qi - 1) * numel(gain_levels) + gi;
            fprintf('[%d/%d] ct_q=%.1f gain=%.1f\n', ci, n_combos, q_levels(qi), gain_levels(gi));

            idx_col = 0;
            for si = 1:numel(prepared)
                inp = prepared{si};
                for r = 1:2
                    overrides = struct(...
                        'imm_ct_fixed_Q_scale', q_levels(qi), ...
                        'imm_transient_gain_max', gain_levels(gi));
                    [pos_sq, ~, ~] = eval_tracker_local(params, inp, r, overrides);
                    pos_rmse_km = sqrt(mean(pos_sq)) / 1000;
                    idx_col = idx_col + 1;
                    avg_rmse_matrix(idx_col, ci) = pos_rmse_km;
                    fprintf('    %-20s R%d: %.3f km\n', scenarios{si}, r, pos_rmse_km);
                    clear pos_sq;
                end
            end
            fprintf('  AVG=%.3f MIN=%.3f\n\n', ...
                mean(avg_rmse_matrix(:, ci)), min(avg_rmse_matrix(:, ci)));
        end
    end

    fprintf('\n============================================================\n');
    fprintf('TOP 10 FACTORIAL COMBINATIONS\n');
    fprintf('============================================================\n\n');
    fprintf('%-6s  %-10s  %-10s  %-10s  %-10s\n', 'Rank', 'ct_q', 'gain', 'avg_rmse', 'min_rmse');
    fprintf('%s\n', repmat('-', 1, 48));

    avg_cols = mean(avg_rmse_matrix, 1);
    min_cols = min(avg_rmse_matrix, [], 1);
    [~, order] = sort(avg_cols);
    for k = 1:min(10, numel(order))
        idx = order(k);
        qi_val = ceil((idx) / numel(gain_levels));
        gi_val = mod(idx - 1, numel(gain_levels)) + 1;
        fprintf('%-6d  %-10.1f  %-10.1f  %-10.3f  %-10.3f\n', ...
            k, q_levels(qi_val), gain_levels(gi_val), avg_cols(idx), min_cols(idx));
    end

    save(fullfile(root, 'imm_sweep_phase2.mat'), ...
        'avg_rmse_matrix', 'q_levels', 'gain_levels', 'scenarios');
end


% =========================================================================
% Phase 3: Refinement Search
% =========================================================================

function run_refinement_search(root, params, scenarios, seed, C)
    fprintf('============================================================\n');
    fprintf('Phase 3: Refinement Search (Top %d configs)\n', C);
    fprintf('============================================================\n\n');

    mat_path = fullfile(root, 'imm_sweep_results.mat');
    if ~exist(mat_path, 'file')
        error('imm_param_sweep:phase1Missing', 'Run phase 1 first.');
    end

    loaded = load(mat_path);
    avg_rmse_all = loaded.avg_rmse_all;
    candidates = load(fullfile(root, 'imm_sweep_candidates.mat')).candidates;
    [~, order] = sort(avg_rmse_all);
    top_indices = order(1:C);

    fprintf('Refining %d best configs from Phase 1...\n\n', C);

    for k = 1:C
        idx = top_indices(k);
        base = candidates{idx};
        fprintf('Base config #%d (%.3f km):\n', idx, avg_rmse_all(idx));
        fnames = fieldnames(base);
        for fi = 1:numel(fnames)
            fprintf('  %-30s = %.2f\n', fnames{fi}, base.(fnames{fi}));
        end

        best_rmse = avg_rmse_all(idx);
        best_config = base;

        steps = [-0.20, -0.10, 0.10, 0.20];
        for p = 1:numel(fnames)
            fname = fnames{p};
            cur = base.(fname);
            for st = 1:numel(steps)
                trial = base;
                new_val = max(1e-6, cur * (1 + steps(st)));
                trial.(fname) = new_val;
                score = eval_trial_quick(params, scenarios, seed, trial);
                if score < best_rmse
                    best_rmse = score;
                    best_config = trial;
                    fprintf('  IMPROVED: %s=%.2f -> %.3f km\n', fname, cur, best_rmse);
                end
            end
        end
        fprintf('  BEST: %.3f km\n\n', best_rmse);
    end
end


% =========================================================================
% Reporting
% =========================================================================

function run_report(root)
    fprintf('Loading saved results...\n');
    mat_path = fullfile(root, 'imm_sweep_results.mat');
    if ~exist(mat_path, 'file')
        fprintf('No results found.\n');
        return;
    end

    loaded = load(mat_path);
    avg_rmse_all = loaded.avg_rmse_all;
    candidates = load(fullfile(root, 'imm_sweep_candidates.mat')).candidates;

    fprintf('\n=== TOP 20 BY AVERAGE RMSE ===\n\n');
    [~, order] = sort(avg_rmse_all);
    for k = 1:min(20, numel(order))
        idx = order(k);
        c = candidates{idx};
        fprintf('%-4d  ', idx);
        fprintf('cv=%d ct=%d q=%.1f g=%.1f ns=%.1f nf=%.1f a=%.2f  ', ...
            c.imm_cv_dwell_time_sec, c.imm_ct_dwell_time_sec, ...
            c.imm_ct_fixed_Q_scale, c.imm_transient_gain_max, ...
            c.imm_transient_nis_start, c.imm_transient_nis_full, ...
            c.imm_transient_ewma_alpha);
        fprintf('AVG=%.3f\n', avg_rmse_all(idx));
    end
end


% =========================================================================
% Evaluation helpers
% =========================================================================

function [pos_sq, speed_sq, nis_arr] = eval_tracker_local(params, inp, radar_id, overrides)
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
    [tracks, ~, snaps] = run_oracle_tracker_sequence( ...
        det, tpl, prm, inp.truth_all, tg, false);

    pos_sq = [];
    speed_sq = [];
    nis_arr = [];

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
            true_lon_rate = interp1(truth.time_sec, truth.lon_rate, t_now, 'linear');
            true_lat_rate = interp1(truth.time_sec, truth.lat_rate, t_now, 'linear');
            if ~all(isfinite([true_lon, true_lat, true_lon_rate, true_lat_rate]))
                continue;
            end

            pos_err = haversine_distance(trk.ukf.x(1), trk.ukf.x(3), true_lon, true_lat);
            pos_sq(end+1) = pos_err^2;

            est_spd = hypot(...
                trk.ukf.x(2)*6371000*pi/180*cosd(trk.ukf.x(3)), ...
                trk.ukf.x(4)*6371000*pi/180);
            tru_spd = hypot(...
                true_lon_rate*6371000*pi/180*cosd(true_lat), ...
                true_lat_rate*6371000*pi/180);
            speed_sq(end+1) = (est_spd - tru_spd)^2;
            nis_arr(end+1) = trk.combined_nis;
        end
    end

    clear tracks tpl det snaps;
end


function score = eval_trial_quick(params, scenarios, seed, trial)
    prepared = {};
    for s = 1:numel(scenarios)
        prepared{s} = prepare_oracle_tracking_inputs(scenarios{s}, ...
            struct('random_seed', seed));
    end

    total_pos_sq = [];
    for si = 1:numel(prepared)
        inp = prepared{si};
        for r = 1:2
            [pos_sq_r, ~, ~] = eval_tracker_local(params, inp, r, trial);
            total_pos_sq = [total_pos_sq; pos_sq_r(:)];
        end
    end

    score = sqrt(mean(total_pos_sq)) / 1000;
    clear prepared;
end


function dist = haversine_distance(lon1, lat1, lon2, lat2)
    dlat = deg2rad(lat2 - lat1);
    dlon = deg2rad(lon2 - lon1);
    a = sin(dlon/2)^2 + cosd(lat1)*cosd(lat2)*sin(dlat/2)^2;
    a = max(0, min(1, a));
    dist = 6371000 * 2 * atan2(sqrt(a), sqrt(1-a));
end
