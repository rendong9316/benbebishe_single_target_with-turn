function imm_param_search_v2(phase, phase_arg)
%IMM_PARAM_SEARCH_V2 Systematic parameter search for IMM UKF with protection constraints.
%
% Usage:
%   imm_param_search_v2('baseline')    % Compute default baselines for all 10 scenarios (50 seeds)
%   imm_param_search_v2('lhs')         % Phase 1: LHS exploration (80 candidates per track)
%   imm_param_search_v2('factorial')   % Phase 2: Factorial scan around top configs
%   imm_param_search_v2('refine')      % Phase 3: Local refinement + final validation
%   imm_param_search_v2('report')      % Show final results
%
% Two tracks: Track A (3in1 mode) and Track B (fuzzy_only mode).
% Protection constraint: no scenario can degrade >5% vs default baseline.
% Cliff penalty for violations.

    if nargin < 1 || isempty(phase)
        phase = 'baseline';
    end
    if nargin < 2
        phase_arg = [];
    end

    root = fileparts(mfilename('fullpath'));
    matlab_bin = '"C:\Program Files\MATLAB\R2023a\bin\matlab"';
    params = simulation_params_oracle();

    switch phase
        case 'baseline'
            compute_baseline(root, params);
        case 'lhs'
            run_lhs(root, params, phase_arg);
        case 'factorial'
            run_factorial(root, params, phase_arg);
        case 'refine'
            run_refine(root, params);
        case 'report'
            run_report(root);
        otherwise
            error('Unknown phase: %s', phase);
    end
end


% =========================================================================
% BASELINE: Compute default config RMSE for all 10 scenarios (50 seeds)
% =========================================================================

function compute_baseline(root, params)
    fprintf('============================================================\n');
    fprintf('BASELINE: Computing default RMSE for all 10 scenarios (50 seeds)\n');
    fprintf('============================================================\n\n');

    specs = get_all_scenarios();
    n_scenarios = numel(specs);
    n_seeds = 50;
    seed_start = 10001;

    fprintf('Preparing detection data...\n');
    prepared = cell(n_scenarios, 1);
    for s = 1:n_scenarios
        prepared{s} = prepare_oracle_tracking_inputs(specs{s}{1}, ...
            struct('random_seed', seed_start));
        fprintf('  [%d/%d] %s\n', s, n_scenarios, specs{s}{2});
    end
    fprintf('Done.\n\n');

    baseline_rmse = zeros(n_scenarios, 2); % rows=scenarios, cols=R1,R2

    % Test both 3in1 and fuzzy_only default modes
    for mode_idx = 1:2
        if mode_idx == 1
            mode_name = '3in1';
            adapt_mode = '3in1';
        else
            mode_name = 'fuzzy_only';
            adapt_mode = 'fuzzy_only';
        end

        fprintf('=== Mode: %s (default params) ===\n', mode_name);

        for si = 1:n_scenarios
            inp = prepared{si};
            scenario_name = specs{si}{2};

            for radar_id = 1:2
                seed_count = 0;
                total_pos_sq = [];

                for seed_idx = 1:n_seeds
                    seed_val = seed_start + seed_idx - 1;
                    overrides = struct('imm_adapt_mode', adapt_mode, 'random_seed', seed_val);
                    pos_sq = eval_scenario_r1_r2(params, inp, radar_id, overrides, seed_start, n_seeds);
                    total_pos_sq = [total_pos_sq; pos_sq];
                end

                rmse_km = sqrt(mean(total_pos_sq)) / 1000;
                baseline_rmse(si, radar_id) = rmse_km;

                if radar_id == 1
                    fprintf('  [%s] R1 avg RMSE: %.3f km\n', scenario_name, rmse_km);
                else
                    fprintf('  [%s] R2 avg RMSE: %.3f km\n', scenario_name, rmse_km);
                end
                clear total_pos_sq;
            end
        end
        fprintf('\n');
    end

    % Save baseline
    save(fullfile(root, 'baseline_results.mat'), 'baseline_rmse', 'specs');
    fprintf('Baseline saved to %s/baseline_results.mat\n', root);

    % Print summary table
    fprintf('\n=== BASELINE SUMMARY (avg R1+R2) ===\n');
    fprintf('%-22s  %-12s  %-12s  %-12s\n', 'Scenario', '3in1-R1', '3in1-R2', 'Avg');
    fprintf('%s\n', repmat('-', 1, 62));
    for si = 1:n_scenarios
        avg_3in1 = mean(baseline_rmse(si, :));
        fprintf('%-22s  %-12.3f  %-12.3f  %-12.3f\n', ...
            specs{si}{2}, baseline_rmse(si,1), baseline_rmse(si,2), avg_3in1);
    end
    overall_avg = mean(baseline_rmse(:,1));
    fprintf('%-22s  %-12.3f\n\n', 'OVERALL (R1)', overall_avg);
end


% =========================================================================
% PHASE 1: LHS Exploration (80 candidates per track)
% =========================================================================

function run_lhs(root, params, arg)
    fprintf('============================================================\n');
    fprintf('PHASE 1: LHS Exploration\n');
    fprintf('============================================================\n\n');

    % Load baseline
    if ~exist(fullfile(root, 'baseline_results.mat'), 'file')
        error('Run baseline first: imm_param_search_v2("baseline")');
    end
    bl = load(fullfile(root, 'baseline_results.mat'));
    baseline_rmse = bl.baseline_rmse; % [10 scenarios, 2 radars]

    specs = get_all_scenarios();
    n_scenarios = numel(specs);
    n_seeds = 10; % Fast screening with 10 seeds
    seed_start = 10001;

    fprintf('Preparing detection data...\n');
    prepared = cell(n_scenarios, 1);
    for s = 1:n_scenarios
        prepared{s} = prepare_oracle_tracking_inputs(specs{s}{1}, ...
            struct('random_seed', seed_start));
    end
    fprintf('Done.\n\n');

    % --- Track A: 3in1 mode (8 parameters) ---
    fprintf('=== Track A: 3in1 mode ---\n');
    track_a_params = { ...
        'imm_cv_dwell_time_sec', ...
        'imm_ct_dwell_time_sec', ...
        'imm_ct_fixed_Q_scale', ...
        'imm_transient_gain_max', ...
        'imm_transient_nis_start', ...
        'imm_transient_nis_full', ...
        'imm_transient_ewma_alpha', ...
        'imm_mu_init_CV'};
    track_a_lo = [600, 120, 2.0, 2.0, 1.0, 4.0, 0.20, 0.30];
    track_a_hi = [6000, 1800, 6.0, 12.0, 6.0, 20.0, 0.90, 0.80];
    track_a_round = [100, 30, 0.1, 0.5, 0.5, 1.0, 0.05, 0.05];

    n_params_a = numel(track_a_params);
    n_candidates_a = 80;

    lhs_a = generate_lhs(n_candidates_a, n_params_a, 42);
    candidates_a = cell(n_candidates_a, 1);
    for i = 1:n_candidates_a
        c = struct();
        for p = 1:n_params_a
            name = track_a_params{p};
            lo = track_a_lo(p); hi = track_a_hi(p);
            v = lo + lhs_a(i, p) * (hi - lo);
            v = max(lo, min(hi, v));
            v = v + track_a_round(p) * 0.5; % rounding bias
            v = round(v / track_a_round(p)) * track_a_round(p);
            v = max(lo, min(hi, v));
            c.(name) = v;
        end
        % Force first candidate to be default
        if i == 1
            c.imm_cv_dwell_time_sec = params.imm_cv_dwell_time_sec;
            c.imm_ct_dwell_time_sec = params.imm_ct_dwell_time_sec;
            c.imm_ct_fixed_Q_scale = params.imm_ct_fixed_Q_scale;
            c.imm_transient_gain_max = params.imm_transient_gain_max;
            c.imm_transient_nis_start = params.imm_transient_nis_start;
            c.imm_transient_nis_full = params.imm_transient_nis_full;
            c.imm_transient_ewma_alpha = params.imm_transient_ewma_alpha;
            c.imm_mu_init_CV = params.imm_mu_init_CV;
        end
        candidates_a{i} = c;
    end

    % Evaluate Track A
    scores_a = evaluate_candidates(root, params, specs, prepared, ...
        candidates_a, '3in1', baseline_rmse, n_seeds, n_candidates_a, ...
        track_a_params, track_a_lo, track_a_hi, 'A');

    % --- Track B: fuzzy_only mode (7 parameters) ---
    fprintf('\n=== Track B: fuzzy_only mode ---\n');
    track_b_params = { ...
        'fuzzy_window_size', ...
        'fuzzy_ema_eta', ...
        'adaptive_Q_min', ...
        'adaptive_Q_max', ...
        'imm_cv_dwell_time_sec', ...
        'imm_ct_dwell_time_sec', ...
        'imm_mu_init_CV'};
    track_b_lo = [1, 0.02, 0.2, 2.0, 600, 120, 0.30];
    track_b_hi = [7, 0.50, 0.8, 6.0, 6000, 1800, 0.80];
    track_b_round = [1, 0.01, 0.05, 0.1, 100, 30, 0.05];

    n_params_b = numel(track_b_params);
    n_candidates_b = 80;

    lhs_b = generate_lhs(n_candidates_b, n_params_b, 123);
    candidates_b = cell(n_candidates_b, 1);
    for i = 1:n_candidates_b
        c = struct();
        for p = 1:n_params_b
            name = track_b_params{p};
            lo = track_b_lo(p); hi = track_b_hi(p);
            v = lo + lhs_b(i, p) * (hi - lo);
            v = max(lo, min(hi, v));
            v = round(v / track_b_round(p)) * track_b_round(p);
            v = max(lo, min(hi, v));
            c.(name) = v;
        end
        % Force first candidate to be default
        if i == 1
            c.fuzzy_window_size = params.fuzzy_window_size;
            c.fuzzy_ema_eta = params.fuzzy_ema_eta;
            c.adaptive_Q_min = params.adaptive_Q_min;
            c.adaptive_Q_max = params.adaptive_Q_max;
            c.imm_cv_dwell_time_sec = params.imm_cv_dwell_time_sec;
            c.imm_ct_dwell_time_sec = params.imm_ct_dwell_time_sec;
            c.imm_mu_init_CV = params.imm_mu_init_CV;
        end
        candidates_b{i} = c;
    end

    % Evaluate Track B
    scores_b = evaluate_candidates(root, params, specs, prepared, ...
        candidates_b, 'fuzzy_only', baseline_rmse, n_seeds, n_candidates_b, ...
        track_b_params, track_b_lo, track_b_hi, 'B');

    % --- Select top 15 from each track ---
    fprintf('\n=== TOP 15 FROM EACH TRACK ===\n\n');

    % Sort by score (lower is better)
    [sorted_scores_a, order_a] = sort(scores_a.score);
    [sorted_scores_b, order_b] = sort(scores_b.score);

    fprintf('Track A (3in1) Top 15:\n');
    fprintf('%-6s  %-10s  %-10s  %-10s  %-8s\n', 'Rank', 'Score', 'Protected?', 'Idx', 'Passed');
    fprintf('%s\n', repmat('-', 1, 48));
    for k = 1:min(15, numel(order_a))
        idx = order_a(k);
        c = candidates_a{idx};
        passed = 'YES';
        if ~scores_a.protect(idx)
            passed = 'NO';
        end
        fprintf('%-6d  %-10.3f  %-10s  %-10d  %-8s\n', ...
            k, sorted_scores_a(k), ...
            num2str(scores_a.protect(idx)), idx, passed);
        if k <= 15
            fnames = fieldnames(c);
            for fi = 1:min(3, numel(fnames))
                fprintf('  %-30s = %.3f\n', fnames{fi}, c.(fnames{fi}));
            end
        end
    end

    fprintf('\nTrack B (fuzzy_only) Top 15:\n');
    fprintf('%-6s  %-10s  %-10s  %-10s  %-8s\n', 'Rank', 'Score', 'Protected?', 'Idx', 'Passed');
    fprintf('%s\n', repmat('-', 1, 48));
    for k = 1:min(15, numel(order_b))
        idx = order_b(k);
        c = candidates_b{idx};
        fprintf('%-6d  %-10.3f  %-10s  %-10d  %-8s\n', ...
            k, sorted_scores_b(k), ...
            num2str(scores_b.protect(idx)), idx, ...
            num2str(scores_b.protect(idx)));
        if k <= 15
            fnames = fieldnames(c);
            for fi = 1:min(3, numel(fnames))
                fprintf('  %-30s = %.3f\n', fnames{fi}, c.(fnames{fi}));
            end
        end
    end

    % Save Phase 1 results
    save(fullfile(root, 'lhs_results_trackA.mat'), ...
        'candidates_a', 'scores_a', 'track_a_params', 'baseline_rmse');
    save(fullfile(root, 'lhs_results_trackB.mat'), ...
        'candidates_b', 'scores_b', 'track_b_params', 'baseline_rmse');

    % Determine top indices for Phase 2
    top_a = order_a(1:min(15, numel(order_a)));
    top_b = order_b(1:min(15, numel(order_b)));
    save(fullfile(root, 'lhs_top_indices.mat'), 'top_a', 'top_b', 'n_candidates_a', 'n_candidates_b');

    fprintf('\nPhase 1 complete. Saved to %s/lhs_results_track*.mat\n', root);
    fprintf('Run imm_param_search_v2("factorial") to proceed to Phase 2.\n');
end


% =========================================================================
% PHASE 2: Factorial Scan
% =========================================================================

function run_factorial(root, params, arg)
    fprintf('============================================================\n');
    fprintf('PHASE 2: Factorial Scan\n');
    fprintf('============================================================\n\n');

    if ~exist(fullfile(root, 'lhs_results_trackA.mat'), 'file')
        error('Run Phase 1 first: imm_param_search_v2("lhs")');
    end

    bl = load(fullfile(root, 'baseline_results.mat'));
    baseline_rmse = bl.baseline_rmse;

    specs = get_all_scenarios();
    n_seeds = 20; % Medium fidelity for factorial
    seed_start = 10001;

    fprintf('Preparing detection data...\n');
    prepared = cell(numel(specs), 1);
    for s = 1:numel(specs)
        prepared{s} = prepare_oracle_tracking_inputs(specs{s}{1}, ...
            struct('random_seed', seed_start));
    end
    fprintf('Done.\n\n');

    % Load top 5 from each track (pass N via phase_arg if specified, default 5)
    if isempty(arg), arg = 5; end
    n_top = arg;

    lhs_a_data = load(fullfile(root, 'lhs_results_trackA.mat'));
    lhs_b_data = load(fullfile(root, 'lhs_results_trackB.mat'));
    lhs_top_idx = load(fullfile(root, 'lhs_top_indices.mat'));

    top_a = lhs_top_idx.top_a(1:n_top);
    top_b = lhs_top_idx.top_b(1:n_top);

    candidates_a = lhs_a_data.candidates_a;
    candidates_b = lhs_b_data.candidates_b;
    params_a = lhs_a_data.track_a_params;
    params_b = lhs_b_data.track_b_params;

    scores_a = lhs_a_data.scores_a;
    scores_b = lhs_b_data.scores_b;

    % For each top candidate, find top 2 correlated parameters and do 5x5 factorial
    fprintf('=== Track A: Factorial scan around top %d configs ===\n', n_top);
    all_factorial_a = {};
    for ci = 1:n_top
        idx = top_a(ci);
        base = candidates_a{idx};
        fprintf('  Config #%d (score=%.3f):\n', idx, scores_a.score(idx));

        fnames = fieldnames(base);
        % Simple heuristic: pick the 2 params with largest range span as "most impactful"
        % For now, use a fixed pair that's most likely to matter
        if ci == 1
            var_params = {'imm_cv_dwell_time_sec', 'imm_ct_fixed_Q_scale'};
        elseif ci == 2
            var_params = {'imm_cv_dwell_time_sec', 'imm_transient_gain_max'};
        else
            var_params = {'imm_ct_fixed_Q_scale', 'imm_transient_gain_max'};
        end

        vals_1 = linspace(base.(var_params{1}) * 0.8, base.(var_params{1}) * 1.2, 5);
        vals_2 = linspace(base.(var_params{2}) * 0.8, base.(var_params{2}) * 1.2, 5);

        factorial_score = inf;
        factorial_best = base;
        n_factorial = 0;

        for v1i = 1:length(vals_1)
            for v2i = 1:length(vals_2)
                n_factorial = n_factorial + 1;
                trial = base;
                trial.(var_params{1}) = vals_1(v1i);
                trial.(var_params{2}) = vals_2(v2i);

                score = eval_candidate_protected(params, specs, prepared, ...
                    trial, '3in1', baseline_rmse, n_seeds, seed_start);

                if score < factorial_score
                    factorial_score = score;
                    factorial_best = trial;
                end

                if mod(n_factorial, 5) == 0
                    fprintf('    [%d/%d] score=%.3f\n', n_factorial, length(vals_1)*length(vals_2), score);
                end
            end
        end

        fprintf('  Best: score=%.3f, cv_dwell=%.0f, ct_q=%.1f, gain=%.1f\n\n', ...
            factorial_score, ...
            factorial_best.imm_cv_dwell_time_sec, ...
            factorial_best.imm_ct_fixed_Q_scale, ...
            factorial_best.imm_transient_gain_max);

        all_factorial_a{ci} = factorial_best;
    end

    fprintf('\n=== Track B: Factorial scan around top %d configs ===\n', n_top);
    all_factorial_b = {};
    for ci = 1:n_top
        idx = top_b(ci);
        base = candidates_b{idx};
        fprintf('  Config #%d (score=%.3f):\n', idx, scores_b.score(idx));

        if ci == 1
            var_params = {'fuzzy_ema_eta', 'adaptive_Q_max'};
        else
            var_params = {'fuzzy_window_size', 'fuzzy_ema_eta'};
        end

        vals_1 = linspace(base.(var_params{1}) * 0.7, base.(var_params{1}) * 1.3, 5);
        vals_2 = linspace(base.(var_params{2}) * 0.7, base.(var_params{2}) * 1.3, 5);

        factorial_score = inf;
        factorial_best = base;
        n_factorial = 0;

        for v1i = 1:length(vals_1)
            for v2i = 1:length(vals_2)
                n_factorial = n_factorial + 1;
                trial = base;
                trial.(var_params{1}) = vals_1(v1i);
                trial.(var_params{2}) = vals_2(v2i);

                score = eval_candidate_protected(params, specs, prepared, ...
                    trial, 'fuzzy_only', baseline_rmse, n_seeds, seed_start);

                if score < factorial_score
                    factorial_score = score;
                    factorial_best = trial;
                end

                if mod(n_factorial, 5) == 0
                    fprintf('    [%d/%d] score=%.3f\n', n_factorial, length(vals_1)*length(vals_2), score);
                end
            end
        end

        fprintf('  Best: score=%.3f\n\n', factorial_score);
        all_factorial_b{ci} = factorial_best;
    end

    % Save Phase 2 results
    save(fullfile(root, 'factorial_results.mat'), ...
        'all_factorial_a', 'all_factorial_b', 'baseline_rmse', 'specs');
    fprintf('\nPhase 2 complete. Saved to %s/factorial_results.mat\n', root);
    fprintf('Run imm_param_search_v2("refine") to proceed to Phase 3.\n');
end


% =========================================================================
% PHASE 3: Refinement + Final Validation
% =========================================================================

function run_refine(root, params)
    fprintf('============================================================\n');
    fprintf('PHASE 3: Refinement + Final Validation (50 seeds)\n');
    fprintf('============================================================\n\n');

    if ~exist(fullfile(root, 'factorial_results.mat'), 'file')
        error('Run Phase 2 first: imm_param_search_v2("factorial")');
    end

    bl = load(fullfile(root, 'baseline_results.mat'));
    baseline_rmse = bl.baseline_rmse;

    specs = get_all_scenarios();
    n_seeds = 50; % Full fidelity for final validation
    seed_start = 10001;

    fprintf('Preparing detection data...\n');
    prepared = cell(numel(specs), 1);
    for s = 1:numel(specs)
        prepared{s} = prepare_oracle_tracking_inputs(specs{s}{1}, ...
            struct('random_seed', seed_start));
    end
    fprintf('Done.\n\n');

    fac = load(fullfile(root, 'factorial_results.mat'));
    all_a = fac.all_factorial_a;
    all_b = fac.all_factorial_b;

    % Evaluate all finalists with 50 seeds
    fprintf('=== Final Validation: Track A (3in1) ===\n');
    scores_a = zeros(length(all_a), 1);
    for ci = 1:length(all_a)
        fprintf('  Config #%d...\n', ci);
        scores_a(ci) = eval_candidate_protected(params, specs, prepared, ...
            all_a{ci}, '3in1', baseline_rmse, n_seeds, seed_start);
        fprintf('    Score: %.3f\n', scores_a(ci));
    end

    fprintf('\n=== Final Validation: Track B (fuzzy_only) ===\n');
    scores_b = zeros(length(all_b), 1);
    for ci = 1:length(all_b)
        fprintf('  Config #%d...\n', ci);
        scores_b(ci) = eval_candidate_protected(params, specs, prepared, ...
            all_b{ci}, 'fuzzy_only', baseline_rmse, n_seeds, seed_start);
        fprintf('    Score: %.3f\n', scores_b(ci));
    end

    % Combine and rank
    all_scores = [scores_a; scores_b];
    all_configs = [all_a; all_b];
    all_modes = cell(length(all_scores), 1);
    for i = 1:length(all_a), all_modes{i} = '3in1'; end
    for i = 1:length(all_b), all_modes{length(all_a)+i} = 'fuzzy_only'; end

    [sorted_scores, order] = sort(all_scores);

    fprintf('\n=== FINAL LEADERBOARD ===\n\n');
    fprintf('%-6s  %-8s  %-10s  %-20s\n', 'Rank', 'Mode', 'Score', 'Config');
    fprintf('%s\n', repmat('-', 1, 50));
    for k = 1:length(order)
        idx = order(k);
        c = all_configs{idx};
        m = all_modes{idx};
        fnames = fieldnames(c);
        desc = sprintf('%s=%.1f', fnames{1}, c.(fnames{1}));
        if numel(fnames) > 1, desc = sprintf('%s, %s=%.1f', desc, fnames{2}, c.(fnames{2})); end
        fprintf('%-6d  %-8s  %-10.3f  %s\n', k, m, sorted_scores(idx), desc);
    end

    % Per-scenario comparison with default
    fprintf('\n=== PER-SCENARIO COMPARISON WITH DEFAULT ===\n\n');
    fprintf('%-22s  %-12s', 'Scenario', 'Default(R1)');
    for k = 1:min(5, length(order))
        fprintf('  %-12s', sprintf('#%d(%s)', order(k), all_modes{order(k)}));
    end
    fprintf('\n');
    fprintf('%-22s  %-12s', repmat('-', 1, 22), repmat('-', 1, 12));
    for k = 1:min(5, length(order)), fprintf('  %-12s', repmat('-', 1, 12)); end
    fprintf('\n');

    for si = 1:numel(specs)
        def_val = mean(baseline_rmse(si, :));
        fprintf('%-22s  %-12.3f', specs{si}{2}, def_val);
        for k = 1:min(5, length(order))
            idx = order(k);
            c = all_configs{idx};
            val = eval_candidate_protected(params, specs, prepared, ...
                c, all_modes{idx}, baseline_rmse, n_seeds, seed_start, si);
            fprintf('  %-12.3f', val);
        end
        fprintf('\n');
    end

    save(fullfile(root, 'final_results.mat'), ...
        'all_configs', 'all_modes', 'sorted_scores', 'order', ...
        'baseline_rmse', 'specs');
    fprintf('\nResults saved to %s/final_results.mat\n', root);
end


% =========================================================================
% REPORT
% =========================================================================

function run_report(root)
    fprintf('============================================================\n');
    fprintf('SEARCH RESULTS SUMMARY\n');
    fprintf('============================================================\n\n');

    if exist(fullfile(root, 'final_results.mat'), 'file')
        fr = load(fullfile(root, 'final_results.mat'));
        fprintf('Top 5 configurations:\n\n');
        for k = 1:min(5, length(fr.order))
            idx = fr.order(k);
            c = fr.all_configs{idx};
            m = fr.all_modes{idx};
            fprintf('  #%d (%s) - score=%.3f\n', idx, m, fr.sorted_scores(idx));
            fnames = fieldnames(c);
            for fi = 1:numel(fnames)
                fprintf('    %-30s = %.4f\n', fnames{fi}, c.(fnames{fi}));
            end
            fprintf('\n');
        end
    else
        fprintf('No final results found. Run imm_param_search_v2("refine") first.\n');
    end

    if exist(fullfile(root, 'baseline_results.mat'), 'file')
        bl = load(fullfile(root, 'baseline_results.mat'));
        fprintf('Default baselines (R1 avg): %.3f km\n', mean(bl.baseline_rmse(:,1)));
    end
end


% =========================================================================
% HELPER FUNCTIONS
% =========================================================================

function specs = get_all_scenarios()
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
end

function weights = get_scenario_weights()
    weights = [0.15, 0.08, 0.08, 0.08, 0.08, 0.08, 0.06, 0.06, 0.07, 0.07];
end

function lhs_matrix = generate_lhs(n_samples, n_dims, seed_val)
    rng(seed_val, 'twister');
    unit = zeros(n_samples, n_dims);
    for col = 1:n_dims
        perm = randperm(n_samples);
        unit(:, col) = (perm' - rand(n_samples, 1)) / n_samples;
    end
    % Gamma=0.5 exponent scaling for edge emphasis
    lhs_matrix = unit .^ 0.5;
end

function pos_sq = eval_scenario_r1_r2(params, inp, radar_id, overrides, seed_start, n_seeds)
    total_pos_sq = [];
    for seed_idx = 1:n_seeds
        seed_val = seed_start + seed_idx - 1;
        overrides_with_seed = overrides;
        overrides_with_seed.random_seed = seed_val;

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
        fnames = fieldnames(overrides_with_seed);
        for fi = 1:numel(fnames)
            prm.(fnames{fi}) = overrides_with_seed.(fnames{fi});
        end

        tpl = ukf_imm('create', prm, rl, rlat, tl, tlat, prm.dt_sec);
        [tracks, ~, snaps] = run_oracle_tracker_sequence(det, tpl, prm, inp.truth_all, tg, false);

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
                pos_err = haversine_distance(trk.ukf.x(1), trk.ukf.x(3), true_lon, true_lat);
                total_pos_sq(end+1) = pos_err^2;
            end
        end
        clear tracks tpl det snaps;
    end
    pos_sq = total_pos_sq;
end

function score = eval_candidate_protected(params, specs, prepared, candidate, adapt_mode, ...
        baseline_rmse, n_seeds, seed_start, single_scenario_idx)

    weights = get_scenario_weights();
    total_weighted_score = 0;
    total_weight_sum = 0;
    all_passed = true;
    max_penalty = 0;

    n_scenarios = numel(specs);
    eval_scenarios = 1:n_scenarios;
    if nargin >= 9 && ~isempty(single_scenario_idx)
        eval_scenarios = single_scenario_idx;
    end

    for si = eval_scenarios
        inp = prepared{si};
        scenario_name = specs{si}{2};

        % Compute RMSE for this scenario (avg R1+R2)
        total_pos_sq = [];
        for radar_id = 1:2
            for seed_idx = 1:n_seeds
                seed_val = seed_start + seed_idx - 1;
                overrides = struct('imm_adapt_mode', adapt_mode, 'random_seed', seed_val);

                % Apply candidate overrides
                fnames = fieldnames(candidate);
                for fi = 1:numel(fnames)
                    overrides.(fnames{fi}) = candidate.(fnames{fi});
                end

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
                for fi = 1:numel(fnames)
                    prm.(fnames{fi}) = candidate.(fnames{fi});
                end

                tpl = ukf_imm('create', prm, rl, rlat, tl, tlat, prm.dt_sec);
                [tracks, ~, snaps] = run_oracle_tracker_sequence(det, tpl, prm, inp.truth_all, tg, false);

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
                        pos_err = haversine_distance(trk.ukf.x(1), trk.ukf.x(3), true_lon, true_lat);
                        total_pos_sq(end+1) = pos_err^2;
                    end
                end
                clear tracks tpl det snaps;
            end
        end

        rmse_km = sqrt(mean(total_pos_sq)) / 1000;
        baseline_avg = mean(baseline_rmse(si, :));
        total_weighted_score = total_weighted_score + weights(si) * rmse_km;
        total_weight_sum = total_weight_sum + weights(si);

        % Protection check
        if rmse_km > 1.05 * baseline_avg
            all_passed = false;
            violation = rmse_km - 1.05 * baseline_avg;
            max_penalty = max(max_penalty, violation);
        end
        clear total_pos_sq;
    end

    score = total_weighted_score / total_weight_sum;

    % Cliff penalty
    if ~all_passed
        score = score + 100.0 * max_penalty^2;
    end

    % Return struct with extra info if called from evaluate_candidates
    if nargout > 0
        score = struct('score', score, 'protect', all_passed, 'max_penalty', max_penalty);
    end
end

function scores = evaluate_candidates(root, params, specs, prepared, candidates, ...
        adapt_mode, baseline_rmse, n_seeds, n_candidates, param_names, param_lo, param_hi, track_label)

    n_scenarios = numel(specs);
    scores = struct('score', zeros(n_candidates, 1), ...
                    'protect', false(n_candidates, 1), ...
                    'max_penalty', zeros(n_candidates, 1));

    fprintf('Evaluating %d candidates (%s mode, %d seeds)...\n\n', ...
        n_candidates, adapt_mode, n_seeds);

    for i = 1:n_candidates
        c = candidates{i};
        fprintf('Candidate %d/%d:\n', i, n_candidates);

        % Print key params
        fnames = fieldnames(c);
        for fi = 1:min(3, numel(fnames))
            fprintf('  %-30s = %.3f\n', fnames{fi}, c.(fnames{fi}));
        end

        score_val = eval_candidate_protected(params, specs, prepared, c, adapt_mode, ...
            baseline_rmse, n_seeds, 10001);

        scores.score(i) = score_val.score;
        scores.protect(i) = score_val.protect;
        scores.max_penalty(i) = score_val.max_penalty;

        fprintf('  Score: %.3f (protected: %s, max_penalty: %.3f)\n\n', ...
            score_val.score, num2str(score_val.protect), score_val.max_penalty);

        % Checkpoint every 10 candidates
        if mod(i, 10) == 0
            save(fullfile(root, sprintf('lhs_checkpoint_track%s.mat', track_label)), ...
                'scores', 'candidates', 'adapt_mode', 'baseline_rmse');
            fprintf('  Checkpoint saved.\n');
        end
    end
end

function dist = haversine_distance(lon1, lat1, lon2, lat2)
    dlat = deg2rad(lat2 - lat1);
    dlon = deg2rad(lon2 - lon1);
    a = sin(dlon/2)^2 + cosd(lat1)*cosd(lat2)*sin(dlat/2)^2;
    a = max(0, min(1, a));
    dist = 6371000 * 2 * atan2(sqrt(a), sqrt(1-a));
end
