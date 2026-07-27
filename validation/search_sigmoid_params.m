function search_sigmoid_params()
% Sigmoid 参数网格搜索: threshold × k × power
% 先用1种子快速排序，再对top-5用5种子验证
    close all;
    addpath(genpath('.'));

    fprintf('========== Sigmoid Parameter Grid Search ==========%s', newline);
    fprintf('%s', newline);

    thresholds = [0.1, 0.2, 0.3, 0.4, 0.5];
    k_vals     = [1, 2, 3, 5, 7];
    power_vals = [0.5, 1.0, 1.5];

    n_t = numel(thresholds);
    n_k = numel(k_vals);
    n_p = numel(power_vals);
    n_combos = n_t * n_k * n_p;

    % 2个关键场景 × 1种子快速跑
    scenes = {'right_rate_0p7', 'left_rate_0p7'};
    seeds_fast = [42];

    % 先跑baseline
    fprintf('Baseline runs...%s', newline);
    baseline_rmse = zeros(numel(scenes), 1);
    for sc_i = 1:numel(scenes)
        p = simulation_params_oracle();
        p.random_seed = 42;
        r = run(scenes{sc_i}, p);
        baseline_rmse(sc_i) = r.errorStats_R1.overall.ukf.rms;
        fprintf('  %-18s: %.2f km%s', scenes{sc_i}, baseline_rmse(sc_i), newline);
    end

    % Step 1: 75 combos × 2 scenes × 1 seed = 150 runs
    fprintf('%sStep 1: Grid sweep (%d runs)%s', newline, n_combos*2, newline);
    scores = NaN(n_combos, 1);  % avg improvement across scenes

    for ci = 1:n_combos
        t = thresholds(mod(ci-1, n_t)+1);
        k = k_vals(mod(floor((ci-1)/n_t), n_k)+1);
        p = power_vals(mod(floor((ci-1)/(n_t*n_k)), n_p)+1);

        scene_imps = zeros(numel(scenes), 1);
        for sc_i = 1:numel(scenes)
            p_sg = simulation_params_oracle();
            p_sg.random_seed = 42;
            p_sg.k_turnrate = k;
            p_sg.imm_adaptive_mode = 'sigmoid';
            p_sg.threshold = t;
            p_sg.power = p;
            p_sg.imm_cv_dwell_time_sec = 2500;
            p_sg.imm_ct_dwell_time_sec = 660;
            p_sg.imm_ct_fixed_Q_scale = 5.3;
            p_sg.imm_transient_gain_max = 11.0;

            try
                r = run(scenes{sc_i}, p_sg);
                v = r.errorStats_R1.overall.ukf.rms;
                scene_imps(sc_i) = (baseline_rmse(sc_i) - v) / baseline_rmse(sc_i) * 100;
            catch ME
                scene_imps(sc_i) = NaN;
            end
        end

        valid_imps = scene_imps(~isnan(scene_imps));
        if ~isempty(valid_imps)
            scores(ci) = mean(valid_imps);
            fprintf('  [rank %2d] t=%.1f k=%2d p=%.1f: avg=%+.1f%% (rr=%+.1f, lr=%+.1f)%s', ...
                ci, t, k, p, scores(ci), scene_imps(1), scene_imps(2), newline);
        end
    end

    % Sort and pick top 5
    [sorted_scores, sorted_idx] = sort(scores, 'descend');
    top_n = min(5, nnz(isfinite(scores)));

    fprintf('%sStep 2: Top %d configs validated with 5 seeds%s', ...
        newline, top_n, newline);

    % Step 2: Top-5 × 2 scenes × 4 more seeds (123, 456, 789, 1024) = 40 runs
    for rank_i = 1:top_n
        ci = sorted_idx(rank_i);
        t = thresholds(mod(ci-1, n_t)+1);
        k = k_vals(mod(floor((ci-1)/n_t), n_k)+1);
        p = power_vals(mod(floor((ci-1)/(n_t*n_k)), n_p)+1);

        fprintf('%s--- Config #%d: t=%.1f k=%d p=%.1f (avg_imp=%+.1f%% from step1) ---%s', ...
            newline, rank_i, t, k, p, scores(ci), newline);

        results = NaN(4, 2);  % [seed_i, scene_i]
        seeds_full = [123, 456, 789, 1024];

        for si = 1:4
            for sc_i = 1:2
                p_sg = simulation_params_oracle();
                p_sg.random_seed = seeds_full(si);
                p_sg.k_turnrate = k;
                p_sg.imm_adaptive_mode = 'sigmoid';
                p_sg.threshold = t;
                p_sg.power = p;
                p_sg.imm_cv_dwell_time_sec = 2500;
                p_sg.imm_ct_dwell_time_sec = 660;
                p_sg.imm_ct_fixed_Q_scale = 5.3;
                p_sg.imm_transient_gain_max = 11.0;

                try
                    r = run(scenes{sc_i}, p_sg);
                    results(si, sc_i) = r.errorStats_R1.overall.ukf.rms;
                catch ME
                    % leave NaN
                end
            end
        end

        % Compute improvements vs baselines
        for sc_i = 1:2
            valid = results(:, sc_i);
            valid = valid(~isnan(valid));
            if ~isempty(valid)
                sg_mean = mean(valid);
                imp = (baseline_rmse(sc_i) - sg_mean) / baseline_rmse(sc_i) * 100;
                sg_std = std(valid);
                fprintf('  %-18s: %.2f ± %.2f (imp=%+.1f%%) [%d seeds]%s', ...
                    scenes{sc_i}, sg_mean, sg_std, imp, length(valid), newline);
            end
        end
    end

    fprintf('%sDone.%s', newline, newline);
end
