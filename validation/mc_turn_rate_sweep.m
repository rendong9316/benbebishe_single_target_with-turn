function mc_turn_rate_sweep(scenario_name, seeds)
    % Turn rate sweep MC: test multiple turn_rate values for a single scenario
    %
    % 用法: mc_turn_rate_sweep('right_rate_0p7', 1:10)
    %
    % 特性:
    %   - 复用 existing mc_rate_scenario 框架，确保兼容性
    %   - 每个种子跑 6 个 turn_rate 候选
    %   - 结果保存为 results/mc_turn_rate_<scenario>_<seeds>.mat

    if nargin < 2 || isempty(seeds)
        seeds = 1:5;
    end

    if nargin < 1 || isempty(scenario_name)
        error('需要提供 scenario_name 参数');
    end

    close all;
    addpath(genpath(pwd));

    n_seeds = length(seeds);
    candidates_deg = [0.5, 0.7, 1.0, 1.3, 1.5, 2.0];
    n_cand = length(candidates_deg);
    configs = cell(1, n_cand);
    for ci = 1:n_cand
        configs{ci} = sprintf('turn_%g', candidates_deg(ci));
    end

    % 结果矩阵: [n_seeds x n_cand]
    rmse_matrix = NaN(n_seeds, n_cand);

    fprintf('========== turn_rate sweep: %s (seeds %d-%d) ==========%s\n', ...
        scenario_name, seeds(1), seeds(end), newline);

    t0 = tic;

    for si = 1:n_seeds
        seed = seeds(si);
        fprintf('\n--- Seed %d ---\n', seed);

        for ci = 1:n_cand
            cfg = configs{ci};
            cand = candidates_deg(ci);

            % 构造参数
            params = simulation_params_oracle();
            params.random_seed = seed;
            params.imm_turn_rate_rad_per_sec = cand * pi / 180.0;

            try
                r = run(scenario_name, params);
                rmse = r.errorStats_R1.overall.ukf.rms;
                rmse_matrix(si, ci) = rmse;
                fprintf('  %-12s: RMSE=%.3f km\n', cfg, rmse);
            catch ME
                fprintf('  %-12s: ERROR %.40s\n', cfg, ME.message);
                rmse_matrix(si, ci) = NaN;
            end
        end

        % 每种子保存进度
        save(sprintf('mr_s%d_seed%d.mat', si, seed), 'rmse_matrix', 'seeds', 'configs', 'scenario_name', '-v7');
    end

    elapsed = toc(t0);
    fprintf('\n=== 批次汇总 (运行时间 %.0fs) ===\n\n', elapsed);

    % 计算每种子内各候选的均值（跨种子）
    means = mean(rmse_matrix, 1, 'omitnan');
    stds = std(rmse_matrix, 0, 1, 'omitnan');
    for ci = 1:n_cand
        fprintf('  %-12s: %.3f ± %.3f km\n', configs{ci}, means(ci), stds(ci));
    end

    % 找出每个种子内最小的（MM上界）
    mm_per_seed = min(rmse_matrix, [], 2);
    mm_mean = mean(mm_per_seed, 'omitnan');
    mm_std = std(mm_per_seed, 0, 'omitnan');
    fprintf('  %-12s: %.3f ± %.3f km (MM上界)\n', 'MIN', mm_mean, mm_std);

    % 计算改善（相对于 1.0°/s 默认值）
    default_ci = find(abs(candidates_deg - 1.0) < 0.01);
    if ~isempty(default_ci)
        default_rmse = means(default_ci);
        fprintf('\n改善 vs Default (1.0°/s): %.3f km\n', default_rmse);
        for ci = 1:n_cand
            if ci ~= default_ci
                imp = (default_rmse - means(ci)) / default_rmse * 100;
                fprintf('  %s: %+1.1f%%\n', configs{ci}, imp);
            end
        end
    end

    % 保存最终结果
    save_file = sprintf('results/mc_turn_rate_%s_%dto%d.mat', scenario_name, seeds(1), seeds(end));
    save(save_file, 'rmse_matrix', 'seeds', 'configs', 'scenario_name', 'candidates_deg', '-v7');
    fprintf('\n结果保存至: %s\n', save_file);
end
