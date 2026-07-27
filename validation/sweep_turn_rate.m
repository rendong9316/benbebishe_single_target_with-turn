function sweep_turn_rate(seeds)
    % turn_rate 离散多模型可行性测试（零源码改动）
    %
    % 原理：ukf_imm 的 CT 模型转弯率由 params.imm_turn_rate_rad_per_sec 控制。
    % 通过 params_override 注入不同候选值，跑多个"3 模型 IMM"实例，
    % 观察 RMSE 随候选 turn_rate 的变化。
    %
    % 关键判据：
    %   1. 注入真实 turn_rate 时 RMSE 是否大幅下降？（证明 mismatch 是瓶颈）
    %   2. RMSE vs turn_rate 曲线在真实值附近是否最低？（证明"选最优"机制成立）
    %   3. min(RMSE over candidates) 就是离散多模型的性能上界
    %
    % 用法: sweep_turn_rate([42]) 或 sweep_turn_rate([42 123 456])

    close all;
    addpath(genpath('.'));

    if nargin < 1 || isempty(seeds)
        seeds = [42];
    end

    % 候选 turn_rate (°/s)：覆盖民航典型范围
    candidates_deg = [0.5, 0.7, 1.0, 1.3, 1.5, 2.0];
    n_cand = length(candidates_deg);

    % 场景及其真实 turn_rate
    scenarios = {'right_rate_0p7', 'left_rate_0p7', 'right_rate_1p3', 'left_rate_1p3'};
    true_rate = [0.7, 0.7, 1.3, 1.3];
    n_sc = length(scenarios);
    n_seeds = length(seeds);

    % RMSE 矩阵: [n_sc x n_cand x n_seeds]
    rmse_mat = NaN(n_sc, n_cand, n_seeds);

    fprintf('==========================================\n');
    fprintf('turn_rate 扫描：离散多模型可行性测试\n');
    fprintf('候选 turn_rate (°/s): [%s]\n', num2str(candidates_deg));
    fprintf('场景数: %d, 候选数: %d, 种子数: %d\n', n_sc, n_cand, n_seeds);
    fprintf('总 run 数: %d\n', n_sc * n_cand * n_seeds);
    fprintf('==========================================\n\n');

    t0 = tic;
    for si = 1:n_seeds
        seed = seeds(si);
        fprintf('>>> Seed %d\n', seed);

        for sci = 1:n_sc
            sname = scenarios{sci};
            fprintf('  场景: %s (真实 turn_rate=%.1f°/s)\n', sname, true_rate(sci));

            for ci = 1:n_cand
                cand = candidates_deg(ci);

                % 注入候选 turn_rate，其他全用默认（基线 IMM）
                params = simulation_params_oracle();
                params.random_seed = seed;
                params.imm_turn_rate_rad_per_sec = cand * pi / 180.0;

                try
                    r = run(sname, params);
                    rmse = r.errorStats_R1.overall.ukf.rms;
                    rmse_mat(sci, ci, si) = rmse;

                    marker = '';
                    if abs(cand - true_rate(sci)) < 1e-6
                        marker = '  ← 真实值(ORACLE)';
                    elseif abs(cand - 1.0) < 1e-6
                        marker = '  ← 默认值(1°/s)';
                    end
                    fprintf('    turn_rate=%.1f°/s → RMSE=%.3f km%s\n', cand, rmse, marker);
                catch ME
                    fprintf('    turn_rate=%.1f°/s → ERROR: %.50s\n', cand, ME.message);
                end
            end
            fprintf('\n');
        end
        fprintf('  [Seed %d done, elapsed %.0fs]\n\n', seed, toc(t0));
    end

    % 保存
    save_file = 'results/sweep_turn_rate.mat';
    save(save_file, 'rmse_mat', 'candidates_deg', 'scenarios', 'true_rate', 'seeds', '-v7');
    fprintf('Saved: %s\n\n', save_file);

    % ====== 汇总 ======
    fprintf('============================================================\n');
    fprintf('汇总（跨种子平均 RMSE, km）\n');
    fprintf('============================================================\n');

    % 表头
    fprintf('%-18s', 'Scenario \\ rate');
    for ci = 1:n_cand
        fprintf(' | %5.1f°/s', candidates_deg(ci));
    end
    fprintf(' | Default | Oracle | MM上界\n');
    fprintf('%s\n', repmat('-', 1, 95));

    default_imp_all = [];
    oracle_imp_all = [];
    mm_bound_imp_all = [];

    for sci = 1:n_sc
        fprintf('%-18s', scenarios{sci});
        row_mean = mean(rmse_mat(sci, :, :), 3, 'omitnan');
        for ci = 1:n_cand
            fprintf(' | %7.3f', row_mean(ci));
        end

        % Default = 1.0°/s 候选
        default_idx = find(abs(candidates_deg - 1.0) < 1e-6);
        default_rmse = row_mean(default_idx);

        % Oracle = 真实 turn_rate 候选
        oracle_idx = find(abs(candidates_deg - true_rate(sci)) < 1e-6);
        oracle_rmse = row_mean(oracle_idx);

        % MM 上界 = min over candidates
        mm_bound = min(row_mean);

        fprintf(' | %7.3f', default_rmse);
        fprintf(' | %7.3f', oracle_rmse);
        fprintf(' | %7.3f', mm_bound);
        fprintf('\n');

        default_imp_all = [default_imp_all; default_rmse];
        oracle_imp_all = [oracle_imp_all; oracle_rmse];
        mm_bound_imp_all = [mm_bound_imp_all; mm_bound];
    end

    % 总体改善
    fprintf('\n');
    fprintf('平均 RMSE:  Default(1°/s) = %.3f km\n', mean(default_imp_all));
    fprintf('            Oracle(真实) = %.3f km  (改善 %+.1f%%)\n', ...
        mean(oracle_imp_all), (mean(default_imp_all)-mean(oracle_imp_all))/mean(default_imp_all)*100);
    fprintf('            MM上界(min)  = %.3f km  (改善 %+.1f%%)\n', ...
        mean(mm_bound_imp_all), (mean(default_imp_all)-mean(mm_bound_imp_all))/mean(default_imp_all)*100);

    fprintf('\n============================================================\n');
    fprintf('结论判读\n');
    fprintf('============================================================\n');
    oracle_gain = (mean(default_imp_all)-mean(oracle_imp_all))/mean(default_imp_all)*100;
    mm_gain = (mean(default_imp_all)-mean(mm_bound_imp_all))/mean(default_imp_all)*100;
    if oracle_gain > 10
        fprintf('✓ Oracle 改善 %.1f%% > 10%%：turn_rate mismatch 确认是主要瓶颈\n', oracle_gain);
    else
        fprintf('⚠ Oracle 改善仅 %.1f%%：turn_rate mismatch 不是唯一瓶颈\n', oracle_gain);
    end
    if mm_gain > 5
        fprintf('✓ 离散多模型上界改善 %.1f%% > 5%%：多模型思路值得实现\n', mm_gain);
    else
        fprintf('⚠ 离散多模型上界改善仅 %.1f%%：多模型收益有限\n', mm_gain);
    end
end
