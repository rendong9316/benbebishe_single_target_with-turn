function mc_large_turn(seeds)
    % 新轨迹（真 180° 大转弯）下的 MC 验证
    % 对比 4 配置 × 4 场景 × N 种子
    %
    % 新验证：turn_rate 离散多模型可行性测试
    % 每个配置对应不同的 turn_rate 值
    % 配置:
    %   1. turn_0.5    - CT模型转弯率0.5°/s
    %   2. turn_0.7    - CT模型转弯率0.7°/s
    %   3. turn_1.0    - CT模型转弯率1.0°/s (默认)
    %   4. turn_1.3    - CT模型转弯率1.3°/s
    %   5. turn_1.5    - CT模型转弯率1.5°/s
    %   6. turn_2.0    - CT模型转弯率2.0°/s
    %
    % 场景: right_rate_0p7, left_rate_0p7, right_rate_1p3, left_rate_1p3

    candidates_deg = [0.5, 0.7, 1.0, 1.3, 1.5, 2.0];
    n_cand = length(candidates_deg);
    configs = cell(1, n_cand);
    for ci = 1:n_cand
        configs{ci} = sprintf('turn_%g', candidates_deg(ci));
    end

    close all;
    addpath(genpath('.'));

    if nargin < 1 || isempty(seeds)
        seeds = [42, 123, 456];
    end

    scenarios = {'right_rate_0p7', 'left_rate_0p7', 'right_rate_1p3', 'left_rate_1p3'};
    n_seeds = length(seeds);
    n_sc = length(scenarios);
    n_cfg = length(configs);  % 6, from turn_rate configs above

    % 结果矩阵: [n_sc x n_cfg x n_seeds]
    rmse_mat = NaN(n_sc, n_cfg, n_seeds);

    fprintf('========================================\n');
    fprintf('MC: 真大转弯场景 (180° 半圆)\n');
    fprintf('Seeds: [%s]\n', num2str(seeds));
    fprintf('Scenarios: %d, Configs: %d\n', n_sc, n_cfg);
    fprintf('Total runs: %d\n', n_sc * n_cfg * n_seeds);
    fprintf('========================================\n\n');

    t_start = tic;

    for si = 1:n_seeds
        seed = seeds(si);
        fprintf('>>> Seed %d (%d/%d)\n', seed, si, n_seeds);

        for sci = 1:n_sc
            sname = scenarios{sci};

            for ci = 1:n_cfg
                cfg = configs{ci};

                % 构造参数
                params = simulation_params_oracle();
                params.random_seed = seed;

                switch cfg
                    case 'turn_0.5'
                        params.imm_turn_rate_rad_per_sec = 0.5 * pi / 180.0;
                    case 'turn_0.7'
                        params.imm_turn_rate_rad_per_sec = 0.7 * pi / 180.0;
                    case 'turn_1.0'
                        params.imm_turn_rate_rad_per_sec = 1.0 * pi / 180.0;
                    case 'turn_1.3'
                        params.imm_turn_rate_rad_per_sec = 1.3 * pi / 180.0;
                    case 'turn_1.5'
                        params.imm_turn_rate_rad_per_sec = 1.5 * pi / 180.0;
                    case 'turn_2.0'
                        params.imm_turn_rate_rad_per_sec = 2.0 * pi / 180.0;
                end

                try
                    r = run(sname, params);
                    rmse = r.errorStats_R1.overall.ukf.rms;
                    rmse_mat(sci, ci, si) = rmse;
                    fprintf('  %-18s | %-14s | RMSE=%.3f\n', sname, cfg, rmse);
                catch ME
                    fprintf('  %-18s | %-14s | ERROR: %.50s\n', sname, cfg, ME.message);
                end
            end
        end
        fprintf('  [Seed %d done, elapsed %.0fs]\n\n', seed, toc(t_start));
    end

    % 保存
    save_file = 'results/mc_large_turn.mat';
    save(save_file, 'rmse_mat', 'seeds', 'scenarios', 'configs', '-v7');
    fprintf('Saved: %s\n\n', save_file);

    % ====== 汇总统计 ======
    fprintf('\n========================================\n');
    fprintf('turn_rate 扫描 (mean RMSE km, 跨种子)\n');
    fprintf('========================================\n');
    fprintf('%-20s', '场景');
    for ci = 1:n_cand
        fprintf(' | %4.1f°/s ', candidates_deg(ci));
    end
    fprintf('\n');
    fprintf('%s\n', repmat('-', 90));

    for sci = 1:n_sc
        fprintf('%-20s', scenarios{sci});
        for ci = 1:n_cand
            vals = rmse_mat(sci, ci, :);
            m = mean(vals, 'omitnan');
            fprintf(' | %6.3f         ', m);
        end
        fprintf('\n');
    end

    % 汇总统计
    fprintf('\n========================================\n');
    fprintf('turn_rate 扫描 (mean RMSE km, 跨种子)\n');
    fprintf('========================================\n');
    fprintf('%-20s', '场景');
    for ci = 1:n_cand
        fprintf(' | %4.1f°/s ', candidates_deg(ci));
    end
    fprintf('\n');
    fprintf('%s\n', repmat('-', 90));

    for sci = 1:n_sc
        fprintf('%-20s', scenarios{sci});
        for ci = 1:n_cand
            vals = rmse_mat(sci, ci, :);
            m = mean(vals, 'omitnan');
            fprintf(' | %6.3f         ', m);
        end
        fprintf('\n');
    end

    % 计算每个场景的默认值（1.0°/s, ci=3）和最佳值（min over candidates）
    fprintf('\n========================================\n');
    fprintf('关键发现（每场景内）\n');
    fprintf('========================================\n');

    for sci = 1:n_sc
        default_ci = find(abs(candidates_deg - 1.0) < 0.01);
        if ~isempty(default_ci)
            def_vals = rmse_mat(sci, default_ci, :);
            def_mean = mean(def_vals, 'omitnan');
        else
            def_mean = NaN;
        end

        mm_vals = min(rmse_mat(sci, :, :, 2), [], 'omitnan');
        mm_mean = mean(mm_vals, 'omitnan');

        oracle_ci = find(abs(candidates_deg - true_rate(sci)) < 0.01);
        if ~isempty(oracle_ci)
            oracle_vals = rmse_mat(sci, oracle_ci, :);
            oracle_mean = mean(oracle_vals, 'omitnan');
        else
            oracle_mean = NaN;
        end

        fprintf('%-20s: Default(1.0)=%.3f, Oracle(%.1f)=%.3f, MM-upper=%.3f\n', ...
            scenarios{sci}, def_mean, true_rate(sci), oracle_mean, mm_mean);
    end
end

