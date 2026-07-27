function mc_phase3_validation()
% ===========================================================================
% Phase 3 验证: Monte Carlo + right_rate_0p7 场景
%
% 实验:
%   A. right_rate_0p7 单种子验证: default vs sigmoid_k3
%   B. 5 seeds MC: 3关键场景(straight, right_short, uturn) × 2配置
%
% 注: MATLAB 长时间运行易 heap corruption, 每个场景独立调用
% ===========================================================================
    close all;
    addpath(genpath('.'));

    fprintf('========== Phase 3 Validation ========== %s', newline);
    fprintf('%s', newline);

    %% ---- Part A: right_rate_0p7 单种子 ----
    fprintf('=== Part A: right_rate_0p7 (the killer scenario) ===%s', newline);
    rng(42);

    % Default baseline
    rb = run('right_rate_0p7');
    bl = rb.errorStats_R1.overall.ukf.rms;
    fprintf('  Default (k=0 fixed): RMSE_R1=%.2fkm%s', bl, newline);

    % Sigmoid k=3
    p2 = simulation_params_oracle();
    p2.k_turnrate = 3;
    p2.imm_adaptive_mode = 'sigmoid';
    p2.threshold = 0.3;
    p2.power = 1.0;
    r2 = run('right_rate_0p7', p2);
    s_rmse = r2.errorStats_R1.overall.ukf.rms;
    imp = (bl - s_rmse)/bl * 100;
    fprintf('  Sigmoid k=3 t=0.3:    RMSE_R1=%.2fkm (%+.1f%%)%s', s_rmse, imp, newline);

    % Sigmoid k=5
    p3 = p2; p3.k_turnrate = 5;
    r3 = run('right_rate_0p7', p3);
    s5_rmse = r3.errorStats_R1.overall.ukf.rms;
    imp5 = (bl - s5_rmse)/bl * 100;
    fprintf('  Sigmoid k=5 t=0.3:    RMSE_R1=%.2fkm (%+.1f%%)%s', s5_rmse, imp5, newline);

    %% ---- Part B: Monte Carlo (5 seeds) ----
    fprintf('%s=== Part B: Monte Carlo (5 seeds) ===%s', newline, newline);
    seeds = [42, 123, 456, 789, 1024];
    scenarios = {'single_straight', 'single_turn_right_short', 'single_uturn'};
    sc_labels = {'straight', 'right_short', 'uturn'};

    mc_results = NaN(length(seeds), length(scenarios), 2);  % [seed, scenario, config]

    for si = 1:length(scenarios)
        sname = scenarios{si};
        slabel = sc_labels{si};
        fprintf('%s--- %-12s ---%s', newline, slabel, newline);

        for seed_i = 1:length(seeds)
            seed = seeds(seed_i);

            % Default baseline
            try
                p_bl = simulation_params_oracle();
                p_bl.random_seed = seed;
                rb_s = run(sname, p_bl);
                mc_results(seed_i, si, 1) = rb_s.errorStats_R1.overall.ukf.rms;
            catch ME
                fprintf('  seed=%d baseline ERROR: %.30s%s', seed, ME.message, newline);
            end

            % Sigmoid k=3
            try
                p_sg = simulation_params_oracle();
                p_sg.random_seed = seed;
                p_sg.k_turnrate = 3;
                p_sg.imm_adaptive_mode = 'sigmoid';
                p_sg.threshold = 0.3;
                p_sg.power = 1.0;
                rs_s = run(sname, p_sg);
                mc_results(seed_i, si, 2) = rs_s.errorStats_R1.overall.ukf.rms;
            catch ME
                fprintf('  seed=%d sigmoid ERROR: %.30s%s', seed, ME.message, newline);
            end

            bl_v = mc_results(seed_i, si, 1);
            sg_v = mc_results(seed_i, si, 2);
            if ~isnan(bl_v) && ~isnan(sg_v)
                imp_v = (bl_v - sg_v)/bl_v * 100;
                fprintf('  seed=%4d  bl=%.2f  sg=%.2f  (%+.1f%%)%s', ...
                    seed, bl_v, sg_v, imp_v, newline);
            end
        end
    end

    %% ---- Part C: 统计汇总 ----
    fprintf('%s%s%s', newline, repmat('=', 70, 1), newline);
    fprintf('SUMMARY (mean +- std across %d seeds):%s', length(seeds), newline);
    fprintf('%-14s | %-12s | %-12s | %-10s | %-8s%s', ...
        'Scenario', 'Baseline', 'Sigmoid_k3', 'Improvement', 'P<0.05', newline);
    fprintf('%s', repmat('-', 70, 1));

    for si = 1:length(scenarios)
        bl_vec = mc_results(:, si, 1);
        sg_vec = mc_results(:, si, 2);
        valid = ~isnan(bl_vec) & ~isnan(sg_vec);

        if sum(valid) >= 2
            bl_m = mean(bl_vec(valid));
            bl_s = std(bl_vec(valid));
            sg_m = mean(sg_vec(valid));
            sg_s = std(sg_vec(valid));
            imp_m = (bl_m - sg_m)/bl_m * 100;

            % 简化版 t 检验 (配对)
            diffs = bl_vec(valid) - sg_vec(valid);
            if std(diffs) > 0
                t_stat = mean(diffs) / (std(diffs)/sqrt(length(diffs)));
                % df=4时 t>2.776 为 p<0.05
                p_signif = abs(t_stat) > 2.776;
            else
                p_signif = false;
            end

            flag = '';
            if p_signif, flag = '*'; end

            fprintf('%-14s | %.2f+-%.2f | %.2f+-%.2f | %+.1f%%     | %s%s', ...
                sc_labels{si}, bl_m, bl_s, sg_m, sg_s, imp_m, ...
                char(flag*'YES'+~flag*'no'), newline);
        end
    end

    fprintf('%s', newline);
    fprintf('* indicates statistical significance (paired t-test, p<0.05, df=%d)%s', ...
        length(seeds)-1);
    fprintf('%s', newline);

    % 保存数据
    save('results/mc_phase3_validation.mat', ...
        'mc_results', 'seeds', 'scenarios', 'sc_labels');
    fprintf('%sResults saved to results/mc_phase3_validation.mat%s', newline, newline);
end
