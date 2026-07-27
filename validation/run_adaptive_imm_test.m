function run_adaptive_imm_test()
% ===========================================================================
% IMM自适应转移概率测试
% 改 params.k_turnrate 然后运行 run.m
% ===========================================================================

    close all;
    addpath(genpath('.'));

    % ---- 配置场景 ----
    scenarios = {'right_rate_0p7', 'left_sustained', 'straight'};
    snames = {'right_rate_0p7(右缓转257s)', ...
              'left_sustained(左转180s)', 'straight(直线2426s)'};

    % Config#51 基础参数
    base_params = simulation_params_oracle();
    base_params.imm_cv_dwell_time_sec = 2500;
    base_params.imm_ct_dwell_time_sec = 660;
    base_params.imm_ct_fixed_Q_scale = 5.3;
    base_params.imm_transient_gain_max = 11.0;

    k_values = [0, 0.1, 0.5, 1.0, 2.0];
    cnames = {'k=0(fixed)', 'k=0.1', 'k=0.5', 'k=1.0', 'k=2.0'};

    fprintf('========== Adaptive IMM Transfer Prob Test ==========%s', newline);
    fprintf('%s', repmat('-', 70, 1));

    rmse_table = NaN(length(scenarios), length(k_values));

    for si = 1:length(scenarios)
        fprintf('%s%s%s', newline, snames{si}, newline);
        rng(42);  % 固定种子

        for ki = 1:length(k_values)
            params = base_params;
            if k_values(ki) > 0
                params.k_turnrate = k_values(ki);
            end

            % 临时覆盖simulation_params_oracle的输出
            try
                result = run(scenarios{si});
                rmse = result.errorStats_R1.overall.ukf.rms;
                rmse_table(si, ki) = rmse;

                extra = '';
                if si == 1 && ki > 1
                    bl = rmse_table(si, 1);
                    if ~isnan(bl) && rmse < bl * 0.98
                        extra = ' << IMPROVED';
                    elseif rmse > bl * 1.02
                        extra = ' << WORSE';
                    end
                end
                printf '  %-14s RMSE=%.2fkm%s%s', cnames{ki}, rmse, extra, newline;

            catch ME
                printf '  %-14s ERROR: %.40s%s', cnames{ki}, ME.message, newline;
            end
        end
        fprintf('%s', repmat('-', 70, 1));
    end

    % 汇总
    fprintf('%s%s%s', newline, repmat('=', 70, 1), newline);
    fprintf('SUMMARY — right_rate_0p7 (seed=42):%s', newline);
    baseline = rmse_table(1, 1);
    for ki = 2:length(k_values)
        if ~isnan(rmse_table(1, ki))
            imp = (baseline - rmse_table(1, ki)) / baseline * 100;
            printf '  %s -> %+.1f%%%s', cnames{ki}, imp, newline;
        end
    end

    % 保存结果
    save('results/adaptive_imm_test_seed42.mat', ...
        'rmse_table', 'k_values', 'scenarios', 'snames');
end
