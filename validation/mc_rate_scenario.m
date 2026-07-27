function mc_rate_scenario(scenario_name, seed_start, seed_end)
% 单场景多种子MC: 3配置(Default/Cfg51/Cfg51+Sigmoid) × N种子
% 增量保存到 results/mc_<scenario>_<seed_start>.mat
%
% 用法:
%   mc_rate_scenario('right_rate_0p7', 1, 5)

    close all;
    addpath(genpath('.'));

    if nargin < 3
        seed_start = 1; seed_end = 5;
    end

    seeds = seed_start:seed_end;
    n_seeds = length(seeds);
    configs = {'Default', 'Config51', 'Sig_k3_p1', 'Sig_k7_p05'};
    n_cfg = length(configs);

    rmse_matrix = NaN(n_seeds, n_cfg);

    fprintf('========== MC: %s (seeds %d-%d) ==========%s', ...
        scenario_name, seed_start, seed_end, newline);

    for si = 1:n_seeds
        seed = seeds(si);
        fprintf('%s--- seed=%d ---%s', newline, seed, newline);

        for ci = 1:n_cfg
            cfg = configs{ci};

            % 构造参数
            params = simulation_params_oracle();
            params.random_seed = seed;

            switch cfg
                case 'Config51'
                    params.imm_cv_dwell_time_sec = 2500;
                    params.imm_ct_dwell_time_sec = 660;
                    params.imm_ct_fixed_Q_scale = 5.3;
                    params.imm_transient_gain_max = 11.0;
                case 'Sig_k3_p1'
                    params.imm_cv_dwell_time_sec = 2500;
                    params.imm_ct_dwell_time_sec = 660;
                    params.imm_ct_fixed_Q_scale = 5.3;
                    params.imm_transient_gain_max = 11.0;
                    params.k_turnrate = 3;
                    params.imm_adaptive_mode = 'sigmoid';
                    params.threshold = 0.3;
                    params.power = 1.0;
                case 'Sig_k7_p05'
                    params.imm_cv_dwell_time_sec = 2500;
                    params.imm_ct_dwell_time_sec = 660;
                    params.imm_ct_fixed_Q_scale = 5.3;
                    params.imm_transient_gain_max = 11.0;
                    params.k_turnrate = 7;
                    params.imm_adaptive_mode = 'sigmoid';
                    params.threshold = 0.3;
                    params.power = 0.5;
            end

            try
                r = run(scenario_name, params);
                rmse = r.errorStats_R1.overall.ukf.rms;
                rmse_matrix(si, ci) = rmse;
                fprintf('  %-16s: RMSE=%.2f%s', cfg, rmse, newline);
            catch ME
                fprintf('  %-16s: ERROR %.40s%s', cfg, ME.message, newline);
            end
        end
    end

    % 保存
    save_file = sprintf('results/mc_%s_%dto%d.mat', scenario_name, seed_start, seed_end);
    save(save_file, 'rmse_matrix', 'seeds', 'configs', 'scenario_name');

    % 当前批次的统计
    fprintf('%s=== Batch Summary ===%s', newline, newline);
    means = mean(rmse_matrix, 1, 'omitnan');
    stds = std(rmse_matrix, 0, 1, 'omitnan');
    for ci = 1:n_cfg
        fprintf('  %-16s: %.2f ± %.2f km%s', configs{ci}, means(ci), stds(ci), newline);
    end

    if all(~isnan(means))
        imp_cfg51 = (means(1) - means(2))/means(1) * 100;
        imp_combined = (means(1) - means(3))/means(1) * 100;
        imp_sigmoid_addon = (means(2) - means(3))/means(2) * 100;
        fprintf('%s', newline);
        fprintf('  Config51 vs Default:       %+.1f%%%s', imp_cfg51, newline);
        fprintf('  Cfg51+Sigmoid vs Default:  %+.1f%%%s', imp_combined, newline);
        fprintf('  Sigmoid net gain (vs Cfg51): %+.1f%%%s', imp_sigmoid_addon, newline);
    end

    fprintf('%sSaved: %s%s', newline, save_file, newline);
end
