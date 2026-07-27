function run_full_adaptive_evaluation()
% 全场景自适应PI评估 — 精简配置（6函数形态）
    close all;
    addpath(genpath('.'));

    fprintf('========== Full Adaptive PI Evaluation ========== %s', newline);

    % 实验配置矩阵 — 6个关键配置覆盖各函数形态+参数范围
    configs = { ...
        'linear_k2';   'linear';   2.0;   0; 0; 1.0;
        'linear_k5';   'linear';   5.0;   0; 0; 1.0;
        'exp_k3_p1.5'; 'exp_decay'; 3.0;   1.5; 0.4; 1.5;
        'exp_k5_p1.5'; 'exp_decay'; 5.0;   1.5; 0.4; 1.5;
        'exp_k5_p2.0'; 'exp_decay'; 5.0;   2.0; 0.4; 1.5;
        'sig_k3_t0.2'; 'sigmoid';   3.0;   1.0; 0.2; 1.0;
        'sig_k3_t0.3'; 'sigmoid';   3.0;   1.0; 0.3; 1.0;
        'sig_k5_t0.3'; 'sigmoid';   5.0;   1.0; 0.3; 1.0;
        'sig_k5_t0.5'; 'sigmoid';   5.0;   1.0; 0.5; 1.0;
        'sig_k3_t0.2p2';'sigmoid';  3.0;   2.0; 0.2; 1.0;
        'sig_k5_t0.3p2';'sigmoid';  5.0;   2.0; 0.3; 1.0;
    };
    n_cfg = size(configs, 1);

    scenarios_data = { ...
        'single_straight',      'straight';
        'single_turn_right_short', 'right_short';
        'single_uturn',         'uturn';
        'single_turn_left_sustained', 'left_sust';
        'multi_cross',          'multi_cross';
    };
    n_scenarios = size(scenarios_data, 1);

    fprintf('Scenarios: %d | Configs: %d | Total: %d runs%s%s', ...
        n_scenarios, n_cfg, n_scenarios*n_cfg, newline, newline);

    rmse_matrix = NaN(n_scenarios, n_cfg);

    for si = 1:n_scenarios
        sname = scenarios_data{si, 1};
        slabel = scenarios_data{si, 2};

        fprintf('%s--- %-14s ---%s', newline, slabel, newline);
        fprintf('%s', repmat('-', 70, 1));

        % baseline
        try
            rb = run(sname);
            bl_rmse(si) = rb.errorStats_R1.overall.ukf.rms;
        catch ME
            bl_rmse(si) = NaN;
        end

        for ci = 1:n_cfg
            k_val = str2double(configs{ci, 3});
            mode = configs{ci, 2};
            power = str2double(configs{ci, 5});
            threshold = str2double(configs{ci, 6});
            cfg_name = configs{ci, 1};

            params = simulation_params_oracle();
            if k_val > 0
                params.k_turnrate = k_val;
                if strcmp(mode, 'exp_decay'), params.power = power; end
                if strcmp(mode, 'sigmoid')
                    params.threshold = threshold;
                    params.power = power;
                end
            end

            try
                rr = run(sname, params);
                rmse = rr.errorStats_R1.overall.ukf.rms;
                rmse_matrix(si, ci) = rmse;
                imp = (bl_rmse(si) - rmse) / bl_rmse(si) * 100;

                marker = '';
                if abs(imp) > 1
                    if imp > 0, marker = '*'; end
                    if imp < 0, marker = '!'; end
                end
                fprintf('  %s: RMSE=%.2f (%+.1f%%)%s', ...
                    cfg_name, rmse, imp, marker);
                fprintf('%s', newline);
            catch ME
                rmse_matrix(si, ci) = NaN;
                fprintf('  %s: ERROR (%.30s)%s', cfg_name, ME.message, newline);
            end
        end
    end

    % ---- Summary table ----
    fprintf('%s%s%s', newline, repmat('=', 80, 1), newline);
    fprintf('SUMMARY TABLE (Baseline = %-4s):%s', ...
        configs{1,1}, newline);
    fprintf('%-14s', 'Scenario');
    for ci = 1:n_cfg
        fprintf('  %s', configs{ci, 1}(1:min(8, length(configs{ci, 1}))));
    end
    fprintf('%s', newline);
    fprintf('%s', repmat('-', 12 + 10*n_cfg, 1));

    for si = 1:n_scenarios
        slabel = scenarios_data{si, 2};
        fprintf('%-14s', slabel);
        for ci = 1:n_cfg
            r = rmse_matrix(si, ci);
            if isnan(r)
                fprintf('  N/A  ');
            else
                bl = bl_rmse(si);
                if isnan(bl), fprintf('  --.--  '); continue; end
                imp = (bl - r)/bl * 100;
                flag = '';
                if abs(imp) > 1
                    if imp > 0, flag = '*'; end
                    if imp < 0, flag = '!'; end
                end
                fprintf('  %.2f(%%+.1f) ', r, imp);
            end
        end
        fprintf('%s', newline);
    end

    fprintf('%sBest config per scenario:%s', newline, newline);
    for si = 1:n_scenarios
        bl = bl_rmse(si);
        row = rmse_matrix(si, :);
        valid = row(~isnan(row));
        idxs = find(~isnan(row));

        if ~isempty(valid)
            best_idx = idxs(min(find(row == min(row))));
            best_imp = (bl - row(best_idx)) / bl * 100;
            fprintf('  %-14s: %s (k=%d) -> %+.1f%%%s', ...
                slabel, configs{best_idx, 1}, k_vals(best_idx), best_imp, newline);
        end
    end
end
