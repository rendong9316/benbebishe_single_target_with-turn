function final_compare_best_configs()
    % 对比: Default+k=0 vs Default+Sigmoid(k=3,t=0.3) vs Config#51
    close all;
    addpath(genpath('.'));
    rng(42);

    scenarios = {'single_straight', 'single_turn_right_short', 'single_uturn', 'multi_cross'};
    labels = {'straight', 'right_short', 'uturn', 'multi_cross'};

    fprintf('========== Final Comparison: Best Adaptive Config ==========%s', newline);
    fprintf('%s', newline);

    rmse_results = zeros(length(scenarios), 3);

    % 测试矩阵
    configs = { ...
        'Default+k=0'; 'Default+Sigmoid(k=3,t=0.3)'; 'Config#51';
        0; 3; 0;
        'none'; 'sigmoid'; 0;
        0.3; 0.4; 5.3;
    };

    for si = 1:length(scenarios)
        sname = scenarios{si};
        label = labels{si};

        fprintf('%s--- %-12s ---%s', newline, label, newline);

        for ci = 1:3
            params = simulation_params_oracle();
            k_val = str2double(configs{2, ci});
            mode = configs{3, ci};
            ct_q = str2double(configs{4, ci});

            if k_val > 0
                params.k_turnrate = k_val;
            end
            if ~strcmp(mode, 'none')
                params.imm_adaptive_mode = mode;
                if strcmp(mode, 'sigmoid'), params.threshold = str2double(configs{3,ci}); end
                if strcmp(mode, 'sigmoid'), params.power = str2double(configs{4,ci}); end
            end

            if k_val == 0 && strcmp(mode, 'none')
                params.imm_cv_dwell_time_sec = 2500;
                params.imm_ct_dwell_time_sec = 660;
                params.imm_ct_fixed_Q_scale = 5.3;
                params.imm_transient_gain_max = 11.0;
            end

            try
                r = run(sname, params);
                rmse = r.errorStats_R1.overall.ukf.rms;
                rmse_results(si, ci) = rmse;

                cfg_name = configs{1, ci};
                if ci == 1
                    baseline = rmse;
                    marker = '';
                else
                    imp = (baseline - rmse)/baseline * 100;
                    marker = sprintf(' (%+.1f%%)', imp);
                end
                fprintf('  %-30s RMSE=%.2f%s%s', cfg_name, rmse, marker, newline);
            catch ME
                fprintf('  %-30s ERROR: %.40s%s', configs{1,ci}, ME.message, newline);
            end
        end
    end

    fprintf('%s%s%s', newline, repmat('=', 70, 1), newline);
    fprintf('SUMMARY TABLE (Default+k=0 / Adaptive / Config#51):%s', newline);
    fprintf('%-12s %-10s %-10s %-10s%s', 'Scenario', 'k=0', 'Sigmoid', 'Config#51', newline);
    fprintf('%s', repmat('-', 48, 1));

    for si = 1:length(scenarios)
        v1 = rmse_results(si,1);
        v2 = rmse_results(si,2);
        v3 = rmse_results(si,3);
        flag2 = ''; flag3 = '';
        if ~isnan(v2) && isnan(v1)==false && v2 < v1*(1-0.01), flag2 = '*'; end
        if ~isnan(v3) && isnan(v1)==false && v3 < v1*(1-0.01), flag3 = '*'; end
        printf('%-12s %-10.2f %-10.2f%s %-10.2f%s%s', ...
            labels{si}, v1, v2, flag2, v3, flag3, newline);
    end
end
