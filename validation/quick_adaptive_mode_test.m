function quick_adaptive_mode_test()
    % 快速测试: default config下3种反馈函数在uturn和right_short的效果
    close all;
    addpath(genpath('.'));
    rng(42);

    baseline_r = run('single_turn_right_short');
    baseline_u = run('single_uturn');

    fprintf('========== Quick Adaptive Mode Test ========== %s', newline);
    fprintf('Default config | Seed=42%s', newline);
    fprintf('%s', repmat('-', 60, 1));
    fprintf('Baseline (k=0 fixed): right_short=%.2f  uturn=%.2f%s', ...
        baseline_r.errorStats_R1.overall.ukf.rms, ...
        baseline_u.errorStats_R1.overall.ukf.rms, newline);
    fprintf('%s', repmat('-', 60, 1));

    modes = {'exp_decay', 'sigmoid', 'linear'};
    powers = [1.5, 2.0];
    thresholds = [0.3, 0.4, 0.5];
    k_vals = [1.0, 2.0, 3.0, 5.0];

    for mi = 1:length(modes)
        mode = modes{mi};
        fprintf('%s--- %s ---%s', newline, upper(mode), newline);

        for ki = 1:length(k_vals)
            k = k_vals(ki);
            params = simulation_params_oracle();
            params.k_turnrate = k;
            params.imm_adaptive_mode = mode;

            if strcmp(mode, 'exp_decay')
                for pj = 1:length(powers)
                    params.power = powers(pj);
                    r_short = run('single_turn_right_short', params);
                    r_uturn = run('single_uturn', params);
                    imp_s = (baseline_r.rmse - r_short.rmse)/baseline_r.rmse*100;
                    imp_u = (baseline_u.rmse - r_uturn.rmse)/baseline_u.rmse*100;
                    marker = '';
                    if abs(imp_u) > 1
                        marker = '*';
                    end
                    fprintf('  k=%.0f power=%.1f RS=%.2f(%+.1f%%) UT=%.2f(%+.1f%%)%s', ...
                        k, powers(pj), r_short.rmse, imp_s, r_uturn.rmse, imp_u, marker);
                end
            elseif strcmp(mode, 'sigmoid')
                for tj = 1:length(thresholds)
                    params.threshold = thresholds(tj);
                    for pj = 1:length(powers)
                        params.power = powers(pj);
                        r_short = run('single_turn_right_short', params);
                        r_uturn = run('single_uturn', params);
                        imp_s = (baseline_r.rmse - r_short.rmse)/baseline_r.rmse*100;
                        imp_u = (baseline_u.rmse - r_uturn.rmse)/baseline_u.rmse*100;
                        marker = '';
                        if abs(imp_u) > 1
                            marker = '*';
                        end
                        fprintf('  k=%.0f thresh=%.1f power=%.1f RS=%.2f(%+.1f%%) UT=%.2f(%+.1f%%)%s', ...
                            k, thresholds(tj), powers(pj), ...
                            r_short.rmse, imp_s, r_uturn.rmse, imp_u, marker);
                    end
                end
            else
                % linear
                r_short = run('single_turn_right_short', params);
                r_uturn = run('single_uturn', params);
                imp_s = (baseline_r.rmse - r_short.rmse)/baseline_r.rmse*100;
                imp_u = (baseline_u.rmse - r_uturn.rmse)/baseline_u.rmse*100;
                marker = '';
                if abs(imp_u) > 1
                    marker = '*';
                end
                fprintf('  k=%.0f linear: RS=%.2f(%+.1f%%) UT=%.2f(%+.1f%%)%s', ...
                    k, r_short.rmse, imp_s, r_uturn.rmse, imp_u, marker);
            end
        end
    end
end
