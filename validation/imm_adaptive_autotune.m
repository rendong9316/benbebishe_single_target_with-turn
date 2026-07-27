function imm_adaptive_autotune()
    close all;
    addpath(genpath('.'));
    rng(42);

    fprintf('========== IMM Adaptive P_ctcv — Auto Tune ========== %s', newline);

    base_params = simulation_params_oracle();
    base_params.imm_cv_dwell_time_sec = 2500;
    base_params.imm_ct_dwell_time_sec = 660;
    base_params.imm_ct_fixed_Q_scale = 5.3;
    base_params.imm_transient_gain_max = 11.0;

    scenarios_data = { ...
        'single_straight',      'straight',      0;
        'single_turn_right_short', 'right_short', 47;
        'single_uturn',         'uturn',        180;
        'multi_cross',          'multi_cross',  0;
    };
    n_scenarios = size(scenarios_data, 1);
    k_values = [0, 0.1, 0.5, 1.0, 2.0];
    n_k = length(k_values);

    rmse_matrix = NaN(n_scenarios, n_k);

    for si = 1:n_scenarios
        sname = scenarios_data{si,1};
        slabel = scenarios_data{si,2};
        expected_turn = str2double(scenarios_data{si,3});

        fprintf('%s--- %-12s (exp=%.0fs) ---%s', newline, slabel, expected_turn, newline);
        fprintf('%s', repmat('-', 60, 1));

        for ki = 1:n_k
            k = k_values(ki);
            params = base_params;
            if k > 0, params.k_turnrate = k; end

            try
                result = run(sname, params);
                rmse_r1 = result.errorStats_R1.overall.ukf.rms;
                rmse_matrix(si, ki) = rmse_r1;

                marker = '';
                baseline = rmse_matrix(si, 1);
                if ~isnan(baseline) && ki > 1
                    pct_diff = (baseline - rmse_r1)/baseline * 100;
                    if pct_diff > 2
                        marker = sprintf(' <-- IMPROVED +%.1f%%', pct_diff);
                    elseif pct_diff < -2
                        marker = sprintf(' <-- WORSE %+.1f%%', pct_diff);
                    end
                end
                fprintf('  k=%.1f -> RMSE=%.2fkm%s%s', k, rmse_r1, marker, newline);

            catch ME
                rmse_matrix(si, ki) = NaN;
                fprintf('  k=%.1f ERROR: %.40s%s', k, ME.message, newline);
            end
        end
        fprintf('%s', repmat('-', 60, 1));
        fprintf('%s', newline);
    end

    % 汇总
    fprintf('%s%s%s', newline, repmat('=', 70, 1), newline);
    fprintf('SUMMARY — Improvement of adaptive over fixed (k=0):%s', newline);
    for si = 1:n_scenarios
        baseline = rmse_matrix(si, 1);
        line = sprintf('  %-12s baseline=%.2fkm:', slabel, baseline);
        for ki = 2:n_k
            r = rmse_matrix(si, ki);
            if ~isnan(r)
                pct = (baseline - r)/baseline * 100;
                line = sprintf('%s k=%.1f:%+.1f%%', line, k_values(ki), pct);
            end
        end
        fprintf('%s', newline);
    end

    fprintf('%sBEST k PER SCENARIO:%s', newline, newline);
    for si = 1:n_scenarios
        baseline = rmse_matrix(si, 1);
        valid = rmse_matrix(si, 2:n_k);
        if any(~isnan(valid))
            diffs = rmse_matrix(si, 2:n_k) - baseline;
            [best_delta, best_idx] = min(diffs);
            best_k = k_values(best_idx + 1);
            if abs(best_delta) > 0.01
                flag = ' << WINNER';
            else
                flag = ' (fixed is best)';
            end
            fprintf('  %-12s: k=%.1f (%+.1f%%)%s%s', ...
                slabel, best_k, -best_delta/baseline*100, flag, newline);
        end
    end

    save('results/imm_adaptive_autotune.mat', 'rmse_matrix', 'k_values');
end
