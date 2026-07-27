function test_default_with_adaptive()
% default config + 自适应PI调参（k=[0, 0.5, 1.0, 2.0]）
% 不用Config#51，用纯默认参数看自适应效果
    close all;
    addpath(genpath('.'));
    rng(42);

    fprintf('========== Default Config + Adaptive P_ctcv ==========%s', newline);
    fprintf('%s', newline);

    % 不需要任何base_params覆盖——完全用simulation_params_oracle()默认值
    scenarios_data = { ...
        'single_turn_right_short','right_short',   47;
        'single_uturn',           'uturn',        180;
    };
    k_values = [0, 0.5, 1.0, 2.0];

    for si = 1:size(scenarios_data,1)
        sname = scenarios_data{si,1};
        slabel = scenarios_data{si,2};
        expected_turn = str2double(scenarios_data{si,3});

        fprintf('%s--- %-12s (exp=%.0fs) ---%s', newline, slabel, expected_turn, newline);

        baseline_rmse = NaN;

        for ki = 1:length(k_values)
            k = k_values(ki);
            params = simulation_params_oracle();
            if k > 0, params.k_turnrate = k; end

            try
                r = run(sname, params);
                rmse = r.errorStats_R1.overall.ukf.rms;

                if ki == 1
                    baseline_rmse = rmse;
                    marker = '';
                else
                    pct = (baseline_rmse - rmse)/baseline_rmse * 100;
                    marker = sprintf(' (%+.1f%% vs fixed)', pct);
                end
                fprintf('  k=%.1f -> RMSE=%.2fkm%s%s', k, rmse, marker, newline);

            catch ME
                fprintf('  k=%.1f ERROR: %.40s%s', k, ME.message, newline);
            end
        end
        fprintf('%s', newline);
    end
end
