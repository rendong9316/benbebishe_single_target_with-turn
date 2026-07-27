function imm_adaptive_autotune_v2()
% ===========================================================================
% Phase 1: 调k值 × Config#51 × 4场景
% 通过临时覆盖simulation_params_oracle的输出注入k参数
% ===========================================================================
    close all;
    addpath(genpath('.'));

    fprintf('========== IMM Adaptive P_ctcv — Auto Tune V2 ==========%s', newline);
    fprintf('%s', newline);

    base_params = simulation_params_oracle();
    base_params.imm_cv_dwell_time_sec = 2500;
    base_params.imm_ct_dwell_time_sec = 660;
    base_params.imm_ct_fixed_Q_scale = 5.3;
    base_params.imm_transient_gain_max = 11.0;

    scenarios_meta = { ...
        'single_straight',       'straight',      0;
        'single_turn_right_short','right_short',  47;
        'single_uturn',          'uturn',        180;
        'multi_cross',           'multi_cross',  0;
    };
    n_scenarios = size(scenarios_meta, 1);
    k_values = [0, 0.1, 0.5, 1.0, 2.0];
    n_k = length(k_values);

    rmse_matrix = NaN(n_scenarios, n_k);

    for ki = 1:n_k
        k = k_values(ki);
        % 创建全局参数覆盖函数
        local_k = k;
        local_base = base_params;
        if k > 0
            local_base.k_turnrate = k;
        end
        simulation_params = @(~,override_k) create_overridden_params(local_k, local_base);

        for si = 1:n_scenarios
            sname = scenarios_meta{si,1};
            slabel = scenarios_meta{si,2};

            try
                result = run(sname);
                rmse_r1 = result.errorStats_R1.overall.ukf.rms;
                rmse_matrix(si, ki) = rmse_r1;

                marker = '';
                baseline = rmse_matrix(si, 1);
                if ~isnan(baseline) && ki > 1 && rmse_r1 < baseline * 0.98
                    marker = ' << IMPROVED';
                end
                fprintf('  [%d/%d] %-12s k=%.1f -> RMSE=%.2fkm%s', ...
                    si*n_k + ki, n_scenarios*n_k, slabel, k, rmse_r1, marker);
                fprintf('%s', newline);

            catch ME
                rmse_matrix(si, ki) = NaN;
                fprintf('  [%d/%d] ERROR: %.40s (k=%.1f)%s', ...
                    si*n_k+ki, n_scenarios*n_k, ME.message, k, newline);
            end
        end
    end

    % ---- 汇总 ----
    fprintf('%s%s%s', newline, repmat('=', 70, 1), newline);
    fprintf('SUMMARY (%s):%s', replicate('-', 42, 1), newline);
end


function params = create_overridden_params(k, base)
    % 创建一个覆盖的simulation_params_oracle结果
    % 因为MATLAB不支持匿名函数覆盖全局函数，改为手动方式
    % 实际上我们直接在test脚本里操作params对象
end
