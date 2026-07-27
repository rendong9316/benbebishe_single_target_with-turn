function test_adaptive_p_ctcv()
% ===========================================================================
% 测试自适应P_ctcv原型 — turn_rate反馈方案
%
% 用法：
%   >> test_adaptive_p_ctcv()
%
% 对比: k=0 (原始固定dwell) vs k=[0.1, 0.5, 1.0, 2.0]
% ===========================================================================

    close all;
    addpath(genpath('.'));

    rng(42);

    key_scenarios = {'right_rate_0p7', 'left_sustained', 'straight'};
    knames = {'right_rate_0p7', 'left_sustained(180s)', 'straight(无转弯)'};
    k_values = [0, 0.1, 0.5, 1.0, 2.0];

    base_params = simulation_params_oracle();

    fprintf('========== Adaptive P_ctcv Test ========== %s', newline);
    fprintf('Seed=42 | Config#51 params | 3 key scenarios x 5 k values%s%s', ...
        newline, newline);
    fprintf('%s', repmat('-', 70, 1));

    rmse_matrix = NaN(numel(key_scenarios), length(k_values));

    for ci = 1:numel(key_scenarios)
        fprintf('%s%s%s', newline, knames{ci}, newline);

        for ki = 1:length(k_values)
            k = k_values(ki);
            params = base_params;
            % Config#51参数
            params.imm_cv_dwell_time_sec = 2500;
            params.imm_ct_dwell_time_sec = 660;
            params.imm_ct_fixed_Q_scale = 5.3;
            params.imm_transient_gain_max = 11.0;
            % 注入k值
            if k > 0
                params.k_turnrate = k;
            end

            try
                result = run(key_scenarios{ci});
                % 需要run支持params传递 — 改为用修改后的config
                rmse = result.errorStats_R1.overall.ukf.rms;
                rmse_matrix(ci, ki) = rmse;

                marker = '';
                if ci == 1 && ki > 1
                    baseline = rmse_matrix(ci, 1);
                    if ~isnan(baseline) && rmse < baseline * 0.98
                        marker = '*IMPROVED*';
                    elseif rmse > baseline * 1.02
                        marker = '*WORSE*';
                    end
                end
                printf '%-4s %-10s RMSE=%.2fkm %s%s', ...
                    ki, num2str(k), rmse, marker, newline;

            catch ME
                printf '%-4s ERROR: %s%s', ki, ME.message(1:40), newline;
            end
        end
        fprintf('%s', repmat('-', 70, 1));
    end

    % ---- 汇总 ----
    fprintf('%s%s%s', newline, repmat('=', 70, 1), newline);
    fprintf('SUMMARY - right_rate_0p7 (seed=42):%s', newline);
    baseline_rmse = rmse_matrix(1, 1);
    for ki = 2:length(k_values)
        if ~isnan(rmse_matrix(1, ki))
            improvement = (baseline_rmse - rmse_matrix(1, ki)) / baseline_rmse * 100;
            printf '%-6s k=%-6s RMSE=%.2fkm (baseline=%.2f) -> %+.1f%%%s', ...
                '', k_values(ki), rmse_matrix(1,ki), baseline_rmse, improvement, newline;
        end
    end
    fprintf('%s', newline);
end
