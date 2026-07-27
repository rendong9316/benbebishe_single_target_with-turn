function quick_verify_adaptive()
% ===========================================================================
% 快速验证自适应P_ctcv: k=0 vs k=1.0 对比single_straight场景
%
% 用法:
%   >> quick_verify_adaptive()
% ===========================================================================
    close all;
    addpath(genpath('.'));
    rng(42);

    base_params = simulation_params_oracle();
    base_params.imm_cv_dwell_time_sec = 2500;
    base_params.imm_ct_dwell_time_sec = 660;
    base_params.imm_ct_fixed_Q_scale = 5.3;
    base_params.imm_transient_gain_max = 11.0;

    fprintf('========== Quick Verify: Adaptive P_ctcv ========== %s', newline);
    fprintf('Scenario: single_straight | Seed=42%s', newline);

    k_values = [0, 1.0];
    for test_k = k_values
        fprintf('%s--- k = %.1f ---%s', newline, test_k, newline);

        % 临时覆盖simulation_params_oracle的param
        params = base_params;
        if test_k > 0
            params.k_turnrate = test_k;
        end

        % Hack: 把params写入workspace再eval run.m
        % 更好的方式: 直接在run.m的Phase 0之后插入params重载
        try
            % 执行仿真
            result = run('single_straight');
            rmse_r1 = result.errorStats_R1.overall.ukf.rms;
            rmse_r2 = result.errorStats_R2.overall.ukf.rms;
            n_tracks_r1 = length(result.trackList_R1);
            n_tracks_r2 = length(result.trackList_R2);

            fprintf('  RMSE R1=%.2fkm R2=%.2fkm | Tracks R1=%d R2=%d%s', ...
                rmse_r1, rmse_r2, n_tracks_r1, n_tracks_r2, newline);

            % 打印mu_history最后10帧看看CT占比变化
            for k_check = min(length(result.trackSnapshots_R1), 5):length(result.trackSnapshots_R1)
                snaps = result.trackSnapshots_R1{k_check};
                for t = 1:min(length(snaps), 5)
                    trk = snaps(t).trackList{t};
                    if isfield(trk, 'ukf') && isfield(trk.ukf, 'mu_history')
                        mu_h = trk.ukf.mu_history;
                        last_row = mu_h(end, :);
                        ct_pct = sum(last_row(2:3)) * 100;
                        if ~isnan(ct_pct)
                            fprintf('  Frame %d: mu=[%.3f %.3f %.3f] CT_p=%.1f%% final_p_ctcv=%.4f', ...
                                k_check, last_row(1), last_row(2), last_row(3), ...
                                ct_pct, trk.ukf.Pi(2,1));
                            if test_k > 0
                                fprintf(' (adaptive!)');
                            end
                            fprintf('%s', newline);
                            break;
                        end
                    end
                end
            end

        catch ME
            fprintf('  ERROR: %s%s', ME.message(1:300), newline);
        end
    end
    fprintf('%sDone.%s', newline, newline);
end
