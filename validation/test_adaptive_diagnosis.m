function test_adaptive_diagnosis()
% ===========================================================================
% 诊断自适应P_ctcv是否生效
% 对比 k=0 (fixed) vs k=[0.5, 1.0] 在不同场景下的mu_history和RMSE
% ===========================================================================
    close all;
    addpath(genpath('.'));
    rng(42);

    base_params = simulation_params_oracle();
    base_params.imm_cv_dwell_time_sec = 2500;
    base_params.imm_ct_dwell_time_sec = 660;
    base_params.imm_ct_fixed_Q_scale = 5.3;
    base_params.imm_transient_gain_max = 11.0;

    scenarios = {'single_straight', 'single_turn_right_short'};
    snames = {'straight(无转弯)', 'right_short(右转47s)'};
    k_vals = [0, 0.5, 1.0];

    fprintf('========== Adaptive P_ctcv Diagnosis ========== %s', newline);
    fprintf('Config#51 params | Seed=42 | 2 scenarios x 3 k values%s%s', ...
        newline, newline);

    for si = 1:length(scenarios)
        fprintf('%s--- %s ---%s', newline, snames{si}, newline);
        fprintf('%s', repmat('-', 60, 1));

        for ki = 1:length(k_vals)
            k = k_vals(ki);
            params = base_params;
            if k > 0
                params.k_turnrate = k;
            end

            try
                result = run(scenarios{si});
                rmse_r1 = result.errorStats_R1.overall.ukf.rms;
                n_frames = size(result.trackSnapshots_R1, 1);

                % 从R1快照提取CT概率统计
                ct_pcts = [];
                for fk = 1:min(n_frames, 20)  % 只看前20帧(加速)
                    snaps = result.trackSnapshots_R1{fk};
                    for ti = 1:min(length(snaps), 3)
                        trk = snaps(ti).trackList{ti};
                        if isfield(trk, 'ukf') && isfield(trk.ukf, 'mu_history')
                            mh = trk.ukf.mu_history;
                            % mu_history长度可能不足n_frames（新航迹起始滞后）
                            if ~isempty(mh) && size(mh, 1) >= fk
                                mu_row = mh(fk, :);
                                ct_p = sum(mu_row(2:3)) * 100;
                                if ~isnan(ct_p) && ct_p > 0
                                    ct_pcts(end+1) = ct_p;
                                end
                            end
                        end
                    end
                end

                avg_ct = mean(ct_pcts) * 100;
                max_ct = max(ct_pcts) * 100;
                n_ct_frames = length(ct_pcts);

                fprintf('  k=%.1f  RMSE=%.2fkm  avg_CT_p=%.1f%%  max_CT_p=%.1f%%  frames_CT>0=%d/%d%s', ...
                    k, rmse_r1, avg_ct, max_ct, n_ct_frames, n_frames, newline);

            catch ME
                fprintf('  k=%.1f  ERROR: %.50s%s', k, ME.message, newline);
            end
        end
        fprintf('%s', repmat('-', 60, 1));
    end
    fprintf('%sDone.%s', newline, newline);
end
