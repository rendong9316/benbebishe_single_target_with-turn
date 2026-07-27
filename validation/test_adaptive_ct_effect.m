function test_adaptive_ct_effect()
    close all;
    addpath(genpath('.'));
    rng(42);

    base_params = simulation_params_oracle();
    base_params.imm_cv_dwell_time_sec = 2500;
    base_params.imm_ct_dwell_time_sec = 660;
    base_params.imm_ct_fixed_Q_scale = 5.3;
    base_params.imm_transient_gain_max = 11.0;

    scenarios = {'single_turn', 'single_turn_right_short'};
    snames = {'turn_120deg', 'right_short'};
    k_vals = [0, 0.5, 1.0];

    fprintf('========== Adaptive P_ctcv Test ========== %s', newline);
    fprintf('Config#51 | Seed=42 | 2 scenarios x 3 k values%s%s', newline, newline);

    rmse_all = NaN(2, 3);

    for si = 1:2
        fprintf('%s--- %s ---%s', newline, snames{si}, newline);
        fprintf('%s', repmat('-', 70, 1));

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

                ct_pcts_last10 = [];
                snaps = result.trackSnapshots_R1;
                for ti = 1:min(length(snaps), 3)
                    trk = snaps{ti}.trackList{1};
                    if isfield(trk, 'ukf') && isfield(trk.ukf, 'mu_history')
                        mh = trk.ukf.mu_history;
                        if ~isempty(mh)
                            last_n = min(10, size(mh, 1));
                            last_rows = mh(end-last_n+1:end, :);
                            ct_rows = sum(last_rows(:, 2:3), 2) * 100;
                            valid_idx = ~isnan(ct_rows(:));
                            ct_pcts_last10 = [ct_pcts_last10; ct_rows(valid_idx)];
                        end
                    end
                end

                avg_ct = mean(ct_pcts_last10) * 100;
                max_ct = max(ct_pcts_last10) * 100;
                n_pos = length(ct_pcts_last10);

                marker = '';
                if ki == 1
                    baseline = rmse_r1;
                else
                    change = (baseline - rmse_r1) / baseline * 100;
                    if abs(change) > 1
                        marker = sprintf(' [%+.1f%%]', change);
                    end
                end

                fprintf('  k=%.1f  RMSE=%.2fkm  CT_avg=%.1f%%  CT_max=%.1f%%  frames_CT>0=%d%s%s', ...
                    k, rmse_r1, avg_ct, max_ct, n_pos, marker, newline);

                rmse_all(si, ki) = rmse_r1;

            catch ME
                fprintf('  k=%.1f  ERROR: %.50s%s', k, ME.message, newline);
                rmse_all(si, ki) = NaN;
            end
        end
        fprintf('%s', repmat('-', 70, 1));
        fprintf('%s', newline);
    end

    fprintf('%s%s%s', newline, repmat('=', 70, 1), newline);
    fprintf('SUMMARY — improvement of adaptive vs fixed (k=0):%s', newline);
    for si = 1:2
        baseline = rmse_all(si, 1);
        if isnan(baseline), continue; end
        s = sprintf('%s:', snames{si});
        for ki = 2:length(k_vals)
            improved = rmse_all(si, ki);
            if ~isnan(improved)
                pct_change = (baseline - improved) / baseline * 100;
                s = sprintf('%s  k=%.1f:%+.1f%%', s, k_vals(ki), pct_change);
            end
        end
        fprintf('  %s%s', s, newline);
    end
end
