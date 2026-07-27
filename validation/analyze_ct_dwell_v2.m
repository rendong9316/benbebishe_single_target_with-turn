function analyze_ct_dwell_v2()
% ===========================================================================
% v2: 使用 trackList_R1/R2（完整ukf结构）而非 snapshot
% trackList保留ukf.mu_history等完整字段
% ===========================================================================
    close all;
    addpath(genpath('.'));
    rng(42);

    base_params = simulation_params_oracle();
    base_params.imm_cv_dwell_time_sec = 2500;
    base_params.imm_ct_dwell_time_sec = 660;
    base_params.imm_ct_fixed_Q_scale = 5.3;
    base_params.imm_transient_gain_max = 11.0;

    n_scenarios = 2;
    scenario_names{n_scenarios,1} = {'single_straight'; 'single_turn_right_short'};
    scene_labels{n_scenarios,1} = {'straight(无转弯)'; 'right_short(右转47s)'};
    expected_turns(n_scenarios,1) = [0; 47];

    fprintf('========== Observed Dwell Analysis V2 ========== %s', newline);
    fprintf('Config#51 | Seed=42 | NO SOURCE CODE CHANGES%s%s', newline, newline);

    for s = 1:n_scenarios
        scenario_name = scenario_names{s};
        scene_label = scene_labels{s};
        expected_turn = expected_turns(s);

        fprintf('%s--- %-18s (exp=%.0fs) ---%s', newline, ...
            scene_label, expected_turn, newline);
        fprintf('%s', repmat('-', 60, 1));

        try
            result = run(scenario_name);
            rmse_r1 = result.errorStats_R1.overall.ukf.rms;

            % 从trackList中提取mu_history
            ct_pcts_all = [];

            % R1 trackList
            for ti = 1:min(length(result.trackList_R1), 3)
                trk = result.trackList_R1{ti};
                if isfield(trk, 'ukf') && isfield(trk.ukf, 'mu_history')
                    mh = trk.ukf.mu_history;
                    if ~isempty(mh) && size(mh,1) >= 3
                        ct_pcts = sum(mh(:,2:3), 2) * 100;
                        valid_idx = ~isnan(ct_pcts(:));
                        ct_pcts_all = [ct_pcts_all; ct_pcts(valid_idx)];
                    end
                end
            end
            % R2 trackList
            for ti = 1:min(length(result.trackList_R2), 3)
                trk = result.trackList_R2{ti};
                if isfield(trk, 'ukf') && isfield(trk.ukf, 'mu_history')
                    mh = trk.ukf.mu_history;
                    if ~isempty(mh) && size(mh,1) >= 3
                        ct_pcts = sum(mh(:,2:3), 2) * 100;
                        valid_idx = ~isnan(ct_pcts(:));
                        ct_pcts_all = [ct_pcts_all; ct_pcts(valid_idx)];
                    end
                end
            end

            if length(ct_pcts_all) == 0
                fprintf('  No mu_history found in trackList!%s', newline);
                continue;
            end

            avg_ct = mean(ct_pcts_all)*100;
            max_ct = max(ct_pcts_all)*100;
            ct_active = sum(ct_pcts_all > 50);
            n_valid = length(ct_pcts_all);

            % 连续段统计
            runs = find_runs_v2(ct_pcts_all > 50);
            run_lens = double(runs) * 30;

            fmt_str = '  RMSE_R1=%.2fkm';
            if length(runs) > 0
                fmt_str = sprintf('%s | avg_CT=%.1f%% max_CT=%.1f%% | active=%d/%d', ...
                    fmt_str, avg_ct, max_ct, ct_active, n_valid);
                if expected_turn > 0 && median(run_lens) > 0
                    status_flag = '';
                    if median(run_lens) >= expected_turn*0.8
                        status_flag = ' [CT驻留够用]';
                    elseif median(run_lens) < expected_turn*0.5
                        status_flag = ' [CT驻留不足!]';
                    else
                        status_flag = ' [边缘]';
                    end
                    fmt_str = sprintf('%s | runs: n=%d avg=%.0fs med=%.0fs max=%.0fs%s', ...
                        fmt_str, length(runs), mean(run_lens), ...
                        median(run_lens), max(run_lens), status_flag);
                end
            end
            fprintf([fmt_str '%s'], rmse_r1, newline);

        catch ME
            fprintf('  ERROR: %s%s', ME.message, newline);
        end
        fprintf('%s', repmat('-', 60, 1));
        fprintf('%s', newline);
    end
end


function r = find_runs_v2(logical_vec)
    r = [];
    count = 0; in_run = false;
    for i = 1:length(logical_vec)
        if logical_vec(i)
            if ~in_run, in_run=true; count=0; end
            count=count+1;
        else
            if in_run, r(end+1)=count; in_run=false; end
        end
    end
    if in_run, r(end+1)=count; end
end
