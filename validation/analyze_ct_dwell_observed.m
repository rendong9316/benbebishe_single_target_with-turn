function analyze_ct_dwell_observed()
% ===========================================================================
% 纯诊断：分析现有仿真中CT占比与固定驻留时间的匹配度
%
% 核心思路：
%   Config#51: imm_ct_dwell=660 → p_ct_cv ≈ 4.4%/帧(固定)
%   → 转弯257s的场景：IMM在CT上平均停留约660s才被强制踢回CV
%   → 如果真实转弯只有47-257s，CT驻留过长/过短导致RMSE恶化
%   → 验证：统计mu_history中CT>0.5的连续帧长分布
% ===========================================================================
    close all;
    addpath(genpath('.'));
    rng(42);

    base_params = simulation_params_oracle();
    base_params.imm_cv_dwell_time_sec = 2500;
    base_params.imm_ct_dwell_time_sec = 660;
    base_params.imm_ct_fixed_Q_scale = 5.3;
    base_params.imm_transient_gain_max = 11.0;

    dt = 30;
    n_scenarios = 4;
    scenario_names = cell(n_scenarios, 1);
    scene_labels = cell(n_scenarios, 1);
    expected_turns = zeros(n_scenarios, 1);

    scenario_names{1} = 'single_straight';       scene_labels{1} = 'straight';         expected_turns(1) = 0;
    scenario_names{2} = 'single_turn_right_short'; scene_labels{2} = 'right_short';    expected_turns(2) = 47;
    scenario_names{3} = 'single_turn_left_sustained'; scene_labels{3} = 'left_sustained'; expected_turns(3) = 180;
    scenario_names{4} = 'multi_cross'; scene_labels{4} = 'multi_cross'; expected_turns(4) = 0;

    fprintf('========== Observed Dwell Analysis ========== %s', newline);
    fprintf('Config#51 params | Seed=42 | NO SOURCE CODE CHANGES%s%s', newline, newline);

    for s = 1:n_scenarios
        scenario_name = scenario_names{s};
        scene_label = scene_labels{s};
        expected_turn = expected_turns(s);

        fprintf('%s--- %-16s (expected_turn=%.0fs) ---%s', newline, ...
            scene_label, expected_turn, newline);

        try
            result = run(scenario_name);
            rmse_r1 = result.errorStats_R1.overall.ukf.rms;
            n_frames = size(result.trackSnapshots_R1, 1);

            % 提取每帧mu_history中的CT概率
            ct_pcts_all = [];
            for fk = 1:n_frames
                snaps = [result.trackSnapshots_R1{fk}(:); result.trackSnapshots_R2{fk}(:)];
                for ti = 1:length(snaps)
                    snap = snaps{ti};
                    if isempty(snap) || ~isfield(snap, 'trackList'), continue; end
                    for tj = 1:min(length(snap.trackList), 3)
                        trk = snap.trackList{tj};
                        if isfield(trk, 'ukf') && isfield(trk.ukf, 'mu_history')
                            mh = trk.ukf.mu_history;
                            if ~isempty(mh) && size(mh,1) >= 3
                                idx = min(fk, size(mh,1));
                                ct_pct = sum(mh(idx,2:3)) * 100;
                                if ~isnan(ct_pct), ct_pcts_all(end+1) = ct_pct; end
                            end
                        end
                    end
                end
            end

            avg_ct = mean(ct_pcts_all)*100;
            max_ct = max(ct_pcts_all)*100;
            ct_active = sum(ct_pcts_all > 50);

            % 连续段统计（用所有可用点，不只是单帧）
            runs = find_runs(ct_pcts_all > 50);
            run_lens = double(runs) * dt;
            if length(run_lens) > 0
                avg_run = mean(run_lens); med_run = median(run_lens); max_run = max(run_lens);
            else
                avg_run=0; med_run=0; max_run=0;
            end

            % 评估匹配度
            status = '';
            if expected_turn > 0 && length(run_lens)>0
                if med_run >= expected_turn*0.8, status = ' OK (CT驻留够用)';
                elseif med_run < expected_turn*0.5, status = ' SHORT! (CT驻留不够)';
                else status = ' MARGINAL'; end
            end

            fprintf('  RMSE_R1=%.2fkm  avg_CT=%.1f%%  max_CT=%.1f%%  frames_active=%d/%d%s%s', ...
                rmse_r1, avg_ct, max_ct, ct_active, n_frames, newline, status);
            fprintf('  CT_runs: n=%d  avg=%.0fs  median=%.0fs  max=%.0fs%s', ...
                length(runs), avg_run, med_run, max_run, newline);

        catch ME
            fprintf('  ERROR: %s%s', ME.message, newline);
        end
    end
    fprintf('%sDone.%s', newline, newline);
end


function r = find_runs(logical_vec)
    % 找连续true段，返回长度为段数的double数组
    r = [];
    count = 0; in_run = false;
    for i = 1:length(logical_vec)
        if logical_vec(i)
            if ~in_run, in_run = true; count = 0; end
            count = count + 1;
        else
            if in_run, r(end+1) = count; in_run = false; end
        end
    end
    if in_run, r(end+1) = count; end
end
