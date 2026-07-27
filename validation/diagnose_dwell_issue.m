function diagnose_dwell_issue()
% ===========================================================================
% 诊断IMM转移驻留问题 — 记录mu_history，统计CT活跃帧长分布
%
% 原理：
%   每帧IMM更新后，imm.mu_history(end,:) = mu' (CV/CT/CT-right概率)
%   如果 CT 概率 > 0.5 说明当前"认为"目标在转弯
%   统计连续CT帧长 -> 与真实转弯时长对比 -> 验证驻留时间是否匹配
%
% 使用方法：
%   需要先用 run('multi_cross') 生成仿真结果
%   run_diagnose_from_saved_data() 自动加载并分析
% ===========================================================================

    fprintf('========== IMM Dwell Issue Diagnosis ==========%s', newline);
    fprintf('Scenarios: single_straight, left_short, right_short,');
    fprintf(' left_sustained, right_sustained, right_rate_0p7%s', newline);

    % ---- 定义要诊断的场景 ----
    scenarios = {'single_straight', 'left_short', 'right_short', ...
                 'left_sustained', 'right_sustained', 'right_rate_0p7'};

    scene_meta = struct( ...
        'name', { ...
            'straight', 'left_short', 'right_short', ...
            'left_sustained', 'right_sustained', 'right_rate_0p7' }, ...
        'duration_sec', [2426, 47, 47, 180, 180, 3783], ...
        'has_turn', {false, true, true, true, true, true}, ...
        'turn_dir', {'-', 'L', 'R', 'L', 'R', 'R'}, ...
        'turn_duration_sec', [0, 47, 47, 180, 180, 257] ...
    );

    results_all = struct();
    for s = 1:length(scenarios)
        fprintf('%sRunning scenario %d/%d: %s...%s', ...
            newline, s, length(scenarios), scene_meta(s).name, newline);
        try
            result = run(scenarios{s});
            results_all{s} = result;

            [ct_stats, overall_stats] = analyze_ct_dwell(result, scenario_meta(s));
            fprintf('  CT stats: avg_run=%.0f frames (%.0fs), median_run=%.0f frames (%.0fs)%s', ...
                ct_stats.mean_run_frames, ct_stats.mean_run_frames*0.3, ...
                ct_stats.median_run_frames, ct_stats.median_run_frames*0.3, newline);
            fprintf('  Overall: P(CT)>0.5 for %.1f%% of frames, mean_p_ct=%.3f%s', ...
                overall_stats.pct_ct_active, overall_stats.mean_p_ct, newline);
        catch ME
            fprintf('  ERROR: %s%s', ME.message, newline);
            results_all{s} = [];
        end
    end
end

function [ct_stats, overall_stats] = analyze_ct_dwell(result, scene_info)
    % 从 result 中提取每帧的 mu_history 并分析
    n_frames = size(result.trackSnapshots_R1, 1);
    n_tracks_total = 0;
    ct_runs = [];
    p_ct_values = [];
    n_ct_active = 0;

    for k = 1:n_frames
        snapshots_R1 = result.trackSnapshots_R1{k};
        snapshots_R2 = result.trackSnapshots_R2{k};
        all_snaps = [snapshots_R1(:); snapshots_R2(:)];

        for i = 1:length(all_snaps)
            snap = all_snaps{i};
            if isempty(snap) || ~isfield(snap, 'trackList')
                continue;
            end
            for j = 1:length(snap.trackList)
                trk = snap.trackList{j};
                if isfield(trk, 'ukf') && isfield(trk.ukf, 'mu_history')
                    mu_h = trk.ukf.mu_history;
                    if ~isempty(mu_h) && size(mu_h, 1) >= k
                        mu_row = mu_h(k, :);
                        if numel(mu_row) >= 3
                            p_ct_values(end+1) = mu_row(2) + mu_row(3);  % CT + CT-right
                            if p_ct_values(end) > 0.5
                                n_ct_active = n_ct_active + 1;
                            end
                        end
                    end
                end
            end
        end
    end
end
