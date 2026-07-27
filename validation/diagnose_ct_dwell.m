function diagnose_ct_dwell()
% ===========================================================================
% 诊断IMM转移驻留问题
%
% 原理：
%   每帧IMM更新后, imm.mu_history(end,:) = [mu_cv; mu_ct; mu_ct_right]'
%   当 CT 概率 > 0.5 时认为滤波器"认为"目标在转弯
%   统计连续CT活跃帧长 -> 与真实转弯时长对比
%
% 使用方法：
%   >> diagnose_ct_dwell('multi_cross', seed=42)
%   >> diagnose_ct_dwell('right_rate_0p7', seed=42)
% ===========================================================================

    close all;
    addpath(genpath('.'));

    % ---- 定义场景 ----
    scene_list = { ...
        'single_straight', 'left_short', 'right_short', ...
        'left_sustained', 'right_sustained', 'right_rate_0p7' };
    scene_info = { ...
        ['straight (2426s, 无转弯)'], ...
        ['left_short (47s转弯)'], ...
        ['right_short (47s转弯)'], ...
        ['left_sustained (180s左转)'], ...
        ['right_sustained (180s右转)'], ...
        ['right_rate_0p7 (257s右缓转)'] ...
    };
    expected_turn_sec = [0, 47, 47, 180, 180, 257];
    dt = 30;  % 采样间隔秒

    % ---- 固定种子跑仿真 ----
    rng(42);

    fprintf('========== IMM Dwell Diagnosis ==========%s', newline);
    fprintf('Seed = 42 | dt = %.0fs | %d scenarios%s', ...
        dt, length(scene_list), newline);
    fprintf('%s', repmat('-', 1, 72));

    all_stats = struct();

    for s = 1:length(scene_list)
        fprintf('%s%s%s', newline, scene_info{s}, newline);
        fprintf('%s', repmat('-', 1, 72));

        try
            result = run(scene_list{s});

            % ---- 提取全局 mu_history ----
            n_frames = size(result.trackSnapshots_R1, 1);
            all_mu = [];  % [n_frames x 3]

            for k = 1:n_frames
                snaps_R1 = result.trackSnapshots_R1{k};
                snaps_R2 = result.trackSnapshots_R2{k};

                % 取R1中最长航迹的mu_history
                [mu_k, ~] = get_max_track_mu(snaps_R1);
                if ~isempty(mu_k) && size(mu_k, 1) >= k
                    all_mu(k, :) = mu_k(k, :);
                else
                    % 如果没有跟踪到的航迹，填充NaN
                    all_mu(k, :) = NaN;
                end
            end

            % ---- 分析 ----
            pct_active = nanmean(all_mu(:, 2) + all_mu(:, 3)) * 100;
            mean_p_ct = nanmean(all_mu(:, 2:3));

            % 找CT活跃段（CT+CTright > 0.5）
            ct_active = sum(all_mu(:, 2:3), 2) > 0.5;
            runs = find_runs(ct_active);

            avg_run = mean(runs.lengths) * dt;
            med_run = median(runs.lengths) * dt;
            max_run = max(runs.lengths) * dt;
            n_runs = numel(runs.lengths);

            fprintf('  Mu avg [CV=%.3f, CT=%.3f, CR=%.3f]%s', ...
                nanmean(all_mu(:,1)), nanmean(all_mu(:,2)), ...
                nanmean(all_mu(:,3)), newline);
            fprintf('  CT(pct): active=%.1f%% of frames, mean_p_ct=%.3f%s', ...
                pct_active, mean_p_ct, newline);
            fprintf('  Runs: n=%d, avg=%.0fs, median=%.0fs, max=%.0fs%s', ...
                n_runs, avg_run, med_run, max_run, newline);
            fprintf('  Expected turn: %.0fs%s', expected_turn_sec(s), newline);

            all_stats(s).pct_active = pct_active;
            all_stats(s).mean_p_ct = mean_p_ct;
            all_stats(s).avg_run_sec = avg_run;
            all_stats(s).median_run_sec = med_run;
            all_stats(s).max_run_sec = max_run;
            all_stats(s).n_runs = n_runs;
            all_stats(s).expected_turn = expected_turn_sec(s);
            all_stats(s).has_enough_ct = med_run >= expected_turn_sec(s) * 0.8;

        catch ME
            fprintf('  ERROR: %s%s', ME.message, newline);
        end
    end

    % ---- 汇总表格 ----
    fprintf('%s%s%s', newline, repmat('=', 1, 72), newline);
    fprintf('SUMMARY:%s', newline);
    printf '%-20s %-8s %-8s %-8s %-8s %-8s %-10s\n', ...
        'Scenario', 'Act%', 'μ(CT)', 'AvgS', 'MedS', 'MaxS', 'Fail?';
    for s = 1:length(scene_list)
        s_name = scene_info{s}(1:min(strfind(scene_info{s}, '(')-2, 1)-1);
        fail = '~';
        if all_stats(s).expected_turn > 0 && ...
           all_stats(s).median_run_sec < all_stats(s).expected_turn * 0.5
            fail = '*YES*';
        end
        printf '%-20s %-8.1f %-8.3f %-8.0f %-8.0f %-8.0f %-10s\n', ...
            s_name, all_stats(s).pct_active, all_stats(s).mean_p_ct, ...
            all_stats(s).avg_run_sec, all_stats(s).median_run_sec, ...
            all_stats(s).max_run_sec, fail;
    end
    fprintf('%s* = CT运行明显短于转弯时长，说明驻留时间不足%s%s', newline, newline, newline);
end


function [mu_last, idx] = get_max_track_mu(snapshots)
    % 从逐帧快照中找到mu_history最长的航迹
    best_n = 0; mu_best = []; best_idx = 0;
    for i = 1:length(snapshots)
        snap = snapshots{i};
        if isempty(snap) || ~isfield(snap, 'trackList')
            continue;
        end
        for j = 1:length(snap.trackList)
            trk = snap.trackList{j};
            if isfield(trk, 'ukf') && isfield(trk.ukf, 'mu_history')
                h = trk.ukf.mu_history;
                if ~isempty(h) && size(h, 1) > best_n
                    best_n = size(h, 1);
                    mu_best = h;
                    best_idx = j;
                end
            end
        end
    end
    mu_last = mu_best;
    idx = best_idx;
end

function runs = find_runs(logical_vec)
    % 找出连续true段的长度
    runs = struct('lengths', {}, 'start', {}, 'end', {});
    in_run = false;
    start_idx = 0;
    count = 0;
    for i = 1:length(logical_vec)
        if logical_vec(i)
            if ~in_run
                in_run = true;
                start_idx = i;
            end
            count = count + 1;
        else
            if in_run
                count = count + 1;
                runs.end(end+1) = start_idx + count - 2;
                runs.start(end+1) = start_idx;
                runs.lengths(end+1) = count - 1;
                in_run = false;
                count = 0;
            end
        end
    end
    if in_run
        count = count + 1;
        runs.end(end+1) = start_idx + count - 1;
        runs.start(end+1) = start_idx;
        runs.lengths(end+1) = count;
    end
    if isempty(runs.lengths)
        runs.lengths = [];
        runs.start = [];
        runs.end = [];
    end
end
