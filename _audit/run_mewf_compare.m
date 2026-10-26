function run_mewf_compare(out_name, s_thr_list)
% run_mewf_compare — MEWF vs NN / Sequential / 三判据 全面对比（从缓存，跳过跟踪）
%
% 用法：
%   run_mewf_compare
%   run_mewf_compare('mewf_compare.csv', [0.50 0.55 0.60 0.65])
%
% 依赖 _audit/cache/*.mat（由 cache_snapshots 生成）。
%
% 对比设计（三层）：
%   【基线】NN / Sequential / DualGate_M5 / DualGate_M8
%   【MEWF 主方案】greedy + 马氏排序，S_thr 扫描
%   【MEWF 消融】排序键（score/dist/maha）、指派方式（hungarian）、
%                特征（去速度 / 去连续性）
%
% 作者：任东  2026-10-26

    if nargin < 1 || isempty(out_name),   out_name   = 'mewf_compare.csv'; end
    if nargin < 2 || isempty(s_thr_list), s_thr_list = [0.50 0.55 0.60 0.65]; end

    repo_root = fileparts(fileparts(mfilename('fullpath')));
    addpath(repo_root);
    code_dirs = {'config','simulation','ukf','tracker','fusion','evaluation', ...
                 'utils','visualization','validation','experiments','io'};
    for cd_i = 1:numel(code_dirs)
        d_i = fullfile(repo_root, code_dirs{cd_i});
        if exist(d_i, 'dir'), addpath(d_i); end
    end

    cache_dir = fullfile(repo_root, '_audit', 'cache');
    files = dir(fullfile(cache_dir, '*.mat'));
    if isempty(files)
        error('缓存为空：请先运行 cache_snapshots(''dense'')');
    end

    rows = {};
    t_all = tic;

    for f = 1:numel(files)
        S = load(fullfile(cache_dir, files(f).name));
        R1 = S.trackSnapshots_R1;
        R2 = S.aligned_R2;
        [n_tb, ~, ~, ~, ~] = pairing.build_truth_pairs(R1, R2);
        fprintf('\n[%s seed %d] 帧 %d | 共现真目标 %d\n', S.scene, S.seed, S.n_frames, n_tb);
        bp = S.params;

        % ================= 基线 =================
        tic; p = track_matcher_nn(R1, R2, bp); t = toc;
        M = pairing.eval_pairing(p, R1, R2, n_tb);
        rows = push(rows, S, 'NN', M, t, n_tb); log_line('NN', '', M, t);

        tic; p = track_matcher_sequential(R1, R2, bp); t = toc;
        M = pairing.eval_pairing(p, R1, R2, n_tb);
        rows = push(rows, S, 'Sequential', M, t, n_tb); log_line('Sequential', '', M, t);

        for Mv = [5 8]
            q = bp; q.dualgate_M = Mv;
            tic; p = track_matcher_dualgate_fast(R1, R2, q); t = toc;
            M = pairing.eval_pairing(p, R1, R2, n_tb);
            rows = push(rows, S, sprintf('DualGate_M%d', Mv), M, t, n_tb);
            log_line('DualGate', sprintf('M=%d ', Mv), M, t);
        end

        % ================= MEWF 主方案：greedy + maha，S_thr 扫描 =================
        for si = 1:numel(s_thr_list)
            q = bp;
            q.mewf_score_thr = s_thr_list(si);
            q.mewf_assign   = 'greedy';
            q.mewf_rank_key = 'maha';
            tic; p = track_matcher_mewf(R1, R2, q); t = toc;
            M = pairing.eval_pairing(p, R1, R2, n_tb);
            rows = push(rows, S, sprintf('MEWF_S%.2f', s_thr_list(si)), M, t, n_tb);
            log_line('MEWF', sprintf('S=%.2f ', s_thr_list(si)), M, t);
        end

        % ================= MEWF 消融（固定中间 S_thr）=================
        s_mid = s_thr_list(ceil(numel(s_thr_list) / 2));
        variants = { ...
            'MEWF_rankScore', 'mewf_rank_key', 'score',      'greedy',    '排序键=综合分'; ...
            'MEWF_rankDist',  'mewf_rank_key', 'dist',       'greedy',    '排序键=欧氏距离'; ...
            'MEWF_hungarian', 'mewf_assign',   'hungarian',  'hungarian', '指派=匈牙利'; ...
            'MEWF_noVel',     'mewf_w_vel',    0,            'greedy',    '去速度特征'; ...
            'MEWF_noCont',    'mewf_w_cont',   0,            'greedy',    '去连续性特征' };
        for vi = 1:size(variants, 1)
            q = bp;
            q.mewf_score_thr = s_mid;
            q.mewf_assign   = variants{vi, 4};
            q.mewf_rank_key = 'maha';
            q.(variants{vi, 2}) = variants{vi, 3};
            tic; p = track_matcher_mewf(R1, R2, q); t = toc;
            M = pairing.eval_pairing(p, R1, R2, n_tb);
            rows = push(rows, S, variants{vi, 1}, M, t, n_tb);
            log_line(variants{vi, 1}, variants{vi, 5}, M, t);
        end
    end

    fprintf('\n[总耗时] %.1fs（%d 场景）\n', toc(t_all), numel(files));

    % ================= 聚合汇总 =================
    fprintf('\n==================== 方法对比汇总（按场景平均）====================\n');
    fprintf('%-16s %8s %8s %8s %8s %8s %9s\n', ...
        '方法', '误配', '漏配', 'P', 'R', 'F1', '加权代价');
    methods = unique(cellfun(@(r) r{3}, rows, 'UniformOutput', false));
    for mi = 1:numel(methods)
        sel = rows(cellfun(@(r) strcmp(r{3}, methods{mi}), rows));
        w  = mean(cellfun(@(r) r{6}, sel));
        ms = mean(cellfun(@(r) r{7}, sel));
        P  = mean(cellfun(@(r) r{8}, sel));
        R  = mean(cellfun(@(r) r{9}, sel));
        F1 = 2 * P * R / max(P + R, eps);
        C  = 5 * w + ms;
        fprintf('%-16s %8.2f %8.2f %8.4f %8.4f %8.4f %9.2f\n', ...
            methods{mi}, w, ms, P, R, F1, C);
    end

    % ================= CSV =================
    out_csv = fullfile(repo_root, '_audit', out_name);
    fid = fopen(out_csv, 'w');
    fwrite(fid, 'scene,seed,method,n_pairs,n_correct,n_wrong,n_missed,precision,recall,time_s,n_true_both', 'char');
    fwrite(fid, sprintf('\n'), 'char');
    for k = 1:numel(rows)
        rw = rows{k};
        line = sprintf('%s,%d,%s,%d,%d,%d,%d,%.4f,%.4f,%.3f,%d', ...
            rw{1}, rw{2}, rw{3}, rw{4}, rw{5}, rw{6}, rw{7}, rw{8}, rw{9}, rw{10}, rw{11});
        fwrite(fid, line, 'char'); fwrite(fid, sprintf('\n'), 'char');
    end
    fclose(fid);
    fprintf('\n结果已保存: %s\n', out_csv);
end

% =========================================================================
function rows = push(rows, S, method, M, t, n_tb)
    rows{end+1} = {S.scene, S.seed, method, M.n_pairs, M.n_correct, ...
                   M.n_wrong, M.n_missed, M.precision, M.recall, t, n_tb}; %#ok<AGROW>
end

function log_line(name, extra, M, t)
    fprintf('  %-16s %-12s %.2fs | pairs=%-4d corr=%-4d WRONG=%-3d miss=%-3d | P=%.3f R=%.3f\n', ...
        name, extra, t, M.n_pairs, M.n_correct, M.n_wrong, M.n_missed, M.precision, M.recall);
end
