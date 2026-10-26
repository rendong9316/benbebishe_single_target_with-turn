function run_mewf_ablation(out_name, s_thr)
% run_mewf_ablation — MEWF 四特征边际贡献消融（固定最优配置）
%
% 用法: run_mewf_ablation                       % 默认 S_thr=0.50
%       run_mewf_ablation('ablation.csv', 0.55)
%
% 固定：指派 = hungarian，排序键 = maha（上一轮全场景标定的最优组合）
% 变量：四个特征的权重组合
%
% 消融项说明：
%   full        w=[1 1 1 1]  完整 MEWF
%   noDist      w=[0 1 1 1]  去掉距离证据（E_d）—— 同时关闭 e_d_min 预筛
%   noMaha      w=[1 0 1 1]  去掉马氏证据（E_m）
%   noVel       w=[1 1 0 1]  去掉速度证据（E_v）
%   noCont      w=[1 1 1 0]  去掉连续性证据（E_c）
%   distOnly    w=[1 0 0 0]  仅距离（≈ 加权最近邻）
%   mahaOnly    w=[0 1 0 0]  仅马氏（≈ Sequential 的判定核心）
%
% 作者：任东  2026-10-26

    if nargin < 1 || isempty(out_name), out_name = 'mewf_ablation.csv'; end
    if nargin < 2 || isempty(s_thr),    s_thr    = 0.50; end

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
    if isempty(files), error('缓存为空'); end

    % 消融配置：名称, w_d, w_m, w_v, w_c, 是否关闭 e_d_min
    cfgs = { ...
        'full',     1, 1, 1, 1, false; ...
        'noDist',   0, 1, 1, 1, true;  ...
        'noMaha',   1, 0, 1, 1, false; ...
        'noVel',    1, 1, 0, 1, false; ...
        'noCont',   1, 1, 1, 0, false; ...
        'distOnly', 1, 0, 0, 0, false; ...
        'mahaOnly', 0, 1, 0, 0, true  };

    rows = {};
    t_all = tic;
    for f = 1:numel(files)
        S = load(fullfile(cache_dir, files(f).name));
        R1 = S.trackSnapshots_R1;
        R2 = S.aligned_R2;
        [n_tb, ~, ~, ~, ~] = pairing.build_truth_pairs(R1, R2);
        fprintf('\n[%s seed %d] 共现真目标 %d\n', S.scene, S.seed, n_tb);

        for ci = 1:size(cfgs, 1)
            q = S.params;
            q.mewf_score_thr = s_thr;
            q.mewf_assign    = 'hungarian';
            q.mewf_rank_key  = 'maha';
            q.mewf_w_dist = cfgs{ci, 2};
            q.mewf_w_maha = cfgs{ci, 3};
            q.mewf_w_vel  = cfgs{ci, 4};
            q.mewf_w_cont = cfgs{ci, 5};
            if cfgs{ci, 6}, q.mewf_min_dist_evidence = 0; end
            tic; p = track_matcher_mewf(R1, R2, q); t = toc;
            M = pairing.eval_pairing(p, R1, R2, n_tb);
            fprintf('  %-10s %.2fs | pairs=%-4d WRONG=%-3d miss=%-3d | P=%.3f R=%.3f\n', ...
                cfgs{ci, 1}, t, M.n_pairs, M.n_wrong, M.n_missed, M.precision, M.recall);
            rows{end+1} = {S.scene, S.seed, cfgs{ci, 1}, M.n_pairs, M.n_correct, ...
                M.n_wrong, M.n_missed, M.precision, M.recall, t, n_tb}; %#ok<AGROW>
        end
    end
    fprintf('\n[总耗时] %.1fs\n', toc(t_all));

    % 聚合
    fprintf('\n============ 特征消融汇总（hungarian + maha，S_thr=%.2f）============\n', s_thr);
    fprintf('%-10s %8s %8s %8s %8s %8s %9s %10s\n', ...
        '配置', '误配', '漏配', 'P', 'R', 'F1', '加权代价', 'Δ误配');
    base_w = NaN;
    for ci = 1:size(cfgs, 1)
        sel = rows(cellfun(@(r) strcmp(r{3}, cfgs{ci, 1}), rows));
        w  = mean(cellfun(@(r) r{6}, sel));
        ms = mean(cellfun(@(r) r{7}, sel));
        P  = mean(cellfun(@(r) r{8}, sel));
        R  = mean(cellfun(@(r) r{9}, sel));
        F1 = 2 * P * R / max(P + R, eps);
        C  = 5 * w + ms;
        if strcmp(cfgs{ci, 1}, 'full'), base_w = w; end
        fprintf('%-10s %8.2f %8.2f %8.4f %8.4f %8.4f %9.2f %+10.2f\n', ...
            cfgs{ci, 1}, w, ms, P, R, F1, C, w - base_w);
    end
    fprintf('（Δ误配 = 该配置误配数 - 完整配置误配数，正值表示去掉该特征后误配增加）\n');

    % CSV
    out_csv = fullfile(repo_root, '_audit', out_name);
    fid = fopen(out_csv, 'w');
    fwrite(fid, 'scene,seed,config,n_pairs,n_correct,n_wrong,n_missed,precision,recall,time_s,n_true_both', 'char');
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
