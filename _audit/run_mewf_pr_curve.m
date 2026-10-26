function run_mewf_pr_curve(out_name, s_list)
% run_mewf_pr_curve — 各特征配置的 P-R 曲线（每个配置扫自己的 S_thr）
%
% 【为什么需要】上一轮消融固定 S_thr=0.50 比较不同特征组合，但 S 是「加权
%   平均」，去掉特征后分母变化，同一个 S_thr 对应的严格程度并不相同：
%     full    S = (E_d+E_m+E_v+E_c)/4
%     noVel   S = (E_d+E_m+E_c)/3
%   若 E_v 通常接近 1，则 full 的 S 被抬高、等效门限更松，单点比较会误判
%   「特征有无贡献」。正确做法是让每个配置扫自己的 S_thr，得到完整 P-R
%   曲线后比较支配关系与最优点。
%
% 用法: run_mewf_pr_curve
%
% 作者：任东  2026-10-26

    if nargin < 1 || isempty(out_name), out_name = 'mewf_pr_curve.csv'; end
    if nargin < 2 || isempty(s_list),   s_list   = [0.40 0.45 0.50 0.55 0.60 0.70]; end

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

    % 配置：名称, w_d, w_m, w_v, w_c, 关闭 e_d_min
    cfgs = { ...
        'full',     1, 1, 1, 1, false; ...
        'noDist',   0, 1, 1, 1, true;  ...
        'noMaha',   1, 0, 1, 1, false; ...
        'noVel',    1, 1, 0, 1, false; ...
        'noCont',   1, 1, 1, 0, false };

    % 先把 5 个场景全部载入内存，避免每个配置重复 load（6.4s/次）
    % 注意：不能用 struct('R1', cellArray, ...) —— struct 构造函数会把 cell
    % 展开成 struct 数组（239 帧 → 239 个元素），导致后续 SC{f}.R1 变成
    % 逗号分隔列表、调用时"输入参数太多"。必须逐字段赋值。
    fprintf('载入缓存 ...\n');
    SC = cell(numel(files), 1);
    for f = 1:numel(files)
        S = load(fullfile(cache_dir, files(f).name));
        [n_tb, ~, ~, ~, ~] = pairing.build_truth_pairs(S.trackSnapshots_R1, S.aligned_R2);
        e = struct();
        e.R1     = S.trackSnapshots_R1;
        e.R2     = S.aligned_R2;
        e.params = S.params;
        e.n_tb   = n_tb;
        e.tag    = sprintf('%s_%d', S.scene, S.seed);
        SC{f} = e;
        fprintf('  %s (n_true_both=%d)\n', e.tag, n_tb);
    end

    rows = {};
    t_all = tic;
    for ci = 1:size(cfgs, 1)
        for si = 1:numel(s_list)
            W = 0; MS = 0; P = 0; R = 0;
            for f = 1:numel(SC)
                q = SC{f}.params;
                q.mewf_score_thr = s_list(si);
                q.mewf_assign    = 'hungarian';
                q.mewf_rank_key  = 'maha';
                q.mewf_w_dist = cfgs{ci, 2};
                q.mewf_w_maha = cfgs{ci, 3};
                q.mewf_w_vel  = cfgs{ci, 4};
                q.mewf_w_cont = cfgs{ci, 5};
                if cfgs{ci, 6}, q.mewf_min_dist_evidence = 0; end
                % 必须先取到局部变量：struct 字段若是 cell，直接当实参会
                % 展开成逗号分隔列表（"输入参数太多"）
                R1 = SC{f}.R1;
                R2 = SC{f}.R2;
                p = track_matcher_mewf(R1, R2, q);
                M = pairing.eval_pairing(p, R1, R2, SC{f}.n_tb);
                W = W + M.n_wrong; MS = MS + M.n_missed;
                P = P + M.precision; R = R + M.recall;
            end
            n = numel(SC);
            W = W / n; MS = MS / n; P = P / n; R = R / n;
            F1 = 2 * P * R / max(P + R, eps);
            C = 5 * W + MS;
            rows{end+1} = {cfgs{ci, 1}, s_list(si), W, MS, P, R, F1, C}; %#ok<AGROW>
            fprintf('  %-8s S=%.2f | 误配 %.2f 漏配 %.2f | P=%.4f R=%.4f F1=%.4f 代价=%.2f\n', ...
                cfgs{ci, 1}, s_list(si), W, MS, P, R, F1, C);
        end
        fprintf('  --- %s 完成 (%.0fs) ---\n', cfgs{ci, 1}, toc(t_all));
    end
    fprintf('\n[总耗时] %.1fs\n', toc(t_all));

    % 每个配置的最优点（按加权代价最小）
    fprintf('\n============ 各配置最优工作点（按加权代价 5×误配+漏配）============\n');
    fprintf('%-10s %6s %8s %8s %8s %8s %8s %9s\n', ...
        '配置', 'S_thr', '误配', '漏配', 'P', 'R', 'F1', '加权代价');
    for ci = 1:size(cfgs, 1)
        sel = rows(cellfun(@(r) strcmp(r{1}, cfgs{ci, 1}), rows));
        costs = cellfun(@(r) r{8}, sel);
        [~, bi] = min(costs);
        r = sel{bi};
        fprintf('%-10s %6.2f %8.2f %8.2f %8.4f %8.4f %8.4f %9.2f\n', ...
            r{1}, r{2}, r{3}, r{4}, r{5}, r{6}, r{7}, r{8});
    end

    % CSV
    out_csv = fullfile(repo_root, '_audit', out_name);
    fid = fopen(out_csv, 'w');
    fwrite(fid, 'config,s_thr,avg_wrong,avg_missed,avg_precision,avg_recall,avg_F1,avg_cost', 'char');
    fwrite(fid, sprintf('\n'), 'char');
    for k = 1:numel(rows)
        r = rows{k};
        line = sprintf('%s,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.2f', ...
            r{1}, r{2}, r{3}, r{4}, r{5}, r{6}, r{7}, r{8});
        fwrite(fid, line, 'char'); fwrite(fid, sprintf('\n'), 'char');
    end
    fclose(fid);
    fprintf('\n结果已保存: %s\n', out_csv);
end
