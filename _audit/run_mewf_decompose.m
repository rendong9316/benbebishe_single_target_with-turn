function run_mewf_decompose(out_name)
% run_mewf_decompose — 拆解 MEWF 优势来源（关键对照实验）
%
% 【为什么必须做这个实验】
%   MEWF 相对 Sequential 有两处改动：① 软门限（距离证据补偿马氏）替代
%   硬马氏通过率门限；② 匈牙利全局指派替代贪心。审稿/答辩必然追问：
%   「你的改进是不是就是把门限调松了 + 换了个指派算法？」
%
%   本实验构造完整 2×N 对照：
%     Sequential 的马氏门限 ratio 与指派方式正交组合，
%     其中 ratio→0.3 相当于「几乎不放行马氏门限」的极限松门限，
%     用于检验 MEWF 的「距离证据补偿」是否还有独立价值。
%
% 判据：
%   若 MEWF 优于 Seq_r0.3_hung，说明软门限设计（用距离证据补偿马氏）有
%   独立贡献，不是单纯放松门限；
%   若两者相当，则须诚实改写贡献表述为「软门限等价于自适应松门限 + 全局指派」。
%
% 作者：任东  2026-10-26

    if nargin < 1 || isempty(out_name), out_name = 'mewf_decompose.csv'; end

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

    % 对照配置：名称, 类型, 参数1, 参数2
    %   'seq'  : 参数1 = maha_ratio, 参数2 = assign
    %   'mewf' : 参数1 = S_thr,      参数2 = assign（固定 maha 排序）
    cfgs = { ...
        'Seq_r0.80_greedy', 'seq',  0.80, 'greedy'; ...
        'Seq_r0.50_greedy', 'seq',  0.50, 'greedy'; ...
        'Seq_r0.30_greedy', 'seq',  0.30, 'greedy'; ...
        'Seq_r0.80_hung',   'seq',  0.80, 'hungarian'; ...
        'Seq_r0.50_hung',   'seq',  0.50, 'hungarian'; ...
        'Seq_r0.30_hung',   'seq',  0.30, 'hungarian'; ...
        'MEWF_S0.40',       'mewf', 0.40, 'hungarian'; ...
        'MEWF_S0.45',       'mewf', 0.45, 'hungarian' };

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
            switch cfgs{ci, 2}
                case 'seq'
                    q.dualgate_maha_ratio = cfgs{ci, 3};
                    q.seq_assign          = cfgs{ci, 4};
                    tic; p = track_matcher_sequential(R1, R2, q); t = toc;
                otherwise
                    q.mewf_score_thr = cfgs{ci, 3};
                    q.mewf_assign    = cfgs{ci, 4};
                    q.mewf_rank_key  = 'maha';
                    tic; p = track_matcher_mewf(R1, R2, q); t = toc;
            end
            M = pairing.eval_pairing(p, R1, R2, n_tb);
            fprintf('  %-18s %.2fs | pairs=%-4d WRONG=%-3d miss=%-3d | P=%.3f R=%.3f\n', ...
                cfgs{ci, 1}, t, M.n_pairs, M.n_wrong, M.n_missed, M.precision, M.recall);
            rows{end+1} = {S.scene, S.seed, cfgs{ci, 1}, M.n_pairs, M.n_correct, ...
                M.n_wrong, M.n_missed, M.precision, M.recall, t, n_tb}; %#ok<AGROW>
        end
    end
    fprintf('\n[总耗时] %.1fs\n', toc(t_all));

    % 聚合
    fprintf('\n============ 优势来源拆解（密集 5 组平均）============\n');
    fprintf('%-18s %8s %8s %8s %8s %8s %9s\n', ...
        '配置', '误配', '漏配', 'P', 'R', 'F1', '加权代价');
    for ci = 1:size(cfgs, 1)
        sel = rows(cellfun(@(r) strcmp(r{3}, cfgs{ci, 1}), rows));
        w  = mean(cellfun(@(r) r{6}, sel));
        ms = mean(cellfun(@(r) r{7}, sel));
        P  = mean(cellfun(@(r) r{8}, sel));
        R  = mean(cellfun(@(r) r{9}, sel));
        F1 = 2 * P * R / max(P + R, eps);
        C  = 5 * w + ms;
        fprintf('%-18s %8.2f %8.2f %8.4f %8.4f %8.4f %9.2f\n', ...
            cfgs{ci, 1}, w, ms, P, R, F1, C);
    end

    % 关键对照结论
    fprintf('\n---- 关键判据 ----\n');
    get = @(nm) cellfun(@(r) [r{6}, r{7}, r{8}, r{9}], ...
        rows(cellfun(@(r) strcmp(r{3}, nm), rows)), 'UniformOutput', false);
    a = cell2mat(get('Seq_r0.30_hung'));
    b = cell2mat(get('MEWF_S0.40'));
    wa = mean(a(:,1)); ma = mean(a(:,2)); Pa = mean(a(:,3)); Ra = mean(a(:,4));
    wb = mean(b(:,1)); mb = mean(b(:,2)); Pb = mean(b(:,3)); Rb = mean(b(:,4));
    fprintf('  Seq_r0.30_hung : 误配 %.2f 漏配 %.2f P=%.4f R=%.4f\n', wa, ma, Pa, Ra);
    fprintf('  MEWF_S0.40     : 误配 %.2f 漏配 %.2f P=%.4f R=%.4f\n', wb, mb, Pb, Rb);
    if Pb >= Pa && Rb >= Ra
        fprintf('  => MEWF 在 P/R 上不劣于「极限松门限+匈牙利」，软门限设计有独立价值\n');
    else
        fprintf('  => MEWF 未占优：需诚实表述为「软门限≈自适应松门限 + 全局指派」\n');
    end

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
