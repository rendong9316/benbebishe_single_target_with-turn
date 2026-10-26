function run_mewf_sparse(out_name)
% run_mewf_sparse — 稀疏场景验证 MEWF 是否退化（P0 收尾实验）
%
% 【为什么必须做】密集场景 MEWF 全面胜出，但稀疏场景（adsb_0002 平均 6 架
%   同时在场、adsb_span 4 架）里 NN 就已经 P=R=1.0（零误配零漏配）。若 MEWF
%   在稀疏场景出现漏配，则"全面优于基线"不成立，论文只能限定为"密集场景
%   更优"。本实验给出明确边界。
%
% 对比项：NN / Sequential / DualGate_M5 / DualGate_M8 /
%         MEWF(greedy,maha) / MEWF(hungarian,maha)
%
% 作者：任东  2026-10-26

    if nargin < 1 || isempty(out_name), out_name = 'mewf_sparse.csv'; end

    repo_root = fileparts(fileparts(mfilename('fullpath')));
    addpath(repo_root);
    code_dirs = {'config','simulation','ukf','tracker','fusion','evaluation', ...
                 'utils','visualization','validation','experiments','io'};
    for cd_i = 1:numel(code_dirs)
        d_i = fullfile(repo_root, code_dirs{cd_i});
        if exist(d_i, 'dir'), addpath(d_i); end
    end

    cache_dir = fullfile(repo_root, '_audit', 'cache');
    files = dir(fullfile(cache_dir, 'adsb_0002_*.mat'));
    files = [files; dir(fullfile(cache_dir, 'adsb_span_*.mat'))];
    if isempty(files)
        error('稀疏缓存为空：请先运行 cache_snapshots(''sparse'')');
    end

    cfgs = { ...
        'NN',              'nn',   0,    ''; ...
        'Sequential',      'seq',  0,    ''; ...
        'DualGate_M5',     'dg',   5,    ''; ...
        'DualGate_M8',     'dg',   8,    ''; ...
        'MEWF_greedy',     'mewf', 0.40, 'greedy'; ...
        'MEWF_hungarian',  'mewf', 0.40, 'hungarian' };

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
                case 'nn'
                    tic; p = track_matcher_nn(R1, R2, q); t = toc;
                case 'seq'
                    tic; p = track_matcher_sequential(R1, R2, q); t = toc;
                case 'dg'
                    q.dualgate_M = cfgs{ci, 3};
                    tic; p = track_matcher_dualgate_fast(R1, R2, q); t = toc;
                otherwise
                    q.mewf_score_thr = cfgs{ci, 3};
                    q.mewf_assign    = cfgs{ci, 4};
                    q.mewf_rank_key  = 'maha';
                    tic; p = track_matcher_mewf(R1, R2, q); t = toc;
            end
            M = pairing.eval_pairing(p, R1, R2, n_tb);
            fprintf('  %-16s %.2fs | pairs=%-4d WRONG=%-3d miss=%-3d | P=%.3f R=%.3f\n', ...
                cfgs{ci, 1}, t, M.n_pairs, M.n_wrong, M.n_missed, M.precision, M.recall);
            rows{end+1} = {S.scene, S.seed, cfgs{ci, 1}, M.n_pairs, M.n_correct, ...
                M.n_wrong, M.n_missed, M.precision, M.recall, t, n_tb}; %#ok<AGROW>
        end
    end
    fprintf('\n[总耗时] %.1fs（%d 场景）\n', toc(t_all), numel(files));

    % 聚合
    fprintf('\n============ 稀疏场景对比（8 组平均）============\n');
    fprintf('%-16s %8s %8s %8s %8s %8s %9s\n', ...
        '方法', '误配', '漏配', 'P', 'R', 'F1', '加权代价');
    for ci = 1:size(cfgs, 1)
        sel = rows(cellfun(@(r) strcmp(r{3}, cfgs{ci, 1}), rows));
        w  = mean(cellfun(@(r) r{6}, sel));
        ms = mean(cellfun(@(r) r{7}, sel));
        P  = mean(cellfun(@(r) r{8}, sel));
        R  = mean(cellfun(@(r) r{9}, sel));
        F1 = 2 * P * R / max(P + R, eps);
        C  = 5 * w + ms;
        fprintf('%-16s %8.2f %8.2f %8.4f %8.4f %8.4f %9.2f\n', ...
            cfgs{ci, 1}, w, ms, P, R, F1, C);
    end

    % 结论判据
    fprintf('\n---- 关键判据：MEWF 在稀疏场景是否退化 ----\n');
    getv = @(nm, col) mean(cellfun(@(r) r{col}, ...
        rows(cellfun(@(r) strcmp(r{3}, nm), rows))));
    for nm = {'MEWF_greedy', 'MEWF_hungarian'}
        w = getv(nm{1}, 6); ms = getv(nm{1}, 7);
        P = getv(nm{1}, 8); R = getv(nm{1}, 9);
        if w <= 1e-9 && ms <= 1e-9
            tag = '与 NN 持平（零误配零漏配）→ 稀疏场景不退化';
        elseif w <= 1e-9
            tag = sprintf('零误配，但平均漏配 %.2f 个 → 略逊于 NN', ms);
        else
            tag = sprintf('有误配 %.2f / 漏配 %.2f → 稀疏场景退化', w, ms);
        end
        fprintf('  %-16s 误配 %.2f 漏配 %.2f P=%.4f R=%.4f | %s\n', nm{1}, w, ms, P, R, tag);
    end

    % CSV
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
