function T = run_mewf_fusion(max_pairs, do_all_methods)
% run_mewf_fusion — MEWF 配对质量是否传导到融合精度（第 4 章闭环证据）
%
% 背景：MEWF 在配对层面（P/R/误配）已证明优于 NN/Sequential/三判据，但论文还需
% 回答"配对指标好，最终融合精度是不是真的更好"。本脚本在同一份对齐快照上，
% 让各配对方法的结果分别进入融合下游，比较 SCC RMSE。
%
% 设计要点：
%   1) 只跑 SCC（融合主算法，第 5 章已证明最优），省去 BC/CI/FCI 的 4 倍开销
%   2) 密集场景配对数可达 200+，逐对逐帧融合极慢，故按 max_pairs 抽样
%      （按 R1 航迹 id 升序取前 N 对，保证可复现、非随机）
%   3) 只有"正确配对"进入融合（与主链路 evaluate_paper_metrics 口径一致）——
%      注意这会产生样本选择偏差：配得越准的方法进入融合的对越多，
%      因此必须同时报告进入融合的对数 n_fused，不能只看 RMSE
%
% 用法：run_mewf_fusion(50, true)
% 输出：_audit/mewf_fusion.csv

    if nargin < 1 || isempty(max_pairs),   max_pairs = 50;   end
    if nargin < 2 || isempty(do_all_methods), do_all_methods = true; end

    here = fileparts(mfilename('fullpath'));
    repo = fileparts(here);
    addpath(repo);
    ds = {'config','simulation','ukf','tracker','fusion','evaluation', ...
          'utils','visualization','validation','experiments','io'};
    for i = 1:numel(ds)
        if exist(fullfile(repo, ds{i}), 'dir'), addpath(fullfile(repo, ds{i})); end
    end

    cache_dir = fullfile(here, 'cache');
    files = dir(fullfile(cache_dir, 'adsb_0*.mat'));
    files = [files; dir(fullfile(cache_dir, 'adsb_span_*.mat'))];
    if isempty(files)
        error('缓存为空，请先运行 cache_snapshots(''dense'') 或 cache_snapshots(''sparse'')');
    end

    out_csv = fullfile(here, 'mewf_fusion.csv');
    fid = fopen(out_csv, 'w');
    fprintf(fid, 'scene,seed,method,n_pairs,n_fused,SCC_rmse,R1_rmse,R2_rmse,cov\n');

    rows = {};
    t_all = tic;
    for f = 1:numel(files)
        fprintf('\n===== [%d/%d] %s =====\n', f, numel(files), files(f).name);
        S = load(fullfile(cache_dir, files(f).name));
        R1 = S.trackSnapshots_R1;
        R2 = S.aligned_R2;
        truthTrajs = S.truthTrajs;
        nf = S.n_frames;
        params = S.params;
        tag = sprintf('%s_%d', S.scene, S.seed);

        [n_tb, ~, ~, ~, ~] = pairing.build_truth_pairs(R1, R2);
        fprintf('  帧 %d | 双站共现真目标 %d\n', nf, n_tb);

        % ---------- 各配对方法 ----------
        names  = {};
        pairs_c = {};
        names{end+1}  = 'MEWF';        pairs_c{end+1} = track_matcher_mewf(R1, R2, params);
        names{end+1}  = 'DualGate_M8'; pairs_c{end+1} = track_matcher_dualgate_fast(R1, R2, params);
        names{end+1}  = 'Sequential';  pairs_c{end+1} = track_matcher_sequential(R1, R2, params);
        if do_all_methods
            names{end+1}  = 'NN';      pairs_c{end+1} = track_matcher_nn(R1, R2, params);
        end

        for m = 1:numel(names)
            pairs = pairs_c{m};
            np = numel(pairs);
            if np == 0
                fprintf('  %-14s 无配对，跳过\n', names{m});
                continue;
            end

            % 抽样：按 R1 航迹 id 升序取前 max_pairs 对
            if np > max_pairs
                ids = zeros(1, np);
                for pi = 1:np, ids(pi) = pairs(pi).R1_track_id; end
                [~, ord] = sort(ids);
                keep = ord(1:max_pairs);
                pairs = pairs(keep);
                np = numel(pairs);
            end

            % 构造极简 matcher（正确配对才进融合）
            n_ac = numel(truthTrajs);
            matcher_min.aligned_R2 = R2;
            matcher_min.pair_to_aircraft = zeros(np, 1);
            for p = 1:np
                t1 = pairing.truth_idx_of(R1, pairs(p).R1_track_id);
                t2 = pairing.truth_idx_of(R2, pairs(p).R2_track_id);
                if ~isnan(t1) && ~isnan(t2) && t1 == t2 && t1 > 0 && t1 <= n_ac
                    matcher_min.pair_to_aircraft(p) = t1;
                end
            end
            n_fused_in = sum(matcher_min.pair_to_aircraft > 0);

            % 融合（只 SCC）
            method_names = {'SCC'};
            fs = cell(np, 1);
            for p = 1:np
                fs{p, 1} = run_track_fusion(pairs(p), R1, R2, params, 'SCC');
            end
            fe = evaluate_all_multi('fusion', fs, method_names, pairs, R1, R2, ...
                truthTrajs, nf, params.dt_sec, matcher_min);

            % fe.overall 顺序：[方法1..方法M, R1_only, R2_only]
            % 动态索引——只跑 SCC 时 overall 仅 3 项，写死 (5)(6) 会越界
            n_m = numel(method_names);
            scc  = fe.overall(1).s;
            r1o  = fe.overall(n_m + 1).s;
            r2o  = fe.overall(n_m + 2).s;
            cov  = scc.n / max(scc.n + r1o.n + r2o.n, 1);

            fprintf('  %-14s 配对 %3d (抽样后) | 进融合 %3d | SCC %.3f  R1 %.3f  R2 %.3f | 覆盖 %.1f%%\n', ...
                names{m}, np, n_fused_in, scc.rms, r1o.rms, r2o.rms, 100*cov);

            fprintf(fid, '%s,%d,%s,%d,%d,%.4f,%.4f,%.4f,%.4f\n', ...
                S.scene, S.seed, names{m}, np, n_fused_in, scc.rms, r1o.rms, r2o.rms, cov);
            rows{end+1} = {S.scene, S.seed, names{m}, np, n_fused_in, scc.rms, r1o.rms, r2o.rms, cov}; %#ok<AGROW>
        end
    end
    fclose(fid);

    fprintf('\n===== 汇总（各方法在所有场景的平均）=====\n');
    fprintf('  %-14s %8s %8s %8s %8s %8s\n', 'method', 'n_fused', 'SCC', 'R1', 'R2', 'gain%');
    uniq = unique(cellfun(@(r) r{3}, rows, 'UniformOutput', false));
    T = struct('method', {}, 'n_fused', {}, 'scc', {}, 'r1', {}, 'r2', {}, 'gain', {});
    for i = 1:numel(uniq)
        sel = rows(strcmp(cellfun(@(r) r{3}, rows, 'UniformOutput', false), uniq{i}));
        nf_ = mean(cellfun(@(r) r{5}, sel));
        sc_ = mean(cellfun(@(r) r{6}, sel));
        r1_ = mean(cellfun(@(r) r{7}, sel));
        r2_ = mean(cellfun(@(r) r{8}, sel));
        best_single = min(r1_, r2_);
        gain = 100 * (best_single - sc_) / best_single;
        T(end+1) = struct('method', uniq{i}, 'n_fused', nf_, 'scc', sc_, ...
                          'r1', r1_, 'r2', r2_, 'gain', gain); %#ok<AGROW>
        fprintf('  %-14s %8.1f %8.3f %8.3f %8.3f %+8.2f\n', uniq{i}, nf_, sc_, r1_, r2_, gain);
    end
    fprintf('\n总耗时 %.1f s | 结果: %s\n', toc(t_all), out_csv);
end
