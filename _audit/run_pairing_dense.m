function run_pairing_dense()
% run_pairing_dense — 密集场景配对质量对比（跳过融合下游，专攻配对算法本身）
%
% 为什么跳过融合：密集场景配对 200 对 × 4 融合算法逐对计算极慢（实测单场景
% >40 分钟），而本章核心结论是「配对正确率随目标密度如何变化」，融合 RMSE
% 已在稀疏场景（adsb_0002/adsb_span）单独给出，密集场景融合可作为后续补充。
%
% 输出：_audit/pairing_dense.csv（列与 pairing_baseline_results.csv 前 11 列一致）
    repo_root = fileparts(fileparts(mfilename('fullpath')));
    addpath(repo_root);
    code_dirs = {'config','simulation','ukf','tracker','fusion','evaluation', ...
                 'utils','visualization','validation','experiments','io'};
    for cd_i = 1:numel(code_dirs)
        d_i = fullfile(repo_root, code_dirs{cd_i});
        if exist(d_i, 'dir'), addpath(d_i); end
    end

    do_fusion = false;   % 关键：跳过融合，只评配对质量

    plan = {...
        'adsb_0608', 301; ...
        'adsb_0608', 302; ...
        'adsb_0608', 303; ...
        'adsb_0911', 401; ...
        'adsb_0911', 402; ...
        };

    rows = {};
    for r = 1:size(plan, 1)
        scenario = plan{r, 1};
        seed = plan{r, 2};
        params = simulation_params_oracle();
        params.random_seed = seed;
        params.imm_ct_omega_slow_deg = 0.05;
        params.imm_ct_omega_fast_deg = 0.2;
        fprintf('\n===== 场景 %s 种子 %d =====\n', scenario, seed);
        tic; S = load_aligned_snapshots(scenario, params); t_load = toc;
        fprintf('[time] 加载(含跟踪) %.1fs\n', t_load);
        cmp = compare_pairing_methods(S, do_fusion);
        for m = 1:numel(cmp.method)
            M = cmp.metrics(m);
            rows{end+1} = {scenario, seed, cmp.method{m}, M.n_pairs, M.n_correct, ...
                           M.n_wrong, M.n_missed, M.precision, M.recall, M.wrong_rate, ...
                           cmp.n_true_both};  %#ok<AGROW>
        end
    end

    % 打印汇总
    fprintf('\n============ 密集场景配对质量汇总 ============\n');
    fprintf('%-10s %4s  %-18s %6s %5s %6s %5s %7s %7s %8s\n', ...
        'scene','seed','method','pairs','corr','wrong','miss','P','R','wrong%');
    for k = 1:numel(rows)
        rw = rows{k};
        fprintf('%-10s %4d  %-18s %6d %5d %6d %5d %7.3f %7.3f %8.2f\n', ...
            rw{1}, rw{2}, rw{3}, rw{4}, rw{5}, rw{6}, rw{7}, rw{8}, rw{9}, 100*rw{10});
    end

    % 保存 CSV
    out_csv = fullfile(repo_root, '_audit', 'pairing_dense.csv');
    fid = fopen(out_csv, 'w');
    fwrite(fid, 'scene,seed,method,n_pairs,n_correct,n_wrong,n_missed,precision,recall,wrong_rate,n_true_both', 'char');
    fwrite(fid, sprintf('\n'), 'char');
    for k = 1:numel(rows)
        rw = rows{k};
        line = sprintf('%s,%d,%s,%d,%d,%d,%d,%.4f,%.4f,%.4f,%d', ...
            rw{1}, rw{2}, rw{3}, rw{4}, rw{5}, rw{6}, rw{7}, rw{8}, rw{9}, rw{10}, rw{11});
        fwrite(fid, line, 'char'); fwrite(fid, sprintf('\n'), 'char');
    end
    fclose(fid);
    fprintf('\n结果已保存: %s\n', out_csv);
end
