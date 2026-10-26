function run_pairing_baseline()
% run_pairing_baseline — 多场景 × 多种子运行 NN / Sequential / 双门限 / 三判据配对对比
%
% 在优化模型集（CT 0.05/0.2，与论文主线一致）下，于同一份对齐航迹快照上
% 对比四种跨站配对方法，输出配对质量表（CSV）与融合下游影响。
% 结果保存到 _audit/pairing_baseline_results.csv 并打印汇总。
    repo_root = fileparts(fileparts(mfilename('fullpath')));   % _audit -> 仓库根
    addpath(repo_root);
    code_dirs = {'config','simulation','ukf','tracker','fusion','evaluation', ...
                 'utils','visualization','validation','experiments','io'};
    for cd_i = 1:numel(code_dirs)
        d_i = fullfile(repo_root, code_dirs{cd_i});
        if exist(d_i, 'dir'), addpath(d_i); end
    end

    do_fusion = true;

    % 场景 / 种子计划（稀疏 → 密集，覆盖论文 4.4 节所需全部密度档位）
    %   adsb_0002  42 目标、同时在场峰值 13  （凌晨稀疏）
    %   adsb_span  15 目标、峰值 10           （跨域长航迹）
    %   adsb_0608 204 目标、峰值 65           （早高峰密集）
    %   adsb_0911 471 目标、峰值 79           （晚高峰高密集）
    plan = {...
        'adsb_0002', 101; ...
        'adsb_0002', 102; ...
        'adsb_0002', 103; ...
        'adsb_0002', 104; ...
        'adsb_0002', 105; ...
        'adsb_span', 201; ...
        'adsb_span', 202; ...
        'adsb_span', 203; ...
        'adsb_0608', 301; ...
        'adsb_0608', 302; ...
        'adsb_0608', 303; ...
        'adsb_0911', 401; ...
        'adsb_0911', 402; ...
        };

    methods = {'NN', 'Sequential', 'DualGate(三判据)', 'DualGate(双门限)'};

    rows = {};
    for r = 1:size(plan, 1)
        scenario = plan{r, 1};
        seed = plan{r, 2};
        params = simulation_params_oracle();
        params.random_seed = seed;
        params.imm_ct_omega_slow_deg = 0.05;   % 优化模型集（与论文主线一致）
        params.imm_ct_omega_fast_deg = 0.2;
        fprintf('\n===== 场景 %s 种子 %d =====\n', scenario, seed);
        S = load_aligned_snapshots(scenario, params);
        cmp = compare_pairing_methods(S, do_fusion);
        for m = 1:numel(cmp.method)
            M = cmp.metrics(m);
            row = {scenario, seed, cmp.method{m}, M.n_pairs, M.n_correct, ...
                   M.n_wrong, M.n_missed, M.precision, M.recall, M.wrong_rate, cmp.n_true_both};
            if do_fusion
                f = cmp.fusion{m};
                row = [row, f(1).rms, f(2).rms, f(3).rms, f(4).rms];  % SCC BC CI FCI
            end
            rows{end+1} = row;  %#ok<AGROW>
        end
    end

    % 打印汇总表
    fprintf('\n================ 配对方法对比汇总 ================\n');
    h1 = 'scene      seed  method               pairs  corr  wrong  miss    P      R   wrong%%';
    if do_fusion, h1 = [h1 '   SCC    BC    CI   FCI']; end
    fprintf('%s\n', h1);
    for k = 1:numel(rows)
        rw = rows{k};
        fprintf('%-10s %4d  %-18s %5d %5d %5d %5d  %.3f %.3f %.3f', ...
            rw{1}, rw{2}, rw{3}, rw{4}, rw{5}, rw{6}, rw{7}, rw{8}, rw{9}, rw{10});
        if do_fusion
            fprintf('  %.3f %.3f %.3f %.3f', rw{12}, rw{13}, rw{14}, rw{15});
        end
        fprintf('\n');
    end

    % 保存 CSV
    out_csv = fullfile(repo_root, '_audit', 'pairing_baseline_v2.csv');
    fid = fopen(out_csv, 'w');
    fwrite(fid, 'scene,seed,method,n_pairs,n_correct,n_wrong,n_missed,precision,recall,wrong_rate,n_true_both', 'char');
    if do_fusion, fwrite(fid, ',SCC,BC,CI,FCI', 'char'); end
    fwrite(fid, sprintf('\n'), 'char');
    for k = 1:numel(rows)
        rw = rows{k};
        line = sprintf('%s,%d,%s,%d,%d,%d,%d,%.4f,%.4f,%.4f,%d', ...
            rw{1}, rw{2}, rw{3}, rw{4}, rw{5}, rw{6}, rw{7}, rw{8}, rw{9}, rw{10}, rw{11});
        if do_fusion
            line = sprintf('%s,%.4f,%.4f,%.4f,%.4f', line, rw{12}, rw{13}, rw{14}, rw{15});
        end
        fwrite(fid, line, 'char'); fwrite(fid, sprintf('\n'), 'char');
    end
    fclose(fid);
    fprintf('\n结果已保存: %s\n', out_csv);
end
