function run_gate_compare(M_list, modes, out_name)
% run_gate_compare — 对比判据①的帧计数模式（continuous 连续 / cumulative 累计容错）
%
% 目的：定位「双门限在低发现概率下吃亏」的真正原因。
%   若 cumulative 明显优于 continuous（漏配大降、误配可控），
%   则证明「断一拍归零」是主因，后续上 SPRT 才有意义。
%
% 依赖 _audit/cache/*.mat（cache_snapshots 生成）。
% 用法: run_gate_compare([5 8 10], {'continuous','cumulative'}, 'gate_compare.csv')
    if nargin < 1 || isempty(M_list),  M_list = [5 8 10]; end
    if nargin < 2 || isempty(modes),   modes = {'continuous', 'cumulative'}; end
    if nargin < 3 || isempty(out_name), out_name = 'gate_compare.csv'; end
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
    if isempty(files), error('缓存为空：请先运行 cache_snapshots'); end

    rows = {};
    t_all = tic;
    for f = 1:numel(files)
        S = load(fullfile(cache_dir, files(f).name));
        R1 = S.trackSnapshots_R1;
        R2 = S.aligned_R2;
        [n_tb, ~, ~, ~, ~] = pairing.build_truth_pairs(R1, R2);
        fprintf('\n[%s seed %d] 共现真目标 %d\n', S.scene, S.seed, n_tb);
        for mi = 1:numel(modes)
            for ki = 1:numel(M_list)
                params = S.params;
                params.dualgate_M = M_list(ki);
                params.dualgate_gate_mode = modes{mi};
                tic; p = track_matcher_dualgate_fast(R1, R2, params); t = toc;
                M = pairing.eval_pairing(p, R1, R2, n_tb);
                fprintf('  %-10s M=%-2d %.2fs | pairs=%-4d WRONG=%-3d miss=%-3d | P=%.3f R=%.3f\n', ...
                    modes{mi}, M_list(ki), t, M.n_pairs, M.n_wrong, M.n_missed, M.precision, M.recall);
                rows{end+1} = {S.scene, S.seed, modes{mi}, M_list(ki), M.n_pairs, ...
                               M.n_correct, M.n_wrong, M.n_missed, M.precision, M.recall, t};  %#ok<AGROW>
            end
        end
    end
    fprintf('\n[总耗时] %.1fs\n', toc(t_all));

    % 按 mode × M 平均
    fprintf('\n============ 计数模式 × M 汇总 ============\n');
    fprintf('%-12s %4s %10s %9s %8s %8s %8s\n', 'mode', 'M', 'avgWrong', 'avgMiss', 'avgP', 'avgR', 'avgF1');
    for mi = 1:numel(modes)
        for ki = 1:numel(M_list)
            sel = rows(cellfun(@(r) strcmp(r{3}, modes{mi}) && r{4} == M_list(ki), rows));
            if isempty(sel), continue; end
            w = mean(cellfun(@(r) r{7}, sel));
            m = mean(cellfun(@(r) r{8}, sel));
            P = mean(cellfun(@(r) r{9}, sel));
            R = mean(cellfun(@(r) r{10}, sel));
            f1 = 2 * P * R / max(P + R, eps);
            fprintf('%-12s %4d %10.2f %9.2f %8.4f %8.4f %8.4f\n', modes{mi}, M_list(ki), w, m, P, R, f1);
        end
    end

    % CSV
    out_csv = fullfile(repo_root, '_audit', out_name);
    fid = fopen(out_csv, 'w');
    fwrite(fid, 'scene,seed,mode,M,n_pairs,n_correct,n_wrong,n_missed,precision,recall,time_s', 'char');
    fwrite(fid, sprintf('\n'), 'char');
    for k = 1:numel(rows)
        rw = rows{k};
        line = sprintf('%s,%d,%s,%d,%d,%d,%d,%d,%.4f,%.4f,%.3f', ...
            rw{1}, rw{2}, rw{3}, rw{4}, rw{5}, rw{6}, rw{7}, rw{8}, rw{9}, rw{10}, rw{11});
        fwrite(fid, line, 'char'); fwrite(fid, sprintf('\n'), 'char');
    end
    fclose(fid);
    fprintf('\n结果已保存: %s\n', out_csv);
end
