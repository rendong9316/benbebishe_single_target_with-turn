function run_m_sweep(M_list, out_name)
% run_m_sweep — 从缓存扫 dualgate_M（完全跳过跟踪，每次几秒）
%
% 依赖 cache_snapshots 生成的 _audit/cache/*.mat。
% 用法: run_m_sweep([4 5 6 8 10])
%       run_m_sweep([3 4 5 6 8 10 12], 'm_sweep_fine.csv')
    if nargin < 1 || isempty(M_list),  M_list = [4 5 6 8 10]; end
    if nargin < 2 || isempty(out_name), out_name = 'm_sweep.csv'; end
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
        fprintf('\n[%s seed %d] 帧 %d, 共现真目标 %d\n', S.scene, S.seed, S.n_frames, n_tb);
        for mi = 1:numel(M_list)
            params = S.params;
            params.dualgate_M = M_list(mi);
            tic; p = track_matcher_dualgate_fast(R1, R2, params); t = toc;
            M = pairing.eval_pairing(p, R1, R2, n_tb);
            fprintf('  M=%-3d %.2fs | pairs=%-4d corr=%-4d WRONG=%-3d miss=%-3d | P=%.3f R=%.3f\n', ...
                M_list(mi), t, M.n_pairs, M.n_correct, M.n_wrong, M.n_missed, M.precision, M.recall);
            rows{end+1} = {S.scene, S.seed, M_list(mi), M.n_pairs, M.n_correct, ...
                           M.n_wrong, M.n_missed, M.precision, M.recall, t};  %#ok<AGROW>
        end
    end
    fprintf('\n[总耗时] %.1fs（%d 组 × %d 个 M）\n', toc(t_all), numel(files), numel(M_list));

    % 按 M 平均汇总
    fprintf('\n============ M 扫描汇总 ============\n');
    fprintf('%-5s %10s %9s %8s %8s\n', 'M', 'avgWrong', 'avgMiss', 'avgP', 'avgR');
    for mi = 1:numel(M_list)
        sel = rows(cellfun(@(r) r{3} == M_list(mi), rows));
        if isempty(sel), continue; end
        w  = mean(cellfun(@(r) r{6}, sel));
        mi_ = mean(cellfun(@(r) r{7}, sel));
        P  = mean(cellfun(@(r) r{8}, sel));
        R  = mean(cellfun(@(r) r{9}, sel));
        fprintf('%-5d %10.2f %9.2f %8.4f %8.4f\n', M_list(mi), w, mi_, P, R);
    end

    % CSV
    out_csv = fullfile(repo_root, '_audit', out_name);
    fid = fopen(out_csv, 'w');
    fwrite(fid, 'scene,seed,M,n_pairs,n_correct,n_wrong,n_missed,precision,recall,time_s', 'char');
    fwrite(fid, sprintf('\n'), 'char');
    for k = 1:numel(rows)
        rw = rows{k};
        line = sprintf('%s,%d,%d,%d,%d,%d,%d,%.4f,%.4f,%.3f', ...
            rw{1}, rw{2}, rw{3}, rw{4}, rw{5}, rw{6}, rw{7}, rw{8}, rw{9}, rw{10});
        fwrite(fid, line, 'char'); fwrite(fid, sprintf('\n'), 'char');
    end
    fclose(fid);
    fprintf('\n结果已保存: %s\n', out_csv);
end
