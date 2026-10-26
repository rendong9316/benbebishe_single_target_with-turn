function smoke_mewf(scene_seed)
% smoke_mewf — 单场景 MEWF 冒烟测试 + S_thr 敏感性速览
%
% 用法: smoke_mewf('adsb_0608_301')
    if nargin < 1 || isempty(scene_seed), scene_seed = 'adsb_0608_301'; end

    repo_root = fileparts(fileparts(mfilename('fullpath')));
    addpath(repo_root);
    code_dirs = {'config','simulation','ukf','tracker','fusion','evaluation', ...
                 'utils','visualization','validation','experiments','io'};
    for cd_i = 1:numel(code_dirs)
        d_i = fullfile(repo_root, code_dirs{cd_i});
        if exist(d_i, 'dir'), addpath(d_i); end
    end

    fpath = fullfile(repo_root, '_audit', 'cache', [scene_seed '.mat']);
    if ~exist(fpath, 'file')
        error('缓存不存在: %s', fpath);
    end
    S = load(fpath);
    R1 = S.trackSnapshots_R1;
    R2 = S.aligned_R2;
    [n_tb, ~, ~, ~, ~] = pairing.build_truth_pairs(R1, R2);
    n1 = numel(pairing.collect_active_ids(R1));
    n2 = numel(pairing.collect_active_ids(R2));
    fprintf('[%s seed %d] 帧 %d | R1 航迹 %d | R2 航迹 %d | 共现真目标 %d\n', ...
        S.scene, S.seed, S.n_frames, n1, n2, n_tb);

    % 基线参照
    p = track_matcher_sequential(R1, R2, S.params);
    M = pairing.eval_pairing(p, R1, R2, n_tb);
    fprintf('  >> Sequential  : pairs=%-4d WRONG=%-3d miss=%-3d P=%.3f R=%.3f\n', ...
        M.n_pairs, M.n_wrong, M.n_missed, M.precision, M.recall);

    % MEWF：判定层固定（四特征融合），扫 指派方式 × 排序键
    combos = { 'hungarian','dist';  'hungarian','maha'; ...
               'greedy','dist';     'greedy','maha'; ...
               'greedy','score' };
    for ci = 1:size(combos, 1)
        fprintf('\n  ---------- %s / rank=%s ----------\n', combos{ci,1}, combos{ci,2});
        for s = [0.45 0.50 0.55 0.60 0.65]
            params = S.params;
            params.mewf_score_thr = s;
            params.mewf_assign = combos{ci,1};
            params.mewf_rank_key = combos{ci,2};
            tic; p = track_matcher_mewf(R1, R2, params); t = toc;
            M = pairing.eval_pairing(p, R1, R2, n_tb);
            fprintf('  S_thr=%.2f %.2fs | pairs=%-4d WRONG=%-3d miss=%-3d | P=%.3f R=%.3f F1=%.4f\n', ...
                s, t, M.n_pairs, M.n_wrong, M.n_missed, M.precision, M.recall, ...
                2 * M.precision * M.recall / max(M.precision + M.recall, eps));
        end
    end
end
