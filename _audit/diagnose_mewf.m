function diagnose_mewf(scene_seed)
% diagnose_mewf — 拆解误配性质：跨目标错配 vs 配到虚警航迹
%
% eval_pairing 把两类完全不同的错误合并计为 n_wrong：
%   Type-A 跨目标错配：t1>0, t2>0, t1≠t2 —— 两个不同真实目标被配上，
%          融合后产生**一条完全错误的状态航迹**，危害最大
%   Type-B 配到虚警：任一侧 truth_idx=0（虚假/碎片航迹）—— 该航迹本身
%          质量差，融合时可被质量门控剔除，危害较小
%   Type-C 无真值标记：truth_idx=NaN（理论上不应出现）
%
% 用法: diagnose_mewf('adsb_0608_301')

    if nargin < 1 || isempty(scene_seed), scene_seed = 'adsb_0608_301'; end

    repo_root = fileparts(fileparts(mfilename('fullpath')));
    addpath(repo_root);
    code_dirs = {'config','simulation','ukf','tracker','fusion','evaluation', ...
                 'utils','visualization','validation','experiments','io'};
    for cd_i = 1:numel(code_dirs)
        d_i = fullfile(repo_root, code_dirs{cd_i});
        if exist(d_i, 'dir'), addpath(d_i); end
    end

    S = load(fullfile(repo_root, '_audit', 'cache', [scene_seed '.mat']));
    R1 = S.trackSnapshots_R1;
    R2 = S.aligned_R2;
    [n_tb, ~, ~, ~, ~] = pairing.build_truth_pairs(R1, R2);
    fprintf('=== %s seed %d | 共现真目标 %d ===\n\n', S.scene, S.seed, n_tb);

    % 两种方法
    p_seq = track_matcher_sequential(R1, R2, S.params);

    params = S.params;
    params.mewf_score_thr = 0.50;
    p_me  = track_matcher_mewf(R1, R2, params);

    report('Sequential', p_seq, R1, R2);
    report('MEWF', p_me, R1, R2);
end

% =========================================================================
function report(name, pairs, R1, R2)
    nA = 0; nB = 0; nC = 0; nOK = 0;
    fprintf('---------- %s：%d 对 ----------\n', name, numel(pairs));
    fprintf('%-8s %-8s %-6s %-6s %-6s %-8s %-9s %-8s\n', ...
        'R1_id', 'R2_id', 'truth1', 'truth2', '类型', 'mean_dist', 'coexist', '备注');
    for p = 1:numel(pairs)
        t1 = pairing.truth_idx_of(R1, pairs(p).R1_track_id);
        t2 = pairing.truth_idx_of(R2, pairs(p).R2_track_id);
        if isnan(t1) || isnan(t2)
            typ = 'C-无真值'; nC = nC + 1;
        elseif t1 == t2 && t1 > 0
            typ = '正确'; nOK = nOK + 1;
        elseif t1 == 0 || t2 == 0
            typ = 'B-虚警'; nB = nB + 1;
        else
            typ = 'A-跨目标'; nA = nA + 1;
        end
        if ~strcmp(typ, '正确')
            md = NaN;
            if isfield(pairs, 'mean_dist_km'), md = pairs(p).mean_dist_km; end
            nc = NaN;
            if isfield(pairs, 'coexist_count'), nc = pairs(p).coexist_count; end
            fprintf('%-8d %-8d %-6d %-6d %-6s %-8.1f %-9d %s\n', ...
                pairs(p).R1_track_id, pairs(p).R2_track_id, ...
                round(t1), round(t2), typ, md, round(nc), ...
                ternary(t1 == 0 || t2 == 0, '一侧为虚警航迹', '两真目标互配'));
        end
    end
    fprintf('  小计: 正确 %d | A-跨目标 %d | B-虚警 %d | C-无真值 %d\n\n', nOK, nA, nB, nC);
end

function out = ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end
