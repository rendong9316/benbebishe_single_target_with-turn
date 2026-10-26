function verify_dualgate_fast(scene, seed, do_old)
% verify_dualgate_fast — 验证 dualgate_fast 与原版结果严格一致，并测密集场景速度
%
% 用法: verify_dualgate_fast('adsb_0002', 101, true)   稀疏场景，对比原版与 fast 版
%       verify_dualgate_fast('adsb_0608', 301, false)  密集场景，只跑 fast 版（原版太慢）
    if nargin < 1 || isempty(scene), scene = 'adsb_0002'; end
    if nargin < 2 || isempty(seed),  seed  = 101; end
    if nargin < 3 || isempty(do_old), do_old = true; end
    repo_root = fileparts(fileparts(mfilename('fullpath')));
    addpath(repo_root);
    dirs = {'config','simulation','ukf','tracker','fusion','evaluation', ...
            'utils','visualization','validation','experiments','io'};
    for i = 1:numel(dirs)
        d = fullfile(repo_root, dirs{i});
        if exist(d, 'dir'), addpath(d); end
    end

    params = simulation_params_oracle();
    params.random_seed = seed;
    params.imm_ct_omega_slow_deg = 0.05;
    params.imm_ct_omega_fast_deg = 0.2;

    S = load_aligned_snapshots(scene, params);
    R1 = S.trackSnapshots_R1; R2 = S.aligned_R2; nf = S.n_frames;
    [n_tb, ~, ~, ~, ~] = pairing.build_truth_pairs(R1, R2);
    fprintf('\n===== %s seed %d | 共现真目标 %d =====\n', scene, seed, n_tb);

    if do_old
        tic; p_old = track_matcher_dualgate(R1, R2, params); t_old = toc;
        fprintf('[old ] %.2fs | %d 对\n', t_old, numel(p_old));
    end

    tic; p_new = track_matcher_dualgate_fast(R1, R2, params); t_new = toc;
    fprintf('[fast] %.2fs | %d 对\n', t_new, numel(p_new));

    M_new = pairing.eval_pairing(p_new, R1, R2, n_tb);
    fprintf('[fast] P=%.3f R=%.3f | pairs=%d corr=%d WRONG=%d\n', ...
        M_new.precision, M_new.recall, M_new.n_pairs, M_new.n_correct, M_new.n_wrong);

    if do_old && ~isempty(p_old) && ~isempty(p_new)
        M_old = pairing.eval_pairing(p_old, R1, R2, n_tb);
        fprintf('[old ] P=%.3f R=%.3f | pairs=%d corr=%d WRONG=%d\n', ...
            M_old.precision, M_old.recall, M_old.n_pairs, M_old.n_correct, M_old.n_wrong);

        % 配对集合一致性
        old_pairs = sortrows([[p_old.R1_track_id]', [p_old.R2_track_id]']);
        new_pairs = sortrows([[p_new.R1_track_id]', [p_new.R2_track_id]']);
        if isequal(old_pairs, new_pairs)
            fprintf('[equiv] 配对集合完全一致 ✓ (%d 对)\n', size(old_pairs, 1));
        else
            fprintf('[equiv] 配对集合不同！old=%d fast=%d\n', size(old_pairs, 1), size(new_pairs, 1));
        end

        % mean_dist 一致性（按 R1 排序后逐对比较）
        [~, io] = sort([p_old.R1_track_id]);
        [~, in] = sort([p_new.R1_track_id]);
        md_o = [p_old(io).mean_dist_km];
        md_n = [p_new(in).mean_dist_km];
        if numel(md_o) == numel(md_n)
            fprintf('[equiv] mean_dist 最大偏差 %.3e km\n', max(abs(md_o - md_n)));
        end
    end
end
