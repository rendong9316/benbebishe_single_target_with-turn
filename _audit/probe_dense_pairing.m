function probe_dense_pairing(scene, seed)
% probe_dense_pairing — 快速探测密集场景下的配对规模与误配情况
% 用法: probe_dense_pairing('adsb_0608', 301)
% 只跑 NN 与三判据（不含融合），计时并输出误配，用于评估密集场景可行性。
    if nargin < 1 || isempty(scene), scene = 'adsb_0608'; end
    if nargin < 2 || isempty(seed),  seed  = 301; end
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

    tic; S = load_aligned_snapshots(scene, params); t_load = toc;
    R1 = S.trackSnapshots_R1; R2 = S.aligned_R2;
    [n_tb, ~, ~, ~, ~] = pairing.build_truth_pairs(R1, R2);
    r1 = pairing.collect_active_ids(R1);
    r2 = pairing.collect_active_ids(R2);
    fprintf('\n[probe] %s seed %d: 加载 %.1fs, 帧 %d, 共现真目标 %d\n', ...
        scene, seed, t_load, S.n_frames, n_tb);
    fprintf('[probe] R1航迹 %d, R2航迹 %d, 候选对 %d (每对算 %d 帧)\n', ...
        numel(r1), numel(r2), numel(r1)*numel(r2), S.n_frames);

    tic; p_nn = track_matcher_nn(R1, R2, params); t_nn = toc;
    M = pairing.eval_pairing(p_nn, R1, R2, n_tb);
    fprintf('[probe] NN     %.1fs  pairs=%d corr=%d WRONG=%d  P=%.3f R=%.3f\n', ...
        t_nn, M.n_pairs, M.n_correct, M.n_wrong, M.precision, M.recall);

    tic; p_dg = track_matcher_dualgate(R1, R2, params); t_dg = toc;
    M2 = pairing.eval_pairing(p_dg, R1, R2, n_tb);
    fprintf('[probe] 三判据  %.1fs  pairs=%d corr=%d WRONG=%d  P=%.3f R=%.3f\n', ...
        t_dg, M2.n_pairs, M2.n_correct, M2.n_wrong, M2.precision, M2.recall);
end
