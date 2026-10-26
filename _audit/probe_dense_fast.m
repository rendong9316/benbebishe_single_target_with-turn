function probe_dense_fast(scene, seed)
% probe_dense_fast — 验证索引化加速的正确性与速度，并测密集场景误配
%
% 用法: probe_dense_fast('adsb_0002', 101)   稀疏场景，验证结果与优化前一致（42 对全对）
%       probe_dense_fast('adsb_0608', 301)   密集场景，看 NN 是否出现误配
%
% 只跑 NN 与 Sequential（已索引化），不含 dualgate（原版逐帧查找仍慢）。
    if nargin < 1 || isempty(scene), scene = 'adsb_0002'; end
    if nargin < 2 || isempty(seed),  seed  = 101; end
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
    R1 = S.trackSnapshots_R1; R2 = S.aligned_R2; nf = S.n_frames;
    [n_tb, ~, ~, ~, ~] = pairing.build_truth_pairs(R1, R2);

    tic; I1 = pairing.build_track_index(R1, nf); I2 = pairing.build_track_index(R2, nf); t_idx = toc;
    fprintf('\n===== %s seed %d =====\n', scene, seed);
    fprintf('[time] 加载(含跟踪) %.1fs | 建索引 %.2fs\n', t_load, t_idx);
    fprintf('[size] 帧 %d | 双站共现真目标 %d | R1航迹 %d | R2航迹 %d | 候选对 %d\n', ...
        nf, n_tb, I1.M, I2.M, I1.M * I2.M);

    % 每帧活跃目标数（密集度证据）
    act1 = sum(I1.has_pos, 2);
    fprintf('[density] R1 每帧活跃: 均值 %.1f, 峰值 %d\n', mean(act1), max(act1));

    % ---- 等价性抽查：新旧 mean_dist 对比（前 min(8,M) 对）----
    ei = min(8, I1.M); ej = min(8, I2.M);
    max_err = 0;
    for i = 1:ei
        for j = 1:ej
            [md_o, nc_o] = pairing.mean_dist(R1, R2, I1.ids(i), I2.ids(j), nf);
            [md_n, nc_n] = pairing.mean_dist_fast(I1, I2, I1.ids(i), I2.ids(j));
            if nc_o ~= nc_n, max_err = max(max_err, 1e9); end
            if isfinite(md_o) && isfinite(md_n)
                max_err = max(max_err, abs(md_o - md_n));
            elseif xor(isfinite(md_o), isfinite(md_n))
                max_err = max(max_err, 1e9);
            end
        end
    end
    fprintf('[equiv] 新旧 mean_dist 抽查 %dx%d 对，最大偏差 %.3e km\n', ei, ej, max_err);

    % ---- NN（索引化）----
    tic; p_nn = track_matcher_nn(R1, R2, params); t_nn = toc;
    M1 = pairing.eval_pairing(p_nn, R1, R2, n_tb);
    fprintf('[NN ] %.2fs | pairs=%d corr=%d WRONG=%d | P=%.3f R=%.3f\n', ...
        t_nn, M1.n_pairs, M1.n_correct, M1.n_wrong, M1.precision, M1.recall);

    % ---- Sequential（索引化）----
    tic; p_sq = track_matcher_sequential(R1, R2, params); t_sq = toc;
    M2 = pairing.eval_pairing(p_sq, R1, R2, n_tb);
    fprintf('[Seq] %.2fs | pairs=%d corr=%d WRONG=%d | P=%.3f R=%.3f\n', ...
        t_sq, M2.n_pairs, M2.n_correct, M2.n_wrong, M2.precision, M2.recall);
end
