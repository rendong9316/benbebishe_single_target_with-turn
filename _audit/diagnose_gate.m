function diagnose_gate(scene, seed, T1)
% diagnose_gate — 诊断判据①：真目标对的「最长连续通过帧数」分布
%
% 目的：解释 M 为何显著影响漏配。对每个「双站都出现且 truth_idx 相同」的真目标对，
% 计算其逐帧距离序列，统计：
%   n_valid   — 两站共现帧数
%   n_under   — 距离 < T1 的帧数
%   n_over    — 距离 >= T1 的帧数
%   max_run   — 最长「连续」距离 < T1 的帧数（决定 M 上限）
%
% 若多数真目标对 max_run < M，则 M 越大漏配越多，这是机制层面的解释。
%
% 用法: diagnose_gate('adsb_0608', 301)      % 默认 T1=35
    if nargin < 1 || isempty(scene), scene = 'adsb_0608'; end
    if nargin < 2 || isempty(seed),  seed  = 301; end
    if nargin < 3 || isempty(T1),    T1    = 35; end
    repo_root = fileparts(fileparts(mfilename('fullpath')));
    addpath(repo_root);
    dirs = {'config','simulation','ukf','tracker','fusion','evaluation', ...
            'utils','visualization','validation','experiments','io'};
    for i = 1:numel(dirs)
        d = fullfile(repo_root, dirs{i});
        if exist(d, 'dir'), addpath(d); end
    end

    f = fullfile(repo_root, '_audit', 'cache', sprintf('%s_%d.mat', scene, seed));
    if ~isfile(f), error('缓存不存在：%s', f); end
    S = load(f);
    R1 = S.trackSnapshots_R1; R2 = S.aligned_R2; nf = S.n_frames;
    I1 = pairing.build_track_index(R1, nf);
    I2 = pairing.build_track_index(R2, nf);

    % 按 truth_idx 配对真目标
    t1 = nan(1, I1.M); t2 = nan(1, I2.M);
    for m = 1:I1.M, t1(m) = pairing.truth_idx_of(R1, I1.ids(m)); end
    for m = 1:I2.M, t2(m) = pairing.truth_idx_of(R2, I2.ids(m)); end

    common = intersect(unique(t1(t1 > 0)), unique(t2(t2 > 0)));
    fprintf('\n===== %s seed %d | T1=%.0fkm | 真目标对 %d =====\n', scene, seed, T1, numel(common));

    stats = zeros(numel(common), 4);   % [n_valid n_under n_over max_run]
    for k = 1:numel(common)
        c1 = find(t1 == common(k), 1);
        c2 = find(t2 == common(k), 1);
        if isempty(c1) || isempty(c2), continue; end
        m = I1.has_pos(:, c1) & I2.has_pos(:, c2);
        n_valid = sum(m);
        d = nan(nf, 1);
        if any(m)
            d(m) = pairing.hav_vec(I1.lon_raw(m, c1), I1.lat_raw(m, c1), ...
                                   I2.lon_raw(m, c2), I2.lat_raw(m, c2));
        end
        under = ~isnan(d) & d < T1;
        over  = ~isnan(d) & d >= T1;
        % 最长连续 under
        max_run = 0; cur = 0;
        for i = 1:nf
            if under(i)
                cur = cur + 1; max_run = max(max_run, cur);
            else
                cur = 0;   % NaN 或超限都中断（与原连续逻辑一致）
            end
        end
        stats(k, :) = [n_valid, sum(under), sum(over), max_run];
    end

    fprintf('共现帧数   : 均值 %.1f, 中位 %.0f\n', mean(stats(:,1)), median(stats(:,1)));
    fprintf('距离<T1 帧 : 均值 %.1f, 中位 %.0f\n', mean(stats(:,2)), median(stats(:,2)));
    fprintf('距离>=T1帧 : 均值 %.1f, 中位 %.0f\n', mean(stats(:,3)), median(stats(:,3)));
    fprintf('最长连续通过: 均值 %.1f, 中位 %.0f, 最小 %.0f, 最大 %.0f\n', ...
        mean(stats(:,4)), median(stats(:,4)), min(stats(:,4)), max(stats(:,4)));

    fprintf('\n按 max_run 分档（能通过多大的 M）：\n');
    edges = [0 4 5 6 8 10 15 20 30 1000];
    for e = 1:numel(edges)-1
        cnt = sum(stats(:,4) >= edges(e) & stats(:,4) < edges(e+1));
        fprintf('  max_run ∈ [%3d,%4d): %3d 个 (%.0f%%)\n', ...
            edges(e), edges(e+1), cnt, 100*cnt/numel(common));
    end
    fprintf('\n→ M=8 只能覆盖 max_run>=8 的目标：%.0f%%\n', ...
        100*sum(stats(:,4) >= 8)/numel(common));
    fprintf('→ M=5 可覆盖 max_run>=5 的目标：%.0f%%\n', ...
        100*sum(stats(:,4) >= 5)/numel(common));
end
