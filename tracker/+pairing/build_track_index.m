function I = build_track_index(snaps, n_frames)
% build_track_index — 预建 (帧 × 航迹) 索引矩阵，消除逐帧线性查找
%
% 【为什么需要】原 mean_dist / pair_stats 对「每个候选对的每一帧」都调用
%   find_track_by_id（O(航迹数) 线性扫描）。密集场景（数百航迹）下退化为
%   上亿次比较：2304 对 × 239 帧 × 203 航迹 ≈ 1.1 亿次，实测单场景 >20 分钟。
%   本函数一次性把每个航迹在每个帧的状态抽成矩阵列，后续候选对只做矩阵切片，
%   复杂度降到 O(帧)，并可用向量化运算。
%
% 【输出】struct I
%   ids        [1×M]   活跃航迹 ID（口径同 collect_active_ids：type≠7 且 lat 有效）
%   id2col     [max_id×1]  ID→列号（0 表示不在索引内）
%   n_frames, M
%   lon_raw/lat_raw   [n_frames×M] 航迹原始经纬度（等价 trk.lon / trk.lat）
%   lon_ukf/lat_ukf   [n_frames×M] UKF 状态位置（等价 ukf.x(1) / ukf.x(3)）
%   vlon/vlat         [n_frames×M] UKF 速度（ukf.x(2) / ukf.x(4)）
%   P11/P12/P22       [n_frames×M] 位置协方差子块元素 ukf.P([1 3],[1 3])
%   has_pos           [n_frames×M] 逻辑：原始位置有效（type≠7 且 lat 非 NaN）
%   has_state         [n_frames×M] 逻辑：extract_state_local 成功
    ids = pairing.collect_active_ids(snaps);
    M = numel(ids);
    I.ids = ids;
    I.n_frames = n_frames;
    I.M = M;
    I.lon_raw = nan(n_frames, M); I.lat_raw = nan(n_frames, M);
    I.lon_ukf = nan(n_frames, M); I.lat_ukf = nan(n_frames, M);
    I.vlon = nan(n_frames, M);    I.vlat = nan(n_frames, M);
    I.P11 = nan(n_frames, M); I.P12 = nan(n_frames, M); I.P22 = nan(n_frames, M);
    I.Pv11 = nan(n_frames, M); I.Pv22 = nan(n_frames, M);   % 速度协方差对角（度/秒)²
    I.has_pos = false(n_frames, M);
    I.has_state = false(n_frames, M);
    if M == 0
        I.id2col = [];
        return;
    end

    max_id = max(ids);
    I.id2col = zeros(max_id, 1);
    for m = 1:M
        I.id2col(ids(m)) = m;
    end

    for k = 1:n_frames
        if ~isfield(snaps{k}, 'trackList') || isempty(snaps{k}.trackList), continue; end
        trks = snaps{k}.trackList;
        for t = 1:numel(trks)
            trk = trks{t};
            if trk.type == 7, continue; end
            id = trk.id;
            if id < 1 || id > max_id, continue; end
            m = I.id2col(id);
            if m == 0, continue; end
            if ~isnan(trk.lat)
                I.lon_raw(k, m) = trk.lon;
                I.lat_raw(k, m) = trk.lat;
                I.has_pos(k, m) = true;
            end
            [s, ok] = pairing.extract_state_local(trk);
            if ok
                I.lon_ukf(k, m) = s.lon;
                I.lat_ukf(k, m) = s.lat;
                I.vlon(k, m) = s.vel(1);
                I.vlat(k, m) = s.vel(2);
                I.P11(k, m) = s.Ppos(1, 1);
                I.P12(k, m) = s.Ppos(1, 2);
                I.P22(k, m) = s.Ppos(2, 2);
                I.Pv11(k, m) = s.Pvel(1, 1);
                I.Pv22(k, m) = s.Pvel(2, 2);
                I.has_state(k, m) = true;
            end
        end
    end
end
