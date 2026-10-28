function [fusion_eval, all_fused_snapshots] = evaluate_online_fusion(fused_snapshots, ctSnap, ...
        method_names, truthTrajs, n_frames, params)
%EVALUATE_ONLINE_FUSION 在线系统航迹的融合误差评估
%
%   [fusion_eval, all_fused_snapshots] = evaluate_online_fusion(...)
%
%   输入：
%     fused_snapshots — {n_frames} cell，每帧是 1×n_methods cell，
%                        每格是含 trackList 的融合快照
%     ctSnap         — {n_frames} cell，每帧是系统航迹结构体数组
%     method_names   — 融合算法名
%
%   输出：
%     fusion_eval       — 与旧批处理口径一致的融合评估结构
%     all_fused_snapshots — {n_pairs × n_methods} cell，
%                        第 p 个配对在第 k 帧的融合快照（传统结构，供既有评估复用）
%
%   时间口径：第 k 帧对应仿真时刻 (k-1)*dt（与 t1_grid 对齐），
%   取真值位置时必须用同一个基准，否则会引入一帧错位。
%
%   口径说明：这里的"配对"是系统航迹层面的——一条系统航迹在确认双源后，
%   其双源帧构成一个评测对。单源帧计入 R1_only / R2_only 统计。

    n_methods = numel(method_names);
    dt = params.dt_sec;

    % ---- 收集每条系统航迹的双源帧 ----
    % 每条系统航迹：truth_idx、双源帧号列表、逐帧误差
    tracks = struct('truth_idx', {}, 'dual_frames', {}, 'err_m', {});
    for m = 1:n_methods
        err_m{m} = {};
    end

    for k = 1:n_frames
        CTk = ctSnap{k};
        if isempty(CTk), continue; end
        for i = 1:numel(CTk)
            if CTk(i).n_src ~= 2, continue; end
            tid = CTk(i).truth_idx;
            if ~isfinite(tid), continue; end
            idx = find([tracks.truth_idx] == tid, 1);
            if isempty(idx)
                tracks(end+1) = struct('truth_idx', tid, 'dual_frames', [], ...
                    'err_m', {cell(1, n_methods)}); %#ok<AGROW>
                idx = numel(tracks);
            end
            % 真值位置（第 k 帧对应时间 (k-1)*dt，与 t1_grid 对齐）
            [tlat, tlon] = truth_at(truthTrajs, tid, (k-1) * dt);
            if isnan(tlat), continue; end
            tracks(idx).dual_frames(end+1) = k; %#ok<AGROW>
            for m = 1:n_methods
                fs = fused_snapshots{k};
                if isempty(fs), continue; end
                trk = find_fused(fs{m}, CTk(i).id);
                if isempty(trk), continue; end
                e = hav(trk.lon, trk.lat, tlon, tlat);
                tracks(idx).err_m{m}(end+1) = e; %#ok<AGROW>
            end
        end
    end

    % ---- 组装 all_fused_snapshots（传统 pair × method 结构）----
    n_pairs = numel(tracks);
    all_fused_snapshots = cell(n_pairs, n_methods);
    for p = 1:n_pairs
        for m = 1:n_methods
            snaps = cell(n_frames, 1);
            for k = 1:n_frames
                fs = fused_snapshots{k};
                if isempty(fs), continue; end
                trk = find_fused(fs{m}, tracks(p).truth_idx, true);
                if isempty(trk), continue; end
                snaps{k} = struct('frameID', k, 'trackList', {{trk}});
            end
            all_fused_snapshots{p, m} = snaps;
        end
    end

    % ---- 统计 ----
    % 每种算法：所有系统航迹双源帧的误差汇总
    overall = struct('method', {}, 's', {}, 'n_both', {}, 'n_r1_only', {}, 'n_r2_only', {});
    for m = 1:n_methods
        all_err = [];
        for p = 1:n_pairs
            all_err = [all_err, tracks(p).err_m{m}]; %#ok<AGROW>
        end
        s = struct('n', numel(all_err), 'median', NaN, 'mean', NaN, 'std', NaN, ...
                   'rms', NaN, 'pct95', NaN, 'min', NaN, 'max', NaN);
        if ~isempty(all_err)
            s.median = median(all_err);
            s.mean = mean(all_err);
            s.std = std(all_err);
            s.rms = sqrt(mean(all_err.^2));
            s.pct95 = prctile(all_err, 95);
            s.min = min(all_err);
            s.max = max(all_err);
        end
        % 单源帧统计
        n_r1 = 0; n_r2 = 0;
        for k = 1:n_frames
            CTk = ctSnap{k};
            if isempty(CTk), continue; end
            for i = 1:numel(CTk)
                if CTk(i).n_src ~= 1, continue; end
                if CTk(i).src1_id > 0, n_r1 = n_r1 + 1; end
                if CTk(i).src2_id > 0, n_r2 = n_r2 + 1; end
            end
        end
        overall(end+1) = struct('method', method_names{m}, 's', s, ...
            'n_both', numel(all_err), 'n_r1_only', n_r1, 'n_r2_only', n_r2); %#ok<AGROW>
    end

    % 逐目标统计
    summary = struct('method', {}, 'aircraft', {}, 's', {});
    for m = 1:n_methods
        for p = 1:n_pairs
            e = tracks(p).err_m{m};
            s = struct('n', numel(e), 'median', NaN, 'mean', NaN, 'std', NaN, ...
                       'rms', NaN, 'pct95', NaN, 'min', NaN, 'max', NaN);
            if ~isempty(e)
                s.median = median(e); s.mean = mean(e); s.std = std(e);
                s.rms = sqrt(mean(e.^2)); s.pct95 = prctile(e, 95);
                s.min = min(e); s.max = max(e);
            end
            summary(end+1) = struct('method', method_names{m}, ...
                'aircraft', tracks(p).truth_idx, 's', s); %#ok<AGROW>
        end
    end

    fusion_eval = struct('method_names', {method_names}, ...
        'summary', summary, 'overall', overall, ...
        'pair_to_aircraft', [tracks.truth_idx], ...
        'n_system_tracks', n_pairs, ...
        'dual_frame_counts', {arrayfun(@(t) numel(t.dual_frames), tracks)});
end

% =====================================================================
function trk = find_fused(snap, id, by_truth)
%FIND_FUSED 在融合快照中找系统航迹
%   by_truth = true 时按 truth_idx 匹配，否则按系统航迹 id 匹配
    trk = [];
    if nargin < 3, by_truth = false; end
    if isempty(snap) || ~isfield(snap, 'trackList'), return; end
    tl = snap.trackList;
    for i = 1:numel(tl)
        if by_truth
            if isfield(tl{i}, 'truth_idx') && isfinite(tl{i}.truth_idx) && ...
                    tl{i}.truth_idx == id
                trk = tl{i};  return;
            end
        else
            if tl{i}.id == id
                trk = tl{i};  return;
            end
        end
    end
end

function [tlat, tlon] = truth_at(truthTrajs, tid, tnow)
    tlat = NaN; tlon = NaN;
    if tid < 1 || tid > numel(truthTrajs), return; end
    tt = truthTrajs{tid};
    if tnow < tt.time_sec(1) || tnow > tt.time_sec(end), return; end
    tlat = interp1(tt.time_sec, tt.lat, tnow, 'linear');
    tlon = interp1(tt.time_sec, tt.lon, tnow, 'linear');
end

function d = hav(lon1, lat1, lon2, lat2)
    R = 6371;
    p1 = deg2rad(lat1); p2 = deg2rad(lat2);
    dp = deg2rad(lat2-lat1); dl = deg2rad(lon2-lon1);
    a = sin(dp/2).^2 + cos(p1).*cos(p2).*sin(dl/2).^2;
    d = 2*R*asin(min(1, sqrt(a)));
end
