% =========================================================================
% evaluate_all_multi.m
% 评估模块 — 过程式调度器，包含 compute_tracking_errors 和 evaluate_fusion。
% =========================================================================
%   errorStats = evaluate_all_multi('tracking_errors', trackSnapshots, detList, truthTrajs, n_frames, dt_sec, radar_label)
%   fusion_eval = evaluate_all_multi('fusion', all_fused_snapshots, method_names, matched_pairs, ...
%                              trackSnapshots_R1, trackSnapshots_R2, truthTrajs, n_frames, dt_sec, matcher)
% =========================================================================

function varargout = evaluate_all_multi(action, varargin)
    switch action
        case 'tracking_errors'
            varargout{1} = compute_tracking_errors_multi(varargin{:});
        case 'fusion'
            varargout{1} = evaluate_fusion_multi(varargin{:});
        otherwise
            error('evaluate_all_multi: unknown action ''%s''', action);
    end
end


% =========================================================================
% compute_tracking_errors — 跟踪误差统计
% =========================================================================
function errorStats = compute_tracking_errors_multi(trackSnapshots, detList, truthTrajs, ...
        n_frames, dt_sec, radar_label)
    n_ac = length(truthTrajs);
    frame_times = (0:n_frames-1) * dt_sec;

    ukf_errs  = cell(n_ac, 1);
    det_errs  = cell(n_ac, 1);
    raw_errs  = cell(n_ac, 1);
    ukf_lats  = cell(n_ac, 1);
    det_lats  = cell(n_ac, 1);
    raw_lats  = cell(n_ac, 1);

    for a = 1:n_ac
        tt = truthTrajs{a};
        ukf_errs{a} = [];  det_errs{a} = [];  raw_errs{a} = [];
        ukf_lats{a} = [];  det_lats{a} = [];  raw_lats{a} = [];

        for k = 1:n_frames
            tnow = frame_times(k);
            if tnow < tt.time_sec(1) || tnow > tt.time_sec(end)
                continue;
            end
            t_true_lat = interp1(tt.time_sec, tt.lat, tnow, 'linear');
            t_true_lon = interp1(tt.time_sec, tt.lon, tnow, 'linear');
            if isnan(t_true_lat) || isnan(t_true_lon)
                continue;
            end

            snap = trackSnapshots{k};
            if ~isempty(snap.trackList)
                best_ukf_dist = inf;
                best_ukf_lat = NaN;
                for t = 1:length(snap.trackList)
                    trk = snap.trackList{t};
                    if trk.type == 7 || isnan(trk.lat), continue; end
                    % 优先使用 truth_idx 字段配对（oracle 模式下可靠）
                    if isfield(trk, 'truth_idx') && trk.truth_idx == a
                        d = haversine_km_eval(trk.lon, trk.lat, t_true_lon, t_true_lat);
                        if d < best_ukf_dist
                            best_ukf_dist = d;
                            best_ukf_lat = trk.lat;
                        end
                    end
                end
                if ~isinf(best_ukf_dist)
                    ukf_errs{a}(end+1) = best_ukf_dist;
                    ukf_lats{a}(end+1) = best_ukf_lat;
                end
            end

            dets = detList{k};
            for d = 1:length(dets)
                dp = dets(d);
                if dp.is_clutter, continue; end
                if ~isfield(dp, 'aircraft_id') || dp.aircraft_id ~= a, continue; end
                if isfield(dp, 'lat') && ~isnan(dp.lat)
                    d_cal = haversine_km_eval(dp.lon, dp.lat, t_true_lon, t_true_lat);
                    det_errs{a}(end+1) = d_cal;
                    det_lats{a}(end+1) = dp.lat;
                end
                if isfield(dp, 'raw_lat') && ~isnan(dp.raw_lat)
                    d_raw = haversine_km_eval(dp.raw_lon, dp.raw_lat, t_true_lon, t_true_lat);
                    raw_errs{a}(end+1) = d_raw;
                    raw_lats{a}(end+1) = dp.raw_lat;
                end
            end
        end
    end

    summary = struct();
    for a = 1:n_ac
        summary(a).aircraft = a;
        s_ukf = compute_summary_eval(ukf_errs{a});
        s_det = compute_summary_eval(det_errs{a});
        s_raw = compute_summary_eval(raw_errs{a});
        summary(a).ukf = s_ukf;
        summary(a).det_calibrated = s_det;
        summary(a).det_raw = s_raw;
        if s_ukf.n > 0 && s_det.n > 0
            summary(a).ukf_vs_det_pct = (1 - s_ukf.median / max(s_det.median, 0.01)) * 100;
        else
            summary(a).ukf_vs_det_pct = 0;
        end
    end

    all_ukf = []; all_det = []; all_raw = [];
    for a = 1:n_ac
        all_ukf = [all_ukf, ukf_errs{a}];
        all_det = [all_det, det_errs{a}];
        all_raw = [all_raw, raw_errs{a}];
    end
    overall.ukf = compute_summary_eval(all_ukf);
    overall.det = compute_summary_eval(all_det);
    overall.raw = compute_summary_eval(all_raw);

    errorStats = struct(...
        'radar', radar_label, ...
        'n_frames', n_frames, ...
        'ukf_errors_km', {ukf_errs}, ...
        'det_errors_km', {det_errs}, ...
        'raw_errors_km', {raw_errs}, ...
        'summary', summary, ...
        'overall', overall);
end


% =========================================================================
% evaluate_fusion — 融合误差评估
% =========================================================================
function fusion_eval = evaluate_fusion_multi(all_fused_snapshots, method_names, ...
        matched_pairs, trackSnapshots_R1, trackSnapshots_R2, ...
        truthTrajs, n_frames, dt_sec, matcher)
    n_methods = length(method_names);
    n_ac = length(truthTrajs);
    frame_times = (0:n_frames-1) * dt_sec;

    if isempty(matched_pairs)
        fusion_errs = cell(n_methods, n_ac);
        r1_errs = cell(1, n_ac);
        r2_errs = cell(1, n_ac);
        for m = 1:n_methods
            for a = 1:n_ac
                fusion_errs{m, a} = [];
            end
        end
        for a = 1:n_ac
            r1_errs{a} = [];
            r2_errs{a} = [];
        end
        summary = struct();
        for m = 1:n_methods
            for a = 1:n_ac
                summary(m,a).method = method_names{m};
                summary(m,a).aircraft = a;
                summary(m,a).s = compute_err_stats_eval([]);
            end
        end
        overall = struct();
        all_methods = [method_names, {'R1_only', 'R2_only'}];
        for m = 1:length(all_methods)
            overall(m).method = all_methods{m};
            overall(m).s = compute_err_stats_eval([]);
        end
        fusion_eval = struct('method_names', {method_names}, ...
            'pair_to_aircraft', [], ...
            'fusion_errors', {fusion_errs}, ...
            'r1_errors', {r1_errs}, ...
            'r2_errors', {r2_errs}, ...
            'summary', summary, ...
            'overall', overall);
        fprintf('\n无跨雷达匹配对，跳过融合误差评估。\n');
        return;
    end

    % Step 1: Map R1-R2 matched pairs to true aircraft
    pair_to_aircraft = zeros(length(matched_pairs), 1);

    % 优先使用 matcher 中已预计算的 pair_to_aircraft（多目标场景）
    if isfield(matcher, 'pair_to_aircraft') && length(matcher.pair_to_aircraft) >= length(matched_pairs)
        pair_to_aircraft = matcher.pair_to_aircraft(1:length(matched_pairs));
    else
        % 原来基于位置的猜测逻辑（单目标场景）
        for p = 1:length(matched_pairs)
            mp = matched_pairs(p);
            if isfield(matcher, 'unique_r1_ids')
                r1_idx = find(matcher.unique_r1_ids == mp.R1_track_id, 1);
            else
                r1_idx = find(matcher.r1_ids(:,1) == mp.R1_track_id, 1);
            end
            if isempty(r1_idx), continue; end
            r1_lons = squeeze(matcher.r1_pos(r1_idx, :, 1))';
            r1_lats = squeeze(matcher.r1_pos(r1_idx, :, 2))';
            best_ac = 0;
            best_dist = inf;
            for a = 1:n_ac
                tt = truthTrajs{a};
                t_lat = interp1(tt.time_sec, tt.lat, frame_times, 'linear', 'extrap');
                t_lon = interp1(tt.time_sec, tt.lon, frame_times, 'linear', 'extrap');
                mean_dist = nanmean(haversine_km_vec_eval(r1_lons, r1_lats, t_lon, t_lat));
                if mean_dist < best_dist
                    best_dist = mean_dist;
                    best_ac = a;
                end
            end
            pair_to_aircraft(p) = best_ac;
        end
    end

    fprintf('\n匹配对 -> 真值飞机映射:\n');
    for p = 1:length(matched_pairs)
        ac = pair_to_aircraft(p);
        if ac < 1 || ac > length(truthTrajs)
            fprintf('  Pair %d (R1#%d <-> R2#%d) -> 未映射\n', ...
                p, matched_pairs(p).R1_track_id, matched_pairs(p).R2_track_id);
            continue;
        end
        fprintf('  Pair %d (R1#%d <-> R2#%d) -> 飞机%s\n', ...
            p, matched_pairs(p).R1_track_id, matched_pairs(p).R2_track_id, ...
            truthTrajs{ac}.label);
    end

    % Step 2: Compute fusion track errors frame by frame
    fusion_errs = cell(n_methods, n_ac);
    n_pairs = size(all_fused_snapshots, 1);
    % 判断索引顺序: {method, pair} (单目标 4×1) vs {pair, method} (多目标 3×4)
    is_multi = (n_pairs > 1);

    for m = 1:n_methods
        for a = 1:n_ac
            fusion_errs{m, a} = [];
        end
        for p = 1:n_pairs
            if is_multi
                fused_snaps = all_fused_snapshots{p, m};
            else
                fused_snaps = all_fused_snapshots{m};
            end
            % 多目标场景下，pair p 对应的 aircraft = pair_to_aircraft(p)
            ac = 0;
            if is_multi && p <= length(pair_to_aircraft)
                ac = pair_to_aircraft(p);
            end
            if ac == 0, continue; end
            for k = 1:n_frames
                t_true_lat_all = zeros(n_ac, 1);
                t_true_lon_all = zeros(n_ac, 1);
                for a = 1:n_ac
                    t_true_lat_all(a) = interp1(truthTrajs{a}.time_sec, ...
                        truthTrajs{a}.lat, frame_times(k), 'linear', 'extrap');
                    t_true_lon_all(a) = interp1(truthTrajs{a}.time_sec, ...
                        truthTrajs{a}.lon, frame_times(k), 'linear', 'extrap');
                end
                snap = fused_snaps{k};
                if isempty(snap.trackList), continue; end
                for t = 1:length(snap.trackList)
                    ftrk = snap.trackList{t};
                    if isnan(ftrk.lon), continue; end
                    d = haversine_km_eval(ftrk.lon, ftrk.lat, t_true_lon_all(ac), t_true_lat_all(ac));
                    fusion_errs{m, ac}(end+1) = d;
                end
            end
        end
    end

    % Step 3: Single-station track errors (baseline)
    r1_errs = cell(1, n_ac);
    r2_errs = cell(1, n_ac);
    r1_snaps = trackSnapshots_R1;
    r2_snaps_aligned = matcher.aligned_R2;
    for a = 1:n_ac
        r1_errs{a} = [];
        r2_errs{a} = [];
    end
    for k = 1:n_frames
        for a = 1:n_ac
            t_true_lat = interp1(truthTrajs{a}.time_sec, truthTrajs{a}.lat, ...
                frame_times(k), 'linear', 'extrap');
            t_true_lon = interp1(truthTrajs{a}.time_sec, truthTrajs{a}.lon, ...
                frame_times(k), 'linear', 'extrap');
            snap_r1 = r1_snaps{k};
            if ~isempty(snap_r1.trackList)
                for p = 1:length(matched_pairs)
                    if pair_to_aircraft(p) == a
                        r1_id = matched_pairs(p).R1_track_id;
                        trk1 = find_track_by_id_eval(snap_r1, r1_id);
                        if ~isempty(trk1) && ~isnan(trk1.lon)
                            d = haversine_km_eval(trk1.lon, trk1.lat, t_true_lon, t_true_lat);
                            r1_errs{a}(end+1) = d;
                        end
                        break;
                    end
                end
            end
            snap_r2 = r2_snaps_aligned{k};
            if ~isempty(snap_r2.trackList)
                for p = 1:length(matched_pairs)
                    if pair_to_aircraft(p) == a
                        r2_id = matched_pairs(p).R2_track_id;
                        trk2 = find_track_by_id_eval(snap_r2, r2_id);
                        if ~isempty(trk2) && ~isnan(trk2.lon)
                            d = haversine_km_eval(trk2.lon, trk2.lat, t_true_lon, t_true_lat);
                            r2_errs{a}(end+1) = d;
                        end
                        break;
                    end
                end
            end
        end
    end

    % Step 4: Summary statistics
    summary = struct();
    for m = 1:n_methods
        for a = 1:n_ac
            summary(m,a).method = method_names{m};
            summary(m,a).aircraft = a;
            summary(m,a).s = compute_err_stats_eval(fusion_errs{m,a});
        end
    end
    for a = 1:n_ac
        summary(n_methods+1, a).method = 'R1_only';
        summary(n_methods+1, a).aircraft = a;
        summary(n_methods+1, a).s = compute_err_stats_eval(r1_errs{a});
    end
    for a = 1:n_ac
        summary(n_methods+2, a).method = 'R2_only';
        summary(n_methods+2, a).aircraft = a;
        summary(n_methods+2, a).s = compute_err_stats_eval(r2_errs{a});
    end

    overall = struct();
    all_methods = [method_names, {'R1_only', 'R2_only'}];
    all_errs = [fusion_errs; r1_errs; r2_errs];
    for m = 1:length(all_methods)
        combined = [];
        for a = 1:n_ac
            combined = [combined, all_errs{m,a}];
        end
        overall(m).method = all_methods{m};
        overall(m).s = compute_err_stats_eval(combined);
    end

    fusion_eval = struct(...
        'method_names', {method_names}, ...
        'pair_to_aircraft', pair_to_aircraft, ...
        'fusion_errors', {fusion_errs}, ...
        'r1_errors', {r1_errs}, ...
        'r2_errors', {r2_errs}, ...
        'summary', summary, ...
        'overall', overall);
end


% =========================================================================
% compute_summary_eval — 汇总统计
% =========================================================================
function s = compute_summary_eval(errs)
    s.n = length(errs);
    if s.n > 0
        s.median = median(errs);
        s.mean = mean(errs);
        s.std = std(errs);
        s.rms = sqrt(mean(errs.^2));
        s.min = min(errs);
        s.max = max(errs);
        s.pct95 = prctile(errs, 95);
    else
        s.median = NaN; s.mean = NaN; s.std = NaN; s.rms = NaN;
        s.min = NaN; s.max = NaN; s.pct95 = NaN;
    end
end


% =========================================================================
% haversine_km_eval — Haversine 距离 (km)
% =========================================================================
function d = haversine_km_eval(lon1, lat1, lon2, lat2)
    R = 6371;
    dlat = deg2rad(lat2 - lat1);
    dlon = deg2rad(lon2 - lon1);
    a = sin(dlat/2)^2 + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * sin(dlon/2)^2;
    a = max(0, min(1, a));
    d = R * 2 * atan2(sqrt(a), sqrt(1 - a));
end


% =========================================================================
% compute_err_stats_eval — 误差统计
% =========================================================================
function s = compute_err_stats_eval(errs)
    s.n = length(errs);
    if s.n > 0
        s.median = median(errs);
        s.mean = mean(errs);
        s.std = std(errs);
        s.rms = sqrt(mean(errs.^2));
        s.pct95 = prctile(errs, 95);
        s.min = min(errs);
        s.max = max(errs);
    else
        s.median = NaN; s.mean = NaN; s.std = NaN; s.rms = NaN;
        s.pct95 = NaN; s.min = NaN; s.max = NaN;
    end
end


% =========================================================================
% find_track_by_id_eval — 按ID查找航迹
% =========================================================================
function trk = find_track_by_id_eval(snap, tid)
    trk = [];
    if isempty(snap) || ~isfield(snap, 'trackList'), return; end
    for t = 1:length(snap.trackList)
        if snap.trackList{t}.id == tid
            trk = snap.trackList{t};
            return;
        end
    end
end


% =========================================================================
% haversine_km_vec_eval — 向量化 Haversine 距离
% =========================================================================
function d_vec = haversine_km_vec_eval(lon1, lat1, lon2, lat2)
    d_vec = zeros(size(lon1));
    for i = 1:length(lon1)
        if isnan(lon1(i)) || isnan(lat1(i)) || isnan(lon2(i)) || isnan(lat2(i))
            d_vec(i) = NaN;
        else
            d_vec(i) = haversine_km_eval(lon1(i), lat1(i), lon2(i), lat2(i));
        end
    end
end
