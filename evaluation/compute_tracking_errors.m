% =========================================================================
% compute_tracking_errors.m
% 定量误差评估: UKF滤波位置 vs 真值, 点迹位置 vs 真值
% =========================================================================
% 输出 errorStats 结构体包含每条航迹的逐帧误差 + 汇总统计
% =========================================================================

function errorStats = compute_tracking_errors(trackSnapshots, detList, truthTrajs, ...
        n_frames, dt_sec, radar_label)

    n_ac = length(truthTrajs);
    frame_times = (0:n_frames-1) * dt_sec;

    % 逐帧逐飞机误差存储
    ukf_errs  = cell(n_ac, 1);  % UKF滤波位置误差 (km)
    det_errs  = cell(n_ac, 1);  % 校准后点迹误差 (km)
    raw_errs  = cell(n_ac, 1);  % 校准前原始点迹误差 (km)
    ukf_lats  = cell(n_ac, 1);  % UKF纬度（用于绘图）
    det_lats  = cell(n_ac, 1);
    raw_lats  = cell(n_ac, 1);

    for a = 1:n_ac
        tt = truthTrajs{a};
        ukf_errs{a} = [];  det_errs{a} = [];  raw_errs{a} = [];
        ukf_lats{a} = [];  det_lats{a} = [];  raw_lats{a} = [];

        for k = 1:n_frames
            % 真值位置
            t_true_lat = interp1(tt.time_sec, tt.lat, frame_times(k), 'linear', 'extrap');
            t_true_lon = interp1(tt.time_sec, tt.lon, frame_times(k), 'linear', 'extrap');

            % UKF: 找最近的活跃航迹
            snap = trackSnapshots{k};
            if ~isempty(snap.trackList)
                best_ukf_dist = inf;
                best_ukf_lat = NaN;
                for t = 1:length(snap.trackList)
                    trk = snap.trackList{t};
                    if trk.type == 7 || isnan(trk.lat), continue; end
                    d = haversine_km(trk.lon, trk.lat, t_true_lon, t_true_lat);
                    if d < best_ukf_dist && d < 200  % 200km上限防止误匹配
                        best_ukf_dist = d;
                        best_ukf_lat = trk.lat;
                    end
                end
                if ~isinf(best_ukf_dist)
                    ukf_errs{a}(end+1) = best_ukf_dist;
                    ukf_lats{a}(end+1) = best_ukf_lat;
                end
            end

            % 点迹: 提取该校准后 + 该校准前误差
            dets = detList{k};
            for d = 1:length(dets)
                dp = dets(d);
                if dp.is_clutter, continue; end
                if ~isfield(dp, 'aircraft_id') || dp.aircraft_id ~= a, continue; end

                % 校准后
                if isfield(dp, 'lat') && ~isnan(dp.lat)
                    d_cal = haversine_km(dp.lon, dp.lat, t_true_lon, t_true_lat);
                    det_errs{a}(end+1) = d_cal;
                    det_lats{a}(end+1) = dp.lat;
                end
                % 校准前
                if isfield(dp, 'raw_lat') && ~isnan(dp.raw_lat)
                    d_raw = haversine_km(dp.raw_lon, dp.raw_lat, t_true_lon, t_true_lat);
                    raw_errs{a}(end+1) = d_raw;
                    raw_lats{a}(end+1) = dp.raw_lat;
                end
            end
        end
    end

    % ---- 汇总统计 ----
    summary = struct();
    for a = 1:n_ac
        summary(a).aircraft = a;

        s_ukf = compute_summary(ukf_errs{a});
        s_det = compute_summary(det_errs{a});
        s_raw = compute_summary(raw_errs{a});

        summary(a).ukf = s_ukf;
        summary(a).det_calibrated = s_det;
        summary(a).det_raw = s_raw;

        % UKF vs 检测改善
        if s_ukf.n > 0 && s_det.n > 0
            summary(a).ukf_vs_det_pct = (1 - s_ukf.median / max(s_det.median, 0.01)) * 100;
        else
            summary(a).ukf_vs_det_pct = 0;
        end
    end

    % 总体统计
    all_ukf = []; all_det = []; all_raw = [];
    for a = 1:n_ac
        all_ukf = [all_ukf, ukf_errs{a}];
        all_det = [all_det, det_errs{a}];
        all_raw = [all_raw, raw_errs{a}];
    end
    overall.ukf = compute_summary(all_ukf);
    overall.det = compute_summary(all_det);
    overall.raw = compute_summary(all_raw);

    errorStats = struct(...
        'radar', radar_label, ...
        'n_frames', n_frames, ...
        'ukf_errors_km', {ukf_errs}, ...
        'det_errors_km', {det_errs}, ...
        'raw_errors_km', {raw_errs}, ...
        'summary', summary, ...
        'overall', overall);
end

function s = compute_summary(errs)
    s.n = length(errs);
    if s.n > 0
        s.median = median(errs);
        s.mean   = mean(errs);
        s.std    = std(errs);
        s.rms    = sqrt(mean(errs.^2));
        s.min    = min(errs);
        s.max    = max(errs);
        s.pct95  = prctile(errs, 95);
    else
        s.median = NaN; s.mean = NaN; s.std = NaN; s.rms = NaN;
        s.min = NaN; s.max = NaN; s.pct95 = NaN;
    end
end

function d = haversine_km(lon1, lat1, lon2, lat2)
    R = 6371;
    dlat = deg2rad(lat2 - lat1);
    dlon = deg2rad(lon2 - lon1);
    a = sin(dlat/2)^2 + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * sin(dlon/2)^2;
    a = max(0, min(1, a));
    d = R * 2 * atan2(sqrt(a), sqrt(1 - a));
end
