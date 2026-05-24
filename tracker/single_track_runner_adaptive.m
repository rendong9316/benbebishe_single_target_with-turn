% =========================================================================
% single_track_runner_adaptive.m
% 机动自适应UKF航迹跟踪器
% =========================================================================
% 与 single_track_runner 使用相同的M/N起始+NN关联+PDA更新框架,
% 但采用 ukf_maneuver_adapt 替代 ukf_fuzzy_adapt, 对转弯机动响应更快.
% =========================================================================

function [trackSnapshots, finalTrack] = single_track_runner_adaptive(detList, ukf_tpl, params, n_frames)
    trackSnapshots = cell(n_frames, 1);
    ukf = [];
    track_state = 'INITIATING';
    life = 0;
    missed = 0;
    quality = 0;

    N = params.tracker_N;
    M = params.tracker_M;
    init_window = {};
    window_has_det = [];

    for k = 1:n_frames
        snap = struct('frameID', k, 'trackList', {{}});
        dets = detList{k};

        switch track_state
            case 'INITIATING'
                init_window{end+1} = dets;
                window_has_det(end+1) = ~isempty(dets);
                if length(init_window) > N
                    init_window(1) = [];
                    window_has_det(1) = [];
                end

                n_with_det = sum(window_has_det);
                if n_with_det >= M && ~isempty(dets)
                    best_prev = [];
                    best_curr_idx = 1;
                    best_support = -1;

                    for curr_idx = 1:length(dets)
                        for i = 1:(length(init_window)-1)
                            prev_dets = init_window{i};
                            if isempty(prev_dets), continue; end
                            for p = 1:length(prev_dets)
                                dp = prev_dets(p);
                                dc = dets(curr_idx);
                                if ~isfield(dp, 'lat') || isnan(dp.lat), continue; end
                                if ~isfield(dc, 'lat') || isnan(dc.lat), continue; end

                                dist = sphere_utils_haversine_distance(dp.lon, dp.lat, dc.lon, dc.lat);
                                dt_frames = length(init_window) - i;
                                est_speed = dist / (dt_frames * params.dt_sec);
                                if est_speed < 30 || est_speed > 600
                                    continue;
                                end

                                support = 0;
                                for jj = 1:(length(init_window)-1)
                                    if jj == i, continue; end
                                    other = init_window{jj};
                                    if isempty(other), continue; end
                                    for oo = 1:length(other)
                                        do = other(oo);
                                        if ~isfield(do, 'lat') || isnan(do.lat), continue; end
                                        d1 = sphere_utils_haversine_distance(dp.lon, dp.lat, do.lon, do.lat);
                                        d2 = sphere_utils_haversine_distance(dc.lon, dc.lat, do.lon, do.lat);
                                        if d1 < 80000 && d2 < 80000
                                            support = support + 1;
                                        end
                                    end
                                end
                                if support > best_support
                                    best_support = support;
                                    best_prev = dp;
                                    best_curr_idx = curr_idx;
                                end
                            end
                        end
                    end

                    if best_support >= 1
                        best_curr = dets(best_curr_idx);
                        ukf = ukf_filter_init(ukf_tpl, best_prev, best_curr);
                        ukf.dt = params.dt_sec;
                        ukf.initialized = true;
                        ukf.Q_base = ukf.Q;
                        ukf.Q_ema = 1.0;
                        ukf.maneuver_active = false;
                        ukf.maneuver_counter = 0;
                        ukf.maneuver_recovery = 0;
                        ukf.suspect_counter = 0;
                        track_state = 'TRACKING';
                        life = 1;  missed = 0;  quality = 5;

                        snap.trackList{1} = make_track_snap_adapt(1, 1, ukf.x(3), ukf.x(1), ...
                            ukf, life, quality, 0, best_curr);

                        init_window = {};
                        window_has_det = [];
                        trackSnapshots{k} = snap;
                        continue;
                    end
                end

                snap.trackList{1} = make_track_snap_adapt(1, 6, NaN, NaN, [], 0, 0, 0, []);
                trackSnapshots{k} = snap;
                continue;

            case 'TRACKING'
                ukf.dt = params.dt_sec;
                [x_pred, P_pred, X_pred, ukf] = ukf_predict_step(ukf);

                z_pred = ukf_measurement_model(ukf, x_pred);
                Z_pred = zeros(ukf.m, 2*ukf.n + 1);
                for s = 1:(2*ukf.n + 1)
                    Z_pred(:, s) = ukf_measurement_model(ukf, X_pred(:, s));
                end
                P_zz = ukf.R;
                for s = 1:(2*ukf.n + 1)
                    dz = Z_pred(:, s) - z_pred;
                    P_zz = P_zz + ukf.Wc(s) * (dz * dz');
                end
                if any(isnan(P_zz(:))), P_zz = ukf.R; end

                best_det = [];
                best_mahal = inf;
                % 机动期间: 渐进放宽门限 (避免突然抓取远处量测)
                if isfield(ukf, 'maneuver_active') && ukf.maneuver_active
                    geo_gate_m = 120000 + ukf.maneuver_counter * 3000;  % 渐进到150km
                    geo_gate_m = min(geo_gate_m, 150000);
                    gate_factor = 1.0 + ukf.maneuver_counter * 0.15;     % 渐进到2.5倍
                    gate_factor = min(gate_factor, 2.5);
                else
                    geo_gate_m = 120000;
                    if life > 15, geo_gate_m = 60000; end
                    gate_factor = 1.0;
                end
                gate_mahal = (params.gate_sigma * gate_factor)^2 * 2;

                % 预扫描: 检测是否有量测在放宽门限内但正常门限外 (机动先兆)
                if ~isfield(ukf, 'maneuver_active') || ~ukf.maneuver_active
                    if ~isfield(ukf, 'suspect_counter'), ukf.suspect_counter = 0; end
                    wide_gate = (params.gate_sigma * 1.8)^2 * 2;
                    any_in_wide = false;
                    for d = 1:length(dets)
                        dp = dets(d);
                        if ~isfield(dp, 'lat') || isnan(dp.lat), continue; end
                        geo_d = sphere_utils_haversine_distance(x_pred(1), x_pred(3), dp.lon, dp.lat);
                        if geo_d > 120000, continue; end
                        z_m = [dp.drange; dp.daz];
                        inno = z_m - z_pred(1:2);
                        if inno(2) > 180, inno(2) = inno(2) - 360;
                        elseif inno(2) < -180, inno(2) = inno(2) + 360; end
                        if inno' * (P_zz(1:2,1:2) \ inno) < wide_gate
                            any_in_wide = true; break;
                        end
                    end
                    if any_in_wide
                        ukf.suspect_counter = ukf.suspect_counter + 1;
                    else
                        ukf.suspect_counter = max(0, ukf.suspect_counter - 1);
                    end
                    % 连续2帧有疑似 → 触发机动 (渐进放宽门限)
                    if ukf.suspect_counter >= 2
                        ukf.maneuver_active = true;
                        ukf.maneuver_counter = 0;
                        ukf.maneuver_recovery = 0;
                        gate_factor = 1.15;  % 初始仅轻微放宽
                        gate_mahal = (params.gate_sigma * gate_factor)^2 * 2;
                        geo_gate_m = 123000;
                    end
                end

                for d = 1:length(dets)
                    dp = dets(d);
                    if ~isfield(dp, 'lat') || isnan(dp.lat), continue; end
                    geo_dist = sphere_utils_haversine_distance(...
                        x_pred(1), x_pred(3), dp.lon, dp.lat);
                    if geo_dist > geo_gate_m, continue; end

                    z_m = [dp.drange; dp.daz];
                    innov = z_m - z_pred(1:2);
                    if innov(2) > 180, innov(2) = innov(2) - 360;
                    elseif innov(2) < -180, innov(2) = innov(2) + 360; end
                    mahal = innov' * (P_zz(1:2,1:2) \ innov);
                    if mahal < gate_mahal && mahal < best_mahal
                        best_mahal = mahal;
                        best_det = dp;
                    end
                end

                if ~isempty(best_det)
                    dets_in_gate = {best_det};
                    gate_threshold = gate_mahal;
                    for d = 1:length(dets)
                        dp = dets(d);
                        if isequal(dp, best_det), continue; end
                        if ~isfield(dp, 'drange') || isnan(dp.drange), continue; end
                        z_m = [dp.drange; dp.daz];
                        innov = z_m - z_pred(1:2);
                        if innov(2) > 180, innov(2) = innov(2) - 360;
                        elseif innov(2) < -180, innov(2) = innov(2) + 360; end
                        if innov' * (P_zz(1:2,1:2) \ innov) < gate_threshold
                            dets_in_gate{end+1} = dp;
                        end
                    end

                    [~, ~, ukf, ~, nis_val] = ukf_pda_update(ukf, dets_in_gate, ...
                        z_pred, Z_pred, X_pred, x_pred, P_pred, P_zz, params);

                    missed = 0;
                    life = life + 1;
                    quality = min(quality + 1, 15);
                    if ~isfield(ukf, 'nis_history'), ukf.nis_history = []; end
                    ukf.nis_history(end+1) = nis_val;
                    if length(ukf.nis_history) > params.fuzzy_window_size
                        ukf.nis_history(1) = [];
                    end

                    % 记录新息历史 (供机动检测用)
                    if ~isfield(ukf, 'innov_history'), ukf.innov_history = {}; end
                    ukf.innov_history{end+1} = [best_det.drange; best_det.daz] - z_pred(1:2);
                    if length(ukf.innov_history) > 10
                        ukf.innov_history(1) = [];
                    end

                    % 机动自适应 (替代基础模糊自适应)
                    if params.use_fuzzy_adaptive && life > 12
                        ukf = ukf_maneuver_adapt(ukf, ukf.nis_history, ukf.innov_history, life, params);
                    end
                else
                    ukf.x = x_pred;
                    ukf.P = P_pred;
                    missed = missed + 1;
                    life = life + 1;
                    quality = max(quality - 1, 0);
                    best_det = [];
                end

                if missed >= params.tracker_K_loss
                    track_state = 'LOST';
                end

                snap.trackList{1} = make_track_snap_adapt(1, 1, ukf.x(3), ukf.x(1), ...
                    ukf, life, quality, missed, best_det);

            case 'LOST'
                track_state = 'INITIATING';
                init_window = {};
                window_has_det = [];
                life = 0; missed = 0; quality = 0;
                snap.trackList{1} = make_track_snap_adapt(1, 7, NaN, NaN, ukf, life, quality, missed, []);
        end

        trackSnapshots{k} = snap;
    end

    finalTrack = struct('id', 1, 'type', iif_adapt(strcmp(track_state,'TRACKING'),1,7), ...
        'quality', quality, 'life', life);
end

function trk = make_track_snap_adapt(id, type, lat, lon, ukf, life, quality, missed, det)
    trk.id = id;
    trk.type = type;
    trk.lat = lat;
    trk.lon = lon;
    trk.ukf = ukf;
    trk.life = life;
    trk.quality = quality;
    trk.missed = missed;
    trk.assoc_det = det;
    if ~isempty(det)
        trk.x_pred = [];
        trk.P_pred = [];
    end
end

function v = iif_adapt(cond, t, f)
    if cond, v = t; else, v = f; end
end
