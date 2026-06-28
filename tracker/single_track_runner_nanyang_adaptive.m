% =========================================================================
% single_track_runner_nanyang_adaptive.m — 混合方案自适应版（简化版）
% =========================================================================
% 与 single_track_runner_nanyang 唯一差异：
%   使用 ukf_zishiying（机动检测+Q自适应提升）替代 ukf_jichu
% 其余完全一致：M/N起始 + 南阳验证 + 简单2态跟踪
% =========================================================================

function [trackSnapshots, finalTrack] = single_track_runner_nanyang_adaptive(detList, ukf_tpl, params, n_frames)
    trackSnapshots = cell(n_frames, 1);
    ukf = [];
    track_state = 'WAITING';
    life = 0;  missed = 0;
    init_state = track_initiation('init', params);
    track_type = 7;

    sysPara = struct();
    sysPara.T_inter = params.dt_sec;
    sysPara.datenum = now;
    sysPara.frameID = 1;
    sysPara.deltaR = 10;
    sysPara.deltaAz = 2;
    sysPara.deltaV = 20;
    sysPara.tx_BLH = [ukf_tpl.tx_lat, ukf_tpl.tx_lon];
    sysPara.rx_BLH = [ukf_tpl.radar_lat, ukf_tpl.radar_lon];
    sysPara.f0 = 10.0;
    sysPara.lambda = 30.0;
    sysPara.prt = 0.05;
    sysPara.fIndex = [0, 0];
    sysPara.aIndex = [0, 360];
    sysPara.rIndex = [0, 5000];
    sysPara.ucMode = 9;
    sysPara.tx_XOY = [0, 0];

    point_history = {};

    for k = 1:n_frames
        snap = struct('frameID', k, 'trackList', {{}});
        dets = detList{k};

        curTime = now + k * params.dt_sec / 86400.0;
        ny_points = det2nanyang_point(dets, k, curTime);
        point_history{k} = ny_points;

        switch track_state
            case 'WAITING'
                [init_state, det1, det2, success] = track_initiation('process', ...
                    init_state, dets, params, k);

                if success
                    candidate = build_candidate_for_validation_ad(...
                        init_state, det1, det2, k, point_history, params, curTime, ukf_tpl);

                    if ~isempty(candidate) && fun_check_track_validation(candidate)
                        ukf = ukf_zishiying('init', ukf_tpl, det1, det2);
                        ukf.dt = params.dt_sec;
                        ukf.initialized = true;
                        ukf.Q_base = ukf.Q;
                        ukf.Q_ema = 1.0;
                        ukf.maneuver_active = false;
                        ukf.maneuver_counter = 0;
                        ukf.maneuver_recovery = 0;
                        ukf.suspect_counter = 0;
                        ukf.life_count = 1;
                        if ~isfield(ukf, 'nis_history'), ukf.nis_history = []; end

                        track_state = 'TRACKING';
                        track_type = 1;
                        life = 1;  missed = 0;

                        snap.trackList{1} = make_snap_ad(1, 1, ...
                            NaN, NaN, ukf, life, 0, det2);
                        trackSnapshots{k} = snap;
                        continue;
                    else
                        init_state = track_initiation('reset', params);
                    end
                end

                snap.trackList{1} = make_snap_ad(1, 7, ...
                    NaN, NaN, [], 0, 0, []);

            case 'TRACKING'
                ukf.dt = params.dt_sec;
                ukf.life_count = life;

                [x_pred, P_pred, X_pred, z_pred, Z_pred, P_zz, ukf] = ukf_jichu('prepare', ukf);
                [best_det, dets_in_gate] = nn_associate(x_pred, z_pred, ...
                    P_zz(1:2, 1:2), dets, params, life);

                if ~isempty(best_det)
                    [innov_w, ~, nis_val] = pda_weight(dets_in_gate, z_pred, P_zz, params);

                    reject_update = false;
                    if life <= 5 && nis_val > 50
                        reject_update = true;
                    end

                    if ~reject_update
                        v_pred_dir = atan2d(x_pred(4), x_pred(2));
                        ukf.last_innov = innov_w;
                        ukf.last_x_pred = x_pred;
                        ukf.last_z_pred = z_pred;
                        ukf.last_P_zz = P_zz;
                        ukf.last_det_list = dets;

                        [lon, lat, ukf] = ukf_zishiying('update', ukf, innov_w, z_pred, ...
                            Z_pred, X_pred, x_pred, P_pred, P_zz, params);

                        % 运动学保护（全生命周期）
                        v_new_dir = atan2d(ukf.x(4), ukf.x(2));
                        if abs(angdiff_ad(v_pred_dir, v_new_dir)) > 90
                            reject_update = true;
                        end
                        if ~reject_update
                            speed_ms = sqrt(ukf.x(2)^2 + ukf.x(4)^2) ...
                                * 111320.0 * cosd(abs(ukf.x(3)));
                            if speed_ms > 500
                                reject_update = true;
                            end
                        end
                        if ~reject_update
                            jump_m = sphere_utils_haversine_distance(x_pred(1), x_pred(3), lon, lat);
                            if jump_m > 50000
                                reject_update = true;
                            end
                        end
                    end

                    if reject_update
                        ukf.x = x_pred;  ukf.P = P_pred;
                        lon = x_pred(1);  lat = x_pred(3);
                        missed = missed + 1;  life = life + 1;
                    else
                        if ~isfield(ukf, 'nis_history'), ukf.nis_history = []; end
                        ukf.nis_history(end+1) = nis_val;
                        missed = 0;  life = life + 1;
                    end
                else
                    ukf.x = x_pred;  ukf.P = P_pred;
                    lon = x_pred(1);  lat = x_pred(3);
                    missed = missed + 1;  life = life + 1;
                end

                if missed >= params.tracker_K_loss
                    track_state = 'LOST';
                end

                snap.trackList{1} = make_snap_ad(1, track_type, ...
                    lat, lon, ukf, life, missed, best_det);

            case 'LOST'
                track_state = 'WAITING';
                init_state = track_initiation('reset', params);
                life = 0;  missed = 0;
                track_type = 7;
                snap.trackList{1} = make_snap_ad(1, 7, ...
                    NaN, NaN, ukf, life, missed, []);
        end

        trackSnapshots{k} = snap;
    end

    finalTrack = struct('id', 1, 'type', track_type, 'life', life);
end


function candidate = build_candidate_for_validation_ad(init_state, det1, det2, k, point_history, params, curTime, ukf_tpl)
    candidate = [];
    if isempty(det1) || isempty(det2), return; end
    point_cells = {};
    for i = 1:length(init_state.window)
        frame_dets = init_state.window{i};
        if isempty(frame_dets), continue; end
        best_dist = inf;  best_pt = [];
        for d = 1:length(frame_dets)
            dp = frame_dets(d);
            if ~isfield(dp, 'lat') || isnan(dp.lat), continue; end
            d1 = sphere_utils_haversine_distance(det1.lon, det1.lat, dp.lon, dp.lat);
            d2 = sphere_utils_haversine_distance(det2.lon, det2.lat, dp.lon, dp.lat);
            if d1 < 80000 && d2 < 80000
                if (d1 + d2) / 2 < best_dist
                    best_dist = (d1 + d2) / 2;  best_pt = dp;
                end
            end
        end
        if ~isempty(best_pt)
            fnum = k - (length(init_state.window) - i);
            if fnum < 1, fnum = 1; end
            ftime = curTime - (k - fnum) * params.dt_sec / 86400.0;
            point_cells{end+1} = det2nanyang_point(best_pt, fnum, ftime);
        end
    end
    ny_det2 = det2nanyang_point(det2, k, curTime);
    has_current = false;
    for c = 1:length(point_cells)
        if point_cells{c}.frameID == k, has_current = true; break; end
    end
    if ~has_current, point_cells{end+1} = ny_det2; end
    if length(point_cells) < 3, return; end
    assc_points = [point_cells{:}];
    [~, sort_idx] = sort([assc_points(:).frameID]);
    candidate.asscPointList = assc_points(sort_idx);
end


function trk = make_snap_ad(id, type, lat, lon, ukf, life, missed, det)
    trk.id = id;
    trk.type = type;
    trk.lat = lat;
    trk.lon = lon;
    trk.ukf = ukf;
    trk.life = life;
    trk.missed = missed;
    trk.assoc_det = det;
end

function d = angdiff_ad(a, b)
    d = mod(b - a + 180, 360) - 180;
end
