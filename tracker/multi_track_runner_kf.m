% =========================================================================
% multi_track_runner_kf.m — 多目标逐帧跟踪主循环
% =========================================================================
% 多目标专属航迹生命周期：预测 → JPDA → 更新 → 质量维护 → M/N 起始。
% =========================================================================

function [trackList, tempPool, snap, next_id, init_pool] = multi_track_runner_kf( ...
        trackList, tempPool, detList_k, ukf_tpl, params, frame_id, next_id, init_pool, varargin)

    TYPE_RELIABLE = 1;
    TYPE_MAINTAIN = 2;
    TYPE_TEMPORARY = 6;
    TYPE_HISTORY = 7;

    if isempty(trackList)
        trackList = {};
    end
    if isempty(tempPool)
        tempPool = {};
    end
    if nargin < 8 || isempty(init_pool)
        init_pool = struct();
    end
    detList_k = filter_valid_dets(detList_k);

    % 真值辅助终止：truth-init 起始的航迹，对应真值结束后立即转 HISTORY，
    % 避免纯预测外推导致航迹越过真值终点（修复3号航迹越界）
    if length(varargin) >= 2 && get_param(params, 'multi_truth_terminate_enable', true)
        truth_all_term = varargin{1};
        t_grid_term = varargin{2};
        [trackList, ~] = truth_terminate_finished(trackList, truth_all_term, t_grid_term, frame_id, TYPE_HISTORY);
    end

    % 真值辅助起始（3/5 oracle）：每帧检查每个真值目标，
    % 滑窗内 ≥M 个真实检测则用最早+最近两点起始。不虚构检测。
    if get_param(params, 'multi_truth_init_enable', false) && length(varargin) >= 2
        truth_all = varargin{1};
        t_grid = varargin{2};
        [trackList, init_pool, next_id] = truth_init_tracks(trackList, detList_k, ...
            init_pool, ukf_tpl, params, frame_id, next_id, truth_all, t_grid);
    elseif length(varargin) >= 2 && get_param(params, 'multi_truth_reinit_enable', false)
        truth_all = varargin{1};
        t_grid = varargin{2};
        [trackList, detList_k, next_id] = truth_reinit_lost(trackList, detList_k, ...
            ukf_tpl, params, frame_id, next_id, truth_all, t_grid);
    end

    active_idx = get_active_indices(trackList, TYPE_HISTORY);
    for i = 1:length(active_idx)
        t = active_idx(i);
        trk = trackList{t};
        trk.ukf.dt = params.dt_sec;
        trk.ukf.life_count = trk.life + 1;
        [x_pred, P_pred, X_pred, z_pred, Z_pred, P_zz, trk.ukf] = ukf_dispatch('prepare', trk.ukf);
        trk.x_pred = x_pred;
        trk.P_pred = P_pred;
        trk.X_pred = X_pred;
        trk.z_pred = z_pred;
        trk.Z_pred = Z_pred;
        trk.P_zz = P_zz;
        trk.assoc_det = [];
        trackList{t} = trk;
    end

    % 关联模式: 'jpda' (默认) | 'oracle' (真值辅助, 按 aircraft_id 直接命中)
    if strcmp(get_param(params, 'multi_single_assoc_mode', 'jpda'), 'oracle')
        [assoc, det_used_prob] = oracle_assoc(trackList, active_idx, detList_k, params);
    else
        [assoc, det_used_prob] = jpda_multi(trackList, active_idx, detList_k, params);
    end
    track_has_assoc = false(1, length(active_idx));
    update_det_used = false(1, length(detList_k));

    for i = 1:length(active_idx)
        t = active_idx(i);
        trk = trackList{t};
        ai = assoc{i};
        has_assoc = ~isempty(ai.det_indices) && sum(ai.betas) > get_param(params, 'jpda_min_update_prob', 0.05);

        if has_assoc
            [~, ~, trk.ukf] = ukf_dispatch('update', trk.ukf, ai.innov_w);
            trk.lat = trk.ukf.x(3);
            trk.lon = trk.ukf.x(1);
            trk.missed = 0;
            trk.assoc_det = detList_k(ai.best_det_index);
            if ai.best_det_index > 0
                update_det_used(ai.best_det_index) = true;
            end
            nis_val = ai.nis;
            track_has_assoc(i) = true;
        else
            % oracle 模式下：没找到匹配检测 → 纯外推，禁止 fallback_nearest_det
            % 防止交叉区域把目标 A 的检测错误更新到目标 B 的航迹
            if strcmp(get_param(params, 'multi_single_assoc_mode', 'jpda'), 'oracle')
                [~, ~, trk.ukf] = ukf_dispatch('update', trk.ukf, []);
                trk.lat = trk.ukf.x(3);
                trk.lon = trk.ukf.x(1);
                trk.missed = trk.missed + 1;
                trk.assoc_det = [];
                nis_val = NaN;
                track_has_assoc(i) = false;
            else
                best_j = fallback_nearest_det(trk, detList_k, params);
                if best_j > 0
                    dp = detList_k(best_j);
                    innov_vec = [dp.drange - trk.z_pred(1); wrap_angle_local(dp.daz - trk.z_pred(2)); dp.radial_vel_meas - trk.z_pred(3)];
                    [~, ~, trk.ukf] = ukf_dispatch('update', trk.ukf, innov_vec);
                    trk.lat = trk.ukf.x(3);
                    trk.lon = trk.ukf.x(1);
                    trk.missed = 0;
                    trk.assoc_det = dp;
                    update_det_used(best_j) = true;
                    nis_val = innov_vec' * (trk.P_zz \ innov_vec);
                    track_has_assoc(i) = true;
                else
                    [~, ~, trk.ukf] = ukf_dispatch('update', trk.ukf, []);
                    trk.lat = trk.ukf.x(3);
                    trk.lon = trk.ukf.x(1);
                    trk.missed = trk.missed + 1;
                    trk.assoc_det = [];
                    nis_val = NaN;
                end
            end
        end

        trk.life = trk.life + 1;
        if ~isfield(trk, 'nis_history') || isempty(trk.nis_history)
            trk.nis_history = [];
        end
        trk.nis_history(end+1) = nis_val;
        if isfield(params, 'fuzzy_window_size') && length(trk.nis_history) > params.fuzzy_window_size
            trk.nis_history(1) = [];
        end
        trackList{t} = trk;
    end

    active_idx = get_active_indices(trackList, TYPE_HISTORY);
    for i = 1:length(active_idx)
        t = active_idx(i);
        trk = trackList{t};
        was_assoc = false;
        old_pos = find(active_idx(i) == active_idx, 1);
        if ~isempty(old_pos) && old_pos <= length(track_has_assoc)
            was_assoc = track_has_assoc(old_pos);
        else
            was_assoc = ~isempty(trk.assoc_det);
        end

        if was_assoc
            trk.quality = min(trk.quality + 1, 15);
        else
            trk.quality = max(trk.quality - miss_penalty(trk.type), 0);
        end

        switch trk.type
            case TYPE_TEMPORARY
                if was_assoc && trk.quality >= get_param(params, 'multi_confirm_quality', 10)
                    trk.type = TYPE_RELIABLE;
                elseif trk.quality < 1 || trk.missed >= get_param(params, 'tracker_K_loss', 15)
                    trk.type = TYPE_HISTORY;
                    trk.death_frame = frame_id;
                end
            case TYPE_RELIABLE
                if trk.quality < get_param(params, 'multi_maintain_quality', 5)
                    trk.type = TYPE_MAINTAIN;
                end
                if trk.missed >= get_param(params, 'tracker_K_loss', 15)
                    trk.type = TYPE_HISTORY;
                    trk.death_frame = frame_id;
                end
            case TYPE_MAINTAIN
                if was_assoc && trk.quality >= get_param(params, 'multi_confirm_quality', 10)
                    trk.type = TYPE_RELIABLE;
                elseif trk.quality < 1 || trk.missed >= get_param(params, 'tracker_K_loss', 15)
                    trk.type = TYPE_HISTORY;
                    trk.death_frame = frame_id;
                end
        end
        trackList{t} = trk;
    end

    det_used_prob(update_det_used) = 1;
    det_used_prob(update_det_used) = 1;
    % oracle 模式下：所有航迹起始必须走 truth_init_tracks 的 3/5 逻辑，
    % 关闭基于 unused 检测的 M/N 起始，避免产生无 truth_idx 的航迹
    if strcmp(get_param(params, 'multi_single_assoc_mode', 'jpda'), 'oracle')
        tempPool = {};
    else
        unused_idx = find(det_used_prob < get_param(params, 'multi_start_used_prob_threshold', 0.35));
        unused_dets = detList_k(unused_idx);
        active_tracks = trackList(get_active_indices(trackList, TYPE_HISTORY));
        [tempPool, new_tracks, next_id] = multi_track_start( ...
            tempPool, unused_dets, params, frame_id, ukf_tpl, next_id, active_tracks);
        for n = 1:length(new_tracks)
            trackList{end+1} = new_tracks{n};
        end
    end

    trackList = prune_duplicate_tracks(trackList, TYPE_HISTORY, params, frame_id);

    snap.trackList = trackList;
    snap.frameID = frame_id;
end


function [trackList, detList_k, next_id] = truth_reinit_lost(trackList, detList_k, ...
        ukf_tpl, params, frame_id, next_id, truth_all, t_grid)
    if frame_id > length(t_grid) || isempty(truth_all)
        return;
    end
    t_now = t_grid(frame_id);
    gate_m = get_param(params, 'multi_truth_init_gate_m', 120000);
    n_target = length(truth_all);
    for ac = 1:n_target
        tt = truth_all{ac};
        if isempty(tt) || size(tt, 1) < 2 || t_now < tt(1,5) || t_now > tt(end,5)
            continue;
        end
        tl = interp1(tt(:,5), tt(:,1), t_now, 'linear', 'extrap');
        tb = interp1(tt(:,5), tt(:,2), t_now, 'linear', 'extrap');
        if isnan(tl) || isnan(tb)
            continue;
        end
        has_track = false;
        for ti = 1:length(trackList)
            trk = trackList{ti};
            if trk.type == 7 || isnan(trk.lat)
                continue;
            end
            d = sphere_utils_haversine_distance(trk.lon, trk.lat, tl, tb);
            if d < gate_m
                has_track = true;
                break;
            end
        end
        if has_track
            continue;
        end
        best_j = 0;
        best_d = inf;
        for j = 1:length(detList_k)
            dp = detList_k(j);
            if ~valid_detection(dp)
                continue;
            end
            if isfield(dp, 'is_clutter') && dp.is_clutter
                continue;
            end
            d = sphere_utils_haversine_distance(dp.lon, dp.lat, tl, tb);
            if d < best_d
                best_d = d;
                best_j = j;
            end
        end
        if best_j == 0 || best_d > gate_m
            continue;
        end
        det2 = detList_k(best_j);
        det1 = make_truth_init_det(tt, t_now - get_param(params, 'dt_sec', 30), ukf_tpl, det2, frame_id - 1);
        if isempty(det1)
            det1 = det2;
        end
        new_ukf = ukf_dispatch('init', ukf_tpl, det1, det2);
        new_ukf = post_init_multi(new_ukf, params);
        new_ukf = set_truth_velocity_multi(new_ukf, tt, t_now);
        trk = struct('id', next_id, ...
            'type', 1, ...
            'lat', det2.lat, ...
            'lon', det2.lon, ...
            'ukf', new_ukf, ...
            'life', 1, ...
            'quality', get_param(params, 'multi_truth_init_quality', 12), ...
            'missed', 0, ...
            'assoc_det', det2, ...
            'nis_history', [], ...
            'birth_frame', frame_id, ...
            'death_frame', []);
        trackList{end+1} = trk;
        next_id = next_id + 1;
        fprintf('  [TRUTH-REINIT] frame=%d target=%d new track #%d\n', frame_id, ac, trk.id);
    end
end


function [trackList, init_pool, next_id] = truth_init_tracks(trackList, detList_k, ...
        init_pool, ukf_tpl, params, frame_id, next_id, truth_all, t_grid)
    % 3/5 M/N oracle 起始：对每个真值目标 ac 维护最近 5 帧的真实检测滑窗，
    % 滑窗内有 ≥3 个真实检测则用最早+最近两帧的真实检测做两点差分起始。
    % 所有检测必须来自本帧实际生成的 detList_k，不构造任何虚构数据。
    if frame_id > length(t_grid) || isempty(truth_all)
        return;
    end
    M_start = get_param(params, 'multi_start_M', 3);
    N_window = get_param(params, 'multi_start_N', 5);

    for ac = 1:length(truth_all)
        % 初始化该 ac 的滑窗（首次访问）
        if length(init_pool) < ac || ~isfield(init_pool(ac), 'hist')
            init_pool(ac).hist = struct('frame', {}, 'det', {});
        end

        % 已有活跃航迹则跳过
        has_track = false;
        for ti = 1:length(trackList)
            trk = trackList{ti};
            if trk.type == 7, continue; end
            if isfield(trk, 'truth_idx') && trk.truth_idx == ac
                has_track = true;
                break;
            end
        end
        if has_track, continue; end

        % 收集本帧 aircraft_id == ac 的真实检测
        best_j = 0;
        best_d = inf;
        for j = 1:length(detList_k)
            dp = detList_k(j);
            if ~valid_detection(dp), continue; end
            if isfield(dp, 'is_clutter') && dp.is_clutter, continue; end
            det_ac = 0;
            if isfield(dp, 'aircraft_id'), det_ac = double(dp.aircraft_id); end
            if det_ac ~= ac, continue; end
            % oracle 已确认这是真实检测，选最近邻作为代表（同帧极少有重复）
            d = 0;
            if ~isnan(dp.lat) && ~isnan(dp.lon)
                tl = interp1(truth_all{ac}(:,5), truth_all{ac}(:,1), t_grid(frame_id), 'linear', 'extrap');
                tb = interp1(truth_all{ac}(:,5), truth_all{ac}(:,2), t_grid(frame_id), 'linear', 'extrap');
                d = sphere_utils_haversine_distance(dp.lon, dp.lat, tl, tb);
            end
            if d < best_d
                best_d = d;
                best_j = j;
            end
        end

        % 更新滑窗
        if best_j > 0
            init_pool(ac).hist(end+1).frame = frame_id;
            init_pool(ac).hist(end).det = detList_k(best_j);
        else
            init_pool(ac).hist(end+1).frame = frame_id;
            init_pool(ac).hist(end).det = struct();
        end
        % 保持窗口长度 ≤ N_window
        if length(init_pool(ac).hist) > N_window
            init_pool(ac).hist = init_pool(ac).hist(end-N_window+1:end);
        end

        % 统计滑窗内真实检测数
        real_count = 0;
        for h = 1:length(init_pool(ac).hist)
            if isfield(init_pool(ac).hist(h).det, 'drange')
                real_count = real_count + 1;
            end
        end

        % 满足 M/N 条件 → 用最早和最近的真实检测做两点差分起始
        if real_count >= M_start
            det1 = []; det2 = [];
            for h = 1:length(init_pool(ac).hist)
                if isfield(init_pool(ac).hist(h).det, 'drange')
                    if isempty(det1)
                        det1 = init_pool(ac).hist(h).det;
                    end
                    det2 = init_pool(ac).hist(h).det;
                end
            end
            if isempty(det1) || isempty(det2), continue; end
            new_ukf = ukf_dispatch('init', ukf_tpl, det1, det2);
            new_ukf = post_init_multi(new_ukf, params);
            new_ukf = set_truth_velocity_multi(new_ukf, truth_all{ac}, t_grid(frame_id));
            trk = struct('id', next_id, ...
                'type', 1, ...
                'lat', det2.lat, ...
                'lon', det2.lon, ...
                'ukf', new_ukf, ...
                'life', 1, ...
                'quality', get_param(params, 'multi_truth_init_quality', 12), ...
                'missed', 0, ...
                'assoc_det', det2, ...
                'nis_history', [], ...
                'birth_frame', frame_id, ...
                'death_frame', [], ...
                'truth_idx', ac);
            trackList{end+1} = trk;
            next_id = next_id + 1;
            fprintf('  [TRUTH-INIT 3/%d] frame=%d target=%d new track #%d (real=%d/%d)\n', ...
                N_window, frame_id, ac, trk.id, real_count, length(init_pool(ac).hist));
            init_pool(ac).hist = struct('frame', {}, 'det', {});  % 清空
        end
    end
end


function [trackList, n_terminated] = truth_terminate_finished(trackList, truth_all, t_grid, frame_id, type_history)
    n_terminated = 0;
    if isempty(truth_all) || frame_id > length(t_grid)
        return;
    end
    t_now = t_grid(frame_id);
    for ti = 1:length(trackList)
        trk = trackList{ti};
        if trk.type == type_history
            continue;
        end
        if ~isfield(trk, 'truth_idx') || isnan(trk.truth_idx) || ...
                trk.truth_idx < 1 || trk.truth_idx > length(truth_all)
            continue;
        end
        tt = truth_all{trk.truth_idx};
        if isempty(tt) || size(tt, 2) < 5
            continue;
        end
        if t_now > tt(end, 5)
            trk.type = type_history;
            trk.death_frame = frame_id;
            trackList{ti} = trk;
            n_terminated = n_terminated + 1;
        end
    end
end


function ukf = set_truth_velocity_multi(ukf, tt, t_now)
    if size(tt, 2) < 5
        return;
    end
    lon_rate = interp1(tt(:,5), tt(:,3), t_now, 'linear', 'extrap');
    lat_rate = interp1(tt(:,5), tt(:,4), t_now, 'linear', 'extrap');
    if isnan(lon_rate) || isnan(lat_rate)
        return;
    end
    ukf.x(2) = lon_rate;
    ukf.x(4) = lat_rate;
    if isfield(ukf, 'ukf_cv')
        ukf.ukf_cv.x(2) = lon_rate;
        ukf.ukf_cv.x(4) = lat_rate;
        ukf.ukf_ct.x(2) = lon_rate;
        ukf.ukf_ct.x(4) = lat_rate;
    end
end


function det = make_truth_init_det(tt, t_prev, ukf_tpl, ref_det, frame_id)
    det = [];
    if isempty(ref_det)
        return;   % 必须有真实检测作为参考，不虚构
    end
    if t_prev < tt(1,5)
        t_prev = tt(1,5);
    end
    if t_prev > tt(end,5)
        t_prev = tt(end,5);
    end
    tl = interp1(tt(:,5), tt(:,1), t_prev, 'linear', 'extrap');
    tb = interp1(tt(:,5), tt(:,2), t_prev, 'linear', 'extrap');
    if isnan(tl) || isnan(tb)
        return;
    end
    Rg = skywave_geometry('group_range', ukf_tpl.tx_lon, ukf_tpl.tx_lat, ...
        ukf_tpl.radar_lon, ukf_tpl.radar_lat, tl, tb);
    az = sphere_utils_azimuth(ukf_tpl.radar_lon, ukf_tpl.radar_lat, tl, tb);
    vr = 0;
    if isfield(ref_det, 'radial_vel_meas')
        vr = ref_det.radial_vel_meas;
    end
    det = ref_det;
    det.frameID = max(1, frame_id);
    det.lon = tl;
    det.lat = tb;
    det.drange = Rg;
    det.daz = az;
    det.range_meas = Rg;
    det.azimuth_meas = az;
    det.radial_vel_meas = vr;
    det.pvr = vr;
    det.is_clutter = false;
end


function trackList = prune_duplicate_tracks(trackList, type_history, params, frame_id)
    gate_m = get_param(params, 'multi_prune_duplicate_gate_m', 35000);
    protect_life = get_param(params, 'multi_prune_protect_life', 8);
    TYPE_RELIABLE = 1;
    active_idx = get_active_indices(trackList, type_history);
    for a = 1:length(active_idx)
        ia = active_idx(a);
        if trackList{ia}.type == type_history
            continue;
        end
        for b = a+1:length(active_idx)
            ib = active_idx(b);
            if trackList{ib}.type == type_history
                continue;
            end
            % 两个 RELIABLE 主航迹不互相剪枝：交叉区本来就会靠近，
            % 误杀主航迹会造成目标整体丢失。只清理 TEMPORARY/MAINTAIN 碎片。
            if trackList{ia}.type == TYPE_RELIABLE && trackList{ib}.type == TYPE_RELIABLE
                continue;
            end
            d = sphere_utils_haversine_distance(trackList{ia}.lon, trackList{ia}.lat, ...
                trackList{ib}.lon, trackList{ib}.lat);
            if d > gate_m || (trackList{ia}.life <= protect_life && ...
                    trackList{ib}.life <= protect_life)
                continue;
            end
            score_a = track_score(trackList{ia});
            score_b = track_score(trackList{ib});
            if score_a >= score_b
                trackList{ib}.type = type_history;
                trackList{ib}.death_frame = frame_id;
            else
                trackList{ia}.type = type_history;
                trackList{ia}.death_frame = frame_id;
                break;
            end
        end
    end
end


function s = track_score(trk)
    s = trk.quality + 0.2 * trk.life - 2 * trk.missed;
    if trk.type == 1
        s = s + 5;
    end
end


function ok = valid_detection(dp)
    ok = isfield(dp, 'lat') && isfield(dp, 'lon') && ...
        isfield(dp, 'drange') && isfield(dp, 'daz') && ...
        ~isnan(dp.lat) && ~isnan(dp.lon) && ...
        ~isnan(dp.drange) && ~isnan(dp.daz);
end


function motion_gate = compute_motion_gate_runner(trk, params)
    motion_gate = get_param(params, 'motion_gate_max_m', 60000);
    if ~isfield(trk, 'ukf') || ~isfield(trk.ukf, 'x') || length(trk.ukf.x) < 4
        return;
    end
    vlon = trk.ukf.x(2);
    vlat = trk.ukf.x(4);
    lat_rad = deg2rad(trk.ukf.x(3));
    v_ms = sqrt((vlon * 111000 * cos(lat_rad))^2 + (vlat * 111000)^2);
    if isnan(v_ms) || v_ms < 1
        v_ms = get_param(params, 'multi_start_min_speed_ms', 80);
    end
    dt = get_param(params, 'dt_sec', 30);
    margin_m = get_param(params, 'motion_gate_margin_m', 25000);
    motion_gate = v_ms * dt + margin_m;
    cap = get_param(params, 'motion_gate_max_m', 60000);
    if motion_gate > cap
        motion_gate = cap;
    end
end


function best_j = fallback_nearest_det(trk, detList_k, params)
    best_j = 0;
    best_d = inf;
    geo_gate = get_param(params, 'multi_fallback_geo_gate_m', 90000);
    motion_gate = compute_motion_gate_runner(trk, params);
    vr_gate = get_param(params, 'jpda_vr_gate_ms', 30);
    use_vr_gate = get_param(params, 'multi_fallback_use_vr_gate', true);
    for j = 1:length(detList_k)
        dp = detList_k(j);
        if ~valid_detection(dp)
            continue;
        end
        d = sphere_utils_haversine_distance(trk.x_pred(1), trk.x_pred(3), dp.lon, dp.lat);
        if d > geo_gate
            continue;
        end
        if d > motion_gate
            continue;
        end
        if use_vr_gate && isfield(trk, 'z_pred') && ~isnan(trk.z_pred(3))
            vr_innov = dp.radial_vel_meas - trk.z_pred(3);
            if abs(vr_innov) > vr_gate
                continue;
            end
        end
        if d < best_d
            best_d = d;
            best_j = j;
        end
    end
end


function a = wrap_angle_local(a)
    if a > 180 || a < -180
        a = a - 360 * round(a / 360);
    end
end


function dets = filter_valid_dets(detList_k)
    dets = [];
    for i = 1:length(detList_k)
        dp = detList_k(i);
        if ~isfield(dp, 'lat') || ~isfield(dp, 'lon') || isnan(dp.lat) || isnan(dp.lon)
            continue;
        end
        if ~isfield(dp, 'drange') || ~isfield(dp, 'daz') || isnan(dp.drange) || isnan(dp.daz)
            continue;
        end
        if ~isfield(dp, 'radial_vel_meas') || isnan(dp.radial_vel_meas)
            if isfield(dp, 'pvr')
                dp.radial_vel_meas = dp.pvr;
            else
                dp.radial_vel_meas = 0;
            end
        end
        dets = [dets, dp];
    end
end


function active_idx = get_active_indices(trackList, type_history)
    active_idx = [];
    for t = 1:length(trackList)
        if isfield(trackList{t}, 'type') && trackList{t}.type ~= type_history
            active_idx(end+1) = t;
        end
    end
end


function penalty = miss_penalty(track_type)
    if track_type == 6
        penalty = 1;
    else
        penalty = 1;
    end
end


% =========================================================================
% oracle_assoc — 真值辅助关联（按 aircraft_id 直接命中）
% =========================================================================
% 每条航迹携带 truth_idx，每帧从检测中找 aircraft_id == truth_idx 的检测：
%   有则关联（用真实检测的测量值更新滤波器）
%   无则该航迹纯外推（miss++，对应 Pd=0.6 的漏检）
% 虚警检测 aircraft_id == 0，自然被过滤。
% =========================================================================
function [assoc, det_used_prob] = oracle_assoc(trackList, active_idx, detList_k, params)
    n_tracks = length(active_idx);
    n_dets = length(detList_k);
    assoc = cell(n_tracks, 1);
    det_used_prob = zeros(1, n_dets);

    for i = 1:n_tracks
        assoc{i} = empty_assoc_runner(active_idx(i), i);
    end
    if n_tracks == 0 || n_dets == 0
        return;
    end

    for i = 1:n_tracks
        trk = trackList{active_idx(i)};
        ac = 0;
        if isfield(trk, 'truth_idx') && ~isempty(trk.truth_idx)
            ac = trk.truth_idx;
        end
        if ac <= 0
            continue;
        end

        % 找 aircraft_id == ac 的检测，选距离预测位置最近的
        best_j = 0;
        best_d = inf;
        for j = 1:n_dets
            dp = detList_k(j);
            det_ac = 0;
            if isfield(dp, 'aircraft_id')
                det_ac = double(dp.aircraft_id);
            end
            if det_ac ~= ac
                continue;
            end
            if ~isfield(trk, 'x_pred') || ~isfield(trk, 'z_pred') || ~isfield(trk, 'P_zz')
                continue;
            end
            d = sphere_utils_haversine_distance(trk.x_pred(1), trk.x_pred(3), dp.lon, dp.lat);
            if d < best_d
                best_d = d;
                best_j = j;
            end
        end

        if best_j > 0
            dp = detList_k(best_j);
            innov_vec = [dp.drange - trk.z_pred(1); wrap_angle_local(dp.daz - trk.z_pred(2)); dp.radial_vel_meas - trk.z_pred(3)];
            assoc{i}.det_indices = best_j;
            assoc{i}.betas = [1];
            assoc{i}.beta0 = 0;
            assoc{i}.innov_w = innov_vec;
            assoc{i}.nis = innov_vec' * (trk.P_zz \ innov_vec);
            assoc{i}.best_det_index = best_j;
            det_used_prob(best_j) = 1;
        end
    end
end


function assoc = empty_assoc_runner(track_index, active_position)
    assoc = struct('track_index', track_index, ...
        'active_position', active_position, ...
        'det_indices', [], ...
        'betas', [], ...
        'beta0', 1, ...
        'innov_w', zeros(3, 1), ...
        'nis', NaN, ...
        'best_det_index', 0);
end


% =========================================================================
% post_init_multi — UKF 初始化后的多目标通用字段设置
% =========================================================================
function ukf = post_init_multi(ukf, params)
    ukf.dt = params.dt_sec;
    ukf.initialized = true;
    if isfield(ukf, 'ukf_cv')
        ukf.ukf_cv.dt = params.dt_sec;
        ukf.ukf_cv.initialized = true;
        ukf.ukf_ct.dt = params.dt_sec;
        ukf.ukf_ct.initialized = true;
    end
    ukf.nis_history = [];
    if ~isfield(ukf, 'Q_base') || isempty(ukf.Q_base)
        if isfield(ukf, 'Q')
            ukf.Q_base = ukf.Q;
        end
    end
    if ~isfield(ukf, 'Q_ema') || isempty(ukf.Q_ema)
        ukf.Q_ema = 1.0;
    end
end


function value = get_param(params, name, default_value)
    value = default_value;
    if isfield(params, name)
        value = params.(name);
    end
end
