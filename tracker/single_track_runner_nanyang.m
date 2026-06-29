% =========================================================================
% single_track_runner_nanyang.m — 混合方案（简化版）
% =========================================================================
% 【功能概述】
%   起始：使用原 track_initiation（地理坐标M/N）+
%         fun_check_track_validation（多维物理验证过滤杂波）
%   跟踪：UKF + PDA（保留 probation 保护）
%   状态：WAITING → TRACKING → LOST（仅2态，无质量积分）
%
% 【为什么简化】
%   单目标场景不需要复杂的质量状态机（TEMPORARY→RELIABLE 晋升等）。
%   南阳质量机是为多目标/高更新率微波雷达设计的，用于天波雷达
%   单目标反而会导致 84% 的"坏种子"仅仅因为从未达到 QUALITY_RELIABLE。
%
% 【状态机】
%   WAITING ──(M/N + 验证通过)──> TRACKING
%       ↑                              │
%       └──────(连续miss≥K_loss)───────┘
% =========================================================================

function [trackSnapshots, finalTrack] = single_track_runner_nanyang(detList, ukf_tpl, params, n_frames, varargin)
    % 可选参数: true_track, t_grid (真值辅助起始需要)
    has_truth = false;
    if ~isempty(varargin) && length(varargin) >= 2
        true_track = varargin{1};
        t_grid = varargin{2};
        has_truth = true;
    end
    % ---- 初始化 ----
    trackSnapshots = cell(n_frames, 1);
    ukf = [];
    track_state = 'WAITING';
    life = 0;  missed = 0;
    init_state = track_initiation('init', params);
    track_type = 7;  % 7 = 非跟踪态（兼容 rmse_tracks 的 type ~= 7 判断）

    % ---- 构建 sysPara（供验证函数使用） ----
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

    % ---- 真值辅助起始用持久变量 ----
    init_det1 = [];
    init_frame1 = 0;
    init_det2 = [];
    first_init_done = false;  % 仅首次起始用真值辅助，重新起始走 M/N

    % =====================================================================
    % 主循环
    % =====================================================================
    for k = 1:n_frames
        snap = struct('frameID', k, 'trackList', {{}});
        dets = detList{k};

        curTime = now + k * params.dt_sec / 86400.0;
        ny_points = det2nanyang_point(dets, k, curTime);
        point_history{k} = ny_points;

        switch track_state
            % =============================================================
            % WAITING — 真值辅助起始 或 M/N 起始 + 南阳验证
            % =============================================================
            case 'WAITING'
                if ~first_init_done && isfield(params, 'use_truth_init') && params.use_truth_init
                    % ---- 真值辅助起始：跳过M/N，用真值位置初始化UKF ----
                    if has_truth
                        if isempty(init_det1)
                            % 用当前帧真值位置构造一个"伪检测"（含range/az字段）
                            tl = interp1(true_track(:,5), true_track(:,1), t_grid(k), 'linear', 'extrap');
                            tb = interp1(true_track(:,5), true_track(:,2), t_grid(k), 'linear', 'extrap');
                            Rg = skywave_geometry('group_range', ukf_tpl.tx_lon, ukf_tpl.tx_lat, ...
                                ukf_tpl.radar_lon, ukf_tpl.radar_lat, tl, tb);
                            az = sphere_utils_azimuth(ukf_tpl.radar_lon, ukf_tpl.radar_lat, tl, tb);
                            init_det1 = struct('lon', tl, 'lat', tb, 'range_meas', Rg, 'azimuth_meas', az, 'frameID', k);
                            init_frame1 = k;
                        elseif isempty(init_det2) && (k - init_frame1) >= 1
                            tl = interp1(true_track(:,5), true_track(:,1), t_grid(k), 'linear', 'extrap');
                            tb = interp1(true_track(:,5), true_track(:,2), t_grid(k), 'linear', 'extrap');
                            Rg = skywave_geometry('group_range', ukf_tpl.tx_lon, ukf_tpl.tx_lat, ...
                                ukf_tpl.radar_lon, ukf_tpl.radar_lat, tl, tb);
                            az = sphere_utils_azimuth(ukf_tpl.radar_lon, ukf_tpl.radar_lat, tl, tb);
                            init_det2 = struct('lon', tl, 'lat', tb, 'range_meas', Rg, 'azimuth_meas', az, 'frameID', k);
                            ukf = ukf_jichu('init', ukf_tpl, init_det1, init_det2);
                            ukf.dt = params.dt_sec;
                            ukf.initialized = true;
                            ukf.Q_base = ukf.Q;
                            ukf.Q_ema = 1.0;
                            if ~isfield(ukf, 'nis_history'), ukf.nis_history = []; end
                            first_init_done = true;
                            track_state = 'TRACKING';
                            track_type = 1;  % 1 = 跟踪态
                            life = 1;  missed = 0;
                            snap.trackList{1} = make_snap(1, 1, ...
                                NaN, NaN, ukf, life, 0, init_det2);
                            trackSnapshots{k} = snap;
                            continue;
                        end
                    else
                        % ---- 无真值数据时的降级：用真实检测起始 ----
                        real_det = [];
                        for d = 1:length(dets)
                            if ~dets(d).is_clutter
                                real_det = dets(d);
                                break;
                            end
                        end
                        if ~isempty(real_det)
                            if isempty(init_det1)
                                init_det1 = real_det;
                                init_frame1 = k;
                            elseif isempty(init_det2) && (k - init_frame1) >= 1
                                init_det2 = real_det;
                                ukf = ukf_jichu('init', ukf_tpl, init_det1, init_det2);
                                ukf.dt = params.dt_sec;
                                ukf.initialized = true;
                                ukf.Q_base = ukf.Q;
                                ukf.Q_ema = 1.0;
                                if ~isfield(ukf, 'nis_history'), ukf.nis_history = []; end
                                first_init_done = true;
                                track_state = 'TRACKING';
                                track_type = 1;  life = 1;  missed = 0;
                                snap.trackList{1} = make_snap(1, 1, ...
                                    NaN, NaN, ukf, life, 0, init_det2);
                                trackSnapshots{k} = snap;
                                continue;
                            end
                        end
                    end
                else
                    % ---- 原始 M/N 起始 + 南阳验证 ----
                    [init_state, det1, det2, success] = track_initiation('process', ...
                        init_state, dets, params, k);

                    if success
                        candidate = build_candidate_for_validation(...
                            init_state, det1, det2, k, point_history, params, curTime, ukf_tpl);

                        if ~isempty(candidate) && fun_check_track_validation(candidate)
                            ukf = ukf_jichu('init', ukf_tpl, det1, det2);
                            ukf.dt = params.dt_sec;
                            ukf.initialized = true;
                            ukf.Q_base = ukf.Q;
                            ukf.Q_ema = 1.0;
                            if ~isfield(ukf, 'nis_history'), ukf.nis_history = []; end

                            track_state = 'TRACKING';
                            track_type = 1;  % 1 = 跟踪态
                            life = 1;  missed = 0;

                            snap.trackList{1} = make_snap(1, 1, ...
                                NaN, NaN, ukf, life, 0, det2);
                            trackSnapshots{k} = snap;
                            continue;
                        else
                            init_state = track_initiation('reset', params);
                        end
                    end
                end

                snap.trackList{1} = make_snap(1, 7, ...
                    NaN, NaN, [], 0, 0, []);

            % =============================================================
            % TRACKING — UKF 预测 + NN 关联 + PDA 更新
            % =============================================================
            case 'TRACKING'
                ukf.dt = params.dt_sec;

                [x_pred, P_pred, X_pred, z_pred, Z_pred, P_zz, ukf] = ukf_jichu('prepare', ukf);
                [best_det, dets_in_gate] = nn_associate(x_pred, z_pred, ...
                    P_zz(1:2, 1:2), dets, params, life);

                if ~isempty(best_det)
                    [innov_w, ~, nis_val] = pda_weight(dets_in_gate, z_pred, P_zz, params);

                    % Probation 保护（仅 life≤5 帧高 NIS）
                    reject_update = false;
                    if life <= 5 && nis_val > 50
                        reject_update = true;
                    end

                    if ~reject_update
                        v_pred_dir = atan2d(x_pred(4), x_pred(2));
                        [lon, lat, ukf] = ukf_jichu('update', ukf, innov_w, ...
                            z_pred, Z_pred, X_pred, x_pred, P_pred, P_zz);

                        % Probation 期速度合理性检查（仅M/N起始需要，真值辅助跳过）
                        %   方向突变 >90° / 速度 >500 m/s → ghost
                        if life <= 10 && ~(isfield(params, 'use_truth_init') && params.use_truth_init)
                            v_new_dir = atan2d(ukf.x(4), ukf.x(2));
                            if abs(angdiff(v_pred_dir, v_new_dir)) > 90
                                reject_update = true;
                            end
                            if ~reject_update
                                speed_ms = sqrt(ukf.x(2)^2 + ukf.x(4)^2) ...
                                    * 111320.0 * cosd(abs(ukf.x(3)));
                                if speed_ms > 500
                                    reject_update = true;
                                end
                            end
                        end
                        % 全生命周期位置跳变保护（>50 km → ghost）
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

                % 模糊自适应 Q
                if params.use_fuzzy_adaptive && life > 12 && isfield(ukf, 'nis_history')
                    ukf = apply_fuzzy_adapt(ukf, params);
                end

                % 连续丢失过多 → LOST
                if missed >= params.tracker_K_loss
                    track_state = 'LOST';
                end

                % 跟踪态始终输出位置
                snap.trackList{1} = make_snap(1, track_type, ...
                    lat, lon, ukf, life, missed, best_det);

            % =============================================================
            % LOST — 重置，回到 WAITING
            % =============================================================
            case 'LOST'
                track_state = 'WAITING';
                init_state = track_initiation('reset', params);
                init_det1 = [];  init_frame1 = 0;  init_det2 = [];
                life = 0;  missed = 0;
                track_type = 7;
                snap.trackList{1} = make_snap(1, 7, ...
                    NaN, NaN, ukf, life, missed, []);
        end

        trackSnapshots{k} = snap;
    end

    finalTrack = struct('id', 1, 'type', track_type, 'life', life);
end


% =========================================================================
% build_candidate_for_validation
% =========================================================================
function candidate = build_candidate_for_validation(init_state, det1, det2, k, point_history, params, curTime, ukf_tpl)
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


% =========================================================================
% 辅助函数
% =========================================================================
function trk = make_snap(id, type, lat, lon, ukf, life, missed, det)
    trk.id = id;
    trk.type = type;
    trk.lat = lat;
    trk.lon = lon;
    trk.ukf = ukf;
    trk.life = life;
    trk.missed = missed;
    trk.assoc_det = det;
end

function d = angdiff(a, b)
    d = mod(b - a + 180, 360) - 180;
end

function ukf = apply_fuzzy_adapt(ukf, params)
    if ~isfield(ukf, 'nis_history') || isempty(ukf.nis_history), return; end
    nis_history = ukf.nis_history;
    if ~isfield(ukf, 'Q_ema') || isempty(ukf.Q_ema), ukf.Q_ema = 1.0; end
    if ~isfield(ukf, 'Q_base') || isempty(ukf.Q_base), ukf.Q_base = ukf.Q; end
    nis_avg = mean(nis_history);
    nis_ratio = nis_avg / 2.0;
    mu_VS = trimf(nis_ratio, 0.0, 0.0, 0.4);
    mu_S  = trimf(nis_ratio, 0.2, 0.5, 0.8);
    mu_M  = trimf(nis_ratio, 0.6, 1.0, 1.5);
    mu_L  = trimf(nis_ratio, 1.3, 2.0, 3.0);
    mu_VL = trimf(nis_ratio, 2.5, 4.0, 4.0);
    total_mu = mu_VS + mu_S + mu_M + mu_L + mu_VL;
    if total_mu < 1e-10, factor_fuzzy = 1.0;
    else
        factor_fuzzy = (mu_VS*0.6 + mu_S*0.8 + mu_M*1.0 + mu_L*1.8 + mu_VL*3.0) / total_mu;
    end
    factor_raw = max(0.5, min(4.0, factor_fuzzy));
    ema_eta = 0.20;
    if isfield(params, 'fuzzy_ema_eta'), ema_eta = params.fuzzy_ema_eta; end
    ukf.Q_ema = ema_eta * factor_raw + (1 - ema_eta) * ukf.Q_ema;
    if abs(ukf.Q_ema - 1.0) < 0.05
        ukf.Q = ukf.Q_base;
    else
        ukf.Q = ukf.Q_base * ukf.Q_ema;
    end
end

function mu = trimf(x, a, b, c)
    if x <= a || x >= c, mu = 0;
    elseif x < b, mu = (x - a) / (b - a);
    else, mu = (c - x) / (c - b); end
end
