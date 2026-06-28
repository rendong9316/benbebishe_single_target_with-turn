% =========================================================================
% single_track_runner_adaptive.m
% =========================================================================
% 【功能概述】
%   机动自适应UKF航迹跟踪器，对 single_track_runner 的增强版本。
%   模块化流水线：prepare → NN关联 → PDA加权 → 自适应UKF更新。
%   使用 ukf_zishiying('update', ...) 替代 ukf_jichu('update', ...)，
%   该函数内部封装了机动检测、渐进波门放宽、机动自适应Q等全部逻辑。
%
% 【与 single_track_runner 的差异】
%   - 使用 ukf_zishiying('update', ...) 替代 ukf_jichu('update', ...)
%   - 机动检测/Q提升全部封装在 ukf_zishiying 内部
%   - 无需单独调用 apply_fuzzy_adapt
% =========================================================================

function [trackSnapshots, finalTrack] = single_track_runner_adaptive(detList, ukf_tpl, params, n_frames)
    % ---- 初始化 ----
    trackSnapshots = cell(n_frames, 1);
    ukf = [];
    track_state = 'INITIATING';
    life = 0;  missed = 0;  quality = 0;

    init_state = track_initiation('init', params);

    % =====================================================================
    % 主循环：逐帧处理
    % =====================================================================
    for k = 1:n_frames
        snap = struct('frameID', k, 'trackList', {{}});
        dets = detList{k};

        switch track_state
            % =============================================================
            % 状态: INITIATING（航迹起始等待）
            % =============================================================
            case 'INITIATING'
                [init_state, det1, det2, success] = track_initiation('process', init_state, dets, params, k);
                if success
                    ukf = ukf_zishiying('init', ukf_tpl, det1, det2);
                    ukf.dt = params.dt_sec;
                    ukf.initialized = true;
                    ukf.Q_base = ukf.Q;
                    ukf.Q_ema = 1.0;

                    % 机动检测相关字段初始化
                    ukf.maneuver_active = false;
                    ukf.maneuver_counter = 0;
                    ukf.maneuver_recovery = 0;
                    ukf.suspect_counter = 0;
                    ukf.life_count = 1;

                    track_state = 'TRACKING';
                    life = 1;  missed = 0;  quality = 5;

                    snap.trackList{1} = make_track_snap_adapt(1, 1, ukf.x(3), ukf.x(1), ...
                        ukf, life, quality, 0, det2);

                    trackSnapshots{k} = snap;
                    continue;
                end

                snap.trackList{1} = make_track_snap_adapt(1, 6, NaN, NaN, [], 0, 0, 0, []);
                trackSnapshots{k} = snap;
                continue;

            % =============================================================
            % 状态: TRACKING（机动自适应跟踪）
            % 模块化流水线: prepare → NN关联 → PDA加权 → 自适应UKF更新
            % =============================================================
            case 'TRACKING'
                ukf.dt = params.dt_sec;
                ukf.life_count = life;

                % 1. UKF: 预测 + 量测统计
                [x_pred, P_pred, X_pred, z_pred, Z_pred, P_zz, ukf] = ukf_jichu('prepare', ukf);

                % 2. 关联: NN 找最佳点迹
                [best_det, dets_in_gate] = nn_associate(x_pred, z_pred, P_zz(1:2, 1:2), dets, params, life);

                if ~isempty(best_det)
                    % 3. 关联: PDA 加权新息
                    [innov_w, ~, nis_val] = pda_weight(dets_in_gate, z_pred, P_zz, params);

                    % 3.5 Probation 期保护（life≤5：NIS过大；life≤10：速度方向突变）
                    probate_nis_limit = 50;
                    reject_update = false;
                    if life <= 5 && nis_val > probate_nis_limit
                        reject_update = true;
                    end

                    if ~reject_update
                        % 预更新速度方向
                        v_pred_dir = atan2d(x_pred(4), x_pred(2));

                        % 设置机动预检测所需字段（供 ukf_zishiying 内部使用）
                        ukf.last_innov = innov_w;
                        ukf.last_x_pred = x_pred;
                        ukf.last_z_pred = z_pred;
                        ukf.last_P_zz = P_zz;
                        ukf.last_det_list = dets;

                        % 4. 自适应 UKF: 纯 Kalman 更新 + 机动自适应 Q
                        [lon, lat, ukf] = ukf_zishiying('update', ukf, innov_w, z_pred, ...
                            Z_pred, X_pred, x_pred, P_pred, P_zz, params);

                        % Probation 期速度合理性检查（life≤10）
                        if life <= 10
                            % 速度方向突变 >90°
                            v_new_dir = atan2d(ukf.x(4), ukf.x(2));
                            dir_change = abs(angdiff_deg_ad(v_pred_dir, v_new_dir));
                            if dir_change > 90
                                reject_update = true;
                            end
                            % 速度大小超限 >500 m/s
                            if ~reject_update
                                speed_ms = sqrt(ukf.x(2)^2 + ukf.x(4)^2) ...
                                    * 111320.0 * cosd(abs(ukf.x(3)));
                                if speed_ms > 500
                                    reject_update = true;
                                end
                            end
                        end
                    end

                    if reject_update
                        ukf.x = x_pred;
                        ukf.P = P_pred;
                        lon = x_pred(1);
                        lat = x_pred(3);
                        missed = missed + 1;
                        life = life + 1;
                        quality = max(quality - 1, 0);
                        best_det = [];
                    else
                        % 航迹维护
                        if ~isfield(ukf, 'nis_history'), ukf.nis_history = []; end
                        ukf.nis_history(end+1) = nis_val;
                        missed = 0;
                        life = life + 1;
                        quality = min(quality + 1, 15);
                    end
                else
                    ukf.x = x_pred;
                    ukf.P = P_pred;
                    lon = x_pred(1);
                    lat = x_pred(3);
                    missed = missed + 1;
                    life = life + 1;
                    quality = max(quality - 1, 0);
                    best_det = [];
                end

                if missed >= params.tracker_K_loss
                    track_state = 'LOST';
                end

                snap.trackList{1} = make_track_snap_adapt(1, 1, lat, lon, ukf, life, quality, missed, best_det);

            % =============================================================
            % 状态: LOST（航迹丢失后重新起始）
            % =============================================================
            case 'LOST'
                track_state = 'INITIATING';
                init_state = track_initiation('reset', params);
                life = 0; missed = 0; quality = 0;
                snap.trackList{1} = make_track_snap_adapt(1, 7, NaN, NaN, ukf, life, quality, missed, []);
        end

        trackSnapshots{k} = snap;
    end

    finalTrack = struct('id', 1, 'type', iif_adapt(strcmp(track_state,'TRACKING'),1,7), ...
        'quality', quality, 'life', life);
end


% =========================================================================
% 辅助函数: make_track_snap_adapt
% =========================================================================
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


% =========================================================================
% 辅助函数: iif_adapt
% =========================================================================
function v = iif_adapt(cond, t, f)
    if cond, v = t; else, v = f; end
end


% =========================================================================
% angdiff_deg_ad — 两个角度（度）之间的最小差值，范围 (-180, 180]
% =========================================================================
function d = angdiff_deg_ad(a, b)
    d = mod(b - a + 180, 360) - 180;
end
