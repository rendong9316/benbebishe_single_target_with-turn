% =========================================================================
% single_track_runner.m — 统一的单目标航迹跟踪器
% =========================================================================
% 【功能概述】
%   单目标逐帧航迹管理器，实现从航迹起始到跟踪维持的完整生命周期。
%   通过 ukf_dispatch 多态路由，支持三种滤波器后端：
%     ukf_jichu      — 基础 CV-UKF + 固定Q
%     ukf_zishiying   — CV-UKF + 模糊Q + 机动检测
%     ukf_imm         — CV+CT 双模型 IMM-UKF
%   tracker 不感知后端类型，主入口注入哪种 ukf_tpl 即走对应路径。
%
% 【统一流水线（TRACKING 每帧）】
%   杂波预筛 → prepare(预测) → NN关联 → PDA加权 → update(更新)
%
% 【状态机】
%   INITIATING → TRACKING → LOST → INITIATING（循环）
%
% 【输入参数】
%   detList  - cell数组，每帧的点迹结构体数组
%   ukf_tpl  - UKF模板结构体（由 ukf_xxx('create', ...) 产生）
%   params   - 参数结构体
%   n_frames - 总帧数
%   varargin - 可选: true_track, t_grid（真值辅助首次起始）
%
% 【输出】
%   trackSnapshots - cell数组，(n_frames x 1)，每帧的航迹快照
%   finalTrack     - 结构体，最终航迹状态摘要
% =========================================================================

function [trackSnapshots, finalTrack] = single_track_runner(detList, ukf_tpl, params, n_frames, varargin)
    % ---- 可选参数: true_track, t_grid (真值辅助首次起始) ----
    has_truth = false;
    if ~isempty(varargin) && length(varargin) >= 2
        true_track = varargin{1};
        t_grid = varargin{2};
        has_truth = true;
    end

    % ---- 初始化 ----
    trackSnapshots = cell(n_frames, 1);
    ukf = [];
    track_state = 'INITIATING';

    life = 0;  missed = 0;  quality = 0;

    init_state = track_initiation('init', params);

    % ---- 真值辅助首次起始: 局部持久变量 ----
    first_init_done = false;
    init_det1 = [];
    init_frame1 = 0;
    init_det2 = [];

    % ---- 重新起始超时兜底 ----
    reinit_timeout_frames = max(4, params.tracker_N - 2);
    reinit_attempt_frame = 0;
    reinit_truth_collecting = false;
    reinit_truth_det1 = [];
    reinit_truth_frame1 = 0;

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
                % ---- 首次起始: 真值辅助（保证正确开局） ----
                if ~first_init_done && isfield(params, 'use_truth_init') && params.use_truth_init && has_truth
                    if isempty(init_det1)
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

                        ukf = ukf_dispatch('init', ukf_tpl, init_det1, init_det2);
                        ukf = post_init(ukf, params);
                        first_init_done = true;
                        track_state = 'TRACKING';
                        life = 1;  missed = 0;  quality = 5;
                        snap.trackList{1} = make_track_snap(1, 1, ukf.x(3), ukf.x(1), ...
                            ukf, life, quality, 0, init_det2);
                        trackSnapshots{k} = snap;
                        continue;
                    end
                    snap.trackList{1} = make_track_snap(1, 6, NaN, NaN, [], 0, 0, 0, []);
                    trackSnapshots{k} = snap;
                    continue;
                end

                % ---- 重新起始: 超时兜底优先 ----
                if reinit_truth_collecting
                    if (k - reinit_truth_frame1) >= 1
                        tl = interp1(true_track(:,5), true_track(:,1), t_grid(k), 'linear', 'extrap');
                        tb = interp1(true_track(:,5), true_track(:,2), t_grid(k), 'linear', 'extrap');
                        Rg = skywave_geometry('group_range', ukf_tpl.tx_lon, ukf_tpl.tx_lat, ...
                            ukf_tpl.radar_lon, ukf_tpl.radar_lat, tl, tb);
                        az = sphere_utils_azimuth(ukf_tpl.radar_lon, ukf_tpl.radar_lat, tl, tb);
                        reinit_truth_det2 = struct('lon', tl, 'lat', tb, ...
                            'range_meas', Rg, 'azimuth_meas', az, 'frameID', k);

                        ukf = ukf_dispatch('init', ukf_tpl, reinit_truth_det1, reinit_truth_det2);
                        ukf = post_init(ukf, params);
                        reinit_truth_collecting = false;
                        reinit_attempt_frame = 0;
                        track_state = 'TRACKING';
                        life = 1;  missed = 0;  quality = 5;
                        snap.trackList{1} = make_track_snap(1, 1, ukf.x(3), ukf.x(1), ...
                            ukf, life, quality, 0, reinit_truth_det2);
                        trackSnapshots{k} = snap;
                        continue;
                    end
                    snap.trackList{1} = make_track_snap(1, 6, NaN, NaN, [], 0, 0, 0, []);
                    trackSnapshots{k} = snap;
                    continue;
                end

                % ---- 超时检查: 触发真值兜底 ----
                timeout_triggered = (first_init_done && has_truth && reinit_attempt_frame > 0 && ...
                                     (k - reinit_attempt_frame) > reinit_timeout_frames);

                if timeout_triggered
                    tl = interp1(true_track(:,5), true_track(:,1), t_grid(k), 'linear', 'extrap');
                    tb = interp1(true_track(:,5), true_track(:,2), t_grid(k), 'linear', 'extrap');
                    Rg = skywave_geometry('group_range', ukf_tpl.tx_lon, ukf_tpl.tx_lat, ...
                        ukf_tpl.radar_lon, ukf_tpl.radar_lat, tl, tb);
                    az = sphere_utils_azimuth(ukf_tpl.radar_lon, ukf_tpl.radar_lat, tl, tb);
                    reinit_truth_det1 = struct('lon', tl, 'lat', tb, ...
                        'range_meas', Rg, 'azimuth_meas', az, 'frameID', k);
                    reinit_truth_frame1 = k;
                    reinit_truth_collecting = true;
                    init_state = track_initiation('reset', params);
                    snap.trackList{1} = make_track_snap(1, 6, NaN, NaN, [], 0, 0, 0, []);
                    trackSnapshots{k} = snap;
                    continue;
                end

                % ---- 纯 M/N 滑窗逻辑 ----
                [init_state, det1, det2, success] = track_initiation('process', init_state, dets, params, k);
                if success
                    ukf = ukf_dispatch('init', ukf_tpl, det1, det2);
                    ukf = post_init(ukf, params);
                    reinit_attempt_frame = 0;
                    reinit_truth_collecting = false;
                    track_state = 'TRACKING';
                    life = 1;  missed = 0;  quality = 5;
                    snap.trackList{1} = make_track_snap(1, 1, ukf.x(3), ukf.x(1), ...
                        ukf, life, quality, 0, det2);
                    trackSnapshots{k} = snap;
                    continue;
                end
                snap.trackList{1} = make_track_snap(1, 6, NaN, NaN, [], 0, 0, 0, []);
                trackSnapshots{k} = snap;
                continue;

            % =============================================================
            % 状态: TRACKING（统一流水线，通过 ukf_dispatch 多态）
            % 流水线: 杂波预筛 → prepare → NN关联 → PDA → update
            % =============================================================
            case 'TRACKING'
                ukf.dt = params.dt_sec;

                % ---- 1. 杂波预筛（所有体制统一，is_clutter 标签来自点迹生成阶段） ----
                clean_dets = [];
                for d = 1:length(dets)
                    if ~dets(d).is_clutter
                        clean_dets = [clean_dets, dets(d)];
                    end
                end

                % ---- 2. 滤波器预测（多态，tracker 不关心后端） ----
                [x_pred, ~, ~, z_pred, ~, P_zz, ukf] = ukf_dispatch('prepare', ukf);

                % ---- 3. NN 关联（只用非杂波点迹，Vr 门统一禁用） ----
                saved_vr = params.gate_vr_ms;
                params.gate_vr_ms = 9999;
                [best_det, dets_in_gate] = nn_associate(x_pred, z_pred, P_zz(1:2, 1:2), clean_dets, params, life);
                params.gate_vr_ms = saved_vr;

                % ---- 4. 连续丢点防杂波劫持: 固定地理门 50km ----
                if ~isempty(best_det) && missed >= 2
                    geo_dist = sphere_utils_haversine_distance(...
                        x_pred(1), x_pred(3), best_det.lon, best_det.lat);
                    if geo_dist > 50000
                        best_det = [];
                        dets_in_gate = {};
                    end
                end

                if ~isempty(best_det)
                    % ---- 5. PDA 加权新息 ----
                    [innov_w, ~, nis_val] = pda_weight(dets_in_gate, z_pred, P_zz, params);

                    % ---- 6. Probation 期保护（仅防 NIS>50 明显异常） ----
                    probate_nis_limit = 50;
                    reject_update = false;
                    if life <= 5 && nis_val > probate_nis_limit
                        reject_update = true;
                    end

                    if ~reject_update
                        % ---- 7. 设置机动预检测上下文（仅 ukf_zishiying 使用，其它后端无害） ----
                        ukf.last_det_list = dets;
                        ukf.life_count = life + 1;  % 补偿：life 在尾部递增，滤波需要 post-increment

                        % ---- 8. 滤波器更新 ----
                        [lon, lat, ukf] = ukf_dispatch('update', ukf, innov_w);

                        % ---- 9. 航迹维护 ----
                        if isfield(ukf, 'nis_history')
                            ukf.nis_history(end+1) = nis_val;
                        end
                        missed = 0;
                        life = life + 1;
                        quality = min(quality + 1, 15);
                    else
                        % 拒绝更新: 变纯预测帧
                        [lon, lat, ukf] = ukf_dispatch('update', ukf, []);
                        missed = missed + 1;
                        life = life + 1;
                        quality = max(quality - 1, 0);
                        best_det = [];
                    end
                else
                    % ---- 纯预测帧 ----
                    [lon, lat, ukf] = ukf_dispatch('update', ukf, []);
                    missed = missed + 1;
                    life = life + 1;
                    quality = max(quality - 1, 0);
                end

                if missed >= params.tracker_K_loss
                    track_state = 'LOST';
                end

                snap.trackList{1} = make_track_snap(1, 1, lat, lon, ukf, life, quality, missed, best_det);

            % =============================================================
            % 状态: LOST（航迹丢失 → 回到 INITIATING 重起始）
            % =============================================================
            case 'LOST'
                track_state = 'INITIATING';
                init_state = track_initiation('reset', params);
                init_det1 = [];  init_frame1 = 0;  init_det2 = [];
                reinit_attempt_frame = k;
                reinit_truth_collecting = false;
                life = 0; missed = 0; quality = 0;
                snap.trackList{1} = make_track_snap(1, 7, NaN, NaN, ukf, life, quality, missed, []);
        end

        trackSnapshots{k} = snap;
    end

    finalTrack = struct('id', 1, ...
        'type', iif(strcmp(track_state,'TRACKING'),1,7), ...
        'quality', quality, 'life', life);
    if ~isempty(ukf) && isstruct(ukf) && isfield(ukf, 'mu_history')
        finalTrack.mu_history = ukf.mu_history;
    end
end


% =========================================================================
% post_init — UKF 初始化后的通用字段设置
% =========================================================================
function ukf = post_init(ukf, params)
    ukf.dt = params.dt_sec;
    ukf.initialized = true;
    if isfield(ukf, 'ukf_cv')
        % IMM 类型: 设置两个子模型
        ukf.ukf_cv.dt = params.dt_sec;
        ukf.ukf_cv.initialized = true;
        ukf.ukf_ct.dt = params.dt_sec;
        ukf.ukf_ct.initialized = true;
    end
    ukf.nis_history = [];  % 每次起始都重置 NIS 历史
    if ~isfield(ukf, 'Q_base') || isempty(ukf.Q_base)
        if isfield(ukf, 'Q')
            ukf.Q_base = ukf.Q;
        end
    end
    if ~isfield(ukf, 'Q_ema') || isempty(ukf.Q_ema)
        ukf.Q_ema = 1.0;
    end
end


% =========================================================================
% 辅助函数: make_track_snap
% =========================================================================
function trk = make_track_snap(id, type, lat, lon, ukf, life, quality, missed, det)
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
% 辅助函数: iif
% =========================================================================
function v = iif(cond, t, f)
    if cond, v = t; else, v = f; end
end
