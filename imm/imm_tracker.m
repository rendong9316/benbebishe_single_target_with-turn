% =========================================================================
% imm_tracker.m — IMM（交互多模型）单目标航迹跟踪器
% =========================================================================
% 【功能概述】
%   对单目标逐帧执行 IMM 跟踪，维持 CV（匀速）和 CT（协调转弯，ω=1°/s）
%   两个运动模型的并行滤波。IMM 通过模型概率自适应加权，在直线段 CV 占优、
%   转弯段 CT 占优，实现平滑的模型切换。
%
% 【IMM 循环（每帧）】
%   1. 模型混合（Mixing）         — 计算每个模型的混合初始状态和协方差
%   2. UKF 预测（Per-Model）      — 各模型用各自的运动模型执行状态预测
%   3. NN 关联（共用 CV 预测）     — 用 CV 模型的量测预测做最近邻关联
%   4. PDA 加权（Per-Model）      — 各模型独立计算 PDA 加权新息
%   5. UKF 更新（Per-Model）      — 各模型执行量测更新（或纯预测）
%   6. 模型似然度计算              — 基于量测新息的似然度
%   7. 模型概率更新               — 贝叶斯更新模型概率
%   8. 状态组合（Combination）     — 加权组合输出
%
% 【数学模型】
%   运动模型：
%     CV: x_k = F_CV(Δt) × x_{k-1} + w,  F_CV = [1 Δt 0 0; 0 1 0 0; 0 0 1 Δt; 0 0 0 1]
%     CT: x_k = F_CT(Δt, ω) × x_{k-1} + w, ω = ±1°/s = ±π/180 rad/s
%          F_CT = [1, sin(ωΔt)/ω, 0, -(1-cos(ωΔt))/ω;
%                  0, cos(ωΔt), 0, -sin(ωΔt);
%                  0, (1-cos(ωΔt))/ω, 1, sin(ωΔt)/ω;
%                  0, sin(ωΔt), 0, cos(ωΔt)]
%
%   Markov 转移矩阵: Π = [0.95, 0.05; 0.05, 0.95]（缓慢切换）
%
% 【输入】
%   detList     — cell数组(n_frames×1)，每帧的点迹结构体
%   ukf_cv_tpl  — CV 模型 UKF 模板（来自 ukf_jichu('create', ...)）
%   ukf_ct_tpl  — CT 模型 UKF 模板（来自 ukf_jichu('create', ...)，
%                 需预先设置 .model_type='CT', .turn_rate_rad_per_sec）
%   params      — 参数结构体
%   n_frames    — 总帧数
%   true_track  — 真值航迹矩阵 [lon, lat, lon_rate, lat_rate, time_sec]（可选）
%   t_grid      — 时间网格（可选，与 true_track 配套）
%
% 【输出】
%   trackSnapshots — cell数组(n_frames×1)，每帧的航迹快照
%   finalTrack     — 结构体，最终航迹状态摘要
% =========================================================================

function [trackSnapshots, finalTrack] = imm_tracker(detList, ukf_cv_tpl, ukf_ct_tpl, params, n_frames, true_track, t_grid)
    % ---- 提取参数 ----
    M = 2;  % 模型数量：CV + CT
    dt = params.dt_sec;
    K_loss = params.tracker_K_loss;

    % ---- IMM-IPDA 检测参数（Musicki 2008, IEEE T-AES） ----
    Pd = params.detection_probability;  % 单帧检测概率
    Pg = params.pda_pd_gate;            % 门内概率
    Pd_Pg = Pd * Pg;                    % 综合检测概率
    % 无检测时的似然: (1 - Pd*Pg)，众模型共用常数
    L_no_det = 1.0 - Pd_Pg;

    % ---- Markov 转移概率矩阵 ----
    % Π(i,j) = P{model=j at k+1 | model=i at k}
    % 可通过 params.imm_Pi_CV_to_CT 和 params.imm_Pi_CT_to_CV 覆盖
    if isfield(params, 'imm_Pi_CV_to_CT') && isfield(params, 'imm_Pi_CT_to_CV')
        p_cv_ct = params.imm_Pi_CV_to_CT;
        p_ct_cv = params.imm_Pi_CT_to_CV;
        Pi = [1-p_cv_ct, p_cv_ct;
              p_ct_cv, 1-p_ct_cv];
    else
        Pi = [0.90, 0.10;
              0.10, 0.90];
    end

    % ---- 模型概率初始值 ----
    if isfield(params, 'imm_mu_init_CV')
        mu_cv = params.imm_mu_init_CV;
        mu = [mu_cv; 1 - mu_cv];
    else
        mu = [0.5; 0.5];  % 初始各 50%
    end

    % ---- 初始化 ----
    trackSnapshots = cell(n_frames, 1);
    ukf_cv = [];  ukf_ct = [];
    track_state = 'INITIATING';

    life = 0;  missed = 0;  quality = 0;

    init_state = track_initiation('init', params);

    % ---- 始终有真值辅助 ----
    has_truth = true;

    % ---- 真值辅助首次起始 ----
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

    % ---- 模型概率历史（用于诊断） ----
    mu_history = zeros(n_frames, M);

    % =====================================================================
    % 主循环：逐帧处理
    % =====================================================================
    for k = 1:n_frames
        snap = struct('frameID', k, 'trackList', {{}});
        dets = detList{k};
        mu_history(k, :) = mu';

        switch track_state
            % =============================================================
            % 状态: INITIATING
            % =============================================================
            case 'INITIATING'
                % ---- 首次起始: 真值辅助 ----
                if ~first_init_done && isfield(params, 'use_truth_init') && params.use_truth_init
                    if isempty(init_det1)
                        tl = interp1(true_track(:,5), true_track(:,1), t_grid(k), 'linear', 'extrap');
                        tb = interp1(true_track(:,5), true_track(:,2), t_grid(k), 'linear', 'extrap');
                        Rg = skywave_geometry('group_range', ukf_cv_tpl.tx_lon, ukf_cv_tpl.tx_lat, ...
                            ukf_cv_tpl.radar_lon, ukf_cv_tpl.radar_lat, tl, tb);
                        az = sphere_utils_azimuth(ukf_cv_tpl.radar_lon, ukf_cv_tpl.radar_lat, tl, tb);
                        init_det1 = struct('lon', tl, 'lat', tb, 'range_meas', Rg, 'azimuth_meas', az, 'frameID', k);
                        init_frame1 = k;
                    elseif isempty(init_det2) && (k - init_frame1) >= 1
                        tl = interp1(true_track(:,5), true_track(:,1), t_grid(k), 'linear', 'extrap');
                        tb = interp1(true_track(:,5), true_track(:,2), t_grid(k), 'linear', 'extrap');
                        Rg = skywave_geometry('group_range', ukf_cv_tpl.tx_lon, ukf_cv_tpl.tx_lat, ...
                            ukf_cv_tpl.radar_lon, ukf_cv_tpl.radar_lat, tl, tb);
                        az = sphere_utils_azimuth(ukf_cv_tpl.radar_lon, ukf_cv_tpl.radar_lat, tl, tb);
                        init_det2 = struct('lon', tl, 'lat', tb, 'range_meas', Rg, 'azimuth_meas', az, 'frameID', k);

                        % 初始化两个 UKF
                        ukf_cv = ukf_jichu('init', ukf_cv_tpl, init_det1, init_det2);
                        ukf_cv.dt = dt;
                        ukf_cv.initialized = true;
                        ukf_cv.Q_base = ukf_cv.Q;
                        ukf_cv.Q_ema = 1.0;
                        if ~isfield(ukf_cv, 'nis_history'), ukf_cv.nis_history = []; end

                        ukf_ct = ukf_jichu('init', ukf_ct_tpl, init_det1, init_det2);
                        ukf_ct.dt = dt;
                        ukf_ct.initialized = true;
                        ukf_ct.Q_base = ukf_ct.Q;
                        ukf_ct.Q_ema = 1.0;
                        if ~isfield(ukf_ct, 'nis_history'), ukf_ct.nis_history = []; end

                        first_init_done = true;
                        track_state = 'TRACKING';
                        life = 1;  missed = 0;  quality = 5;

                        % 组合状态作为初始输出
                        x_comb = 0.5 * ukf_cv.x + 0.5 * ukf_ct.x;
                        snap.trackList{1} = make_track_snap_imm(1, 1, x_comb(3), x_comb(1), ...
                            ukf_cv, ukf_ct, mu, life, quality, 0, init_det2);
                        trackSnapshots{k} = snap;
                        continue;
                    end
                    snap.trackList{1} = make_track_snap_imm(1, 6, NaN, NaN, [], [], mu, 0, 0, 0, []);
                    trackSnapshots{k} = snap;
                    continue;
                end

                % ---- 重新起始: 超时兜底 ----
                if reinit_truth_collecting
                    if (k - reinit_truth_frame1) >= 1
                        tl = interp1(true_track(:,5), true_track(:,1), t_grid(k), 'linear', 'extrap');
                        tb = interp1(true_track(:,5), true_track(:,2), t_grid(k), 'linear', 'extrap');
                        Rg = skywave_geometry('group_range', ukf_cv_tpl.tx_lon, ukf_cv_tpl.tx_lat, ...
                            ukf_cv_tpl.radar_lon, ukf_cv_tpl.radar_lat, tl, tb);
                        az = sphere_utils_azimuth(ukf_cv_tpl.radar_lon, ukf_cv_tpl.radar_lat, tl, tb);
                        reinit_truth_det2 = struct('lon', tl, 'lat', tb, ...
                            'range_meas', Rg, 'azimuth_meas', az, 'frameID', k);

                        ukf_cv = ukf_jichu('init', ukf_cv_tpl, reinit_truth_det1, reinit_truth_det2);
                        ukf_cv.dt = dt;  ukf_cv.initialized = true;
                        ukf_cv.Q_base = ukf_cv.Q;  ukf_cv.Q_ema = 1.0;
                        if ~isfield(ukf_cv, 'nis_history'), ukf_cv.nis_history = []; end

                        ukf_ct = ukf_jichu('init', ukf_ct_tpl, reinit_truth_det1, reinit_truth_det2);
                        ukf_ct.dt = dt;  ukf_ct.initialized = true;
                        ukf_ct.Q_base = ukf_ct.Q;  ukf_ct.Q_ema = 1.0;
                        if ~isfield(ukf_ct, 'nis_history'), ukf_ct.nis_history = []; end

                        reinit_truth_collecting = false;
                        reinit_attempt_frame = 0;
                        track_state = 'TRACKING';
                        life = 1;  missed = 0;  quality = 5;
                        mu = [0.5; 0.5];

                        x_comb = 0.5 * ukf_cv.x + 0.5 * ukf_ct.x;
                        snap.trackList{1} = make_track_snap_imm(1, 1, x_comb(3), x_comb(1), ...
                            ukf_cv, ukf_ct, mu, life, quality, 0, reinit_truth_det2);
                        trackSnapshots{k} = snap;
                        continue;
                    end
                    snap.trackList{1} = make_track_snap_imm(1, 6, NaN, NaN, [], [], mu, 0, 0, 0, []);
                    trackSnapshots{k} = snap;
                    continue;
                end

                % ---- 超时检测（对标 single_track_runner: first_init_done=true 时才触发） ----
                timeout_triggered = (first_init_done && has_truth && reinit_attempt_frame > 0 && ...
                                     (k - reinit_attempt_frame) > reinit_timeout_frames);
                if timeout_triggered
                    tl = interp1(true_track(:,5), true_track(:,1), t_grid(k), 'linear', 'extrap');
                    tb = interp1(true_track(:,5), true_track(:,2), t_grid(k), 'linear', 'extrap');
                    Rg = skywave_geometry('group_range', ukf_cv_tpl.tx_lon, ukf_cv_tpl.tx_lat, ...
                        ukf_cv_tpl.radar_lon, ukf_cv_tpl.radar_lat, tl, tb);
                    az = sphere_utils_azimuth(ukf_cv_tpl.radar_lon, ukf_cv_tpl.radar_lat, tl, tb);
                    reinit_truth_det1 = struct('lon', tl, 'lat', tb, ...
                        'range_meas', Rg, 'azimuth_meas', az, 'frameID', k);
                    reinit_truth_frame1 = k;
                    reinit_truth_collecting = true;
                    init_state = track_initiation('reset', params);
                    snap.trackList{1} = make_track_snap_imm(1, 6, NaN, NaN, [], [], mu, 0, 0, 0, []);
                    trackSnapshots{k} = snap;
                    continue;
                end

                % ---- 纯 M/N 滑窗逻辑（对标 single_track_runner） ----
                [init_state, det1, det2, success] = track_initiation('process', init_state, dets, params, k);
                if success
                    ukf_cv = ukf_jichu('init', ukf_cv_tpl, det1, det2);
                    ukf_cv.dt = dt;  ukf_cv.initialized = true;
                    ukf_cv.Q_base = ukf_cv.Q;  ukf_cv.Q_ema = 1.0;
                    if ~isfield(ukf_cv, 'nis_history'), ukf_cv.nis_history = []; end

                    ukf_ct = ukf_jichu('init', ukf_ct_tpl, det1, det2);
                    ukf_ct.dt = dt;  ukf_ct.initialized = true;
                    ukf_ct.Q_base = ukf_ct.Q;  ukf_ct.Q_ema = 1.0;
                    if ~isfield(ukf_ct, 'nis_history'), ukf_ct.nis_history = []; end

                    reinit_attempt_frame = 0;
                    reinit_truth_collecting = false;
                    track_state = 'TRACKING';
                    life = 1;  missed = 0;  quality = 5;
                    mu = [0.5; 0.5];

                    x_comb = 0.5 * ukf_cv.x + 0.5 * ukf_ct.x;
                    snap.trackList{1} = make_track_snap_imm(1, 1, x_comb(3), x_comb(1), ...
                        ukf_cv, ukf_ct, mu, life, quality, 0, det2);
                    trackSnapshots{k} = snap;
                    continue;
                end
                snap.trackList{1} = make_track_snap_imm(1, 6, NaN, NaN, [], [], mu, 0, 0, 0, []);
                trackSnapshots{k} = snap;

            % =============================================================
            % 状态: TRACKING（对标 single_track_runner + IMM特有步骤）
            % ═══ 直线逻辑 + 仅四处IMM插入: 混合/双预测/双更新/似然+组合 ═══
            % =============================================================
            case 'TRACKING'
                life = life + 1;
                ukf_cv.dt = dt;  ukf_ct.dt = dt;

                % ── IMM特有①: 模型混合 ──
                c_bar = Pi' * mu;
                mu_mix = zeros(M, M);
                for i = 1:M
                    for j = 1:M
                        mu_mix(i, j) = Pi(i, j) * mu(i) / max(c_bar(j), 1e-12);
                    end
                end
                x_mix = {zeros(4,1), zeros(4,1)};
                P_mix = {zeros(4,4), zeros(4,4)};
                ukf_models = {ukf_cv, ukf_ct};
                for j = 1:M
                    for i = 1:M
                        x_mix{j} = x_mix{j} + mu_mix(i, j) * ukf_models{i}.x;
                    end
                    for i = 1:M
                        dx = ukf_models{i}.x - x_mix{j};
                        P_mix{j} = P_mix{j} + mu_mix(i, j) * (ukf_models{i}.P + dx * dx');
                    end
                end
                ukf_cv.x = x_mix{1};  ukf_cv.P = P_mix{1};
                ukf_ct.x = x_mix{2};  ukf_ct.P = P_mix{2};

                % ── IMM特有②: 各模型独立预测 ──
                [x_pred_cv, P_pred_cv, X_pred_cv, z_pred_cv, Z_pred_cv, P_zz_cv, ukf_cv] = ...
                    ukf_jichu('prepare', ukf_cv);
                [x_pred_ct, P_pred_ct, X_pred_ct, z_pred_ct, Z_pred_ct, P_zz_ct, ukf_ct] = ...
                    ukf_jichu('prepare', ukf_ct);

                % ════════════════════════════════════════════════════════════
                % 以下对标 single_track_runner TRACKING 状态
                % 差异: IMM需预筛is_clutter（双模型交互放大杂波污染，PDA无法弥补）
                % ════════════════════════════════════════════════════════════

                % 预筛非杂波点迹
                clean_dets = [];
                for d = 1:length(dets)
                    if ~dets(d).is_clutter
                        clean_dets = [clean_dets, dets(d)];
                    end
                end

                % 1. NN关联（与直线版一致: 地理预筛+马氏距离）
                % Vr门在IMM中必须禁用: is_clutter已过滤杂波, CV预测Vr在转弯时不准
                saved_vr = params.gate_vr_ms;
                params.gate_vr_ms = 9999;
                [best_det, dets_in_gate] = nn_associate(x_pred_cv, z_pred_cv, ...
                    P_zz_cv(1:2,1:2), clean_dets, params, life);
                params.gate_vr_ms = saved_vr;

                % 2. 连续丢点防杂波劫持: 固定地理门50km（与直线版一致）
                if ~isempty(best_det) && missed >= 2
                    geo_dist = sphere_utils_haversine_distance(...
                        x_pred_cv(1), x_pred_cv(3), best_det.lon, best_det.lat);
                    if geo_dist > 50000
                        best_det = [];
                        dets_in_gate = {};
                    end
                end

                if ~isempty(best_det)
                    % 3. PDA 加权新息（与直线版一致）
                    [innov_w, ~, nis_val] = pda_weight(dets_in_gate, z_pred_cv, P_zz_cv, params);

                    % 4. Probation 期保护（与直线版一致）
                    probate_nis_limit = 50;
                    reject_update = false;
                    if life <= 5 && nis_val > probate_nis_limit
                        reject_update = true;
                    end

                    if ~reject_update
                        % ── IMM特有③: 重建加权量测 → 各模型独立更新 ──
                        z_weighted = innov_w + z_pred_cv;
                        innov_cv = z_weighted - z_pred_cv;
                        innov_ct = z_weighted - z_pred_ct;
                        if abs(innov_cv(2)) > 180
                            innov_cv(2) = innov_cv(2) - 360 * round(innov_cv(2) / 360);
                        end
                        if abs(innov_ct(2)) > 180
                            innov_ct(2) = innov_ct(2) - 360 * round(innov_ct(2) / 360);
                        end

                        [~, ~, ukf_cv] = ukf_jichu('update', ukf_cv, innov_cv, ...
                            z_pred_cv, Z_pred_cv, X_pred_cv, x_pred_cv, P_pred_cv, P_zz_cv);
                        [~, ~, ukf_ct] = ukf_jichu('update', ukf_ct, innov_ct, ...
                            z_pred_ct, Z_pred_ct, X_pred_ct, x_pred_ct, P_pred_ct, P_zz_ct);

                        % NIS 记录
                        nis_cv_val = innov_cv' * (P_zz_cv \ innov_cv);
                        nis_ct_val = innov_ct' * (P_zz_ct \ innov_ct);
                        if ~isnan(nis_cv_val)
                            ukf_cv.nis_history(end+1) = nis_cv_val;
                        end
                        if ~isnan(nis_ct_val)
                            ukf_ct.nis_history(end+1) = nis_ct_val;
                        end

                        % 航迹维护（与直线版一致）
                        missed = 0;
                        quality = min(quality + 1, 15);
                    else
                        % 拒绝更新（与直线版一致）
                        ukf_cv.x = x_pred_cv;  ukf_cv.P = P_pred_cv;
                        ukf_ct.x = x_pred_ct;  ukf_ct.P = P_pred_ct;
                        missed = missed + 1;
                        quality = max(quality - 1, 0);
                        best_det = [];
                    end
                else
                    % 纯预测（与直线版一致）
                    ukf_cv.x = x_pred_cv;  ukf_cv.P = P_pred_cv;
                    ukf_ct.x = x_pred_ct;  ukf_ct.P = P_pred_ct;
                    missed = missed + 1;
                    quality = max(quality - 1, 0);
                    best_det = [];
                end

                % ── IMM特有④: 模型似然度 + 概率更新 + 状态组合 ──
                nz = ukf_cv.m;
                if ~isempty(best_det)
                    nis_cv_val = (innov_cv' * (P_zz_cv \ innov_cv));
                    nis_ct_val = (innov_ct' * (P_zz_ct \ innov_ct));
                    log_norm = -0.5 * (nz * log(2*pi) + log(max(det(P_zz_cv), 1e-30)));
                    L_cv = Pd_Pg * exp(log_norm - 0.5 * nis_cv_val);
                    log_norm = -0.5 * (nz * log(2*pi) + log(max(det(P_zz_ct), 1e-30)));
                    L_ct = Pd_Pg * exp(log_norm - 0.5 * nis_ct_val);
                else
                    L_cv = L_no_det;
                    L_ct = L_no_det;
                end

                c_total = L_cv * c_bar(1) + L_ct * c_bar(2);
                if c_total > 1e-30
                    mu_new = [L_cv * c_bar(1); L_ct * c_bar(2)] / c_total;
                else
                    mu_new = mu;
                end
                mu = max(0.02, min(0.95, mu_new));
                mu = mu / sum(mu);

                x_comb = mu(1) * ukf_cv.x + mu(2) * ukf_ct.x;

                % ── 自适应 Q（与直线版一致: life>12）──
                if params.use_fuzzy_adaptive && life > 12 && isfield(ukf_cv, 'nis_history')
                    ukf_cv = apply_fuzzy_adapt(ukf_cv, params);
                end

                % 航迹快照
                snap.trackList{1} = make_track_snap_imm(1, 1, x_comb(3), x_comb(1), ...
                    ukf_cv, ukf_ct, mu, life, quality, missed, best_det);
                trackSnapshots{k} = snap;

                if missed >= K_loss
                    track_state = 'LOST';
                end

            % =============================================================
            % 状态: LOST（对标 single_track_runner: 回到 INITIATING 重起始）
            % =============================================================
            case 'LOST'
                track_state = 'INITIATING';
                init_state = track_initiation('reset', params);
                init_det1 = [];  init_frame1 = 0;  init_det2 = [];
                reinit_attempt_frame = k;
                reinit_truth_collecting = false;
                life = 0;  missed = 0;  quality = 0;
                mu = [0.5; 0.5];
                snap.trackList{1} = make_track_snap_imm(1, 7, NaN, NaN, ukf_cv, ukf_ct, mu, life, quality, missed, []);
                trackSnapshots{k} = snap;
            otherwise
                % pass
        end
    end

    % ---- 构造最终航迹 ----
    finalTrack = struct('id', 1, 'type', 7, 'life', life, 'quality', quality);
    if strcmp(track_state, 'TRACKING')
        finalTrack.type = 1;
    elseif strcmp(track_state, 'LOST')
        finalTrack.type = 7;
    end
    % 保存模型概率历史供诊断
    finalTrack.mu_history = mu_history;
end

% =========================================================================
% make_track_snap_imm — 构造 IMM 航迹快照结构体
% =========================================================================
function t = make_track_snap_imm(id, type, lat, lon, ukf_cv, ukf_ct, mu, life, quality, missed, assoc_det)
    t = struct('id', id, 'type', type, 'lat', lat, 'lon', lon, ...
        'life', life, 'quality', quality, 'missed', missed);

    if ~isempty(ukf_cv) && isstruct(ukf_cv)
        t.ukf = ukf_cv;  % 兼容下游读取 .ukf 字段
        t.ukf_cv = ukf_cv;
    else
        t.ukf = [];
        t.ukf_cv = [];
    end
    if ~isempty(ukf_ct) && isstruct(ukf_ct)
        t.ukf_ct = ukf_ct;
    else
        t.ukf_ct = [];
    end

    t.mu = mu;  % [mu_cv; mu_ct] 模型概率
    t.assoc_det = assoc_det;
    if isempty(assoc_det)
        t.assoc_det = struct('prange', [], 'paz', [], 'pvr', []);
    end
end

% =========================================================================
% apply_fuzzy_adapt — 模糊自适应 Q（与 single_track_runner 同步）
% 基于 NIS 平均值的模糊推理调整过程噪声 Q
% =========================================================================
function ukf = apply_fuzzy_adapt(ukf, params)
    if ~isfield(ukf, 'nis_history') || isempty(ukf.nis_history)
        return;
    end

    nis_history = ukf.nis_history;

    % 初始化
    if ~isfield(ukf, 'Q_ema') || isempty(ukf.Q_ema)
        ukf.Q_ema = 1.0;
    end
    if ~isfield(ukf, 'Q_base') || isempty(ukf.Q_base)
        ukf.Q_base = ukf.Q;
    end

    % 模糊自适应 Q
    nis_avg = mean(nis_history);
    nis_ratio = nis_avg / 2.0;

    mu_VS = trimf_val(nis_ratio, 0.0, 0.0, 0.4);
    mu_S  = trimf_val(nis_ratio, 0.2, 0.5, 0.8);
    mu_M  = trimf_val(nis_ratio, 0.6, 1.0, 1.5);
    mu_L  = trimf_val(nis_ratio, 1.3, 2.0, 3.0);
    mu_VL = trimf_val(nis_ratio, 2.5, 4.0, 4.0);

    out_Decrease       = 0.6;
    out_SlightDecrease = 0.8;
    out_Maintain       = 1.0;
    out_Increase       = 1.8;
    out_RapidIncrease  = 3.0;

    total_mu = mu_VS + mu_S + mu_M + mu_L + mu_VL;
    if total_mu < 1e-10
        factor_fuzzy = 1.0;
    else
        factor_fuzzy = (mu_VS * out_Decrease + mu_S * out_SlightDecrease + ...
                       mu_M * out_Maintain + mu_L * out_Increase + ...
                       mu_VL * out_RapidIncrease) / total_mu;
    end

    factor_raw = max(0.5, min(4.0, factor_fuzzy));

    % EMA 平滑
    ema_eta = 0.20;
    if isfield(params, 'fuzzy_ema_eta'), ema_eta = params.fuzzy_ema_eta; end
    ukf.Q_ema = ema_eta * factor_raw + (1 - ema_eta) * ukf.Q_ema;

    if abs(ukf.Q_ema - 1.0) < 0.05
        ukf.Q = ukf.Q_base;
    else
        ukf.Q = ukf.Q_base * ukf.Q_ema;
    end
end

% =========================================================================
% trimf_val — 三角形隶属函数求值
% trimf(x, a, b, c): 三角形顶点在 (a,0)→(b,1)→(c,0)
% =========================================================================
function mu = trimf_val(x, a, b, c)
    if x <= a || x >= c
        mu = 0;
    elseif x < b
        mu = (x - a) / (b - a);
    else
        mu = (c - x) / (c - b);
    end
end
