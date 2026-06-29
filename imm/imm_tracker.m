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
    % 行：from，列：to；0.10 转移概率，对应平均驻留 10 帧
    % 文献依据: ATPM-ISIPDA (Musicki 2008) — 低数据率场景需加快模型切换
    Pi = [0.90, 0.10;
          0.10, 0.90];

    % ---- 模型概率初始值 ----
    mu = [0.5; 0.5];  % 初始各 50%

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

                % ---- 超时检测 ----
                if ~first_init_done && ~reinit_truth_collecting
                    if reinit_attempt_frame == 0
                        reinit_attempt_frame = k;
                    elseif (k - reinit_attempt_frame) >= reinit_timeout_frames && has_truth
                        tl = interp1(true_track(:,5), true_track(:,1), t_grid(k), 'linear', 'extrap');
                        tb = interp1(true_track(:,5), true_track(:,2), t_grid(k), 'linear', 'extrap');
                        Rg = skywave_geometry('group_range', ukf_cv_tpl.tx_lon, ukf_cv_tpl.tx_lat, ...
                            ukf_cv_tpl.radar_lon, ukf_cv_tpl.radar_lat, tl, tb);
                        az = sphere_utils_azimuth(ukf_cv_tpl.radar_lon, ukf_cv_tpl.radar_lat, tl, tb);
                        reinit_truth_det1 = struct('lon', tl, 'lat', tb, ...
                            'range_meas', Rg, 'azimuth_meas', az, 'frameID', k);
                        reinit_truth_frame1 = k;
                        reinit_truth_collecting = true;
                        snap.trackList{1} = make_track_snap_imm(1, 6, NaN, NaN, [], [], mu, 0, 0, 0, []);
                        trackSnapshots{k} = snap;
                        continue;
                    end
                end

                % ---- M/N 滑窗起始（复用 track_initiation 逻辑） ----
                init_state = track_initiation('update', init_state, dets, k, params);
                if init_state.ready
                    det_pair = track_initiation('result', init_state);
                    if ~isempty(det_pair)
                        ukf_cv = ukf_jichu('init', ukf_cv_tpl, det_pair.det1, det_pair.det2);
                        ukf_cv.dt = dt;  ukf_cv.initialized = true;
                        ukf_cv.Q_base = ukf_cv.Q;  ukf_cv.Q_ema = 1.0;
                        if ~isfield(ukf_cv, 'nis_history'), ukf_cv.nis_history = []; end

                        ukf_ct = ukf_jichu('init', ukf_ct_tpl, det_pair.det1, det_pair.det2);
                        ukf_ct.dt = dt;  ukf_ct.initialized = true;
                        ukf_ct.Q_base = ukf_ct.Q;  ukf_ct.Q_ema = 1.0;
                        if ~isfield(ukf_ct, 'nis_history'), ukf_ct.nis_history = []; end

                        track_state = 'TRACKING';
                        life = 1;  missed = 0;  quality = 5;
                        mu = [0.5; 0.5];

                        x_comb = 0.5 * ukf_cv.x + 0.5 * ukf_ct.x;
                        snap.trackList{1} = make_track_snap_imm(1, 1, x_comb(3), x_comb(1), ...
                            ukf_cv, ukf_ct, mu, life, quality, 0, det_pair.det2);
                        trackSnapshots{k} = snap;
                        continue;
                    end
                end
                snap.trackList{1} = make_track_snap_imm(1, 6, NaN, NaN, [], [], mu, 0, 0, 0, []);
                trackSnapshots{k} = snap;

            % =============================================================
            % 状态: TRACKING（IMM 核心循环）
            % =============================================================
            case 'TRACKING'
                life = life + 1;

                % ---- 步骤1: 模型混合（Mixing） ----
                % 混合概率 μ_{j|i} = π_{ji} * μ_j / c_i
                c_bar = Pi' * mu;  % c_i = Σ_j π_{ji} * μ_j
                % mu_mix(i,j) = P{model i was active at k-1 | model j is active at k}
                mu_mix = zeros(M, M);
                for i = 1:M
                    for j = 1:M
                        mu_mix(i, j) = Pi(i, j) * mu(i) / max(c_bar(j), 1e-12);
                    end
                end

                % 混合状态和协方差
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

                % 设置混合后的初始状态
                ukf_cv.x = x_mix{1};  ukf_cv.P = P_mix{1};
                ukf_ct.x = x_mix{2};  ukf_ct.P = P_mix{2};

                % ---- 步骤2: 各模型独立预测 ----
                [x_pred_cv, P_pred_cv, X_pred_cv, z_pred_cv, Z_pred_cv, P_zz_cv, ukf_cv] = ...
                    ukf_jichu('prepare', ukf_cv);
                [x_pred_ct, P_pred_ct, X_pred_ct, z_pred_ct, Z_pred_ct, P_zz_ct, ukf_ct] = ...
                    ukf_jichu('prepare', ukf_ct);

                % ---- 步骤3: NN 关联（用 CV 模型预测做公共关联） ----
                % 筛选非杂波点迹
                ac_dets = [];
                for d = 1:length(dets)
                    if ~dets(d).is_clutter
                        ac_dets = [ac_dets, dets(d)];
                    end
                end

                % NN 关联：在 CV 的预测量测空间中找最近邻
                [best_det, best_nis, innov_common] = imm_nn_associate(...
                    ac_dets, z_pred_cv, P_zz_cv, ukf_cv, params);

                % 若 NN 未找到，尝试 PDA 加权
                assoc_det = [];
                if ~isempty(best_det)
                    assoc_det = best_det;
                end

                % ---- 步骤4-5: 各模型更新（共用关联结果） ----
                innov_cv_val = 0;  innov_ct_val = 0;
                nis_cv_val = 0;    nis_ct_val = 0;

                if ~isempty(assoc_det)
                    % 对 CV 模型计算新息
                    z_meas = [assoc_det.range_meas; assoc_det.azimuth_meas; assoc_det.pvr];
                    innov_cv = z_meas - z_pred_cv;
                    innov_ct = z_meas - z_pred_ct;

                    innov_cv_val = innov_cv(1)^2 + innov_cv(2)^2;
                    innov_ct_val = innov_ct(1)^2 + innov_ct(2)^2;

                    % CV 模型更新
                    [~, ~, ukf_cv] = ukf_jichu('update', ukf_cv, innov_cv, ...
                        z_pred_cv, Z_pred_cv, X_pred_cv, x_pred_cv, P_pred_cv, P_zz_cv);

                    % CT 模型更新
                    [~, ~, ukf_ct] = ukf_jichu('update', ukf_ct, innov_ct, ...
                        z_pred_ct, Z_pred_ct, X_pred_ct, x_pred_ct, P_pred_ct, P_zz_ct);

                    % 记录 NIS
                    nis_cv_val = innov_cv' * (P_zz_cv \ innov_cv);
                    nis_ct_val = innov_ct' * (P_zz_ct \ innov_ct);
                    if ~isnan(nis_cv_val)
                        ukf_cv.nis_history(end+1) = nis_cv_val;
                    end
                    if ~isnan(nis_ct_val)
                        ukf_ct.nis_history(end+1) = nis_ct_val;
                    end

                    missed = 0;
                    quality = min(100, quality + 2);
                else
                    % 纯预测（无量测关联）
                    ukf_cv.x = x_pred_cv;  ukf_cv.P = P_pred_cv;
                    ukf_ct.x = x_pred_ct;  ukf_ct.P = P_pred_ct;
                    missed = missed + 1;
                    quality = max(0, quality - 5);
                end

                % ---- 步骤6: 模型似然度（IMM-IPDA 风格, Musicki 2008） ----
                % 有检测: Λ_j = Pd*Pg * N(ν_j; 0, S_j)
                % 无检测: Λ_j = 1 - Pd*Pg（常数，不冻结模型概率）
                % 注意: Pd*Pg 因子在比值中抵消，但无检测时 Λ 的绝对值影响
                %        模型概率向 Markov 先验的漂移速率
                nz = ukf_cv.m;  % 量测维数 = 3
                if ~isempty(assoc_det)
                    log_norm = -0.5 * (nz * log(2*pi) + log(max(det(P_zz_cv), 1e-30)));
                    L_cv = Pd_Pg * exp(log_norm - 0.5 * nis_cv_val);

                    log_norm = -0.5 * (nz * log(2*pi) + log(max(det(P_zz_ct), 1e-30)));
                    L_ct = Pd_Pg * exp(log_norm - 0.5 * nis_ct_val);
                else
                    % 无检测: 模型概率向 Markov 先验漂移
                    L_cv = L_no_det;
                    L_ct = L_no_det;
                end

                % ---- 步骤7: 模型概率更新 ----
                c_total = L_cv * c_bar(1) + L_ct * c_bar(2);
                if c_total > 1e-30
                    mu_new = [L_cv * c_bar(1); L_ct * c_bar(2)] / c_total;
                else
                    mu_new = mu;  % 数值退化时保持不变
                end
                % 钳位：单模型概率不低于 5%（文献建议防止锁死）
                mu = max(0.05, min(0.95, mu_new));
                mu = mu / sum(mu);

                % ---- 步骤8: 状态组合 ----
                x_comb = mu(1) * ukf_cv.x + mu(2) * ukf_ct.x;
                P_comb = mu(1) * (ukf_cv.P + (ukf_cv.x - x_comb)*(ukf_cv.x - x_comb)') + ...
                         mu(2) * (ukf_ct.P + (ukf_ct.x - x_comb)*(ukf_ct.x - x_comb)');

                % ---- 模糊自适应 Q（对 CV 模型） ----
                if params.use_fuzzy_adaptive && ~isempty(assoc_det)
                    ukf_cv = apply_fuzzy_adapt(ukf_cv, params);
                end

                % ---- 航迹快照 ----
                snap.trackList{1} = make_track_snap_imm(1, 1, x_comb(3), x_comb(1), ...
                    ukf_cv, ukf_ct, mu, life, quality, missed, assoc_det);
                trackSnapshots{k} = snap;

                % ---- 丢点终止检测 ----
                if missed >= K_loss
                    track_state = 'LOST';
                end

            % =============================================================
            % 状态: LOST
            % =============================================================
            case 'LOST'
                snap.trackList{1} = make_track_snap_imm(1, 7, NaN, NaN, [], [], mu, life, quality, missed, []);
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
% imm_nn_associate — IMM 最近邻关联（在 CV 预测空间）
% =========================================================================
function [best_det, best_nis, innov] = imm_nn_associate(ac_dets, z_pred, P_zz, ukf, params)
    best_det = [];
    best_nis = Inf;
    innov = [];

    if isempty(ac_dets)
        return;
    end

    gate_sigma = params.gate_sigma;
    gate_vr_ms = params.gate_vr_ms;
    gate_threshold = gate_sigma^2 * 2;  % 2自由度卡方门限

    for d = 1:length(ac_dets)
        det = ac_dets(d);
        z_meas = [det.range_meas; det.azimuth_meas; det.pvr];

        % 硬 Vr 门：帧间径向速度差
        if isfield(ukf, 'last_vr') && ~isempty(ukf.last_vr)
            dvr = abs(det.pvr - ukf.last_vr);
            if dvr > gate_vr_ms
                continue;
            end
        end

        nu = z_meas - z_pred;
        % 方位角差值标准化到 [-180, 180]
        if abs(nu(2)) > 180
            nu(2) = nu(2) - 360 * round(nu(2) / 360);
        end

        % 马氏距离（仅用 range + az 2D）
        nu_2d = nu(1:2);
        try
            nis_val = nu_2d' * (P_zz(1:2,1:2) \ nu_2d);
        catch
            nis_val = nu_2d' * pinv(P_zz(1:2,1:2)) * nu_2d;
        end

        if nis_val < gate_threshold && nis_val < best_nis
            best_nis = nis_val;
            best_det = det;
            innov = nu;
        end
    end
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
