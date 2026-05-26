% =========================================================================
% single_track_runner.m
% =========================================================================
% 【功能概述】
%   单目标逐帧航迹管理器，实现从航迹起始到跟踪维持的完整生命周期。
%   采用 M/N 滑窗起始策略 + 模块化 UKF+关联流水线：
%     prepare (预测+量测统计) → NN 关联 → PDA 加权 → UKF 更新
%
% 【数学原理】
%   1. M/N滑窗起始 (Track Initiation):
%      在连续N帧的滑窗中，若至少M帧检测到点迹，触发起始尝试。
%      对首帧x末帧的所有点迹对进行速度检验（30-600 m/s），
%      中间帧点迹靠近配对轨迹的"共识评分"决定最优起始对。
%      共识评分 >= 1 -> 两点差分初始化UKF。
%
%   2. 模块化 UKF+关联流水线 (Tracking):
%      ukf_jichu('prepare', ...) → 预测 + 量测统计
%      nn_associate(...) → 地理预筛 + 马氏距离 NN 关联
%      pda_weight(...) → PDA β 加权新息
%      ukf_jichu('update', ...) → 纯 Kalman 状态/协方差更新
%
% 【输入参数】
%   detList  - cell数组，每帧的点迹结构体数组
%   ukf_tpl  - UKF模板结构体
%   params   - 参数结构体
%   n_frames - 总帧数
%
% 【输出】
%   trackSnapshots - cell数组，(n_frames x 1)，每帧的航迹快照
%   finalTrack     - 结构体，最终航迹状态摘要
% =========================================================================

function [trackSnapshots, finalTrack] = single_track_runner(detList, ukf_tpl, params, n_frames)
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
                    ukf = ukf_jichu('init', ukf_tpl, det1, det2);
                    ukf.dt = params.dt_sec;
                    ukf.initialized = true;
                    ukf.Q_base = ukf.Q;
                    ukf.Q_ema = 1.0;
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
            % 状态: TRACKING（正常跟踪）
            % 模块化流水线：prepare → NN关联 → PDA加权 → UKF更新
            % =============================================================
            case 'TRACKING'
                ukf.dt = params.dt_sec;

                % 1. UKF: 预测 + 量测统计
                [x_pred, P_pred, X_pred, z_pred, Z_pred, P_zz, ukf] = ukf_jichu('prepare', ukf);

                % 2. 关联: NN 找最佳点迹
                [best_det, dets_in_gate] = nn_associate(x_pred, z_pred, P_zz(1:2, 1:2), dets, params, life);

                if ~isempty(best_det)
                    % 3. 关联: PDA 加权新息
                    [innov_w, ~, nis_val] = pda_weight(dets_in_gate, z_pred, P_zz, params);

                    % 4. UKF: 纯 Kalman 更新
                    [lon, lat, ukf] = ukf_jichu('update', ukf, innov_w, z_pred, Z_pred, X_pred, x_pred, P_pred, P_zz);

                    % 航迹维护
                    if ~isfield(ukf, 'nis_history'), ukf.nis_history = []; end
                    ukf.nis_history(end+1) = nis_val;
                    missed = 0;
                    life = life + 1;
                    quality = min(quality + 1, 15);
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

                % 自适应 Q（若启用）
                if params.use_fuzzy_adaptive && life > 12 && isfield(ukf, 'nis_history')
                    ukf = apply_fuzzy_adapt(ukf, params);
                end

                if missed >= params.tracker_K_loss
                    track_state = 'LOST';
                end

                snap.trackList{1} = make_track_snap(1, 1, lat, lon, ukf, life, quality, missed, best_det);

            % =============================================================
            % 状态: LOST（航迹丢失）
            % =============================================================
            case 'LOST'
                track_state = 'INITIATING';
                init_state = track_initiation('reset', params);
                life = 0; missed = 0; quality = 0;
                snap.trackList{1} = make_track_snap(1, 7, NaN, NaN, ukf, life, quality, missed, []);
        end

        trackSnapshots{k} = snap;
    end

    finalTrack = struct('id', 1, ...
        'type', iif(strcmp(track_state,'TRACKING'),1,7), ...
        'quality', quality, 'life', life);
end


% =========================================================================
% apply_fuzzy_adapt — 模糊自适应 Q（简化版，不含机动检测）
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
