% =========================================================================
% single_track_runner.m
% =========================================================================
% 【功能概述】
%   单目标逐帧航迹管理器，实现从航迹起始到跟踪维持的完整生命周期。
%   采用 M/N 滑窗起始策略 + UKF预测 + 最近邻（NN）关联 + PDA更新
%   的经典框架，适用于单目标场景的航迹处理。
%
% 【数学原理】
%   1. M/N滑窗起始 (Track Initiation):
%      在连续N帧的滑窗中，若至少M帧检测到点迹，触发起始尝试。
%      对首帧×末帧的所有点迹对进行速度检验（30-600 m/s），
%      中间帧点迹靠近配对轨迹的"共识评分"决定最优起始对。
%      共识评分 ≥ 1 → 两点差分初始化UKF。
%
%      速度估计: v_est = dist_haversine / (dt * n_frames_between)
%
%   2. UKF预测-更新循环:
%      - 预测: Sigma点传播状态方程得到先验估计
%      - 量测预测: Sigma点传播量测方程得到 z_pred, P_zz
%      - 最近邻关联: 地理距离预筛选 + 马氏距离门限选择最佳点迹
%      - PDA更新: 收集波门内所有点迹，加权融合更新状态
%
%   3. 航迹终止:
%      连续漏检帧数 ≥ K_loss → 状态转为 LOST → 重新起始
%
% 【输入参数】
%   detList  - cell数组，每帧的点迹结构体数组
%              detList{k} = 第k帧的点迹结构体数组
%   ukf_tpl  - UKF模板结构体
%   params   - 参数结构体，需包含:
%              .tracker_N: M/N起始滑窗帧数N
%              .tracker_M: M/N起始最少点迹数M
%              .dt_sec: 帧时间间隔（秒）
%              .gate_sigma: 波门Sigma倍数
%              .tracker_K_loss: 最大连续漏检帧数
%              .fuzzy_window_size: 模糊自适应窗口大小
%              .use_fuzzy_adaptive: 是否启用模糊自适应Q
%   n_frames - 总帧数
%
% 【输出】
%   trackSnapshots - cell数组，(n_frames x 1)，每帧的航迹快照
%                    snap.trackList{1} 为单条航迹的结构体
%   finalTrack     - 结构体，最终航迹状态摘要
%                    .id: 航迹编号 = 1
%                    .type: 1=TRACKING, 7=LOST/HISTORY
%                    .quality: 最终质量评分
%                    .life: 总生命周期（帧数）
%
% 【调用关系】
%   被主脚本或单目标评估脚本调用
%   子调用: sphere_utils_haversine_distance, ukf_filter_init,
%           ukf_predict_step, ukf_measurement_model, ukf_pda_update,
%           ukf_fuzzy_adapt, make_track_snap, iif
% =========================================================================

function [trackSnapshots, finalTrack] = single_track_runner(detList, ukf_tpl, params, n_frames)
    % ---- 初始化 ----
    trackSnapshots = cell(n_frames, 1);  % 每帧航迹快照存储
    ukf = [];                             % UKF滤波器状态
    track_state = 'INITIATING';           % 初始状态：等待起始

    % 航迹元数据
    life = 0;        % 航迹生命周期计数器（帧数）
    missed = 0;      % 连续漏检帧数计数器
    quality = 0;     % 航迹质量评分

    % ---- M/N起始参数提取 ----
    N = params.tracker_N;   % 滑窗总帧数（如 N=5）
    M = params.tracker_M;   % 最少有点迹的帧数（如 M=3），满足即尝试起始
    init_window = {};       % 滑窗内每帧的点迹列表
    window_has_det = [];    % 滑窗内每帧是否有检测（逻辑值向量）

    % =====================================================================
    % 主循环：逐帧处理
    % =====================================================================
    for k = 1:n_frames
        snap = struct('frameID', k, 'trackList', {{}});
        dets = detList{k};  % 当前帧的点迹

        switch track_state
            % =============================================================
            % 状态: INITIATING（航迹起始等待）
            % =============================================================
            case 'INITIATING'
                % ---- 滑窗收集：将当前帧点迹加入滑窗 ----
                init_window{end+1} = dets;
                window_has_det(end+1) = ~isempty(dets);

                % 保持滑窗大小为N帧（先进先出）
                if length(init_window) > N
                    init_window(1) = [];       % 移除最旧帧
                    window_has_det(1) = [];
                end

                % ---- 检查M/N条件 ----
                n_with_det = sum(window_has_det);  % 滑窗内有点迹的帧数
                if n_with_det >= M && ~isempty(dets)
                    % 满足M/N条件且当前帧有点迹，尝试寻找最优起始配对

                    % ---- 多假设配对遍历 ----
                    % 对当前帧每个点迹 vs 滑窗内之前各帧的每个点迹
                    % 选择"共识评分"最高的配对作为起始
                    best_prev = [];        % 最佳配对中较早的点迹
                    best_curr_idx = 1;     % 最佳配对中当前帧点迹的索引
                    best_support = -1;     % 最佳配对的支持度

                    for curr_idx = 1:length(dets)
                        % 遍历滑窗内除当前帧外的每一帧
                        for i = 1:(length(init_window)-1)
                            prev_dets = init_window{i};
                            if isempty(prev_dets), continue; end

                            for p = 1:length(prev_dets)
                                dp = prev_dets(p);   % 历史帧点迹
                                dc = dets(curr_idx); % 当前帧点迹

                                % 验证点迹有有效经纬度
                                if ~isfield(dp, 'lat') || isnan(dp.lat), continue; end
                                if ~isfield(dc, 'lat') || isnan(dc.lat), continue; end

                                % ---- 速度合理性检验 ----
                                % 球面距离 / 时间间隔 = 估计速度
                                % 只有合理速度范围 [30, 600] m/s 才考虑
                                dist = sphere_utils_haversine_distance(dp.lon, dp.lat, dc.lon, dc.lat);
                                dt_frames = length(init_window) - i;
                                est_speed = dist / (dt_frames * params.dt_sec);
                                if est_speed < 30 || est_speed > 600
                                    continue;  % 速度不合理，跳过该配对
                                end

                                % ---- 共识评分 ----
                                % 检查滑窗内其他帧有多少点迹靠近该配对
                                % 连线，以此排除杂波配对
                                support = 0;
                                for jj = 1:(length(init_window)-1)
                                    if jj == i, continue; end
                                    other = init_window{jj};
                                    if isempty(other), continue; end
                                    for oo = 1:length(other)
                                        do = other(oo);
                                        if ~isfield(do, 'lat') || isnan(do.lat), continue; end
                                        % 中间帧点迹距配对两个端点的距离
                                        d1 = sphere_utils_haversine_distance(dp.lon, dp.lat, do.lon, do.lat);
                                        d2 = sphere_utils_haversine_distance(dc.lon, dc.lat, do.lon, do.lat);
                                        % 两端距离都在80km内视为支持该配对
                                        if d1 < 80000 && d2 < 80000
                                            support = support + 1;
                                        end
                                    end
                                end

                                % 更新最佳配对
                                if support > best_support
                                    best_support = support;
                                    best_prev = dp;
                                    best_curr_idx = curr_idx;
                                end
                            end
                        end
                    end

                    % ---- 共识条件判断 ----
                    % 仅当至少1个其他帧的点迹支持该配对时才起始
                    % 这能有效排除孤立的杂波配对
                    if best_support >= 1
                        best_curr = dets(best_curr_idx);

                        % 两点差分初始化UKF
                        % 利用配对的两点估计初始位置和速度
                        ukf = ukf_filter_init(ukf_tpl, best_prev, best_curr);
                        ukf.dt = params.dt_sec;
                        ukf.initialized = true;

                        % 保存基础过程噪声协方差，用于后续自适应调整
                        ukf.Q_base = ukf.Q;
                        ukf.Q_ema = 1.0;  % Q的EMA缩放因子初始值

                        % 状态转换：起始成功 → 跟踪状态
                        track_state = 'TRACKING';
                        life = 1;  missed = 0;  quality = 5;

                        % 记录第一帧快照
                        snap.trackList{1} = make_track_snap(1, 1, ukf.x(3), ukf.x(1), ...
                            ukf, life, quality, 0, best_curr);

                        % 清空滑窗，航迹已成功起始
                        init_window = {};
                        window_has_det = [];
                        trackSnapshots{k} = snap;
                        continue;
                    end
                end

                % 未触发起始：记录空快照（type=6 表示TEMPORARY/未确认）
                snap.trackList{1} = make_track_snap(1, 6, NaN, NaN, [], 0, 0, 0, []);
                trackSnapshots{k} = snap;
                continue;

            % =============================================================
            % 状态: TRACKING（正常跟踪）
            % =============================================================
            case 'TRACKING'
                % ---- UKF预测步骤 ----
                ukf.dt = params.dt_sec;
                [x_pred, P_pred, X_pred, ukf] = ukf_predict_step(ukf);

                % ---- 量测预测及协方差计算 ----
                z_pred = ukf_measurement_model(ukf, x_pred);
                Z_pred = zeros(ukf.m, 2*ukf.n + 1);
                for s = 1:(2*ukf.n + 1)
                    Z_pred(:, s) = ukf_measurement_model(ukf, X_pred(:, s));
                end

                % 量测预测协方差 P_zz = R + sum_i Wc_i * (Z_i - z_pred)*(Z_i - z_pred)'
                P_zz = ukf.R;
                for s = 1:(2*ukf.n + 1)
                    dz = Z_pred(:, s) - z_pred;
                    P_zz = P_zz + ukf.Wc(s) * (dz * dz');
                end
                if any(isnan(P_zz(:))), P_zz = ukf.R; end

                % ---- 最近邻（NN）关联 ----
                % 两阶段筛选: 地理距离预筛选 → 马氏距离精筛选
                best_det = [];
                best_mahal = inf;

                % 自适应地理波门: 初始阶段宽(120km)，收敛后窄(60km)
                geo_gate_m = 120000;  % 初始阶段120km地理波门
                if life > 15, geo_gate_m = 60000; end  % UKF收敛后缩小到60km

                for d = 1:length(dets)
                    dp = dets(d);
                    if ~isfield(dp, 'lat') || isnan(dp.lat), continue; end

                    % 第一阶段: 地理距离预筛选（快速排除）
                    geo_dist = sphere_utils_haversine_distance(...
                        x_pred(1), x_pred(3), dp.lon, dp.lat);
                    if geo_dist > geo_gate_m, continue; end

                    % 第二阶段: 马氏距离精筛选（统计检验）
                    z_m = [dp.drange; dp.daz];
                    innov = z_m - z_pred(1:2);
                    if innov(2) > 180, innov(2) = innov(2) - 360;
                    elseif innov(2) < -180, innov(2) = innov(2) + 360; end
                    mahal = innov' * (P_zz(1:2,1:2) \ innov);

                    % 同时满足: 在波门内 AND 马氏距离最小
                    if mahal < params.gate_sigma^2 * 2 && mahal < best_mahal
                        best_mahal = mahal;
                        best_det = dp;
                    end
                end

                % ---- 根据关联结果进行更新或预测 ----
                if ~isempty(best_det)
                    % ---- 关联成功: PDA加权更新 ----
                    dets_in_gate = {best_det};  % 门内点迹集合

                    % 收集波门内其他点迹（用于PDA多假设）
                    gate_threshold = params.gate_sigma^2 * 2;
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

                    % PDA更新（融合所有门内点迹）
                    [~, ~, ukf, ~, nis_val] = ukf_pda_update(ukf, dets_in_gate, ...
                        z_pred, Z_pred, X_pred, x_pred, P_pred, P_zz, params);

                    % 更新元数据
                    missed = 0;                         % 关联成功，清零漏检
                    life = life + 1;
                    quality = min(quality + 1, 15);     % 质量递增，上限15

                    % 维护NIS历史记录
                    if ~isfield(ukf, 'nis_history'), ukf.nis_history = []; end
                    ukf.nis_history(end+1) = nis_val;
                    if length(ukf.nis_history) > params.fuzzy_window_size
                        ukf.nis_history(1) = [];
                    end

                    % 模糊自适应Q（在航迹稳定后启用）
                    if params.use_fuzzy_adaptive && life > 12
                        ukf = ukf_fuzzy_adapt(ukf, ukf.nis_history, life, params);
                    end
                else
                    % ---- 漏检: 纯预测（不融合观测） ----
                    ukf.x = x_pred;
                    ukf.P = P_pred;
                    missed = missed + 1;                % 漏检计数+1
                    life = life + 1;
                    quality = max(quality - 1, 0);      % 质量递减，下限0
                    best_det = [];
                end

                % ---- 航迹终止检查 ----
                % K_loss 连续漏检 → 标记为 LOST
                if missed >= params.tracker_K_loss
                    track_state = 'LOST';
                end

                % 记录当前帧快照
                snap.trackList{1} = make_track_snap(1, 1, ukf.x(3), ukf.x(1), ...
                    ukf, life, quality, missed, best_det);

            % =============================================================
            % 状态: LOST（航迹丢失）
            % =============================================================
            case 'LOST'
                % 航迹终止后重新尝试起始
                track_state = 'INITIATING';  % 回到起始状态
                init_window = {};
                window_has_det = [];
                life = 0; missed = 0; quality = 0;

                % 记录type=7（HISTORY/LOST）快照
                snap.trackList{1} = make_track_snap(1, 7, NaN, NaN, ukf, life, quality, missed, []);
        end

        trackSnapshots{k} = snap;
    end

    % ---- 构建最终航迹状态摘要 ----
    finalTrack = struct('id', 1, ...
        'type', iif(strcmp(track_state,'TRACKING'),1,7), ...
        'quality', quality, 'life', life);
end


% =========================================================================
% 辅助函数: make_track_snap
% 创建单条航迹的快照结构体，统一字段格式
% =========================================================================
function trk = make_track_snap(id, type, lat, lon, ukf, life, quality, missed, det)
    trk.id = id;           % 航迹编号
    trk.type = type;       % 航迹类型 (1=TRACKING, 6=TEMPORARY, 7=HISTORY)
    trk.lat = lat;         % 当前纬度
    trk.lon = lon;         % 当前经度
    trk.ukf = ukf;         % UKF滤波器状态
    trk.life = life;       % 生命周期（已完成的总帧数）
    trk.quality = quality; % 质量评分
    trk.missed = missed;   % 连续漏检帧数
    trk.assoc_det = det;   % 关联到的量测
    if ~isempty(det)
        trk.x_pred = [];   % 有量测时不需要存储预测值
        trk.P_pred = [];
    end
end


% =========================================================================
% 辅助函数: iif
% 内联条件判断（Inline IF），返回 cond ? t : f
% =========================================================================
function v = iif(cond, t, f)
    if cond, v = t; else, v = f; end
end
