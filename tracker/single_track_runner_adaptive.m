% =========================================================================
% single_track_runner_adaptive.m
% =========================================================================
% 【功能概述】
%   机动自适应UKF航迹跟踪器，对 single_track_runner 的增强版本。
%   在相同的 M/N起始 + NN关联 + PDA更新 框架基础上，增加了两项
%   针对目标转弯机动的自适应机制：
%     1. 渐进波门放宽：检测到机动先兆后，逐渐扩大关联波门
%     2. 机动自适应Q：使用 ukf_maneuver_adapt 替代 ukf_fuzzy_adapt，
%        对转弯机动响应更快速
%
% 【数学原理】
%   1. 机动预检测（Pre-detection of Maneuver）:
%      在未激活机动状态时，维护一个"疑似计数器" suspect_counter:
%      - 本帧有任何量测在1.8倍放宽波门（即 wide_gate）内 → counter+1
%      - 本帧无任何量测在放宽波门内 → counter-1（不低于0）
%      - counter ≥ 2 → 判定为机动开始，激活 maneuver_active
%
%   2. 渐进波门放宽（Progressive Gate Relaxation）:
%      机动激活后，波门并非瞬间跳到最大值，而是每帧渐进增加:
%      - 地理波门: 120km + maneuver_counter * 3km，上限 150km
%      - 马氏距离倍数: 1.0 + maneuver_counter * 0.15，上限 2.5
%      渐进策略避免了突然大幅放宽导致错误关联远处杂波点。
%
%   3. 机动适应Q更新:
%      ukf_maneuver_adapt 根据NIS和新息历史联合判断机动类型
%      （直线加速/减速转弯），针对性调整过程噪声协方差Q。
%
% 【输入参数】
%   detList  - cell数组，每帧的点迹结构体数组
%   ukf_tpl  - UKF模板结构体
%   params   - 参数结构体，需包含:
%              .tracker_N, .tracker_M, .dt_sec, .gate_sigma,
%              .tracker_K_loss, .fuzzy_window_size, .use_fuzzy_adaptive
%   n_frames - 总帧数
%
% 【输出】
%   trackSnapshots - cell数组，(n_frames x 1)，每帧的航迹快照
%   finalTrack     - 最终航迹状态摘要结构体
%
% 【调用关系】
%   被主脚本或单目标评估脚本调用
%   子调用: sphere_utils_haversine_distance, ukf_filter_init,
%           ukf_predict_step, ukf_measurement_model, ukf_pda_update,
%           ukf_maneuver_adapt, make_track_snap_adapt, iif_adapt
%
% 【与 single_track_runner 的差异】
%   - 使用 ukf_maneuver_adapt 替代 ukf_fuzzy_adapt
%   - 增加 ukf.maneuver_active, ukf.maneuver_counter 等机动状态字段
%   - 增加渐进波门放宽逻辑
%   - 增加机动预检测（suspect_counter机制）
%   - 维护 innov_history（新息历史）供机动检测用
% =========================================================================

function [trackSnapshots, finalTrack] = single_track_runner_adaptive(detList, ukf_tpl, params, n_frames)
    % ---- 初始化 ----
    trackSnapshots = cell(n_frames, 1);
    ukf = [];
    track_state = 'INITIATING';
    life = 0;
    missed = 0;
    quality = 0;

    % M/N起始参数
    N = params.tracker_N;
    M = params.tracker_M;
    init_window = {};
    window_has_det = [];

    % =====================================================================
    % 主循环：逐帧处理
    % =====================================================================
    for k = 1:n_frames
        snap = struct('frameID', k, 'trackList', {{}});
        dets = detList{k};

        switch track_state
            % =============================================================
            % 状态: INITIATING（航迹起始等待）
            % 与 single_track_runner 相同的M/N滑窗起始逻辑
            % =============================================================
            case 'INITIATING'
                % ---- 滑窗收集 ----
                init_window{end+1} = dets;
                window_has_det(end+1) = ~isempty(dets);
                if length(init_window) > N
                    init_window(1) = [];
                    window_has_det(1) = [];
                end

                n_with_det = sum(window_has_det);
                if n_with_det >= M && ~isempty(dets)
                    % ---- 多假设配对遍历（寻找共识起始对） ----
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

                                % 速度合理性检验: v ∈ [30, 600] m/s
                                dist = sphere_utils_haversine_distance(dp.lon, dp.lat, dc.lon, dc.lat);
                                dt_frames = length(init_window) - i;
                                est_speed = dist / (dt_frames * params.dt_sec);
                                if est_speed < 30 || est_speed > 600
                                    continue;
                                end

                                % 共识评分: 其他帧点迹是否靠近轨迹线
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

                    % 共识≥1 → 两点差分初始化UKF并起始航迹
                    if best_support >= 1
                        best_curr = dets(best_curr_idx);
                        ukf = ukf_filter_init(ukf_tpl, best_prev, best_curr);
                        ukf.dt = params.dt_sec;
                        ukf.initialized = true;
                        ukf.Q_base = ukf.Q;
                        ukf.Q_ema = 1.0;

                        % ---- 机动检测相关字段初始化 ----
                        ukf.maneuver_active = false;     % 机动激活标志
                        ukf.maneuver_counter = 0;        % 机动持续帧数
                        ukf.maneuver_recovery = 0;       % 机动恢复计数
                        ukf.suspect_counter = 0;         % 疑似机动计数

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

            % =============================================================
            % 状态: TRACKING（正常跟踪，含机动自适应）
            % =============================================================
            case 'TRACKING'
                % ---- UKF预测 ----
                ukf.dt = params.dt_sec;
                [x_pred, P_pred, X_pred, ukf] = ukf_predict_step(ukf);

                % ---- 量测预测及协方差 ----
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

                % ---- 关联参数：根据机动状态自适应调整 ----
                best_det = [];
                best_mahal = inf;

                % 机动期间: 渐进放宽门限（避免突然抓取远处量测）
                % 原理: 目标转弯时预测位置偏离实际位置，传统固定波门
                %       可能无法捕获机动中的量测。渐进放宽在扩大搜索
                %       范围的同时保持了连续性。
                if isfield(ukf, 'maneuver_active') && ukf.maneuver_active
                    % 地理波门: 从120km逐步放宽到150km
                    geo_gate_m = 120000 + ukf.maneuver_counter * 3000;
                    geo_gate_m = min(geo_gate_m, 150000);

                    % 马氏距离倍数: 从1.0逐步放宽到2.5
                    gate_factor = 1.0 + ukf.maneuver_counter * 0.15;
                    gate_factor = min(gate_factor, 2.5);
                else
                    % 正常模式: 收敛前宽，收敛后窄
                    geo_gate_m = 120000;
                    if life > 15, geo_gate_m = 60000; end
                    gate_factor = 1.0;
                end
                gate_mahal = (params.gate_sigma * gate_factor)^2 * 2;

                % ---- 机动预检测：预扫描宽门限量测 ----
                % 在非机动状态下，检测是否有量测在比正常门限更宽的
                % 区域内。若连续2帧有此类量测，判定为机动开始。
                if ~isfield(ukf, 'maneuver_active') || ~ukf.maneuver_active
                    if ~isfield(ukf, 'suspect_counter'), ukf.suspect_counter = 0; end

                    % 宽门限: 1.8倍正常门限
                    wide_gate = (params.gate_sigma * 1.8)^2 * 2;
                    any_in_wide = false;

                    % 扫描所有点迹
                    for d = 1:length(dets)
                        dp = dets(d);
                        if ~isfield(dp, 'lat') || isnan(dp.lat), continue; end
                        geo_d = sphere_utils_haversine_distance(x_pred(1), x_pred(3), dp.lon, dp.lat);
                        if geo_d > 120000, continue; end  % 超出最大地理门限
                        z_m = [dp.drange; dp.daz];
                        inno = z_m - z_pred(1:2);
                        if inno(2) > 180, inno(2) = inno(2) - 360;
                        elseif inno(2) < -180, inno(2) = inno(2) + 360; end
                        if inno' * (P_zz(1:2,1:2) \ inno) < wide_gate
                            any_in_wide = true; break;  % 发现疑似机动的量测
                        end
                    end

                    % 更新疑似计数器
                    if any_in_wide
                        ukf.suspect_counter = ukf.suspect_counter + 1;
                    else
                        % 未检测到则递减（不低于0），防止偶发误触发
                        ukf.suspect_counter = max(0, ukf.suspect_counter - 1);
                    end

                    % 连续2帧有疑似 → 触发机动状态
                    if ukf.suspect_counter >= 2
                        ukf.maneuver_active = true;
                        ukf.maneuver_counter = 0;       % 重置机动计数器
                        ukf.maneuver_recovery = 0;      % 重置恢复计数器

                        % 初始仅轻微放宽（factor=1.15，比正常大15%）
                        gate_factor = 1.15;
                        gate_mahal = (params.gate_sigma * gate_factor)^2 * 2;
                        geo_gate_m = 123000;            % 123km
                    end
                end

                % ---- 最近邻关联（使用当前门限） ----
                for d = 1:length(dets)
                    dp = dets(d);
                    if ~isfield(dp, 'lat') || isnan(dp.lat), continue; end

                    % 地理距离预筛选
                    geo_dist = sphere_utils_haversine_distance(...
                        x_pred(1), x_pred(3), dp.lon, dp.lat);
                    if geo_dist > geo_gate_m, continue; end

                    % 马氏距离精筛选
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

                % ---- 根据关联结果 ----
                if ~isempty(best_det)
                    % ---- 关联成功: PDA更新 ----
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

                    % 维护NIS历史
                    if ~isfield(ukf, 'nis_history'), ukf.nis_history = []; end
                    ukf.nis_history(end+1) = nis_val;
                    if length(ukf.nis_history) > params.fuzzy_window_size
                        ukf.nis_history(1) = [];
                    end

                    % ---- 记录新息历史（供机动检测用） ----
                    % innov_history 保存最近10帧的原始新息向量
                    % 用于 ukf_maneuver_adapt 分析转弯方向
                    if ~isfield(ukf, 'innov_history'), ukf.innov_history = {}; end
                    ukf.innov_history{end+1} = [best_det.drange; best_det.daz] - z_pred(1:2);
                    if length(ukf.innov_history) > 10
                        ukf.innov_history(1) = [];
                    end

                    % ---- 机动自适应Q更新 ----
                    % 使用 ukf_maneuver_adapt 替代基础版的 ukf_fuzzy_adapt
                    % 该函数综合考虑 NIS历史 + 新息历史，能更快识别转弯机动
                    if params.use_fuzzy_adaptive && life > 12
                        ukf = ukf_maneuver_adapt(ukf, ukf.nis_history, ukf.innov_history, life, params);
                    end
                else
                    % ---- 漏检: 纯预测 ----
                    ukf.x = x_pred;
                    ukf.P = P_pred;
                    missed = missed + 1;
                    life = life + 1;
                    quality = max(quality - 1, 0);
                    best_det = [];
                end

                % 终止检查: K_loss连续漏检 → LOST
                if missed >= params.tracker_K_loss
                    track_state = 'LOST';
                end

                snap.trackList{1} = make_track_snap_adapt(1, 1, ukf.x(3), ukf.x(1), ...
                    ukf, life, quality, missed, best_det);

            % =============================================================
            % 状态: LOST（航迹丢失后重新起始）
            % =============================================================
            case 'LOST'
                track_state = 'INITIATING';
                init_window = {};
                window_has_det = [];
                life = 0; missed = 0; quality = 0;
                snap.trackList{1} = make_track_snap_adapt(1, 7, NaN, NaN, ukf, life, quality, missed, []);
        end

        trackSnapshots{k} = snap;
    end

    % ---- 构建最终航迹状态摘要 ----
    finalTrack = struct('id', 1, 'type', iif_adapt(strcmp(track_state,'TRACKING'),1,7), ...
        'quality', quality, 'life', life);
end


% =========================================================================
% 辅助函数: make_track_snap_adapt
% 创建单条航迹的快照结构体（自适应版本）
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
% 内联条件判断
% =========================================================================
function v = iif_adapt(cond, t, f)
    if cond, v = t; else, v = f; end
end
