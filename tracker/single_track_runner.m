% =========================================================================
% single_track_runner.m
% 单目标逐帧航迹管理 (M/N滑窗起始 + UKF预测更新)
% =========================================================================
% 起始策略:
%   不依赖 is_clutter 标记 (真实雷达不存在此信息).
%   使用 M/N 滑窗收集全部点迹, 通过时空一致性自然排除杂波:
%     1. 滑窗N帧, 至少M帧有点迹 → 触发起始尝试
%     2. 遍历首帧×末帧所有点迹对, 估计速度在 [30,600] m/s 之间
%     3. 中间帧有点迹靠近配对轨迹 → 支持度最高的配对获胜
%     4. 支持度 >= 1 → 两点差分初始化UKF; 否则继续滑窗
% 跟踪策略:
%   UKF预测 → 地理预筛选 → 马氏距离NN关联 → PDA加权更新 / 纯预测
%   K_loss 连续漏检 → LOST → 重新起始
% =========================================================================

function [trackSnapshots, finalTrack] = single_track_runner(detList, ukf_tpl, params, n_frames)
    trackSnapshots = cell(n_frames, 1);
    ukf = [];
    track_state = 'INITIATING';
    life = 0;
    missed = 0;
    quality = 0;

    % M/N起始参数
    N = params.tracker_N;
    M = params.tracker_M;
    init_window = {};     % 每帧的点迹列表
    window_has_det = [];  % 每帧是否有点迹 (逻辑值)

    for k = 1:n_frames
        snap = struct('frameID', k, 'trackList', {{}});
        dets = detList{k};

        switch track_state
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
                    % 多假设配对: 当前帧每个点迹 vs 窗内之前各帧每个点迹
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

                                % 速度合理性检验
                                dist = sphere_utils_haversine_distance(dp.lon, dp.lat, dc.lon, dc.lat);
                                dt_frames = length(init_window) - i;
                                est_speed = dist / (dt_frames * params.dt_sec);
                                if est_speed < 30 || est_speed > 600
                                    continue;
                                end

                                % 共识评分: 窗内其他帧有多少点迹靠近轨迹
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

                    % 仅当共识配对存在 (>=1个其他帧支持) 才起始
                    if best_support >= 1
                        best_curr = dets(best_curr_idx);
                        ukf = ukf_filter_init(ukf_tpl, best_prev, best_curr);
                        ukf.dt = params.dt_sec;
                        ukf.initialized = true;
                        ukf.Q_base = ukf.Q;
                        ukf.Q_ema = 1.0;
                        track_state = 'TRACKING';
                        life = 1;  missed = 0;  quality = 5;

                        snap.trackList{1} = make_track_snap(1, 1, ukf.x(3), ukf.x(1), ...
                            ukf, life, quality, 0, best_curr);

                        init_window = {};  % 清空, 不再使用
                        window_has_det = [];
                        trackSnapshots{k} = snap;
                        continue;
                    end
                end

                % 未触发起始: 空快照
                snap.trackList{1} = make_track_snap(1, 6, NaN, NaN, [], 0, 0, 0, []);
                trackSnapshots{k} = snap;
                continue;

            case 'TRACKING'
                ukf.dt = params.dt_sec;
                [x_pred, P_pred, X_pred, ukf] = ukf_predict_step(ukf);

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

                % 最近邻关联 (地理预筛选 + 马氏距离)
                best_det = [];
                best_mahal = inf;
                geo_gate_m = 120000;  % 初始阶段120km
                if life > 15, geo_gate_m = 60000; end  % 收敛后60km

                for d = 1:length(dets)
                    dp = dets(d);
                    if ~isfield(dp, 'lat') || isnan(dp.lat), continue; end
                    % 地理距离预筛选
                    geo_dist = sphere_utils_haversine_distance(...
                        x_pred(1), x_pred(3), dp.lon, dp.lat);
                    if geo_dist > geo_gate_m, continue; end

                    z_m = [dp.drange; dp.daz];
                    innov = z_m - z_pred(1:2);
                    if innov(2) > 180, innov(2) = innov(2) - 360;
                    elseif innov(2) < -180, innov(2) = innov(2) + 360; end
                    mahal = innov' * (P_zz(1:2,1:2) \ innov);
                    if mahal < params.gate_sigma^2 * 2 && mahal < best_mahal
                        best_mahal = mahal;
                        best_det = dp;
                    end
                end

                if ~isempty(best_det)
                    % PDA 更新
                    dets_in_gate = {best_det};
                    % 收集门内其他点迹 (用于PDA加权)
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

                    [~, ~, ukf, ~, nis_val] = ukf_pda_update(ukf, dets_in_gate, ...
                        z_pred, Z_pred, X_pred, x_pred, P_pred, P_zz, params);

                    missed = 0;
                    life = life + 1;
                    quality = min(quality + 1, 15);
                    if ~isfield(ukf, 'nis_history'), ukf.nis_history = []; end
                    ukf.nis_history(end+1) = nis_val;
                    if length(ukf.nis_history) > params.fuzzy_window_size
                        ukf.nis_history(1) = [];
                    end
                    if params.use_fuzzy_adaptive && life > 12
                        ukf = ukf_fuzzy_adapt(ukf, ukf.nis_history, life, params);
                    end
                else
                    % 纯预测
                    ukf.x = x_pred;
                    ukf.P = P_pred;
                    missed = missed + 1;
                    life = life + 1;
                    quality = max(quality - 1, 0);
                    best_det = [];
                end

                % 终止检查
                if missed >= params.tracker_K_loss
                    track_state = 'LOST';
                end

                snap.trackList{1} = make_track_snap(1, 1, ukf.x(3), ukf.x(1), ...
                    ukf, life, quality, missed, best_det);

            case 'LOST'
                % 航迹终止后重新起始
                track_state = 'INITIATING';
                init_window = {};
                window_has_det = [];
                life = 0; missed = 0; quality = 0;
                snap.trackList{1} = make_track_snap(1, 7, NaN, NaN, ukf, life, quality, missed, []);
        end

        trackSnapshots{k} = snap;
    end

    finalTrack = struct('id', 1, 'type', iif(strcmp(track_state,'TRACKING'),1,7), ...
        'quality', quality, 'life', life);
end

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

function v = iif(cond, t, f)
    if cond, v = t; else, v = f; end
end
