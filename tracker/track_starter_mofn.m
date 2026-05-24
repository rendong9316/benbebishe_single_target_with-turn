% =========================================================================
% track_starter_mofn.m
% M/N逻辑航迹起始器（tempPool回溯法）
% =========================================================================
% 参考项目架构：维护tempPool保存最近N帧的未关联点迹候选序列。
% 每帧新未关联点迹回溯搜索tempPool中候选，形成点迹链。
% 满足M/N条件（≥3点/5帧）后，两点差分初始化UKF，创建TEMPORARY航迹。
% =========================================================================

function [trackList, tempPool] = track_starter_mofn(trackList, tempPool, ...
        unused_dets, ukf_tpl, params, frame_id)

    if isempty(unused_dets)
        % 仍需要清理过期候选
        tempPool = cleanup_stale_candidates(tempPool, frame_id, params.tracker_N);
        return;
    end

    init_gate_m = 60000;  % 起始地理距离门限 60km

    % ---- 步骤1：每个未使用点迹回溯匹配tempPool候选 ----
    for d = 1:length(unused_dets)
        dp = unused_dets(d);
        if ~isfield(dp, 'lat') || isnan(dp.lat), continue; end

        best_cand = 0;
        best_dist = inf;

        % 在tempPool中找最近候选
        for c = 1:length(tempPool)
            cand = tempPool{c};
            if isempty(cand.points), continue; end
            last_pt = cand.points{end};
            if ~isfield(last_pt, 'lat') || isnan(last_pt.lat), continue; end
            dist = sphere_utils_haversine_distance(dp.lon, dp.lat, last_pt.lon, last_pt.lat);
            if dist < init_gate_m && dist < best_dist
                best_dist = dist;
                best_cand = c;
            end
        end

        if best_cand > 0
            % 追加到已有候选
            cand = tempPool{best_cand};
            cand.points{end+1} = dp;
            cand.frames(end+1) = frame_id;
            cand.lastFrame = frame_id;
            tempPool{best_cand} = cand;
        else
            % 创建新候选
            cand.points = {dp};
            cand.frames = frame_id;
            cand.lastFrame = frame_id;
            tempPool{end+1} = cand;
        end
    end

    % ---- 步骤2：检查M/N条件，起始新航迹 ----
    M = params.tracker_M;
    N = params.tracker_N;
    promoted = false(1, length(tempPool));
    n_promoted = 0;

    for c = 1:length(tempPool)
        cand = tempPool{c};
        n_pts = length(cand.points);

        if n_pts < M, continue; end
        frame_span = cand.frames(end) - cand.frames(1);
        if frame_span > N - 1, continue; end

        first_det = cand.points{1};
        last_det  = cand.points{end};

        dist_2pt = sphere_utils_haversine_distance(...
            first_det.lon, first_det.lat, last_det.lon, last_det.lat);
        if dist_2pt > 300000, continue; end

        % 候选验证：点迹序列需呈近似直线运动（杂波序列不满足）
        if ~validate_candidate_sequence(cand), continue; end

        new_ukf = ukf_filter_init(ukf_tpl, first_det, last_det);

        % ---- 航迹复活检查: 候选是否为已死亡航迹的延续 ----
        revived = false;
        revival_gate_m = 100000;   % 100km接续判定门限
        revival_window   = 15;      % 死亡15帧内可复活

        for t = 1:length(trackList)
            trk = trackList{t};
            if trk.type ~= 7, continue; end  % 只查HISTORY
            if ~isfield(trk, 'death_frame'), continue; end
            if frame_id - trk.death_frame > revival_window, continue; end

            % 候选首点 vs 死亡航迹最后位置
            dist = sphere_utils_haversine_distance(...
                first_det.lon, first_det.lat, trk.lon, trk.lat);
            if dist < revival_gate_m
                % 复活!
                trk.type = 6;           % 恢复为TEMPORARY
                trk.quality = 9;        % 重置质量
                trk.missed = 0;         % 清零漏检
                trk.ukf = new_ukf;      % 用新UKF重新初始化
                trk.lat = new_ukf.x(3);
                trk.lon = new_ukf.x(1);
                trk.life = trk.life;    % 保留历史life计数
                trk.assoc_det = last_det;
                trk.nis_history = [];
                trk.init_points = n_pts;
                if ~isfield(trk, 'revived')
                    trk.revived = 0;
                end
                trk.revived = trk.revived + 1;
                trackList{t} = trk;
                promoted(c) = true;
                revived = true;
                fprintf('  [复活] Frame %d: HISTORY#%d 复活 (距死亡%d帧, 接续距离%.0fkm)\n', ...
                    frame_id, trk.id, frame_id - trk.death_frame, dist/1000);
                break;
            end
        end

        if revived, continue; end

        next_id = length(trackList) + 1;
        new_trk.id = next_id;
        new_trk.type = 6;
        new_trk.quality = 9;  % 仅需1次关联即可升级RELIABLE(≥10)
        new_trk.ukf = new_ukf;
        new_trk.life = 0;
        new_trk.missed = 0;
        new_trk.lat = new_ukf.x(3);
        new_trk.lon = new_ukf.x(1);
        new_trk.assoc_det = last_det;
        new_trk.nis_history = [];
        new_trk.birth_frame = frame_id;
        new_trk.init_points = n_pts;
        new_trk.death_frame = NaN;  % 未死亡
        trackList{end+1} = new_trk;
        promoted(c) = true;
    end

    % ---- 步骤3：移除已起始的候选 + 清理过期候选 ----
    tempPool = tempPool(~promoted);
    tempPool = cleanup_stale_candidates(tempPool, frame_id, N);
end

function tempPool = cleanup_stale_candidates(tempPool, current_frame, N)
    keep = true(1, length(tempPool));
    for c = 1:length(tempPool)
        if current_frame - tempPool{c}.lastFrame > N
            keep(c) = false;
        end
    end
    tempPool = tempPool(keep);
end

function ok = validate_candidate_sequence(cand)
    % 验证候选点迹序列的直线运动一致性
    % 用robust regression拟合直线，检验残差和速度合理性
    n = length(cand.points);
    if n < 3, ok = true; return; end  % M=3时无法做残差检验

    lats = zeros(1, n); lons = zeros(1, n);
    for i = 1:n
        lats(i) = cand.points{i}.lat;
        lons(i) = cand.points{i}.lon;
    end

    % 拟合 lon = a*lat + b (在经纬度空间近似直线)
    p = polyfit(lats, lons, 1);
    lon_pred = polyval(p, lats);
    residuals_deg = abs(lons - lon_pred);

    % 残差转换为km (1° ≈ 111 km)
    residuals_km = max(residuals_deg) * 111;

    % 序列总跨度
    total_dist_km = sphere_utils_haversine_distance(...
        lons(1), lats(1), lons(end), lats(end)) / 1000;

    % 验证条件:
    % 1. 最大残差 < 30 km (严格共线性)
    % 2. 若总跨度>100km，要求残差<跨度的30%
    if residuals_km > 30000
        ok = false; return;
    end
    if total_dist_km > 100 && residuals_km > 0.3 * total_dist_km
        ok = false; return;
    end
    if total_dist_km > 500
        ok = false; return;  % 30秒内目标移动不应超过500km
    end
    ok = true;
end
