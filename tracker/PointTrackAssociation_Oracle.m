function [TPmatch_result, singlePointsIndex, used_det] = PointTrackAssociation_Oracle( ...
        trackList, pointList, frame_id)
    % Oracle 模式点迹-航迹关联
    %
    % 核心逻辑：基于真值 ID 的最近邻匹配
    %   1. 从航迹列表提取 truth_idx，建立"航迹→真值ID"映射
    %   2. 从点迹列表按 aircraft_id 分组候选点迹
    %   3. 对每条航迹，在其对应真值 ID 的候选点迹中找最近邻
    %   4. 每个点迹只能被一条航迹关联（先到先得）
    %
    % 与普通关联算法的区别：
    %   - 不使用马氏距离门限过滤（Oracle 模式下 Pd=0.6 已通过检测阶段处理）
    %   - 直接用真值 ID 筛选候选，避免了虚假关联
    %   - 在候选集中按几何距离排序，保证最优匹配

    n_tracks = length(trackList);
    n_points = length(pointList);
    % TPmatch_result 格式：[track_idx, point_idx]，point_idx=0 表示未关联
    % 初始化所有航迹为未关联状态（第 2 列全为 0）
    TPmatch_result = [(1:n_tracks)', zeros(n_tracks, 1)];
    used_det = false(1, n_points);  % 点迹消耗掩码，初始全部未使用

    % ---- 步骤1：从航迹提取 truth_idx，建立真值ID映射 ----
    % 遍历航迹列表，收集每条航迹关联的真值目标编号
    % truth_ids(i) 表示第 i 条航迹对应的真值 ID，nan 表示无真值映射
    truth_ids = nan(1, n_tracks);
    max_truth_id = 0;
    for i = 1:n_tracks
        trk = trackList{i};
        % 跳过没有 truth_idx 或值无效的航迹（如 HISTORY 航迹）
        if ~isfield(trk, 'truth_idx') || ~isscalar(trk.truth_idx) || ...
                ~isfinite(double(trk.truth_idx))
            continue;
        end
        truth_id = double(trk.truth_idx);
        % 确保 truth_idx 是有效的正整数
        if truth_id < 1 || truth_id ~= floor(truth_id)
            continue;
        end
        truth_ids(i) = truth_id;
        max_truth_id = max(max_truth_id, truth_id);
    end

    % ---- 步骤2：按真值 ID 分组候选点迹 ----
    % candidates{truth_id} = [point_idx1, point_idx2, ...]
    % 将点迹按 aircraft_id（即真值 ID）分组，方便后续快速查找
    candidates = cell(1, max_truth_id);
    for j = 1:n_points
        dp = pointList(j);
        % 只考虑当前帧的真实检测（非杂波、非过期帧）
        % is_current_real_detection 验证 aircraft_id 有效、frameID 匹配、非杂波
        if ~is_current_real_detection(dp, frame_id)
            continue;
        end
        truth_id = double(dp.aircraft_id);
        % 将点迹索引归入对应真值 ID 的候选组
        if truth_id <= max_truth_id
            candidates{truth_id}(end+1) = j;
        end
    end

    % ---- 步骤3：逐航迹匹配最近邻点迹 ----
    % 对每条航迹，在其对应真值 ID 的候选点迹中找到距离最近的未用点迹
    for i = 1:n_tracks
        truth_id = truth_ids(i);
        if isnan(truth_id)
            continue;  % 跳过无 truth_idx 的航迹
        end
        candidate_indices = candidates{truth_id};
        best_j = 0;
        best_d = inf;
        % 在候选集中找到未消耗的最近点迹
        for j = candidate_indices
            if used_det(j)
                continue;  % 已被其他航迹消耗（先到先得原则）
            end
            % 计算航迹预测位置与点迹之间的 Haversine 距离
            d = oracle_point_distance(trackList{i}, pointList(j));
            if d < best_d
                best_d = d;
                best_j = j;
            end
        end
        % 记录匹配结果：更新 TPmatch_result 的第 2 列为点迹索引
        if best_j > 0
            TPmatch_result(i, 2) = best_j;
            used_det(best_j) = true;  % 标记点迹已被消耗
        end
    end

    % 返回未被关联的点迹索引（送入航迹起始模块）
    % singlePointsIndex 包含所有未被任何航迹关联的点迹
    singlePointsIndex = find(~used_det);
end

function tf = is_current_real_detection(dp, frame_id)
    % 验证点迹是否为当前帧的真实检测
    % 条件：
    %   1. 有有效的 aircraft_id（标量、有限、>=1）
    %   2. frameID 等于当前帧号
    %   3. 不是杂波（is_clutter=false 或不存在该字段）
    tf = isfield(dp, 'aircraft_id') && isscalar(dp.aircraft_id) && ...
        isfinite(double(dp.aircraft_id)) && double(dp.aircraft_id) >= 1 && ...
        isfield(dp, 'frameID') && isscalar(dp.frameID) && ...
        double(dp.frameID) == double(frame_id) && ...
        ~(isfield(dp, 'is_clutter') && dp.is_clutter);
end

function d = oracle_point_distance(trk, dp)
    % 计算航迹预测位置与点迹之间的球面距离（km）
    % 优先使用 x_pred 字段（UKF 预测状态），其次使用 lon/lat 字段
    % 如果字段不存在或值为 NaN，返回 0（表示距离无效）
    if isfield(trk, 'x_pred') && numel(trk.x_pred) >= 3 && ...
            isfield(dp, 'lon') && isfield(dp, 'lat') && ...
            isfinite(dp.lon) && isfinite(dp.lat)
        % 使用 UKF 预测状态的经纬度（x_pred[1]=lon, x_pred[3]=lat）
        d = sphere_utils_haversine_distance(trk.x_pred(1), trk.x_pred(3), dp.lon, dp.lat);
    elseif isfield(trk, 'lon') && isfield(trk, 'lat') && ...
            isfield(dp, 'lon') && isfield(dp, 'lat') && ...
            isfinite(trk.lon) && isfinite(trk.lat) && ...
            isfinite(dp.lon) && isfinite(dp.lat)
        % 回退到航迹顶层的 lon/lat 字段
        d = sphere_utils_haversine_distance(trk.lon, trk.lat, dp.lon, dp.lat);
    else
        % 字段缺失或无效，返回 0 表示无法计算距离
        d = 0;
    end
end
