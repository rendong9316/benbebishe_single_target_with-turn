% =========================================================================
% track_starter_mofn.m
% =========================================================================
% 【功能概述】
%   基于 M/N 逻辑的航迹起始器，采用 tempPool（临时候选池）回溯法
%   在多目标跟踪框架中创建新航迹。每个未关联的点迹或是加入已有
%   候选序列，或是创建新候选。满足 M/N 条件（N帧内至少M个点迹）
%   时，用首末两点差分初始化UKF并创建 TEMPORARY 航迹。
%   支持航迹复活（Death Track Revival）机制。
%
% 【数学原理】
%   1. 航迹起始问题 (Track Initiation):
%      在无先验状态的情况下，从连续多帧的点迹序列中识别出真实目标
%      轨迹。M/N 逻辑是经典解法：
%      - 在 N 帧的观察窗内，若至少 M 帧有点迹落入某一运动假设，
%        则宣告航迹起始
%      - 典型参数: M=3, N=5（5帧内至少3帧有检测）
%
%   2. 两点差分初始化:
%      利用首末两个点迹的位置和时间差:
%        初始位置 = first_det.position
%        初始速度 = (last_det - first_det) / delta_t
%      问题在于量测只有(x,y)二维位置，需结合运动模型估计速度。
%
%   3. 候选验证:
%      对收集到的点迹序列，用robust linear regression在经纬度空间
%      拟合直线：
%         lon = a * lat + b
%      检验条件:
%        (a) 最大拟合残差 < 30 km（点迹序列基本共线）
%        (b) 跨度>100km时，残差 < 跨度的30%
%        (c) 总跨度 < 500 km（避免把不同目标误连）
%      这些条件有效排除杂波的随机散布序列。
%
%   4. 航迹复活:
%      当新候选的首点位于死亡航迹（HISTORY状态）最后已知位置
%      100km以内，且死亡不超过15帧时，直接复活该航迹。
%      复活的航迹以 TEMPORARY 状态重新进入跟踪，quality重置为9。
%
% 【输入参数】
%   trackList  - cell数组，当前所有航迹（原地修改）
%   tempPool   - cell数组，临时候选池，每个元素含:
%                .points: 点迹cell数组
%                .frames: 对应的帧号数组
%                .lastFrame: 最后更新帧号
%   unused_dets - 结构体数组，当前帧未被关联的点迹
%   ukf_tpl    - UKF模板结构体
%   params     - 参数结构体，需含:
%                .tracker_N: M/N窗长N
%                .tracker_M: M/N最少点数M
%   frame_id   - 当前帧编号
%
% 【输出】
%   trackList  - 更新后的航迹列表（可能新增或复活了航迹）
%   tempPool   - 更新后的临时候选池（移除了已起始的候选）
%
% 【调用关系】
%   被 multi_track_manager.m 在 Step 2（无活跃航迹）和 Step 8
%     （剩余点迹起始）调用
%   子调用: sphere_utils_haversine_distance, ukf_filter_init,
%           validate_candidate_sequence, cleanup_stale_candidates
% =========================================================================

function [trackList, tempPool] = track_starter_mofn(trackList, tempPool, ...
        unused_dets, ukf_tpl, params, frame_id)

    % ---- 无未关联点迹时仅清理过期候选 ----
    if isempty(unused_dets)
        tempPool = cleanup_stale_candidates(tempPool, frame_id, params.tracker_N);
        return;
    end

    % 起始过程的地理距离门限: 60km
    % 候选点迹之间的几何距离必须在此门限内才能归为同一候选
    init_gate_m = 60000;

    % ---- 步骤1：每个未使用点迹回溯匹配tempPool候选 ----
    % 对每个新点迹，在已有候选池中寻找最近的候选序列
    for d = 1:length(unused_dets)
        dp = unused_dets(d);
        if ~isfield(dp, 'lat') || isnan(dp.lat), continue; end

        best_cand = 0;
        best_dist = inf;

        % 遍历tempPool，找与当前点迹最近的候选
        for c = 1:length(tempPool)
            cand = tempPool{c};
            if isempty(cand.points), continue; end

            % 取候选序列的最后一点（最新帧的点迹）
            last_pt = cand.points{end};
            if ~isfield(last_pt, 'lat') || isnan(last_pt.lat), continue; end

            % 球面距离
            dist = sphere_utils_haversine_distance(dp.lon, dp.lat, last_pt.lon, last_pt.lat);
            if dist < init_gate_m && dist < best_dist
                best_dist = dist;
                best_cand = c;
            end
        end

        if best_cand > 0
            % 追加到已有候选序列
            cand = tempPool{best_cand};
            cand.points{end+1} = dp;        % 添加点迹
            cand.frames(end+1) = frame_id;   % 记录帧号
            cand.lastFrame = frame_id;       % 更新最后活跃帧
            tempPool{best_cand} = cand;
        else
            % 创建新候选（新目标可能出现）
            cand.points = {dp};
            cand.frames = frame_id;
            cand.lastFrame = frame_id;
            tempPool{end+1} = cand;
        end
    end

    % ---- 步骤2：检查M/N条件，起始新航迹 ----
    M = params.tracker_M;  % 最少有点迹的帧数
    N = params.tracker_N;  % 滑窗总帧数

    % 标记哪些候选已被提升为航迹（避免重复处理）
    promoted = false(1, length(tempPool));
    n_promoted = 0;

    for c = 1:length(tempPool)
        cand = tempPool{c};
        n_pts = length(cand.points);  % 该候选序列包含的点迹数

        % ---- M条件检查：至少有M个点迹 ----
        if n_pts < M, continue; end

        % ---- N条件检查：点迹分布在N帧以内 ----
        frame_span = cand.frames(end) - cand.frames(1);
        if frame_span > N - 1, continue; end

        % ---- 提取首末点迹 ----
        first_det = cand.points{1};
        last_det  = cand.points{end};

        % ---- 距离合理性：首末点不超过300km ----
        % 防止将两个不同飞机的点迹误连为同一航迹
        dist_2pt = sphere_utils_haversine_distance(...
            first_det.lon, first_det.lat, last_det.lon, last_det.lat);
        if dist_2pt > 300000, continue; end

        % ---- 候选验证：点迹序列一致性检验 ----
        % 要求点迹在经纬度空间呈近似直线运动（杂波散布不满足）
        % 通过polynomial fitting检验残差和跨度
        if ~validate_candidate_sequence(cand), continue; end

        % ---- UKF初始化：利用首末两点差分 ----
        new_ukf = ukf_filter_init(ukf_tpl, first_det, last_det);

        % =============================================================
        % 航迹复活检查
        % =============================================================
        % 原理: 目标可能短暂消失（如低可观测性、雷达遮蔽）后重新
        %       出现。死亡航迹保留在trackList中（type=7），当新候选
        %       的起始位置靠近死亡航迹最后已知位置时，复活该航迹
        %       而不是创建全新航迹，保持航迹ID连续性。
        % =============================================================
        revived = false;
        revival_gate_m = 100000;   % 复活判断门限: 100km
        revival_window   = 15;     % 死亡后15帧内可复活

        for t = 1:length(trackList)
            trk = trackList{t};
            if trk.type ~= 7, continue; end         % 只检查HISTORY航迹
            if ~isfield(trk, 'death_frame'), continue; end
            if frame_id - trk.death_frame > revival_window, continue; end

            % 候选首点 vs 死亡航迹最后位置的距离
            dist = sphere_utils_haversine_distance(...
                first_det.lon, first_det.lat, trk.lon, trk.lat);
            if dist < revival_gate_m
                % ---- 航迹复活！ ----
                trk.type = 6;           % 恢复为TEMPORARY暂定航迹
                trk.quality = 9;        % 重置质量（接近RELIABLE升级阈值10）
                trk.missed = 0;         % 清零漏检计数
                trk.ukf = new_ukf;      % 用新UKF重新初始化滤波器
                trk.lat = new_ukf.x(3);
                trk.lon = new_ukf.x(1);
                trk.life = trk.life;    % 保留历史life计数
                trk.assoc_det = last_det;
                trk.nis_history = [];   % 清空NIS历史（新滤波器无历史）
                trk.init_points = n_pts;
                if ~isfield(trk, 'revived')
                    trk.revived = 0;
                end
                trk.revived = trk.revived + 1;  % 复活次数+1（用于统计）
                trackList{t} = trk;
                promoted(c) = true;
                revived = true;
                fprintf('  [复活] Frame %d: HISTORY#%d 复活 (距死亡%d帧, 接续距离%.0fkm)\n', ...
                    frame_id, trk.id, frame_id - trk.death_frame, dist/1000);
                break;
            end
        end

        % 如果已复活，跳过新建航迹
        if revived, continue; end

        % ---- 创建全新航迹 ----
        next_id = length(trackList) + 1;  % 顺序分配航迹ID
        new_trk.id = next_id;
        new_trk.type = 6;                  % TEMPORARY暂定航迹
        new_trk.quality = 9;               % 初始quality=9，仅需1次关联即可≥10升级RELIABLE
        new_trk.ukf = new_ukf;
        new_trk.life = 0;                  % 生命周期从0开始
        new_trk.missed = 0;
        new_trk.lat = new_ukf.x(3);
        new_trk.lon = new_ukf.x(1);
        new_trk.assoc_det = last_det;
        new_trk.nis_history = [];
        new_trk.birth_frame = frame_id;    % 出生帧记录
        new_trk.init_points = n_pts;       % 起始使用的点迹数
        new_trk.death_frame = NaN;         % 存活中（NaN表示未死亡）
        trackList{end+1} = new_trk;        % 追加到航迹列表
        promoted(c) = true;
    end

    % ---- 步骤3：移除已起始的候选 + 清理过期候选 ----
    % 已提升为航迹的候选需要从tempPool中移除
    tempPool = tempPool(~promoted);
    % 清理超过N帧未更新的过期候选（可能是杂波或消失的目标）
    tempPool = cleanup_stale_candidates(tempPool, frame_id, N);
end


% =========================================================================
% 辅助函数: cleanup_stale_candidates
% 清理tempPool中超过N帧未更新的过期候选序列
% 这些候选大概率是杂波形成的虚假序列，继续保留只会消耗资源
% =========================================================================
function tempPool = cleanup_stale_candidates(tempPool, current_frame, N)
    keep = true(1, length(tempPool));
    for c = 1:length(tempPool)
        if current_frame - tempPool{c}.lastFrame > N
            keep(c) = false;
        end
    end
    tempPool = tempPool(keep);
end


% =========================================================================
% 辅助函数: validate_candidate_sequence
% 验证候选点迹序列的空间一致性
%
% 原理: 真实目标的迹点序列在短时间跨度内应近似共线（直线运动）。
%       杂波由于随机散布，不会展现出这种一致性。通过多项式拟合检验
%       残差大小来区分真实目标与杂波。
%
% 检验条件:
%   1. 最大拟合残差 < 30 km（所有点迹基本在一条直线上）
%   2. 总跨度>100km时，残差 < 跨度的30%（相对误差约束）
%   3. 总跨度 < 500 km（30秒内飞行的物理上限）
% =========================================================================
function ok = validate_candidate_sequence(cand)
    n = length(cand.points);

    % M=3时仅3个点，做残差检验意义不大（2点必共线）
    if n < 3, ok = true; return; end

    % 提取经纬度序列
    lats = zeros(1, n); lons = zeros(1, n);
    for i = 1:n
        lats(i) = cand.points{i}.lat;
        lons(i) = cand.points{i}.lon;
    end

    % 在经纬度空间做一阶多项式拟合（直线）
    % 拟合模型: lon = a * lat + b
    p = polyfit(lats, lons, 1);
    lon_pred = polyval(p, lats);

    % 计算残差（度）
    residuals_deg = abs(lons - lon_pred);

    % 残差转km: 1° ≈ 111 km（赤道附近近似）
    residuals_km = max(residuals_deg) * 111;

    % 序列总跨度（km）
    total_dist_km = sphere_utils_haversine_distance(...
        lons(1), lats(1), lons(end), lats(end)) / 1000;

    % ---- 检验1: 最大残差 < 30 km ----
    if residuals_km > 30000
        ok = false; return;
    end

    % ---- 检验2: 长跨度时残差比例约束 ----
    % 跨度>100km时要求残差小于跨度的30%，防止点迹序列过度弯曲
    if total_dist_km > 100 && residuals_km > 0.3 * total_dist_km
        ok = false; return;
    end

    % ---- 检验3: 总跨度上限 ----
    % 30秒内（默认帧间隔约1-3秒）目标移动超过500km在物理上不可能
    if total_dist_km > 500
        ok = false; return;
    end

    ok = true;
end
