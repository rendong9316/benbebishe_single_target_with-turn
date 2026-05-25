% =========================================================================
% jnn_association.m
% =========================================================================
% 【功能概述】
%   基于JNN（Joining Nearest Neighbor / 联合最近邻）策略的全局点迹-
%   航迹关联算法。对所有活跃航迹与未使用点迹之间计算马氏距离代价矩阵，
%   通过贪心全局分配算法解决多航迹竞争同一点迹的冲突问题。
%
% 【数学原理】
%   1. 马氏距离（Mahalanobis Distance）:
%         mahal = innov' * inv(P_zz) * innov
%      其中 innov = z_meas - z_pred 为新息（Innovation），
%      P_zz 为量测预测协方差矩阵（由UKF Sigma点传播得到）。
%      马氏距离服从自由度为量测维度的卡方分布，因此门限可设为
%      gate_threshold = gate_sigma^2 * dim(z)，此处 dim(z)=2。
%
%   2. 地理距离预筛选:
%      采用Haversine球面距离公式，在进入马氏距离计算之前先用地理
%      距离快速排除不可能关联的点迹，大幅降低计算量。
%
%   3. 贪心全局分配:
%      迭代选取代价矩阵中最小元素 → 分配该(track, point)对 →
%      移除该行和该列 → 重复直到剩余代价均为无穷大。
%      保证: (a) 每个点迹最多关联一条航迹
%            (b) 每条航迹最多关联一个点迹
%
% 【输入参数】
%   trackList  - cell数组，所有航迹结构体列表（含活跃和死亡航迹）
%   active_idx - 向量，活跃航迹在trackList中的索引
%   detList    - 结构体数组，当前帧所有未关联的点迹
%   params     - 参数结构体，需包含字段:
%                .gate_sigma: 波门Sigma倍数（如3表示3-sigma门）
%
% 【输出】
%   assoc_pairs - Nx2矩阵，每行 [航迹索引, 点迹索引]
%                 存储成功关联的配对。空矩阵表示无关联。
%
% 【调用关系】
%   被 multi_track_manager.m 在Step 4调用
%   调用 sphere_utils_haversine_distance (外部)
% =========================================================================

function assoc_pairs = jnn_association(trackList, active_idx, detList, params)
    % ---- 初始化：获取航迹数和点迹数 ----
    n_tracks = length(active_idx);
    n_dets = length(detList);

    % 初始化关联配对矩阵为空（0行2列）
    assoc_pairs = zeros(0, 2);
    if n_tracks == 0 || n_dets == 0, return; end

    % ---- 步骤1：计算马氏距离代价矩阵 ----
    % 代价矩阵 cost(i,j) = 航迹i与点迹j之间的马氏距离
    % 初始化为无穷大，表示不可关联
    cost = inf(n_tracks, n_dets);

    % 波门阈值: 对于2维量测，卡方分布下 gate_sigma^2 * 2
    % 例如 gate_sigma=3 时阈值为 9*2=18，对应约99.7%置信区间
    gate_threshold = params.gate_sigma^2 * 2;

    % ---- 遍历每条活跃航迹 ----
    for i = 1:n_tracks
        trk = trackList{active_idx(i)};

        % 如果航迹没有量测预测协方差P_zz，跳过（未完成预测步骤）
        if ~isfield(trk, 'P_zz'), continue; end

        % 提取距离-方位角的2x2协方差子矩阵
        P_zz_2d = trk.P_zz(1:2, 1:2);
        if any(isnan(P_zz_2d(:))), continue; end

        % 量测预测值 z_pred = [预测距离; 预测方位角]
        z_pred = trk.z_pred;

        % ---- 自适应地理距离波门 ----
        % 动态调整波门大小：收敛期宽，成熟期窄，漏检时放宽
        % 原理：UKF初始化后初期协方差较大，需要更宽的搜索范围
        %       连续漏检意味着目标可能发生了机动，应扩大搜索区域
        if trk.life <= 15
            geo_gate_m = 120000;  % UKF收敛期: 120km地理波门
        else
            geo_gate_m = 80000;   % 航迹成熟后: 80km地理波门
        end
        if trk.missed > 0
            % 每漏检一帧，地理波门扩大15km
            geo_gate_m = geo_gate_m + trk.missed * 15000;
        end

        % ---- 遍历每个点迹 ----
        for j = 1:n_dets
            dp = detList(j);
            % 跳过无效点迹（无距离量测或NaN）
            if ~isfield(dp, 'drange') || isnan(dp.drange), continue; end

            % ---- 地理距离预筛选 ----
            % 使用Haversine公式计算球面距离，快速排除空间上不可能
            % 关联的点迹。这一步计算量远小于矩阵求逆，可大幅加速。
            if isfield(dp, 'lat') && ~isnan(dp.lat) && ...
                    isfield(trk, 'x_pred')
                geo_dist = sphere_utils_haversine_distance(...
                    trk.x_pred(1), trk.x_pred(3), dp.lon, dp.lat);
                if geo_dist > geo_gate_m, continue; end  % 超出地理波门，跳过
            end

            % ---- 计算马氏距离 ----
            z_meas = [dp.drange; dp.daz];     % 量测向量 [距离; 方位角]
            innov = z_meas - z_pred(1:2);      % 新息 = 量测 - 预测

            % 角度新息归一化到 [-180, 180] 区间，处理0°/360°环绕
            if innov(2) > 180, innov(2) = innov(2) - 360;
            elseif innov(2) < -180, innov(2) = innov(2) + 360; end

            % 马氏距离: mahal = innov' * inv(P_zz_2d) * innov
            % 使用左除运算符 \ 等价于 inv(P_zz_2d)*innov，数值更稳定
            mahal = innov' * (P_zz_2d \ innov);
            if mahal < gate_threshold
                cost(i, j) = mahal;  % 落入波门，记录代价
            end
        end
    end

    % ---- 步骤2：贪心全局分配 ----
    % 维护两个布尔向量标记可用性:
    %   available_trk(i) = true 表示航迹i尚未分配
    %   available_pt(j)  = true 表示点迹j尚未分配
    available_trk = true(n_tracks, 1);
    available_pt  = true(n_dets, 1);

    while true
        % 寻找当前可用配对中的最小代价
        best_val = inf;
        best_i = 0;
        best_j = 0;
        for i = 1:n_tracks
            if ~available_trk(i), continue; end
            for j = 1:n_dets
                if ~available_pt(j), continue; end
                if cost(i, j) < best_val
                    best_val = cost(i, j);
                    best_i = i;
                    best_j = j;
                end
            end
        end

        % 如果所有剩余代价都是无穷大，表示没有更多有效配对，退出
        if isinf(best_val), break; end

        % 记录配对: [原始航迹索引, 点迹索引]
        % 注意 active_idx(best_i) 将局部索引转换回全局航迹索引
        assoc_pairs(end+1, :) = [active_idx(best_i), best_j];

        % 移除已分配的行和列，确保一对一约束
        available_trk(best_i) = false;
        available_pt(best_j) = false;
    end
end
