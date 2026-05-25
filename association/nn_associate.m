% =========================================================================
% nn_associate.m
% =========================================================================
% 功能概要：
%   Nearest Neighbor (NN) 点迹-航迹关联 — 纯过程式函数。
%   两步筛选：地理距离预筛 + 马氏距离精筛，返回最佳点迹和波门内所有点迹。
%
% 输入：
%   x_pred    - 预测状态 [lon; lon_dot; lat; lat_dot] (4x1)
%   z_pred    - 预测量测 [range; azimuth; radial_vel] (3x1)
%   P_zz_2d   - 量测协方差 2D 子矩阵 (2x2)
%   det_list  - 当前帧检测点迹结构体数组
%   params    - 参数结构体（含 gate_sigma）
%   track_life - 航迹生命期（帧数），用于地理波门自适应
%
% 输出：
%   best_det     - 马氏距离最小的点迹（若门内无点迹则为空）
%   dets_in_gate - cell 数组，波门内所有点迹
% =========================================================================

function [best_det, dets_in_gate] = nn_associate(x_pred, z_pred, P_zz_2d, det_list, params, track_life)

    % ---- Step 1: 地理距离预筛选波门大小 ----
    geo_gate_m = 120000;  % 初始阶段 120km 地理波门
    if track_life > 15
        geo_gate_m = 60000;  % UKF 收敛后缩小到 60km
    end

    % ---- Step 2: NN 关联（地理预筛选 + 马氏距离精筛选） ----
    best_det = [];
    best_mahal = inf;

    for d = 1:length(det_list)
        dp = det_list(d);
        if ~isfield(dp, 'lat') || isnan(dp.lat)
            continue;
        end

        % 第一阶段: 地理距离预筛选
        geo_dist = sphere_utils_haversine_distance(...
            x_pred(1), x_pred(3), dp.lon, dp.lat);
        if geo_dist > geo_gate_m
            continue;
        end

        % 第二阶段: 马氏距离精筛选
        z_m = [dp.drange; dp.daz];
        innov = z_m - z_pred(1:2);
        if innov(2) > 180
            innov(2) = innov(2) - 360;
        elseif innov(2) < -180
            innov(2) = innov(2) + 360;
        end
        mahal = innov' * (P_zz_2d \ innov);

        gate_threshold = params.gate_sigma^2 * 2;
        if mahal < gate_threshold && mahal < best_mahal
            best_mahal = mahal;
            best_det = dp;
        end
    end

    % ---- Step 3: 收集波门内所有点迹（用于 PDA 多假设） ----
    dets_in_gate = {};
    if ~isempty(best_det)
        dets_in_gate = {best_det};
    end
    gate_threshold = params.gate_sigma^2 * 2;
    for d = 1:length(det_list)
        dp = det_list(d);
        if ~isempty(best_det) && isequal(dp, best_det)
            continue;
        end
        if ~isfield(dp, 'drange') || isnan(dp.drange)
            continue;
        end
        z_m = [dp.drange; dp.daz];
        innov = z_m - z_pred(1:2);
        if innov(2) > 180
            innov(2) = innov(2) - 360;
        elseif innov(2) < -180
            innov(2) = innov(2) + 360;
        end
        if innov' * (P_zz_2d \ innov) < gate_threshold
            dets_in_gate{end+1} = dp;
        end
    end
end
