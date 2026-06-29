% =========================================================================
% nn_associate.m — 最近邻点迹-航迹关联（马氏距离 + 硬Vr门）
% =========================================================================
% 两步筛选：地理距离预筛 → 2D马氏距离精筛 → 硬Vr门过滤杂波
% 马氏距离自适应UKF不确定度，Vr硬门补强杂波抑制。
%
% 输入：
%   x_pred    - 预测状态 [lon; lon_dot; lat; lat_dot] (4x1)
%   z_pred    - 预测量测 [range; azimuth; radial_vel] (3x1)
%   P_zz_2d   - 量测协方差 2D 子矩阵 (2x2)
%   det_list  - 当前帧检测点迹结构体数组
%   params    - 参数结构体（含 gate_sigma, gate_vr_ms）
%   track_life - 航迹生命期（帧数），用于地理波门自适应
%
% 输出：
%   best_det     - 马氏距离最小的点迹（若门内无点迹则为空）
%   dets_in_gate - cell 数组，波门内所有点迹
% =========================================================================

function [best_det, dets_in_gate] = nn_associate(x_pred, z_pred, P_zz_2d, det_list, params, track_life)

    % ---- Step 1: 地理距离预筛选波门 ----
    geo_gate_m = 120000;
    if track_life > 15
        geo_gate_m = 60000;
    end

    % ---- Step 2: 马氏距离门（2D: range+az，自适应UKF不确定度） ----
    gate_threshold = params.gate_sigma^2 * 2;

    % ---- Step 3: 硬Vr门（额外杂波过滤） ----
    % probation期(life≤8)速度初值不可靠，Vr门放宽到200=不过滤
    % UKF收敛后恢复收紧，滤除杂波Vr随机[-200,200]
    gate_vr_ms = params.gate_vr_ms;
    if track_life <= 8
        gate_vr_ms = max(gate_vr_ms, 200);
    end

    % ---- Step 4: NN 关联 ----
    best_det = [];
    best_mahal = inf;

    for d = 1:length(det_list)
        dp = det_list(d);
        if ~isfield(dp, 'lat') || isnan(dp.lat)
            continue;
        end

        % 4.1 地理距离预筛选
        geo_dist = sphere_utils_haversine_distance(...
            x_pred(1), x_pred(3), dp.lon, dp.lat);
        if geo_dist > geo_gate_m
            continue;
        end

        % 4.2 硬Vr门（杂波Vr随机[-200,200]，真目标帧间Vr变化<5m/s）
        vr_diff = abs(dp.radial_vel_meas - z_pred(3));
        if vr_diff > gate_vr_ms
            continue;
        end

        % 4.3 2D马氏距离精筛（range + azimuth）
        z_m = [dp.drange; dp.daz];
        innov = z_m - z_pred(1:2);
        if innov(2) > 180
            innov(2) = innov(2) - 360;
        elseif innov(2) < -180
            innov(2) = innov(2) + 360;
        end
        mahal = innov' * (P_zz_2d \ innov);

        if mahal < gate_threshold && mahal < best_mahal
            best_mahal = mahal;
            best_det = dp;
        end
    end

    % ---- Step 5: 收集波门内所有点迹（用于 PDA） ----
    dets_in_gate = {};
    if ~isempty(best_det)
        dets_in_gate = {best_det};
    end
    for d = 1:length(det_list)
        dp = det_list(d);
        if ~isempty(best_det) && isequal(dp, best_det)
            continue;
        end
        if ~isfield(dp, 'drange') || isnan(dp.drange)
            continue;
        end

        geo_dist = sphere_utils_haversine_distance(...
            x_pred(1), x_pred(3), dp.lon, dp.lat);
        if geo_dist > geo_gate_m, continue; end

        vr_diff = abs(dp.radial_vel_meas - z_pred(3));
        if vr_diff > gate_vr_ms, continue; end

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
