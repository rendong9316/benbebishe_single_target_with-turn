% =========================================================================
% generate_frame_detections_multi.m — 多目标单帧点迹生成函数
% =========================================================================
%
% 【功能概述】
%   对单部双基地雷达、单次扫描（一帧）生成完整的点迹列表，包含：
%     1. 多个目标点迹 — 每个目标经检测概率 P_d 筛选后，叠加系统偏差和随机噪声
%     2. 虚警杂波 — 在雷达威力覆盖区内按泊松分布随机生成
%   每个检测点迹带有 aircraft_id 字段，标记所属的目标编号（1-based）。
%
% 【与 generate_frame_detections.m 的区别】
%   - 输入为 N 个目标的状态数组（每个目标独立检测），而非单个目标
%   - 每个检测输出带有 .aircraft_id 字段
%   - 虚警杂波的 aircraft_id = 0（或 -1）
%
% 【输入参数】
%   rx_lon, rx_lat    — 接收站经纬度（度）
%   tx_lon, tx_lat    — 发射站经纬度（度）
%   tgt_states        — N_targets x 5 矩阵，每行 [lon, lat, lon_rate, lat_rate, aircraft_id]
%   frameID           — 帧编号
%   time_sec          — 当前帧的仿真时间（秒）
%   range_bias, az_bias — 系统偏差
%   beam_center, params — 雷达参数
%   range_noise, az_noise — 噪声标准差（可选）
%
% 【输出】
%   detList — 结构体数组，每帧所有点迹（目标+杂波）
%   has_target_dets — N_targets x 1 逻辑数组，每个目标是否被检测到
% =========================================================================

function [detList, has_target_dets] = generate_frame_detections_multi(rx_lon, rx_lat, ...
        tx_lon, tx_lat, tgt_states, frameID, time_sec, range_bias, az_bias, ...
        beam_center, params, range_noise, az_noise)

    if nargin < 15, range_noise = params.radar1_range_noise_std_m; end
    if nargin < 16, az_noise = params.radar1_azimuth_noise_std_deg; end

    detList = [];
    n_targets = size(tgt_states, 1);
    has_target_dets = false(n_targets, 1);

    half_beam = params.beam_width_deg / 2;
    n_false = poissrnd(params.n_resolution_cells * params.false_alarm_rate);

    % ---- 第 1 部分：逐个目标检测 ----
    for t = 1:n_targets
        tgt_lon = tgt_states(t, 1);
        tgt_lat = tgt_states(t, 2);
        tgt_lr = tgt_states(t, 3);
        tgt_latr = tgt_states(t, 4);
        ac_id = tgt_states(t, 5);

        [in_cov, ~, ~] = radar_coverage_check(rx_lon, rx_lat, tgt_lon, tgt_lat, ...
            beam_center, params);

        if in_cov && rand() <= params.detection_probability
            has_target_dets(t) = true;

            Rg_true = skywave_geometry('group_range', tx_lon, tx_lat, ...
                rx_lon, rx_lat, tgt_lon, tgt_lat);
            az_true = skywave_geometry('azimuth', rx_lon, rx_lat, tgt_lon, tgt_lat);
            vd_true = skywave_geometry('doppler', tx_lon, tx_lat, rx_lon, rx_lat, ...
                tgt_lon, tgt_lat, tgt_lr, tgt_latr);

            Rg_meas = Rg_true + range_bias + randn() * range_noise;
            az_meas = az_true + az_bias + randn() * az_noise;
            vd_meas = vd_true + randn() * params.radial_vel_noise_std_ms;

            det = struct('frameID', frameID, 'time_sec', time_sec, ...
                'prange', Rg_meas, 'paz', az_meas, 'pvr', vd_meas, ...
                'range_meas', Rg_meas, 'azimuth_meas', az_meas, 'radial_vel_meas', vd_meas, ...
                'range_true', Rg_true, 'azimuth_true', az_true, 'radial_vel_true', vd_true, ...
                'lat_true', tgt_lat, 'lon_true', tgt_lon, ...
                'lat', NaN, 'lon', NaN, ...
                'is_clutter', false, ...
                'aircraft_id', int32(ac_id));
            detList = [detList, det];
        end
    end

    % ---- 第 2 部分：虚警杂波 ----
    for f = 1:n_false
        fake_r1 = params.range_min_m + rand() * (params.range_max_m - params.range_min_m);
        fake_az = beam_center - half_beam + rand() * params.beam_width_deg;

        [clut_lon, clut_lat] = sphere_utils_destination_point(rx_lon, rx_lat, fake_r1, fake_az);

        fake_Rg = skywave_geometry('group_range', tx_lon, tx_lat, rx_lon, rx_lat, ...
            clut_lon, clut_lat);
        fake_vr = -200 + rand() * 400;

        det = struct('frameID', frameID, 'time_sec', time_sec, ...
            'prange', fake_Rg + range_bias, ...
            'paz', fake_az + az_bias, ...
            'pvr', fake_vr, ...
            'range_meas', NaN, 'azimuth_meas', NaN, 'radial_vel_meas', fake_vr, ...
            'range_true', NaN, 'azimuth_true', NaN, 'radial_vel_true', NaN, ...
            'lat_true', clut_lat, 'lon_true', clut_lon, ...
            'lat', clut_lat, 'lon', clut_lon, ...
            'is_clutter', true, ...
            'aircraft_id', int32(0));
        detList = [detList, det];
    end
end
