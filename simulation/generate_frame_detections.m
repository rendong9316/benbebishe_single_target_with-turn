% =========================================================================
% generate_frame_detections.m
% 对单部雷达、单帧生成点迹列表（目标点迹 + 虚警杂波）
% =========================================================================

function [detList, has_target_det] = generate_frame_detections(rx_lon, rx_lat, ...
        tx_lon, tx_lat, tgt_lon, tgt_lat, tgt_lon_rate, tgt_lat_rate, ...
        frameID, time_sec, range_bias, az_bias, beam_center, params, ...
        range_noise, az_noise)

    if nargin < 16, range_noise = params.radar1_range_noise_std_m; end
    if nargin < 17, az_noise = params.radar1_azimuth_noise_std_deg; end

    detList = [];
    has_target_det = false;

    % ---- 检查目标是否在威力范围内 ----
    [in_cov, ~, ~] = radar_coverage_check(rx_lon, rx_lat, tgt_lon, tgt_lat, beam_center, params);

    if in_cov
        % ---- 检测概率判断 ----
        if rand() <= params.detection_probability
            has_target_det = true;

            r0 = sphere_utils_haversine_distance(tx_lon, tx_lat, tgt_lon, tgt_lat);
            r1_dist = sphere_utils_haversine_distance(rx_lon, rx_lat, tgt_lon, tgt_lat);
            Rg_true = r0 + r1_dist;
            az_true = sphere_utils_azimuth(rx_lon, rx_lat, tgt_lon, tgt_lat);

            az_tx = sphere_utils_azimuth(tx_lon, tx_lat, tgt_lon, tgt_lat);
            rv_tx = sphere_utils_radial_velocity(tgt_lon_rate, tgt_lat_rate, tgt_lat, az_tx);
            rv_rx = sphere_utils_radial_velocity(tgt_lon_rate, tgt_lat_rate, tgt_lat, az_true);
            vd_true = rv_tx + rv_rx;

            Rg_meas = Rg_true + range_bias + randn() * range_noise;
            az_meas = az_true + az_bias + randn() * az_noise;
            vd_meas = vd_true + randn() * params.radial_vel_noise_std_ms;

            det = struct('frameID', frameID, 'time_sec', time_sec, ...
                'prange', Rg_meas, 'paz', az_meas, 'pvr', vd_meas, ...
                'range_meas', Rg_meas, 'azimuth_meas', az_meas, 'radial_vel_meas', vd_meas, ...
                'range_true', Rg_true, 'azimuth_true', az_true, 'radial_vel_true', vd_true, ...
                'lat_true', tgt_lat, 'lon_true', tgt_lon, ...
                'lat', NaN, 'lon', NaN, 'is_clutter', false);
            detList = [detList, det];
        end
    end

    % ---- 虚警杂波生成（在(r1, az)空间均匀采样，保证威力范围内地理分布可控）----
    n_false = poissrnd(params.n_resolution_cells * params.false_alarm_rate);
    half_beam = params.beam_width_deg / 2;
    for f = 1:n_false
        % 在接收站极坐标(r1, az)中均匀采样
        fake_r1 = params.range_min_m + rand() * (params.range_max_m - params.range_min_m);
        fake_az = beam_center - half_beam + rand() * params.beam_width_deg;

        % 由(r1, az)正算杂波地理坐标
        [clut_lon, clut_lat] = sphere_utils_destination_point(rx_lon, rx_lat, fake_r1, fake_az);

        % 计算对应的双基地群距离 Rg = r0 + r1
        r0 = sphere_utils_haversine_distance(tx_lon, tx_lat, clut_lon, clut_lat);
        fake_Rg = r0 + fake_r1;

        % 杂波多普勒：OTH雷达杂波多普勒展宽通常不超过±200 m/s
        fake_vr = -200 + rand() * 400;

        % 掺入真实系统偏差，使主循环偏差校正后 drange≈fake_Rg, daz≈fake_az
        % 从而量测空间与地理空间保持一致
        det = struct('frameID', frameID, 'time_sec', time_sec, ...
            'prange', fake_Rg + range_bias, ...
            'paz', fake_az + az_bias, ...
            'pvr', fake_vr, ...
            'range_meas', NaN, 'azimuth_meas', NaN, 'radial_vel_meas', fake_vr, ...
            'range_true', NaN, 'azimuth_true', NaN, 'radial_vel_true', NaN, ...
            'lat_true', clut_lat, 'lon_true', clut_lon, ...
            'lat', clut_lat, 'lon', clut_lon, ...
            'is_clutter', true);
        detList = [detList, det];
    end
end
