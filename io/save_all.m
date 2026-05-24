% save_data.m
% 数据持久化模块（面向过程+函数式版本）
% ==============
% 将仿真结果保存为 CSV 和 MAT 格式。
% - CSV: 人类可读，方便 Excel 或其他工具打开
% - MAT: MATLAB 原生格式，供后续分析模块直接加载
% 等价于 Python 版 simulation/io.py
%
% 本模块将原函数保持不变，函数式接口已适用。

function save_all(true_track, r1_meas_list, r2_meas_list, params, out_dir)
    % 保存全部仿真数据到指定目录
    %
    % 输入:
    %   true_track: 真实航迹数组 (n_steps, 7)
    %               列: [lon, lat, 0, lon_rate, lat_rate, 0, time]
    %   r1_meas_list: 雷达1量测 cell array of struct
    %   r2_meas_list: 雷达2量测 cell array of struct
    %   params: simulation_params 结构体
    %   out_dir: 输出目录路径

    % 确保输出目录存在
    if ~exist(out_dir, 'dir'), mkdir(out_dir); end

    n = size(true_track, 1);

    % ==================== CSV: 真实航迹 ====================
    fid = fopen(fullfile(out_dir, 'true_track.csv'), 'w');
    fprintf(fid, 'time_str,lon_deg,lat_deg,lon_rate_dps,lat_rate_dps,speed_ms\n');
    R = sphere_utils_get_earth_radius();               % 地球半径（米）
    for i = 1:n
        lat_rad = deg2rad(true_track(i, 2));           % 当前纬度（弧度）
        v_east  = true_track(i, 3) * (pi/180) * R * cos(lat_rad);  % 东向速度（m/s）
        v_north = true_track(i, 4) * (pi/180) * R;                 % 北向速度（m/s）
        speed_ms = sqrt(v_east^2 + v_north^2);                      % 合速度（m/s）
        time_str = sphere_utils_seconds_to_datetime_str(true_track(i, 5), params.ref_start_time);
        fprintf(fid, '%s,%.8f,%.8f,%.6f,%.6f,%.6f\n', ...
                time_str, ...
                true_track(i, 1), true_track(i, 2), ...
                true_track(i, 3), true_track(i, 4), ...
                speed_ms);
    end
    fclose(fid);

    % ==================== CSV: 两部雷达的量测 ====================
    radars = {'radar1', r1_meas_list; 'radar2', r2_meas_list};
    for r = 1:2
        rname = radars{r, 1};
        rmeas = radars{r, 2};
        fid = fopen(fullfile(out_dir, sprintf('%s_measurements.csv', rname)), 'w');
        fprintf(fid, ['time_str,range_meas_m,azimuth_meas_deg,radial_vel_meas_ms,' ...
                      'range_true_m,azimuth_true_deg,radial_vel_true_ms,lat_deg,lon_deg\n']);
        for i = 1:length(rmeas)
            m = rmeas{i};
            if isempty(m)
                fprintf(fid, 'nan,,,,,,,,\n');
                continue;
            end
            % 对齐后数据可能不含量测字段，只写经纬度
            if isfield(m, 'range_meas')
                fprintf(fid, '%s,%.3f,%.6f,%.4f', m.time_str, ...
                        m.range_meas, m.azimuth_meas, m.radial_vel_meas);
                if isfield(m, 'range_true')
                    fprintf(fid, ',%.3f,%.6f,%.4f', ...
                            m.range_true, m.azimuth_true, m.radial_vel_true);
                else
                    fprintf(fid, ',,,');
                end
                fprintf(fid, ',%.8f,%.8f\n', m.lat, m.lon);
            else
                fprintf(fid, '%s,,,,,,,,%.8f,%.8f\n', m.time_str, m.lat, m.lon);
            end
        end
        fclose(fid);
    end

    % 辅助函数：从量测列表中提取指定字段，漏检(NaN)填 NaN
    % 已提取到 extract_measurement_field.m

    % ==================== MAT: MATLAB 原生格式 ====================
    % 提取各字段
    r1_time = extract_measurement_field(r1_meas_list, 'time_sec');
    r1_range_meas = extract_measurement_field(r1_meas_list, 'range_meas');
    r1_az_meas = extract_measurement_field(r1_meas_list, 'azimuth_meas');
    r1_rv_meas = extract_measurement_field(r1_meas_list, 'radial_vel_meas');
    r1_range_true = extract_measurement_field(r1_meas_list, 'range_true');
    r1_az_true = extract_measurement_field(r1_meas_list, 'azimuth_true');
    r1_rv_true = extract_measurement_field(r1_meas_list, 'radial_vel_true');
    r1_lat = extract_measurement_field(r1_meas_list, 'lat');
    r1_lon = extract_measurement_field(r1_meas_list, 'lon');

    r2_time = extract_measurement_field(r2_meas_list, 'time_sec');
    r2_range_meas = extract_measurement_field(r2_meas_list, 'range_meas');
    r2_az_meas = extract_measurement_field(r2_meas_list, 'azimuth_meas');
    r2_rv_meas = extract_measurement_field(r2_meas_list, 'radial_vel_meas');
    r2_range_true = extract_measurement_field(r2_meas_list, 'range_true');
    r2_az_true = extract_measurement_field(r2_meas_list, 'azimuth_true');
    r2_rv_true = extract_measurement_field(r2_meas_list, 'radial_vel_true');
    r2_lat = extract_measurement_field(r2_meas_list, 'lat');
    r2_lon = extract_measurement_field(r2_meas_list, 'lon');

    % 保存 MAT 文件
    save(fullfile(out_dir, 'simulation_data.mat'), ...
         'true_track', ...
         'r1_time', 'r1_range_meas', 'r1_az_meas', 'r1_rv_meas', ...
         'r1_range_true', 'r1_az_true', 'r1_rv_true', ...
         'r1_lat', 'r1_lon', ...
         'r2_time', 'r2_range_meas', 'r2_az_meas', 'r2_rv_meas', ...
         'r2_range_true', 'r2_az_true', 'r2_rv_true', ...
         'r2_lat', 'r2_lon');

    % ==================== JSON: 元数据 ====================
    meta = struct();
    meta.project = 'HF Passive Radar Dual-Illuminator Track Association and Fusion';
    meta.phase = '1_trajectory_simulation';
    meta.generated_at = datestr(now, 'yyyy-mm-dd HH:MM:SS');
    meta.ref_start_time = datestr(params.ref_start_time, 'yyyy-mm-dd HH:MM:SS');
    meta.params = struct( ...
        'duration_sec', params.duration_sec, ...
        'dt_sec', params.dt_sec, ...
        'radar1_lon', params.radar1_lon, 'radar1_lat', params.radar1_lat, ...
        'radar2_lon', params.radar2_lon, 'radar2_lat', params.radar2_lat, ...
        'range_noise_std_m', params.range_noise_std_m, ...
        'azimuth_noise_std_deg', params.azimuth_noise_std_deg, ...
        'radial_vel_noise_std_ms', params.radial_vel_noise_std_ms, ...
        'detection_probability', params.detection_probability, ...
        'random_seed', params.random_seed);

    fid = fopen(fullfile(out_dir, 'simulation_metadata.json'), 'w');
    fprintf(fid, '%s', jsonencode(meta, 'PrettyPrint', true));
    fclose(fid);

    fprintf('数据已保存到 %s/\n', out_dir);
end
