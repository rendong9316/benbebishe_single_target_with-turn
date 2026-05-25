% =========================================================================
% measurement_simulator.m
% 量测仿真器模块 — 包含创建和量测两个函数
% =========================================================================
% 本文件合并了原 measurement_simulator_create.m 和
% measurement_simulator_measure.m，通过字符串分发统一入口。
%
% 调用方式:
%   sim = measurement_simulator('create', radar, params, rng_seed, range_bias, azimuth_bias, tx_lon, tx_lat)
%   result = measurement_simulator('measure', sim, target_lon, target_lat, lon_rate, lat_rate, time_sec)
% =========================================================================

function varargout = measurement_simulator(action, varargin)
    switch action
        case 'create'
            varargout{1} = measurement_simulator_create(varargin{:});
        case 'measure'
            varargout{1} = measurement_simulator_measure(varargin{:});
        otherwise
            error('measurement_simulator: unknown action "%s"', action);
    end
end

% ========================================================================
% measurement_simulator_create - 创建单雷达量测仿真器
% ========================================================================
% 将雷达定义、仿真参数和随机性控制封装到一个结构体中，
% 形成完整的"量测仿真器"对象。调用 measurement_simulator_measure
% 时只需要传递该结构体和目标状态即可完成一次仿真量测。
%
% 输入:
%   radar        - 雷达站结构体，包含 .lon（经度）和 .lat（纬度）
%   params       - 仿真参数结构体
%   rng_seed     - 整数，随机数生成器种子值
%   range_bias   - 标量（可选，默认 0），距离系统偏置（米）
%   azimuth_bias - 标量（可选，默认 0），方位系统偏置（度）
%   tx_lon       - 照射源（发射站）经度（度）
%   tx_lat       - 照射源（发射站）纬度（度）
% 输出:
%   sim - 量测仿真器结构体
% ========================================================================
function sim = measurement_simulator_create(radar, params, rng_seed, ...
                                            range_bias, azimuth_bias, tx_lon, tx_lat)
    sim.radar = radar;
    sim.params = params;

    if nargin < 4, range_bias = 0.0; end
    if nargin < 5, azimuth_bias = 0.0; end

    sim.range_bias = range_bias;
    sim.azimuth_bias = azimuth_bias;

    sim.tx_lon = tx_lon;
    sim.tx_lat = tx_lat;

    sim.rng_seed = rng_seed;
    rng(rng_seed);
    sim.rng_stream = rng_seed;
end

% ========================================================================
% measurement_simulator_measure - 雷达执行一次探测周期
% ========================================================================
% 模拟雷达一次完整的扫描/探测过程，包括：
%   1. 检测概率判断（可能漏检）
%   2. 计算真实极坐标量测值
%   3. 添加系统偏置和随机噪声
%   4. 将含噪量测反算回经纬度坐标
%
% 输入:
%   sim        - 量测仿真器结构体
%   target_lon - 目标当前经度（度）
%   target_lat - 目标当前纬度（度）
%   lon_rate   - 目标经度变化率（度/秒）
%   lat_rate   - 目标纬度变化率（度/秒）
%   time_sec   - 标量（可选，默认 0），当前仿真时间（秒）
% 输出:
%   result - 量测结果结构体（漏检时返回 []）
% ========================================================================
function result = measurement_simulator_measure(sim, target_lon, target_lat, ...
                                  lon_rate, lat_rate, time_sec)
    if nargin < 6, time_sec = 0.0; end

    % 第1步：检测概率判断——模拟漏检
    if sim.params.detection_probability < 1.0
        if rand() > sim.params.detection_probability
            result = [];
            return;
        end
    end

    % 第2步：计算真实极坐标量测值（无噪声真值）
    [true_rng, true_az, true_rv] = radar_station_true_polar( ...
        sim.radar, sim.tx_lon, sim.tx_lat, target_lon, target_lat, lon_rate, lat_rate);

    % 第3步：添加噪声——从真值到量测值
    noisy_rng = true_rng + sim.range_bias + randn() * sim.params.range_noise_std_m;
    noisy_az = true_az + sim.azimuth_bias + randn() * sim.params.azimuth_noise_std_deg;
    noisy_rv = true_rv + randn() * sim.params.radial_vel_noise_std_ms;

    % 第4步：双基地反解——从含噪群距离和方位角求目标经纬度
    baseline = sphere_utils_haversine_distance(sim.tx_lon, sim.tx_lat, ...
        sim.radar.lon, sim.radar.lat);
    tx_az = sphere_utils_azimuth(sim.radar.lon, sim.radar.lat, ...
        sim.tx_lon, sim.tx_lat);
    phi = noisy_az - tx_az;
    r1 = 0.5 * (noisy_rng^2 - baseline^2) / (noisy_rng - baseline * cosd(phi));

    [lon, lat] = sphere_utils_destination_point(sim.radar.lon, sim.radar.lat, r1, noisy_az);

    % 第5步：组装返回结构体
    result = struct( ...
        'time_sec', time_sec, ...
        'time_str', sphere_utils_seconds_to_datetime_str(time_sec, sim.params.ref_start_time), ...
        'range_true', true_rng, ...
        'azimuth_true', true_az, ...
        'radial_vel_true', true_rv, ...
        'range_meas', noisy_rng, ...
        'azimuth_meas', noisy_az, ...
        'radial_vel_meas', noisy_rv, ...
        'lat', lat, ...
        'lon', lon ...
    );
end
