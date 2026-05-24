% ========================================================================
% measurement_simulator_measure.m
% ========================================================================
%
% 【功能概述】
% 量测仿真执行函数。模拟雷达对单个目标执行一次完整的探测周期，
% 生成包含噪声的量测数据。本函数是仿真系统中"从真实值到量测值"
% 的核心转换环节。
%
% 【数学原理】
% 本函数实现了以下数学模型（雷达量测模型）：
%
% 1. 检测判断（Detection Gate）：
%    如果检测概率 p < 1.0，则生成一个 [0, 1] 均匀分布随机数 u。
%    若 u > p，则判定为"漏检"（Missed Detection），返回空数组 []。
%    这模拟了雷达并非每次扫描都能检测到目标的事实。
%
% 2. 真值计算（见 radar_station_true_polar）：
%    [true_rng, true_az, true_rv] = f(radar, target_lon, target_lat, ...)
%
% 3. 噪声添加（独立高斯噪声模型）：
%    noisy_rng = true_rng + bias_range + w_range
%    noisy_az  = true_az  + bias_azimuth + w_azimuth
%    noisy_rv  = true_rv  + w_radial_vel
%    其中：
%      w_range      ~ N(0, σ_range²)        (距离噪声，米)
%      w_azimuth    ~ N(0, σ_azimuth²)      (方位噪声，度)
%      w_radial_vel ~ N(0, σ_radial_vel²)   (径向速度噪声，m/s)
%    randn() 产生标准正态分布 N(0,1) 随机数，乘以 σ 后获得目标方差
%
% 4. 反向地理定位（球面正算）：
%    利用 sphere_utils_destination_point，根据雷达位置、含噪距离
%    和含噪方位角，反算目标的经纬度估计值：
%      (meas_lon, meas_lat) = f_inv(radar_lon, radar_lat, noisy_rng, noisy_az)
%    这与定位算法（如卡尔曼滤波）的量测反算步骤一致。
%
% 【在项目流水线中的位置】
% 本函数位于仿真系统的"传感器模拟"核心位置：
%
%   [真实目标轨迹]  →  aircraft_trajectory_generate
%         ↓
%   [雷达量测仿真]  →  measurement_simulator_measure ★本函数★
%         ↓
%   [航迹关联/融合]  →  滤波算法、航迹管理
%
% 【输入参数】
%   sim        - 量测仿真器结构体（由 measurement_simulator_create 创建），
%                包含雷达信息、噪声参数、检测概率和系统偏置
%   target_lon - 标量双精度浮点数，目标当前经度（度）
%   target_lat - 标量双精度浮点数，目标当前纬度（度）
%   lon_rate   - 标量双精度浮点数，目标经度变化率（度/秒）
%   lat_rate   - 标量双精度浮点数，目标纬度变化率（度/秒）
%   time_sec   - 标量双精度浮点数（可选，默认 0），当前仿真时间（秒），
%                相对于参考起始时间的偏移量
%
% 【返回值】
%   result - 结构体（struct），包含以下字段（如果漏检则返回 []）：
%     .time_sec        : 仿真时间（秒）
%     .time_str        : 格式化的时间字符串（如 '2001-09-11 08:05:30'）
%     .range_true      : 真实斜距（米）
%     .azimuth_true    : 真实方位角（度）
%     .radial_vel_true : 真实径向速度（m/s）
%     .range_meas      : 含噪量测斜距（米）= 真值 + 偏置 + 噪声
%     .azimuth_meas    : 含噪量测方位角（度）= 真值 + 偏置 + 噪声
%     .radial_vel_meas : 含噪量测径向速度（m/s）= 真值 + 噪声
%     .lat             : 从含噪量测反算得到的纬度（度）
%     .lon             : 从含噪量测反算得到的经度（度）
%
% 【使用方法】
%   sim = measurement_simulator_create(radar, params, 42, 0, 0);
%   result = measurement_simulator_measure(sim, 113.0, 34.0, 0.001, 0.0008, 120.0);
%   if isempty(result)
%       disp('目标漏检');
%   else
%       fprintf('量测距离: %.1f m\n', result.range_meas);
%   end
%
% 【注意事项】
%   - 如果检测概率 < 1.0，本函数可能返回空数组 []，调用方必须检查
%   - 噪声使用 randn() 生成，其状态受 measurement_simulator_create
%     中设置的 rng_seed 控制
%   - 从含噪距离和方位角反算经纬度的精度取决于球面正算公式的近似程度
%   - 三个噪声分量相互独立（在数学上），但在一次调用中使用同一条
%     随机数流，因此彼此之间也有统计独立性
% ========================================================================

function result = measurement_simulator_measure(sim, target_lon, target_lat, ...
                                  lon_rate, lat_rate, time_sec)
    % ----------------------------------------------------------------
    % measurement_simulator_measure - 雷达执行一次探测周期
    % ----------------------------------------------------------------
    % 本函数模拟雷达一次完整的扫描/探测过程，包括：
    %   1. 检测概率判断（可能漏检）
    %   2. 计算真实极坐标量测值
    %   3. 添加系统偏置和随机噪声
    %   4. 将含噪量测反算回经纬度坐标
    %
    % 这是一个随机函数：即使输入相同（同样的目标位置和仿真器），
    % 由于随机噪声的存在，每次调用的输出也可能不同。
    % （除非仿真器中固定了随机数种子，此时同一序列可精确复现）
    % ----------------------------------------------------------------

    % ---- 处理可选参数：仿真时间 ----
    % if nargin < 6：如果调用者未提供第6个参数（time_sec）
    %   则默认为 0.0 秒（仿真起始时刻）
    if nargin < 6, time_sec = 0.0; end

    % ================================================================
    % 第1步：检测概率判断——模拟漏检
    % ================================================================
    % 如果检测概率小于 100%（即 sim.params.detection_probability < 1.0），
    % 则有概率发生漏检（目标存在但雷达未检测到）
    if sim.params.detection_probability < 1.0
        % rand()：产生 [0, 1] 区间内均匀分布的随机数
        % 如果该随机数 > 检测概率，则判定为漏检
        % 例如：检测概率 = 0.9，则大约有 10% 的概率漏检
        if rand() > sim.params.detection_probability
            % 漏检：返回空数组 []，表示本次探测没有产生量测
            % 调用方需要通过 isempty(result) 检查这种情况
            result = [];
            return;  % 提前返回，不执行后续的量测计算
        end
    end

    % ================================================================
    % 第2步：计算真实极坐标量测值（无噪声真值）
    % ================================================================
    % 调用 radar_station_true_polar 计算双基地量测真值：
    %   群距离 Rg = distance(Tx,target) + distance(target,Rx)
    %   方位角（接收站测量）
    %   双基地径向速度 = 两路径径向速度之和
    [true_rng, true_az, true_rv] = radar_station_true_polar( ...
        sim.radar, sim.tx_lon, sim.tx_lat, target_lon, target_lat, lon_rate, lat_rate);

    % ================================================================
    % 第3步：添加噪声——从真值到量测值
    % ================================================================

    % ---- 3a. 含噪斜距计算 ----
    % true_rng                     : 真实斜距（米）
    % + sim.range_bias             : 距离系统偏置（米，常数偏移）
    % + randn() * sim.params.range_noise_std_m
    %                               : 高斯随机噪声
    % randn()：产生标准正态分布 N(0,1) 的随机数
    % 乘以 range_noise_std_m 后变为 N(0, σ_range²)
    noisy_rng = true_rng + sim.range_bias + randn() * sim.params.range_noise_std_m;

    % ---- 3b. 含噪方位角计算 ----
    % true_az                      : 真实方位角（度）
    % + sim.azimuth_bias           : 方位系统偏置（度，常数偏移）
    % + randn() * sim.params.azimuth_noise_std_deg
    %                               : 高斯随机噪声
    % 乘以 azimuth_noise_std_deg 后变为 N(0, σ_az²)
    noisy_az = true_az + sim.azimuth_bias + randn() * sim.params.azimuth_noise_std_deg;

    % ---- 3c. 含噪径向速度计算 ----
    % true_rv                      : 真实径向速度（m/s）
    % + randn() * sim.params.radial_vel_noise_std_ms
    %                               : 高斯随机噪声（径向速度无系统偏置）
    % 乘以 radial_vel_noise_std_ms 后变为 N(0, σ_rv²)
    noisy_rv = true_rv + randn() * sim.params.radial_vel_noise_std_ms;

    % ================================================================
    % 第4步：双基地反解——从含噪群距离和方位角求目标经纬度
    % ================================================================
    % 双基地群距离 Rg = r0 + r1 定义了一个以Tx和Rx为焦点的椭圆。
    % 接收站方位角 θ 确定了从Rx出发的射线方向。
    % 目标位于椭圆与射线的交点处。
    %
    % 已知: 基线 d = distance(Tx, Rx)
    %       Tx 在 Rx 坐标系中的方位 β = azimuth(Rx, Tx)
    %       目标方位与基线夹角 φ = noisy_az - β
    % 由椭圆方程: r1² + d² - 2·r1·d·cos(φ) = (Rg - r1)²
    % 解出 r1（目标到接收站的地表距离）:
    %       r1 = (Rg² - d²) / (2·(Rg - d·cos(φ)))
    baseline = sphere_utils_haversine_distance(sim.tx_lon, sim.tx_lat, ...
        sim.radar.lon, sim.radar.lat);
    tx_az = sphere_utils_azimuth(sim.radar.lon, sim.radar.lat, ...
        sim.tx_lon, sim.tx_lat);
    phi = noisy_az - tx_az;
    r1 = 0.5 * (noisy_rng^2 - baseline^2) / (noisy_rng - baseline * cosd(phi));

    % 从接收站沿方位角方向前进 r1 距离，得到目标经纬度
    [lon, lat] = sphere_utils_destination_point(sim.radar.lon, sim.radar.lat, r1, noisy_az);

    % ================================================================
    % 第5步：组装返回结构体
    % ================================================================
    % struct('field1', val1, ...)：创建一个包含所有量测信息的结构体
    % 该结构体同时保存了真实值（用于评估算法性能）和含噪量测值
    % （用于模拟实际雷达输出），方便后续的滤波和航迹关联算法使用
    result = struct( ...
        'time_sec', time_sec, ...
        ... % ---- 格式化时间字符串 ----
        ... % sphere_utils_seconds_to_datetime_str 将秒转换为可读的 datetime 字符串
        ... % 例如：time_sec=120.0, ref_start_time='2001-09-11 08:00:00'
        ... %   → time_str = '2001-09-11 08:02:00'
        'time_str', sphere_utils_seconds_to_datetime_str(time_sec, sim.params.ref_start_time), ...
        ... % ---- 真实值（Ground Truth） ----
        'range_true', true_rng, ...        % 真实群距离 Rg = r0+r1（米）
        'azimuth_true', true_az, ...       % 真实方位角（度）
        'radial_vel_true', true_rv, ...    % 真实双基地径向速度（m/s）
        ... % ---- 含噪量测值（模拟雷达实际输出） ----
        'range_meas', noisy_rng, ...       % 含噪群距离（米）
        'azimuth_meas', noisy_az, ...      % 含噪方位角（度）
        'radial_vel_meas', noisy_rv, ...   % 含噪双基地径向速度（m/s）
        ... % ---- 从含噪量测反算的经纬度 ----
        'lat', lat, ...                    % 量测推算纬度（度）
        'lon', lon ...                     % 量测推算经度（度）
    );
end
% ========================================================================
% 文件结束
% ========================================================================
