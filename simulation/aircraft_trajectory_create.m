% ========================================================================
% aircraft_trajectory_create.m
% ========================================================================
%
% 【功能概述】
% 飞机真实航迹生成模块——航迹结构体创建函数。
% 本函数是整个仿真系统的核心数据生成入口，负责根据给定的航路点和
% 巡航速度，创建一个完整的航迹结构体，供后续的插值、量测仿真和
% 航迹关联算法使用。
%
% 【数学原理】
%
% 1. 航迹分段模型：
%    航迹由 N 个航路点（Waypoints）定义了 (N-1) 个航段（Segments）。
%    第 i 个航段从 waypoints(i) 到 waypoints(i+1)，在航段内部飞机以
%    恒定速率匀速直线飞行（恒向线 / Rhumb Line）。
%
% 2. 球面距离计算（Haversine 公式）：
%    对于地球上两点 (lon1, lat1) 和 (lon2, lat2)，球面距离为：
%      a = sin²(Δlat/2) + cos(lat1)*cos(lat2)*sin²(Δlon/2)
%      c = 2 * atan2(√a, √(1-a))
%      d = R * c
%    其中 R = 6371000 米（地球平均半径）
%
% 3. 航段时长计算：
%    dur(i) = dist(i) / speed_ms
%    其中 dist(i) 为第 i 个航段的球面距离（米），speed_ms 为巡航速度
%
% 4. 经纬度变化率：
%    lon_rate(i) = (lon_{i+1} - lon_i) / dur(i)   [度/秒]
%    lat_rate(i) = (lat_{i+1} - lat_i) / dur(i)   [度/秒]
%    在航段内部，这两个变化率保持恒定，实现匀速直线运动
%
% 5. 累计时间：
%    t_start(i) = sum_{j=1}^{i-1} dur(j)          [秒]
%    即第 i 个航段的起始绝对时间 = 之前所有航段时长之和
%
% 6. 时间数组（用于后续批量采样）：
%    time_array = [0, dt_sec, 2*dt_sec, ..., duration_sec]
%    共 n_steps 个采样点，采样间隔为 dt_sec
%
% 【在项目流水线中的位置】
% 本函数是航迹生成流水线的第一环，位于整个仿真流程的最前端：
%
%  ┌─────────────────────────────────────────────────────────┐
%  │  数据生成流水线（Simulation Pipeline）                    │
%  │                                                         │
%  │  [1] radar_station_create        创建雷达站              │
%  │         ↓                                               │
%  │  [2] aircraft_trajectory_create  创建航迹结构 ★本文件★   │
%  │         ↓                                               │
%  │  [3] aircraft_trajectory_interpolate('generate', ...) 生成完整轨迹采样        │
%  │         ↓                                               │
%  │  [4] measurement_simulator_create 创建量测仿真器          │
%  │         ↓                                               │
%  │  [5] measurement_simulator_measure 执行量测（逐点）       │
%  │         ↓                                               │
%  │  [6] 航迹关联算法（滤波、关联、融合）                     │
%  └─────────────────────────────────────────────────────────┘
%
% 【输入参数】
%   waypoints_lla - N×3 双精度矩阵，每行表示一个航路点，
%                   格式为 [lon_i, lat_i, alt_i（通常为0）]
%                   N >= 2（至少需要2个航路点）
%   speed_ms      - 标量双精度浮点数，飞机巡航速度（单位：m/s）
%                   所有航段使用相同的巡航速度
%   dt_sec        - 标量双精度浮点数，时间采样步长（单位：秒）
%                   决定了最终生成轨迹的时间分辨率
%
% 【返回值】
%   traj          - 结构体（struct），包含以下字段：
%     .speed        : 巡航速度（m/s）
%     .dt_sec       : 时间步长（秒）
%     .waypoints    : (N×2) 航路点经纬度矩阵 [lon, lat]
%     .segments     : cell 数组，大小为 (N-1)×1
%                     每个元素为一个结构体，包含：
%                       .start    : 1×2 向量 [lon, lat] 航段起点
%                       .end      : 1×2 向量 [lon, lat] 航段终点
%                       .lon_rate : 经度变化率（度/秒）
%                       .lat_rate : 纬度变化率（度/秒）
%                       .dur      : 航段时长（秒）
%                       .t_start  : 航段起始绝对时间（秒）
%     .duration_sec : 航迹总时长（秒）
%     .n_segments   : 航段数量（= 航路点数 - 1）
%     .time_array   : 1×M 行向量，均匀采样时间序列 [秒]
%     .n_steps      : 总采样步数
%
% 【使用方法】
%   % 定义航路点：南阳 → 郑州 → 武汉
%   waypoints = [112.50, 33.00, 0;
%                113.65, 34.76, 0;
%                114.30, 30.60, 0];
%   traj = aircraft_trajectory_create(waypoints, 250.0, 30.0);
%   % traj 现在包含完整的航迹信息，可供后续函数使用
%
% 【注意事项】
%   - waypoints_lla 的第三列（高度）在函数内部被忽略，仅取前两列
%   - 所有航段使用相同的巡航速度 speed_ms，不模拟加速/减速
%   - 经纬度变化率直接使用度/秒，在创建航段时预计算好，
%     后续插值时无需重复计算，提高了批量插值的效率
%   - 航段起点/终点的经纬度直接来自航路点，不进行额外变换
% ========================================================================

function varargout = aircraft_trajectory_create(varargin)
    % ----------------------------------------------------------------
    % aircraft_trajectory_create - 从航路点创建飞机航迹结构体
    % ----------------------------------------------------------------
    % 支持两种调用方式：
    %   1. traj = aircraft_trajectory_create(waypoints_lla, speed_ms, dt_sec)
    %      从航路点创建直线航迹结构体
    %   2. [traj, waypoints] = aircraft_trajectory_create('turn', params)
    %      创建带拐弯的3航路点航迹结构体

    % ---- 字符串分发 ----
    if nargin >= 1 && ischar(varargin{1})
        switch varargin{1}
            case 'turn'
                [varargout{1}, varargout{2}] = create_turn_trajectory(varargin{2});
            case 'gradual_turn'
                [varargout{1}, varargout{2}] = create_gradual_turn_trajectory(varargin{2});
            case 'uturn'
                [varargout{1}, varargout{2}] = create_uturn_trajectory(varargin{2});
            otherwise
                error('aircraft_trajectory_create: unknown action "%s"', varargin{1});
        end
        return;
    end

    % ---- 原始调用路径 ----
    waypoints_lla = varargin{1};
    speed_ms = varargin{2};
    dt_sec = varargin{3};
    % 本函数的核心任务是将离散的航路点序列转换为一个结构化的
    % 航迹对象。每个相邻航路点之间定义为一个"航段"，在航段内
    % 飞机以恒定速率匀速飞行。
    %
    % 计算流程：
    %   1. 保存基本参数（速度、步长）
    %   2. 提取航路点的经纬度（忽略高度）
    %   3. 遍历每对相邻航路点，为每个航段：
    %      a. 用 Haversine 公式计算球面距离
    %      b. 距离 / 速度 = 航段时长
    %      c. 经纬度差 / 时长 = 经纬度变化率
    %      d. 记录航段信息到结构体
    %      e. 累加累计时间
    %   4. 生成均匀时间采样数组
    % ----------------------------------------------------------------

    % ================================================================
    % 第1步：保存基本飞行参数
    % ================================================================
    traj.speed  = speed_ms;    % 巡航速率（m/s），所有航段统一使用
    traj.dt_sec = dt_sec;     % 时间采样步长（秒），决定输出轨迹的时间分辨率

    % ---- 航路点提取：仅取经纬度，忽略第三列高度 ----
    % waypoints_lla(:, 1:2) 含义：
    %   冒号 :  → 取所有行（即所有航路点）
    %   1:2     → 取第1列和第2列（即经度和纬度）
    %   第三列（通常为高度 0）被丢弃，因为本系统使用球面二维模型
    traj.waypoints = waypoints_lla(:, 1:2);

    % ================================================================
    % 第2步：计算每个航段的参数
    % ================================================================

    % ---- 航路点数量 ----
    % size(traj.waypoints, 1) 返回矩阵的第1维大小（行数），即航路点个数
    % 例如：3个航路点 → n_wp = 3 → 将有2个航段
    n_wp = size(traj.waypoints, 1);

    % ---- 预分配 cell 数组存储航段信息 ----
    % cell(n_wp - 1, 1)：创建 (n_wp-1) 行 × 1 列的 cell 数组
    % cell 数组是 MATLAB 中一种特殊的容器类型，每个单元格（cell）
    % 可以存储任意类型和任意大小的数据，不同于普通矩阵的固定类型
    % 预分配空间可以避免在循环中动态扩展数组，提高内存效率
    traj.segments = cell(n_wp - 1, 1);

    % ---- 累计时间初始化 ----
    % t_cum 记录从航迹起点到当前航段起点经过的总时间（秒）
    % 初始为 0，表示第一个航段从时间 0 开始
    t_cum = 0.0;

    % ---- 循环处理每个航段 ----
    % for i = 1:(n_wp - 1)：遍历每对相邻航路点
    % i=1 表示第1个航段（waypoints(1) → waypoints(2)）
    % i=2 表示第2个航段（waypoints(2) → waypoints(3)），依此类推
    for i = 1:(n_wp - 1)
        % ---- 取航段起点经纬度 ----
        % traj.waypoints(i, 1)：第 i 行第 1 列，即第 i 个航路点的经度
        % traj.waypoints(i, 2)：第 i 行第 2 列，即第 i 个航路点的纬度
        lon0 = traj.waypoints(i, 1); lat0 = traj.waypoints(i, 2);

        % ---- 取航段终点经纬度 ----
        % (i+1) 表示下一个航路点，即第 i 个航段的终点
        lon1 = traj.waypoints(i+1, 1); lat1 = traj.waypoints(i+1, 2);

        % ---- 计算球面距离（Haversine 公式） ----
        % sphere_utils_haversine_distance 使用 Haversine 公式计算
        % 地球表面两点间的大圆距离，返回值为米（m）
        % 参数顺序：(起始经度, 起始纬度, 终点经度, 终点纬度)
        dist = sphere_utils_haversine_distance(lon0, lat0, lon1, lat1);

        % ---- 计算航段时长 ----
        % dur = 球面距离（米）/ 巡航速度（米/秒）
        % 结果单位为秒（s）
        dur = dist / speed_ms;

        % ---- 计算经纬度变化率（度/秒） ----
        % 经度变化率 = (终点经度 - 起点经度) / 航段时长
        % 单位为度每秒（deg/s），表示在该航段内经度每秒变化多少度
        lon_rate = (lon1 - lon0) / dur;

        % 纬度变化率 = (终点纬度 - 起点纬度) / 航段时长
        % 单位为度每秒（deg/s），表示在该航段内纬度每秒变化多少度
        lat_rate = (lat1 - lat0) / dur;

        % ---- 将航段信息存入 cell 数组 ----
        % struct('field1', val1, 'field2', val2, ...)：
        %   创建一个 MATLAB 结构体，每个字段存储一种信息
        % traj.segments{i}：使用花括号 {} 将结构体存入 cell 数组的第 i 格
        %
        % 结构体各字段说明：
        %   start    : [lon0, lat0] 航段起点经纬度（度）
        %   end      : [lon1, lat1] 航段终点经纬度（度）
        %   lon_rate : 经度变化率（度/秒）——在航段内为常数
        %   lat_rate : 纬度变化率（度/秒）——在航段内为常数
        %   dur      : 航段持续时间（秒）
        %   t_start  : 航段起始绝对时间（秒），等于前面所有航段时长之和
        traj.segments{i} = struct('start', [lon0, lat0], ...
                                 'end', [lon1, lat1], ...
                                 'lon_rate', lon_rate, ...
                                 'lat_rate', lat_rate, ...
                                 'dur', dur, ...
                                 't_start', t_cum);

        % ---- 更新累计时间 ----
        % 将当前航段的时长累加到 t_cum 中
        % 下一次循环时，t_cum 就是下一航段的起始时间
        t_cum = t_cum + dur;
    end

    % ================================================================
    % 第3步：记录航迹总体属性
    % ================================================================

    % ---- 航迹总时长 ----
    % 所有航段时长之和（即最后一个航段结束时的时间）
    % 单位：秒
    traj.duration_sec = t_cum;

    % ---- 航段总数 ----
    % length(traj.segments)：返回 cell 数组的元素个数
    % 等于 n_wp - 1（航路点数减一）
    traj.n_segments = length(traj.segments);

    % ================================================================
    % 第4步：生成均匀时间采样数组
    % ================================================================

    % ---- 构造时间数组 ----
    % 语法：start:step:end
    %   0：起始时间（航迹起点，t=0 秒）
    %   dt_sec：采样间隔（步长）
    %   traj.duration_sec：结束时间（航迹终点）
    % 结果：一个行向量，包含从 0 到 duration_sec 的等间隔时间点
    % 例如：dt_sec=30, duration_sec=300
    %   → time_array = [0, 30, 60, 90, ..., 300]
    traj.time_array = 0:dt_sec:traj.duration_sec;

    % ---- 总采样步数 ----
    % length(traj.time_array)：时间数组中元素的个数
    % 即最终输出的轨迹将包含 n_steps 个采样点
    traj.n_steps = length(traj.time_array);
    varargout{1} = traj;
end

% =========================================================================
% create_turn_trajectory - 拐弯航迹生成器（内部子函数）
% =========================================================================
% 通过3个航路点定义拐弯航迹，中间点形成约120°拐角。
% 航路点经覆盖校验确保全部位于双雷达威力范围内。
% =========================================================================
function [traj, waypoints] = create_turn_trajectory(params)
    % 拐弯航路点: 中间点形成约120°拐角
    % 全部位于双雷达威力范围内 (1000-2000km, 15°波束)
    waypoints = [126.0, 32.5, 0;   % 起点 (西南)
                 128.5, 33.5, 0;   % 拐点 (东北) — 入向~62°, 出向~182°
                 128.6, 31.7, 0];  % 终点 (南偏东) — 拐角约120°

    % 拐弯场景降低航速以保持帧数
    speed_ms = 140.0;

    traj = aircraft_trajectory_create(waypoints, speed_ms, params.dt_sec);
end

% =========================================================================
% create_uturn_trajectory - 回头弯（180°）航迹生成器
% =========================================================================
% 【功能】生成180°回头弯航迹（1°/s转弯率，左转）
%   航路点: W1(起点) 正东飞行 → 左转180°半圆 → 正西飞回W3
%   转弯模型: 左转半圆，ω=+1°/s, R=v/ω
%   航迹结构: ①W1→入弯点(直线东飞) ②入弯点→出弯点(180°左转半圆) ③出弯点→W3(直线西飞)
%   全部航路点位于双雷达威力范围交汇区内（~127°E,33°N区域）
%
% 【输入】params — 参数结构体，使用 .aircraft_speed_ms, .dt_sec
% 【输出】traj — 航迹结构体（与 aircraft_trajectory_interpolate 兼容）
%         waypoints — 3×2 航路点矩阵 [lon, lat]（不含高度）
% =========================================================================
function [traj, waypoints] = create_uturn_trajectory(params)
    speed_ms = params.aircraft_speed_ms;
    dt = params.dt_sec;
    omega_deg = 1.0;  % 1°/s 标准转弯率
    omega_rad = omega_deg * pi / 180.0;
    R_turn_m = speed_ms / omega_rad;  % 转弯半径 (m)
    turn_dur_sec = 180.0;  % 180°转弯 = 180秒
    arc_length_m = pi * R_turn_m;  % 半圆弧长

    % ---- 几何参数（180°左转回头弯） ----
    %  圆心固定，起终点经度固定，纬度由几何自动确定
    bearing_in = 90.0;    % 入向：正东
    turn_dir = +1;        % +1=左转(CCW)
    bearing_out = mod(bearing_in + 180.0 * turn_dir, 360);  % 出向：正西(270°)

    % 圆心（固定）
    center_lon = 131.44;
    center_lat = 31.75;

    % 左转圆心在入向左侧：entry→center = bearing_in - 90° = 0° (正北)
    center_bearing = bearing_in - 90.0 * turn_dir;  % = 0° (正北)

    % 入弯点：圆心正南R处 (center→entry = center_bearing + 180° = 180°)
    % 出弯点：圆心正北R处 (center→exit = center_bearing = 0°)
    [entry_lon, entry_lat] = haversine_forward(center_lon, center_lat, ...
        center_bearing + 180.0, R_turn_m);
    [exit_lon, exit_lat] = haversine_forward(center_lon, center_lat, ...
        center_bearing, R_turn_m);

    % 起终点经度（沿用户指定），纬度 = 入弯/出弯纬度（在各自直线上）
    W1_lon = 127.0284;  W1_lat = entry_lat;   % 起点：正东直飞到入弯点
    W3_lon = 127.2735;  W3_lat = exit_lat;    % 终点：正西从出弯点飞来

    % 直线段长度（从起点到入弯点，从出弯点到终点）
    straight_approach_m = sphere_utils_haversine_distance(W1_lon, W1_lat, entry_lon, entry_lat);
    straight_exit_m = sphere_utils_haversine_distance(exit_lon, exit_lat, W3_lon, W3_lat);

    % ---- 构建航段 ----
    segments = {};
    t_cum = 0;

    % 航段1：入弯直线
    dur1 = straight_approach_m / speed_ms;
    segments{1} = struct('start', [W1_lon, W1_lat], ...
        'end', [entry_lon, entry_lat], ...
        'lon_rate', (entry_lon - W1_lon) / dur1, ...
        'lat_rate', (entry_lat - W1_lat) / dur1, ...
        'dur', dur1, 't_start', 0);
    t_cum = t_cum + dur1;

    % 航段2：180°半圆（从圆心直接计算每个弧点，消除增量累积误差）
    %  入弯点在圆心 center_bearing+180° 方向（即正南），t=0
    %  t_arc秒后，飞机绕圆心转过 turn_dir*omega*t_arc 度
    %  弧点方位 = center_bearing + 180° - turn_dir*omega*t_arc
    arc_step = 1.0;
    n_arc_pts = floor(turn_dur_sec / arc_step);
    arc_pts = zeros(n_arc_pts, 2);
    for i = 1:n_arc_pts
        t_arc = i * arc_step;
        bearing_from_center = center_bearing + 180.0 - turn_dir * omega_deg * t_arc;
        if bearing_from_center >= 360, bearing_from_center = bearing_from_center - 360; end
        if bearing_from_center < 0, bearing_from_center = bearing_from_center + 360; end
        [arc_pts(i,1), arc_pts(i,2)] = haversine_forward(center_lon, center_lat, ...
            bearing_from_center, R_turn_m);
    end

    % 按 dt 秒一组打包弧段
    pts_per_seg = max(1, round(dt / arc_step));
    seg_start_lon = entry_lon; seg_start_lat = entry_lat;
    for i_start = 1:pts_per_seg:n_arc_pts
        i_end = min(i_start + pts_per_seg - 1, n_arc_pts);
        seg_end_lon = arc_pts(i_end, 1); seg_end_lat = arc_pts(i_end, 2);
        seg_dur = (i_end - i_start + 1) * arc_step;
        if seg_dur < 1e-6, continue; end
        segments{end+1} = struct('start', [seg_start_lon, seg_start_lat], ...
            'end', [seg_end_lon, seg_end_lat], ...
            'lon_rate', (seg_end_lon - seg_start_lon) / seg_dur, ...
            'lat_rate', (seg_end_lat - seg_start_lat) / seg_dur, ...
            'dur', seg_dur, 't_start', t_cum);
        t_cum = t_cum + seg_dur;
        seg_start_lon = seg_end_lon; seg_start_lat = seg_end_lat;
    end

    % 航段3：出弯直线
    dur3 = straight_exit_m / speed_ms;
    segments{end+1} = struct('start', [exit_lon, exit_lat], ...
        'end', [W3_lon, W3_lat], ...
        'lon_rate', (W3_lon - exit_lon) / dur3, ...
        'lat_rate', (W3_lat - exit_lat) / dur3, ...
        'dur', dur3, 't_start', t_cum);
    t_cum = t_cum + dur3;

    % ---- 构建航迹结构体 ----
    traj.speed = speed_ms;
    traj.dt_sec = dt;
    traj.segments = segments';
    traj.waypoints = [W1_lon, W1_lat; NaN, NaN; W3_lon, W3_lat];
    traj.duration_sec = t_cum;
    traj.n_segments = length(segments);
    traj.time_array = 0:dt:t_cum;
    traj.n_steps = length(traj.time_array);

    waypoints = [W1_lon, W1_lat, 0; NaN, NaN, 0; W3_lon, W3_lat, 0];

    % ---- 打印 ----
    fprintf('  回头弯航迹生成 (180度左转):\n');
    fprintf('    圆心:  (%.2fE, %.2fN) 固定\n', center_lon, center_lat);
    fprintf('    起点:  (%.2fE, %.2fN) 入弯点: (%.2fE, %.2fN)\n', ...
        W1_lon, W1_lat, entry_lon, entry_lat);
    fprintf('    终点:  (%.2fE, %.2fN) 出弯点: (%.2fE, %.2fN)\n', ...
        W3_lon, W3_lat, exit_lon, exit_lat);
    fprintf('    入向: %.0f度(正东) 出向: %.0f度(正西) 转弯率: %.1f度/s\n', ...
        bearing_in, bearing_out, omega_deg);
    fprintf('    转弯半径: %.1f km, 转弯时长: %.0f s, 弧长: %.1f km\n', ...
        R_turn_m/1000, turn_dur_sec, arc_length_m/1000);
    fprintf('    入弯直线: %.0f km, 出弯直线: %.0f km, 总航程: %.0f km, 总时长: %.0f s\n', ...
        straight_approach_m/1000, straight_exit_m/1000, ...
        (straight_approach_m + arc_length_m + straight_exit_m)/1000, t_cum);
end

% =========================================================================
% create_gradual_turn_trajectory - 渐进拐弯航迹生成器
% =========================================================================
% 【功能】生成民航客机式渐进拐弯航迹（1°/s转弯率）
%   航路点: W1(起点) → W2(拐弯顶点) → W3(终点)
%   转弯模型: 协调转弯，转弯率 ω = 1°/s，转弯半径 R = v/ω
%   航迹结构: ①W1→入弯点(直线) ②入弯点→出弯点(圆弧) ③出弯点→W3(直线)
%
% 【几何推导】
%   转弯角 θ = |bearing_out − bearing_in|
%   转弯半径 R = speed_ms / (turn_rate_deg_per_sec × π/180)
%   转弯提前量 d_anticipate = R × tan(θ/2)（沿入向从W2退后）
%   弧长 = R × θ_rad
%
% 【轨迹生成】
%   第1段（入弯直线）：W1到入弯点，恒定航向 = bearing_in
%   第2段（转弯圆弧）：以1°/s匀速改变航向，dt_sec采样
%   第3段（出弯直线）：出弯点到W3，恒定航向 = bearing_out
%
% 【输入】params — 参数结构体，使用 .aircraft_speed_ms, .dt_sec
% 【输出】traj — 航迹结构体（与 aircraft_trajectory_create 标准输出兼容）
%         waypoints — 3×2 航路点矩阵 [lon, lat]（不含高度）
% =========================================================================
function [traj, waypoints] = create_gradual_turn_trajectory(params)
    % ---- 航路点定义 ----
    % W1: 起点, W2: 拐弯顶点, W3: 终点
    W1 = [126.6685, 32.2184];  % 起点
    W2 = [128.2501, 31.0887];  % 拐弯顶点
    W3 = [132.0502, 31.4379];  % 终点
    waypoints = [W1(1), W1(2), 0;
                 W2(1), W2(2), 0;
                 W3(1), W3(2), 0];

    speed_ms = params.aircraft_speed_ms;
    dt = params.dt_sec;
    turn_rate_deg_per_sec = 1.0;  % 民航标准转弯率

    % ---- 第1步：计算入向和出向方位角 ----
    bearing_in  = sphere_utils_azimuth(W1(1), W1(2), W2(1), W2(2));
    bearing_out = sphere_utils_azimuth(W2(1), W2(2), W3(1), W3(2));

    % ---- 第2步：计算转弯角度（标准化到 [0, 180]） ----
    delta_heading = bearing_out - bearing_in;
    % 标准化到 [-180, 180]
    if delta_heading > 180
        delta_heading = delta_heading - 360;
    elseif delta_heading < -180
        delta_heading = delta_heading + 360;
    end
    turn_angle_deg = abs(delta_heading);
    turn_sign = sign(delta_heading);
    if turn_sign == 0
        turn_sign = 1;  % 退化情况：无转弯
    end

    % ---- 第3步：转弯几何参数 ----
    omega_rad_per_sec = turn_rate_deg_per_sec * pi / 180.0;  % 转弯率 (rad/s)
    R_turn_m = speed_ms / omega_rad_per_sec;  % 转弯半径 (m)
    turn_angle_rad = turn_angle_deg * pi / 180.0;
    d_anticipate_m = R_turn_m * tan(turn_angle_rad / 2.0);  % 转弯提前量 (m)
    turn_arc_length_m = R_turn_m * turn_angle_rad;  % 弧长 (m)
    turn_duration_sec = turn_angle_deg / turn_rate_deg_per_sec;  % 转弯持续时间 (s)

    % ---- 第4步：计算第1段（W1 → 入弯点）的距离和时长 ----
    dist_W1_W2_m = sphere_utils_haversine_distance(W1(1), W1(2), W2(1), W2(2));
    seg1_dist_m = dist_W1_W2_m - d_anticipate_m;
    if seg1_dist_m < 0
        warning('create_gradual_turn: 转弯提前量超过W1→W2距离，截断为0');
        seg1_dist_m = 0;
    end
    seg1_dur_sec = seg1_dist_m / speed_ms;

    % ---- 第5步：计算第3段（出弯点 → W3）的距离和时长 ----
    dist_W2_W3_m = sphere_utils_haversine_distance(W2(1), W2(2), W3(1), W3(2));
    seg3_dist_m = dist_W2_W3_m - d_anticipate_m;
    if seg3_dist_m < 0
        warning('create_gradual_turn: 转弯提前量超过W2→W3距离，截断为0');
        seg3_dist_m = 0;
    end
    seg3_dur_sec = seg3_dist_m / speed_ms;

    % ---- 第6步：生成入弯点坐标（从W1沿bearing_in走seg1_dist_m） ----
    [turn_start_lon, turn_start_lat] = haversine_forward(W1(1), W1(2), ...
        bearing_in, seg1_dist_m);

    % ---- 第7步：生成转弯弧段点迹（以dt_sec采样） ----
    % 从入弯点开始，每dt_sec改变航向 turn_rate_deg_per_sec × dt_sec 度
    % 在每个dt_sec微段内，航向从段首线性变化到段尾
    arc_points = {};
    current_lon = turn_start_lon;
    current_lat = turn_start_lat;
    current_bearing = bearing_in;
    t_in_arc = 0;
    arc_step = 1.0;  % 弧段内部采样步长 (s)，小于dt时细化

    while t_in_arc < turn_duration_sec - 1e-6
        step_sec = min(arc_step, turn_duration_sec - t_in_arc);
        % 该微段内的平均航向（首尾航向的中间值）
        heading_start = current_bearing;
        heading_end = bearing_in + turn_sign * turn_rate_deg_per_sec * (t_in_arc + step_sec);
        heading_mid = (heading_start + heading_end) / 2.0;
        if abs(heading_end - heading_start) > 180
            % 跨越360°边界
            if heading_start < heading_end
                heading_mid = heading_start + (heading_end - heading_start - 360) / 2.0;
            else
                heading_mid = heading_start + (heading_end - heading_start + 360) / 2.0;
            end
            if heading_mid < 0, heading_mid = heading_mid + 360; end
            if heading_mid >= 360, heading_mid = heading_mid - 360; end
        end
        % 沿平均航向移动 step_sec × speed_ms
        [next_lon, next_lat] = haversine_forward(current_lon, current_lat, ...
            heading_mid, speed_ms * step_sec);
        arc_points{end+1} = struct('lon', next_lon, 'lat', next_lat);
        current_lon = next_lon;
        current_lat = next_lat;
        current_bearing = heading_end;
        % 标准化 current_bearing
        if current_bearing >= 360, current_bearing = current_bearing - 360; end
        if current_bearing < 0, current_bearing = current_bearing + 360; end
        t_in_arc = t_in_arc + step_sec;
    end
    turn_end_lon = current_lon;
    turn_end_lat = current_lat;
    bearing_out_actual = current_bearing;

    % ---- 第8步：确认出弯点可直接飞向W3 ----
    % 出弯航向应已等于或接近 bearing_out
    % 出弯点到W3的方位用于第3段
    bearing_to_W3 = sphere_utils_azimuth(turn_end_lon, turn_end_lat, W3(1), W3(2));
    seg3_dist_actual_m = sphere_utils_haversine_distance(turn_end_lon, turn_end_lat, W3(1), W3(2));

    % ---- 第9步：构建航迹结构体（兼容 aircraft_trajectory_interpolate） ----
    traj.speed  = speed_ms;
    traj.dt_sec = dt;

    % 构建航段cell数组
    segments = {};

    % 航段1：直线入弯（W1 → 入弯点）
    if seg1_dur_sec > 1e-6
        dur1 = seg1_dur_sec;
        lon_rate1 = (turn_start_lon - W1(1)) / dur1;
        lat_rate1 = (turn_start_lat - W1(2)) / dur1;
        segments{end+1} = struct('start', [W1(1), W1(2)], ...
            'end', [turn_start_lon, turn_start_lat], ...
            'lon_rate', lon_rate1, 'lat_rate', lat_rate1, ...
            'dur', dur1, 't_start', 0);
    end

    % 航段2：转弯弧段（用弧段微段拼接）
    % 将弧段按dt_sec分组打包成大航段
    t_cum = seg1_dur_sec;  % 累计时间
    prev_pt_lon = turn_start_lon;
    prev_pt_lat = turn_start_lat;
    n_arc = length(arc_points);
    % 按dt_sec秒一组打包
    pts_per_seg = max(1, round(dt / arc_step));
    for i_start = 1:pts_per_seg:n_arc
        i_end = min(i_start + pts_per_seg - 1, n_arc);
        seg_start_lon = prev_pt_lon;
        seg_start_lat = prev_pt_lat;
        seg_end_lon = arc_points{i_end}.lon;
        seg_end_lat = arc_points{i_end}.lat;
        seg_dur = (i_end - i_start + 1) * arc_step;
        if seg_dur < 1e-6, continue; end
        lon_rate_s = (seg_end_lon - seg_start_lon) / seg_dur;
        lat_rate_s = (seg_end_lat - seg_start_lat) / seg_dur;
        segments{end+1} = struct('start', [seg_start_lon, seg_start_lat], ...
            'end', [seg_end_lon, seg_end_lat], ...
            'lon_rate', lon_rate_s, 'lat_rate', lat_rate_s, ...
            'dur', seg_dur, 't_start', t_cum);
        t_cum = t_cum + seg_dur;
        prev_pt_lon = seg_end_lon;
        prev_pt_lat = seg_end_lat;
    end

    % 航段3：直线出弯（出弯点 → W3）
    if seg3_dist_actual_m > 1e-6
        dur3 = seg3_dist_actual_m / speed_ms;
        lon_rate3 = (W3(1) - turn_end_lon) / dur3;
        lat_rate3 = (W3(2) - turn_end_lat) / dur3;
        segments{end+1} = struct('start', [turn_end_lon, turn_end_lat], ...
            'end', [W3(1), W3(2)], ...
            'lon_rate', lon_rate3, 'lat_rate', lat_rate3, ...
            'dur', dur3, 't_start', t_cum);
        t_cum = t_cum + dur3;
    end

    traj.segments = segments';
    traj.waypoints = waypoints(:, 1:2);
    traj.duration_sec = t_cum;
    traj.n_segments = length(segments);
    traj.time_array = 0:dt:t_cum;
    traj.n_steps = length(traj.time_array);

    % ---- 第10步：打印轨迹信息 ----
    fprintf('  渐进拐弯航迹生成:\n');
    fprintf('    入向方位: %.1f°, 出向方位: %.1f°\n', bearing_in, bearing_out);
    fprintf('    拐角: %.1f°, 转弯率: %.1f°/s\n', turn_angle_deg, turn_rate_deg_per_sec);
    fprintf('    转弯半径: %.1f km, 转弯时长: %.1f s\n', R_turn_m/1000, turn_duration_sec);
    fprintf('    总航程: %.1f km, 总时长: %.0f s\n', ...
        (seg1_dist_m + turn_arc_length_m + seg3_dist_actual_m)/1000, t_cum);
end

% =========================================================================
% haversine_forward — 球面正算（从起点沿方位角前进给定距离）
% =========================================================================
% 【输入】
%   lon0, lat0 — 起点经纬度（度）
%   bearing    — 方位角（度，真北为0，顺时针）
%   dist_m     — 前进距离（米）
% 【输出】
%   lon1, lat1 — 终点经纬度（度）
% 【公式】
%   lat1 = asin(sin(lat0)*cos(d/R) + cos(lat0)*sin(d/R)*cos(bearing))
%   lon1 = lon0 + atan2(sin(bearing)*sin(d/R)*cos(lat0), cos(d/R)-sin(lat0)*sin(lat1))
% =========================================================================
function [lon1, lat1] = haversine_forward(lon0, lat0, bearing, dist_m)
    R = 6371000.0;  % 地球平均半径 (m)
    lat0_rad = lat0 * pi / 180.0;
    lon0_rad = lon0 * pi / 180.0;
    bearing_rad = bearing * pi / 180.0;
    dR = dist_m / R;

    lat1_rad = asin(sin(lat0_rad) * cos(dR) + ...
        cos(lat0_rad) * sin(dR) * cos(bearing_rad));
    lon1_rad = lon0_rad + atan2(sin(bearing_rad) * sin(dR) * cos(lat0_rad), ...
        cos(dR) - sin(lat0_rad) * sin(lat1_rad));

    lat1 = lat1_rad * 180.0 / pi;
    lon1 = lon1_rad * 180.0 / pi;
end
% ========================================================================
% 文件结束
% ========================================================================
