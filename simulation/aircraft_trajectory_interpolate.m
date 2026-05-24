% ========================================================================
% aircraft_trajectory_interpolate.m
% ========================================================================
%
% 【功能概述】
% 单点航迹插值函数。给定航迹结构体和一个时间 t（秒），利用线性
% 插值计算出飞机在该时刻的经纬度坐标和速度。
%
% 【数学原理】
% 本函数采用分段线性插值（Piecewise Linear Interpolation）：
%
% 假设时间 t 落在第 i 个航段内，该航段有：
%   start = [lon0, lat0]  : 航段起始点经纬度（度）
%   lon_rate              : 经度变化率（度/秒）
%   lat_rate              : 纬度变化率（度/秒）
%   t_seg                 : 从航段起点开始的时间偏移量（秒）
%
% 则插值公式为：
%   lon = lon0 + lon_rate * t_seg      ... (1) 经度线性插值
%   lat = lat0 + lat_rate * t_seg      ... (2) 纬度线性插值
%
% 速度和坐标使用相同的插值公式，速度在航段内为常数：
%   lon_rate = (lon1 - lon0) / dur     ... (3) 预计算的经度变化率
%   lat_rate = (lat1 - lat0) / dur     ... (4) 预计算的纬度变化率
%
% 这种插值方式实际上模拟了恒向线（Rhumb Line）上的匀速运动。
% 在短距离（< 500 km）内，恒向线与大圆航线的差异很小，
% 对于雷达仿真而言精度完全足够。
%
% 【在项目流水线中的位置】
% 本函数位于航迹生成流水线的中间层：
%   aircraft_trajectory_create（创建航迹结构）
%     → aircraft_trajectory_locate（定位时间所在航段）
%       → aircraft_trajectory_interpolate（单点插值）
%         → aircraft_trajectory_interpolate_batch（批量插值）
%           → 量测仿真器使用插值结果
%
% 【输入参数】
%   traj - 航迹结构体（由 aircraft_trajectory_create 创建）
%   t    - 标量双精度浮点数，插值时间（单位：秒）
%
% 【返回值】
%   pos     - 1x2 双精度行向量 [lon, lat]，插值得到的经纬度（度）
%   vel_deg - 1x2 双精度行向量 [lon_rate, lat_rate]，
%             经纬度变化率（单位：度/秒）
%
% 【使用方法】
%   [pos, vel] = aircraft_trajectory_interpolate(traj, 150.0);
%   % pos = [112.5, 33.2], vel = [0.001, 0.0008]
%
% 【注意事项】
%   - t_seg 会被限制在 [0, seg.dur] 范围内，防止因浮点误差
%     导致插值结果超出航段端点
%   - 该函数假设航段内为匀速直线运动，不模拟加速/减速过程
%   - 经纬度变化率直接使用球面度/秒，未做 cos(lat) 修正
% ========================================================================

function [pos, vel_deg] = aircraft_trajectory_interpolate(traj, t)
    % ----------------------------------------------------------------
    % aircraft_trajectory_interpolate - 在给定时刻对航迹进行线性插值
    % ----------------------------------------------------------------
    % 工作流程：
    %   1. 调用 aircraft_trajectory_locate 确定时间 t 所在的航段
    %   2. 取出该航段的结构体数据
    %   3. 使用线性插值公式计算经度和纬度
    %   4. 返回位置和速度向量
    %
    % 插值公式（航段内匀速运动）：
    %   lon = start_lon + lon_rate * t_seg
    %   lat = start_lat + lat_rate * t_seg
    % ----------------------------------------------------------------

    % ---- 第1步：定位时间 t 所在的航段 ----
    % 调用 locate 函数获取：
    %   idx   : 航段索引（在 traj.segments 中的位置）
    %   t_seg : 在航段内的时间偏移量（秒）
    [idx, t_seg] = aircraft_trajectory_locate(traj, t);

    % ---- 第2步：获取该航段的结构体数据 ----
    % 从 cell 数组 traj.segments 中取出第 idx 个元素
    % seg 结构体包含字段：start, end, lon_rate, lat_rate, dur, t_start
    seg = traj.segments{idx};

    % ---- 第3步：时间偏移量安全钳制 ----
    % 将 t_seg 限制在 [0, seg.dur] 范围内
    % 防止因浮点舍入误差导致 t_seg 略微超出该航段时长
    % 例如 seg.dur=100.0，但 t_seg 可能因误差为 100.0000001
    t_seg = min(t_seg, seg.dur);

    % ---- 第4步：线性插值计算经度 ----
    % seg.start(1)：该航段起始点的经度（第1个元素，索引1表示经度）
    % seg.lon_rate：经度变化率（度/秒），由创建时预计算
    % lon = 起始经度 + 经度变化率 * 时间偏移量
    lon = seg.start(1) + seg.lon_rate * t_seg;

    % ---- 第5步：线性插值计算纬度 ----
    % seg.start(2)：该航段起始点的纬度（第2个元素，索引2表示纬度）
    % seg.lat_rate：纬度变化率（度/秒），由创建时预计算
    % lat = 起始纬度 + 纬度变化率 * 时间偏移量
    lat = seg.start(2) + seg.lat_rate * t_seg;

    % ---- 第6步：组装返回值 ----
    % pos: 1x2 行向量 [经度, 纬度]，单位为度
    pos = [lon, lat];

    % vel_deg: 1x2 行向量 [经度变化率, 纬度变化率]，单位为度/秒
    % 在航段内速度恒定，直接取 seg 中预计算的值即可
    vel_deg = [seg.lon_rate, seg.lat_rate];
end
% ========================================================================
% 文件结束
% ========================================================================
