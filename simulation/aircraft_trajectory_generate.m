% ========================================================================
% aircraft_trajectory_generate.m
% ========================================================================
%
% 【功能概述】
% 完整航迹生成函数。本函数是一个薄封装层，调用
% aircraft_trajectory_interpolate_batch 对航迹中预定义的所有
% 采样时间点（traj.time_array）进行批量插值，生成完整的轨迹数据。
%
% 【数学原理】
% 本函数不涉及独立的数学计算，其结果等价于对航迹的所有采样点
% 依次应用分段线性插值。具体公式参见 aircraft_trajectory_interpolate。
%
% 【在项目流水线中的位置】
% 本函数位于航迹创建和量测仿真之间，是"数据生成"步骤的最后一环：
%
%   aircraft_trajectory_create  →  创建航迹结构体（定义航段）
%         ↓
%   aircraft_trajectory_generate →  生成完整轨迹采样 ★本函数★
%         ↓
%   measurement_simulator_measure → 对每个采样点模拟雷达量测
%
% 简而言之：create 定义了"飞机怎么飞"（航段规划），
%           generate 输出了"飞机每一秒在哪"（离散采样）。
%
% 【输入参数】
%   traj - 航迹结构体（由 aircraft_trajectory_create 创建），
%          必须包含以下字段：
%          .time_array : 预定义的采样时间数组 (1×N 行向量)
%          .segments   : cell 数组，每个元素是航段结构体
%          .n_segments : 航段数量
%          .duration_sec : 总时长（秒）
%
% 【返回值】
%   out  - N×5 双精度矩阵，每一行的格式为：
%          [lon, lat, lon_rate, lat_rate, time]
%          其中 N = traj.n_steps（= length(traj.time_array)）
%
% 【使用方法】
%   % 首先创建航迹
%   traj = aircraft_trajectory_create(waypoints, 250.0, 30.0);
%   % 然后生成完整轨迹
%   true_track = aircraft_trajectory_generate(traj);
%   % true_track 是 N×5 矩阵，包含所有采样点的 (lon,lat,v_lon,v_lat,t)
%
% 【注意事项】
%   - 本函数不执行任何额外计算，仅传递参数给 interpolate_batch
%   - 直接使用 traj.time_array 作为时间序列，保证采样点与
%     创建时指定的 dt_sec 步长一致
%   - 返回的矩阵可直接用于绘图（plot(true_track(:,1), true_track(:,2))）
%     或输入到量测仿真器
% ========================================================================

function out = aircraft_trajectory_generate(traj)
    % ----------------------------------------------------------------
    % aircraft_trajectory_generate - 生成完整的飞机轨迹采样数据
    % ----------------------------------------------------------------
    % 这是一个便捷的封装函数，它将 traj.time_array（航迹中预定义的
    % 均匀时间序列）传递给批量插值函数，从而生成一条完整的、
    % 按固定时间步长采样的轨迹。
    %
    % 输出矩阵 out 的每一行代表一个采样时刻的飞机状态：
    %   [经度, 纬度, 经度变化率, 纬度变化率, 时间]
    %
    % 这个矩阵是后续所有处理（量测仿真、滤波、航迹关联）的
    % 真实值（Ground Truth）输入。
    % ----------------------------------------------------------------

    % ---- 调用批量插值函数 ----
    % aircraft_trajectory_interpolate_batch(traj, traj.time_array)：
    %   对 traj.time_array 中的每一个时间点进行插值
    % traj.time_array：在 create 阶段生成的均匀时间序列
    %   例如 [0, 30, 60, 90, ..., 1800]（dt_sec=30, 总时长1800秒）
    % 返回 out：N×5 矩阵，格式为 [lon, lat, lon_rate, lat_rate, time]
    out = aircraft_trajectory_interpolate_batch(traj, traj.time_array);
end
% ========================================================================
% 文件结束
% ========================================================================
