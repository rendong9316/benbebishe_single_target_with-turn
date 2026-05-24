% ========================================================================
% aircraft_trajectory_interpolate_batch.m
% ========================================================================
%
% 【功能概述】
% 批量航迹插值函数。对给定的时间数组中的每个时间点，调用单点
% 插值函数 aircraft_trajectory_interpolate 逐一计算位置和速度，
% 将所有结果汇总为一个 N×5 的矩阵返回。
%
% 【数学原理】
% 本函数本身不涉及额外的数学计算，其数学基础完全依赖于
% aircraft_trajectory_interpolate 中的分段线性插值公式：
%   lon_i = start_lon + lon_rate * t_seg_i
%   lat_i = start_lat + lat_rate * t_seg_i
% 对时间数组 t_array 中的每个元素 t_i，分别应用上述公式。
%
% 【在项目流水线中的位置】
% 本函数是航迹插值的批量封装层，位于：
%   aircraft_trajectory_create → aircraft_trajectory_generate
%     → aircraft_trajectory_interpolate_batch（本函数）
%       → aircraft_trajectory_interpolate（单点插值）
%         → aircraft_trajectory_locate（时间定位）
%
% 它被 aircraft_trajectory_generate 调用，用于一次性生成完整的
% 航迹采样数据，供后续的量测仿真使用。
%
% 【输入参数】
%   traj    - 航迹结构体（由 aircraft_trajectory_create 创建）
%   t_array - 1×N 双精度行向量，包含 N 个时间采样点（单位：秒）
%             通常就是 traj.time_array（即 0:dt_sec:duration_sec）
%
% 【返回值】
%   out     - N×5 双精度矩阵，每行对应一个时间点的插值结果：
%             第1列: lon（经度，度）
%             第2列: lat（纬度，度）
%             第3列: lon_rate（经度变化率，度/秒）
%             第4列: lat_rate（纬度变化率，度/秒）
%             第5列: time（时间，秒）
%
% 【使用方法】
%   t_array = 0:30:300;  % 0到300秒，步长30秒
%   out = aircraft_trajectory_interpolate_batch(traj, t_array);
%   % out 的每行格式：[lon, lat, lon_rate, lat_rate, time]
%
% 【注意事项】
%   - 使用 for 循环逐个处理，而非向量化。这是因为每个时间点
%     可能落在不同的航段，需要分别调用 locate 查找
%   - 输出矩阵的第5列为输入时间本身，方便后续绘制时间序列图
%   - 如果 t_array 较大（如数千个点），可考虑改用 arrayfun 或
%     parfor 进行加速，但当前场景中采样点数通常不多（一般 < 1000）
% ========================================================================

function out = aircraft_trajectory_interpolate_batch(traj, t_array)
    % ----------------------------------------------------------------
    % aircraft_trajectory_interpolate_batch - 批量航迹插值
    % ----------------------------------------------------------------
    % 本函数是对 aircraft_trajectory_interpolate 的批量封装。
    % 对输入的时间数组中的每一个时间点，依次调用单点插值函数，
    % 然后将所有的位置和速度结果汇总到一个矩阵中。
    %
    % 使用循环而非向量化的原因：
    %   每个时间点可能位于不同的航段（segment），航段之间
    %   的 lon_rate 和 lat_rate 各不相同，无法用单一的矩阵
    %   运算同时处理所有时间点。因此逐个处理是合理的选择。
    % ----------------------------------------------------------------

    % ---- 获取时间数组的长度 ----
    % n：时间采样点的总数量
    % length(t_array)：返回数组中元素个数（对于向量即为其长度）
    n = length(t_array);

    % ---- 预分配输出矩阵 ----
    % zeros(n, 5)：创建一个 n 行 × 5 列的零矩阵
    % 预分配内存可以提高性能，避免在循环中动态扩展矩阵
    % 5 列分别对应：[lon, lat, lon_rate, lat_rate, time]
    out = zeros(n, 5);

    % ---- 循环遍历每个时间采样点 ----
    % for i = 1:n：从第 1 个采样点处理到第 n 个
    for i = 1:n
        % ---- 取出当前时间点 ----
        % t_array(i)：数组中第 i 个元素的值，即第 i 个采样时间
        t = t_array(i);

        % ---- 调用单点插值函数 ----
        % aircraft_trajectory_interpolate(traj, t) 返回：
        %   pos : 1×2 向量 [lon, lat]（插值经纬度）
        %   vel : 1×2 向量 [lon_rate, lat_rate]（经纬度变化率）
        [pos, vel] = aircraft_trajectory_interpolate(traj, t);

        % ---- 将插值结果填入输出矩阵的第 i 行 ----
        % out(i, 1) = pos(1)：第 i 行第 1 列 → 经度
        out(i, 1) = pos(1);

        % out(i, 2) = pos(2)：第 i 行第 2 列 → 纬度
        out(i, 2) = pos(2);

        % out(i, 3) = vel(1)：第 i 行第 3 列 → 经度变化率（度/秒）
        out(i, 3) = vel(1);

        % out(i, 4) = vel(2)：第 i 行第 4 列 → 纬度变化率（度/秒）
        out(i, 4) = vel(2);

        % out(i, 5) = t：第 i 行第 5 列 → 采样时间（秒）
        out(i, 5) = t;
    end
end
% ========================================================================
% 文件结束
% ========================================================================
