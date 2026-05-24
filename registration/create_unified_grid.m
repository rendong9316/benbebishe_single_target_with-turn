% ============================================================================
% create_unified_grid.m
% 创建统一时间网格函数
% ============================================================================
%
% 【功能概述】
%   根据指定的起始时间偏移、采样间隔和步数，生成一个均匀等间隔的
%   统一时间网格数组。该网格将作为两部异步采样的雷达航迹进行时间对齐
%   时的"参考时间坐标系"——两部雷达各自通过球面大圆插值，将自身航迹
%   推算到这个统一时间网格的每个时间点上。
%
% 【在空间配准流程中的角色】
%   空间配准流程中，两部雷达是异步采样的（采样时刻不完全相同），
%   在进行航迹融合之前，必须将两部雷达的量测统一到相同的时间基准上。
%   本函数生成的就是这个"共同的时间基准"。
%
%   典型使用方式：
%     unified_grid = create_unified_grid(0, 1.0, 100);
%     aligned_r1 = align_radar_to_grid(r1_corrected, unified_grid, ref_time);
%     aligned_r2 = align_radar_to_grid(r2_corrected, unified_grid, ref_time);
%
%   这样 aligned_r1 和 aligned_r2 在时间上完全对齐，可以逐点比较/融合。
%
% 【数学原理 —— 时间网格的生成】
%   时间网格是从 offset_start 开始、以 dt_sec 为步长的等差数列：
%     t_k = offset_start + k * dt_sec，其中 k = 0, 1, 2, ..., n_steps-1
%
%   例如：
%     offset_start = 10.0（秒），dt_sec = 0.5（秒），n_steps = 5
%     生成 grid = [10.0, 10.5, 11.0, 11.5, 12.0]
%
%   这是一个纯数学函数，不涉及雷达物理模型。
%
% 【输入参数】
%   offset_start  - 统一时间网格的起始时间偏移（秒）
%                   通常设为 0，表示从参考时间点开始
%                   或者从第一部雷达的第一帧时间开始
%   dt_sec        - 采样时间间隔（秒），即相邻两个网格点的时间差
%                   值越小，时间分辨率越高，插值越精细，但计算量也越大
%                   典型值：0.1 ~ 1.0 秒，取决于雷达原始采样率和需求精度
%   n_steps       - 时间网格的总点数（步数）
%                   最终网格长度为 n_steps，时间跨度为 (n_steps - 1) * dt_sec
%
% 【返回值】
%   grid          - 统一时间网格数组，类型为 double 的行向量
%                   长度为 n_steps，元素值从 offset_start 开始
%                   步长为 dt_sec 的等差数列
%                   格式示例：[0, 1, 2, 3, 4]（当 offset_start=0, dt_sec=1, n_steps=5）
%
% 【MATLAB 语法要点】
%   (0:n_steps-1) * dt_sec + offset_start
%   这是 MATLAB 的向量化表达式，分步解析：
%   1. 0:n_steps-1：生成行向量 [0, 1, 2, ..., n_steps-1]（共 n_steps 个元素）
%   2. * dt_sec：标量乘法，向量每个元素乘以 dt_sec
%   3. + offset_start：标量加法，向量每个元素加上 offset_start
%   最终得到等差数列 [offset_start, offset_start+dt_sec, ..., offset_start+(n_steps-1)*dt_sec]
%
% ============================================================================

function grid = create_unified_grid(offset_start, dt_sec, n_steps)
    % 生成统一时间网格数组
    %
    % 输入:
    %   offset_start: 起始时间偏移（秒），即第一个网格点的时间值
    %   dt_sec:       时间步长（秒），相邻网格点的间隔
    %   n_steps:      网格点数，生成的数组长度
    %
    % 返回:
    %   grid: 均匀时间网格，从 offset_start 开始，步长 dt_sec，共 n_steps 个点

    %% ---- 向量化生成等差数列 ----
    % 使用 MATLAB 的向量化语法一步生成时间网格
    % 不使用 for 循环，而是直接利用冒号表达式和广播运算
    %
    % 生成过程：
    % (0:n_steps-1)     → [0, 1, 2, ..., n_steps-1]         （索引序列）
    % * dt_sec          → [0, dt_sec, 2*dt_sec, ...]         （时间偏移序列）
    % + offset_start    → [t0, t0+dt_sec, t0+2*dt_sec, ...]  （最终时间网格）
    %
    % 这种向量化写法的效率远高于 for 循环（MATLAB 内部使用 SIMD 优化）
    grid = (0:n_steps-1) * dt_sec + offset_start;

end  % 函数 create_unified_grid 结束
