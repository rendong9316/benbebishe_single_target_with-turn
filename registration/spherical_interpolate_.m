% ============================================================================
% spherical_interpolate_.m
% 球面大圆插值函数（带下划线后缀以避免命名冲突）
% ============================================================================
%
% 【功能概述】
%   本函数在给定目标时刻 T 处，利用雷达航迹中 T 前后最近的两个有效采样点，
%   沿球面大圆按时间比例进行插值，推算出目标在该时刻的经纬度坐标。
%   支持三种模式：内插（T 在两个有效点之间）、前向外推（T 在所有点之前）、
%   后向外推（T 在所有点之后）。
%
% 【数学原理 —— 球面大圆插值 / SLERP】
%   球面大圆插值（Spherical Linear Interpolation, SLERP）是在单位球面上
%   两点之间按弧长比例求中间点的算法。设球面上两点对应的单位向量为
%   u₀ 和 u₁，插值比例 ratio ∈ [0, 1]，则中间点 u_T 为：
%
%     Ω = arccos(u₀ · u₁)                         ← 两点间的球面夹角
%     u_T = [sin((1 - ratio) * Ω) / sin(Ω)] * u₀
%         + [sin(ratio * Ω) / sin(Ω)] * u₁         ← SLERP 公式
%
%   然后从单位向量 u_T 反算出经纬度 (lon_T, lat_T)。
%   当 ratio = 0 时，u_T = u₀（回到起点）；
%   当 ratio = 1 时，u_T = u₁（到达终点）。
%
%   时间比例 ratio 的计算：
%     ratio = (T - t₀) / (t₁ - t₀)
%   其中 t₀ 和 t₁ 分别是 T 前后两个航迹点的时间戳。
%   这等价于假设目标在大圆上匀速运动。
%
% 【输入参数】
%   T          - 目标插值时刻（秒），类型为 double 标量
%                通常是统一时间网格中的某个时间点
%   t_valid    - 有效航迹点的时间数组，类型为 double 行向量
%                必须是升序排列的（按时间先后顺序）
%                例如：[10.2, 10.7, 11.3, 11.9, ...]
%   valid_meas - 有效航迹点的量测数据，类型为 cell array of struct
%                与 t_valid 一一对应，每个 struct 包含字段：
%                  lon: 经度（度）
%                  lat: 纬度（度）
%   ref_time   - （可选）参考起始时间，类型为 datetime
%                用于生成可读时间字符串；为空时不生成 time_str
%
% 【返回值】
%   result     - 插值结果结构体 struct，包含字段：
%                  time_sec: T（秒），即插值时刻
%                  time_str: 格式化时间字符串（仅当 ref_time 非空时）
%                  lat:      插值得到的纬度（度）
%                  lon:      插值得到的经度（度）
%                  aligned:  逻辑值 true，表示该点由插值得到
%
% 【调用关系】
%   本函数调用：sphere_utils_interpolate_great_circle（球面大圆插值核心算法）
%              sphere_utils_seconds_to_datetime_str（时间戳转字符串）
%   本函数被调用：align_radar_to_grid.m（雷达航迹时间对齐主函数）
%
% 【特殊处理】
%   1. T 在所有采样点之前：用第一和第二点做外推（前向线性外推，ratio < 0）
%   2. T 在所有采样点之后：用倒数第二和最后一点做外推（后向线性外推，ratio > 1）
%   3. T 在两点之间：正常内插（0 <= ratio <= 1）
%   虽然外推的精度不如内插可靠，但在航迹边缘能提供连续的时间覆盖。
%
% ============================================================================

function result = spherical_interpolate_(T, t_valid, valid_meas, ref_time)
    % 在时刻 T 处做球面大圆插值
    % 在有效航迹点数组中搜索 T 前后的最近两点，沿球面大圆按时间比例插值。
    %
    % 输入:
    %   T:          目标插值时刻（秒）
    %   t_valid:    有效航迹点的时间数组（升序）
    %   valid_meas: 有效航迹点的量测数据（cell array of struct）
    %   ref_time:   参考时间（datetime），可选
    %
    % 返回:
    %   result: 插值结果结构体，包含 time_sec / time_str / lat / lon / aligned

    %% ---- 确定有效航迹点数量 ----
    n_valid = length(t_valid);   % 有效采样点的总数

    %% ---- 二分查找 T 在时间数组中的位置 ----
    % 线性搜索：从 1 开始递增 idx，直到找到第一个 t_valid(idx) >= T 的位置
    % 如果 T 大于所有有效点，idx 会超出 n_valid（即 idx = n_valid + 1）
    %
    % 算法效率：O(n)，其中 n = n_valid
    % 由于每个雷达的航迹点数量通常不大（数十到数百帧），线性搜索足够快
    idx = 1;                                           % 从第一个元素开始查找
    while idx <= n_valid && t_valid(idx) < T           % 只要当前时间小于 T，继续向前
        idx = idx + 1;                                 % 索引递增
    end
    % 循环结束时，idx 指向第一个满足 t_valid(idx) >= T 的位置（1-based 索引）

    %% ---- 根据 idx 位置决定插值/外推策略 ----
    % 策略分为三种情况：
    %   idx == 1:        T 在所有采样点之前 → 外推（用最近两个点：第1和第2点）
    %   idx > n_valid:   T 在所有采样点之后 → 外推（用最近两个点：倒数第1和第2点）
    %   1 < idx <= n_valid: T 在两个采样点之间 → 内插（用 idx-1 和 idx 点）

    if idx == 1
        %% 情况1：T 在所有采样点之前，用第1和第2个点做前向外推
        % 外推公式与内插相同，只是 ratio < 0（T 在第1个点之前）
        m0 = valid_meas{1};       m1 = valid_meas{2};        % 最近的两个采样点
        t0 = t_valid(1);          t1 = t_valid(2);           % 对应的时间戳
        ratio = (T - t0) / (t1 - t0);                        % 时间比例（此时 ratio < 0，表示外推）

    elseif idx > n_valid
        %% 情况2：T 在所有采样点之后，用倒数第1和第2个点做后向外推
        % 外推公式与内插相同，只是 ratio > 1（T 在最后一个点之后）
        m0 = valid_meas{n_valid - 1}; m1 = valid_meas{n_valid};  % 最后两个采样点
        t0 = t_valid(n_valid - 1);    t1 = t_valid(n_valid);     % 对应的时间戳
        ratio = (T - t0) / (t1 - t0);                             % 时间比例（此时 ratio > 1，表示外推）

    else
        %% 情况3：T 在两个有效采样点之间，正常内插
        % idx-1 是 T 之前最近的点，idx 是 T 之后最近的点
        % 0 <= ratio <= 1，是标准的球面大圆内插
        m0 = valid_meas{idx - 1};   m1 = valid_meas{idx};            % T 前后的两个采样点
        t0 = t_valid(idx - 1);      t1 = t_valid(idx);              % 对应的时间戳
        ratio = (T - t0) / (t1 - t0);                                % 时间比例（0 <= ratio <= 1）
    end

    %% ---- 球面大圆插值核心计算 ----
    % 调用底层工具函数 sphere_utils_interpolate_great_circle 进行 SLERP 计算
    % 输入：起点经纬度 (m0.lon, m0.lat)、终点经纬度 (m1.lon, m1.lat)、
    %       插值比例 ratio
    % 输出：插值得到的经纬度 (lon, lat)，单位：度
    [lon, lat] = sphere_utils_interpolate_great_circle( ...
        m0.lon, m0.lat, m1.lon, m1.lat, ratio);

    %% ---- 生成可读的时间字符串 ----
    % 如果调用方提供了参考时间 ref_time，则根据 T 生成 "YYYY-MM-DD HH:MM:SS" 格式的字符串
    % 如果没有提供，则生成简单的 "%6.1fs" 格式字符串（如 " 12.5s"）
    if ~isempty(ref_time)
        % sphere_utils_seconds_to_datetime_str 将秒偏移加在 ref_time 上，
        % 转为 datetime 后再格式化为字符串
        time_str = sphere_utils_seconds_to_datetime_str(T, ref_time);
    else
        % sprintf('%.1fs', T) 将数值格式化为带1位小数的秒数字符串
        % 例如：T = 12.5 → "12.5s"
        time_str = sprintf('%.1fs', T);
    end

    %% ---- 打包返回结果 ----
    % struct() 创建结构体，字段名用单引号括起来，多个字段用逗号分隔
    % aligned 字段设为 true，表示该航迹点由插值得到（非原始量测点）
    result = struct('time_sec', T, 'time_str', time_str, ...
                    'lat', lat, 'lon', lon, 'aligned', true);

end  % 函数 spherical_interpolate_ 结束
