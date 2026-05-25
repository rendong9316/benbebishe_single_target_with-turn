% ============================================================================
% align_radar_to_grid.m
% 雷达航迹时间对齐主函数（球面大圆插值版本）
% ============================================================================
%
% 【功能概述】
%   本函数将一部异步采样的雷达航迹统一对齐到指定的均匀时间网格上。
%   对每个网格时间点 T，在雷达原有采样点之间沿球面大圆做线性插值，
%   推算出雷达在 T 时刻"本该看到"的目标经纬度坐标。
%   不使用 ENU 切平面近似，全部使用球面公式，适合大范围（数百公里）场景。
%
% 【数学原理 —— 球面大圆插值】
%   球面大圆插值（Great Circle Interpolation）是在球面上两点之间按比例
%   求中间点的方法，等价于球面线性插值（SLERP，Spherical Linear Interpolation）：
%     设两点 A(lon0, lat0) 和 B(lon1, lat1)，插值比例 ratio ∈ [0, 1]，
%     则中间点 P 位于通过 A、B 的大圆（great circle）上，且弧长 AP : PB = ratio : (1-ratio)。
%
%   时间对齐的做法：
%     设雷达在时刻 t_k 和 t_{k+1} 各有一帧航迹点 (单位向量 u_k, u_{k+1})，
%     要推算网格时刻 T（t_k < T < t_{k+1}）处的目标位置，比例系数为：
%       ratio = (T - t_k) / (t_{k+1} - t_k)  ← 时间比例
%     然后用 SLERP 在 u_k 和 u_{k+1} 之间按 ratio 插值，得到 u_T，
%     再将单位向量 u_T 还原为经纬度 (lon_T, lat_T)。
%
%   相对于 ENU 切平面插值的优势：
%     1. 球面大圆是两点之间在球面上的最短路径，ENU 切平面近似会在远距离引入畸变
%     2. 不依赖雷达站本地切平面，适用于两个站点间距较大的多基地雷达配准
%     3. 避免 ENU 投影的保角/保距误差
%
% 【输入参数】
%   meas_list    - 单部雷达的航迹列表，类型为 cell array of struct
%                  每个 cell 元素是一帧航迹点，struct 包含字段：
%                    time_sec: 该帧的时间戳（秒，相对于某个参考起点）
%                    lat:      目标纬度（度）
%                    lon:      目标经度（度）
%                  漏检帧（无目标检测）则为空数组 []，将被自动跳过
%   unified_time - 统一时间网格数组，类型为 double 行向量
%                  包含一系列等间隔的时间点（秒），由 create_unified_grid.m 生成
%                  例如：[0, 1, 2, 3, ..., 99]（步长1秒，共100个点）
%   ref_time     - （可选）参考起始时间，类型为 datetime
%                  用于生成可读的时间字符串 time_str
%                  如果为空或未提供，则不生成 time_str 字段
%
% 【返回值】
%   aligned      - 对齐后的航迹列表，类型为 cell array of struct
%                  长度与 unified_time 相同
%                  每个 cell 元素为一个 struct，包含字段：
%                    time_sec: 当前网格时间点
%                    time_str: 时间字符串（仅当 ref_time 非空时存在）
%                    lat:      插值得到的纬度（度）
%                    lon:      插值得到的经度（度）
%                    aligned:  逻辑值，true 表示该点为插值结果
%                  若该网格时间点超出雷达航迹的有效时间范围，则填入空数组 []
%
% 【调用关系】
%   本函数调用：spherical_interpolate_（球面大圆插值子函数）
%   本函数被调用：main.m 或空间配准流程脚本，作为时间对齐步骤
%   通常用法：
%     aligned_r1 = align_radar_to_grid(r1_corrected, unified_grid, ref_time);
%     aligned_r2 = align_radar_to_grid(r2_corrected, unified_grid, ref_time);
%   调用后 aligned_r1 和 aligned_r2 在时间上完全对齐，可逐点比较/融合。
%
% 【注意事项】
%   1. 本函数不对统一时间网格范围外的点做外推（外推精度不可靠），
%      超出雷达有效航迹范围的时间点直接返回空数组。
%   2. 至少需要2个有效航迹点才能进行插值（interpolation），
%      不足2个点时，全部网格点返回空数组。
%   3. 漏检帧（空数组）和经纬度为 NaN 的帧都会被跳过。
%
% ============================================================================

function aligned = align_radar_to_grid(meas_list, unified_time, ref_time)
    % 将一部雷达的航迹对齐到统一时间网格（球面大圆插值）
    % 对统一时间网格的每个时刻 T，在雷达原有采样点之间做球面大圆插值，
    % 推算出雷达在 T 时刻"本该看到"的目标经纬度。
    %
    % 输入:
    %   meas_list: 单部雷达航迹列表（cell array of struct），漏检为 []
    %   unified_time: 统一时间网格（秒），double 行向量
    %   ref_time: 参考时间（datetime），可选参数
    %
    % 返回:
    %   aligned: 对齐后的航迹 cell 数组，长度 = length(unified_time)
    %            超出有效范围的点填 []，有效点填插值结果 struct

    if nargin < 3, ref_time = []; end

    %% ---- 过滤有效航迹点 ----
    % 遍历所有航迹帧，剔除漏检帧和经纬度无效帧，
    % 收集有效的航迹点和对应的时间戳
    %
    % valid:  cell 数组，按顺序存放每个有效帧的 struct（含经纬度、时间等字段）
    % t_valid: double 数组，按顺序存放每个有效帧的时间戳（秒）
    valid = {};          % 用 {} 初始化为空 cell 数组
    t_valid = [];        % 用 [] 初始化为空矩阵（即 0x0 的空 double 数组）
    for i = 1:length(meas_list)
        m = meas_list{i};                                 % 用花括号 {} 读取 cell 数组第 i 个元素
        if isempty(m), continue; end                      % 跳过漏检帧（无目标检测）
        if ~isfield(m, 'lat') || isnan(m.lat), continue; end  % 跳过经纬度无效帧
        valid{end+1} = m;                                 % cell 数组用 {end+1} 在末尾追加元素
        t_valid(end+1) = m.time_sec;                      % 普通数组用 (end+1) 在末尾追加元素
    end

    %% ---- 安全性检查：有效航迹点不足2个则无法插值 ----
    % 球面大圆插值至少需要2个端点来确定大圆路径，
    % 如果只有0或1个有效点，全部网格点返回空数组
    if length(valid) < 2
        aligned = cell(1, length(unified_time));          % 创建 1×N 空 cell 数组，每个元素为 []
        return;                                           % 提前返回
    end

    %% ---- 确定有效航迹的时间范围 ----
    % t_min: 第一个有效帧的时间戳（秒），即雷达航迹的起始时间
    % t_max: 最后一个有效帧的时间戳（秒），即雷达航迹的结束时间
    % 网格时间点落在 [t_min, t_max] 之外时不进行插值（避免不可靠的外推）
    t_min = t_valid(1);       % 取第一个有效时间
    t_max = t_valid(end);     % 取最后一个有效时间

    %% ---- 逐点插值：遍历统一时间网格的每个时刻 ----
    % 为统一时间网格中的每个时间点创建插值结果
    % aligned{k} = []         表示时间点 k 超出雷达覆盖范围，无有效数据
    % aligned{k} = result    表示时间点 k 在雷达覆盖范围内，插值成功
    aligned = cell(1, length(unified_time));   % 预分配输出 cell 数组
    for k = 1:length(unified_time)
        T = unified_time(k);                    % 当前要插值的目标时间点
        if T < t_min || T > t_max               % 如果 T 在有效航迹范围之外
            aligned{k} = [];                    % 不进行外推，返回空数组
        else                                    % T 在有效航迹范围内
            % 调用球面大圆插值子函数，在 T 时刻进行插值
            % spherical_interpolate_ 会在 t_valid 中找到 T 前后的最近两点，
            % 然后沿大圆按时间比例插值出 T 时刻的经纬度
            result = spherical_interpolate_(T, t_valid, valid, ref_time);
            aligned{k} = result;                % 存放插值结果 struct
        end
    end

end  % 函数 align_radar_to_grid 结束


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
%   u0 和 u1，插值比例 ratio ∈ [0, 1]，则中间点 u_T 为：
%
%     Omega = arccos(u0 . u1)                          ← 两点间的球面夹角
%     u_T = [sin((1 - ratio) * Omega) / sin(Omega)] * u0
%         + [sin(ratio * Omega) / sin(Omega)] * u1      ← SLERP 公式
%
%   然后从单位向量 u_T 反算出经纬度 (lon_T, lat_T)。
%   当 ratio = 0 时，u_T = u0（回到起点）；
%   当 ratio = 1 时，u_T = u1（到达终点）。
%
%   时间比例 ratio 的计算：
%     ratio = (T - t0) / (t1 - t0)
%   其中 t0 和 t1 分别是 T 前后两个航迹点的时间戳。
%   这等价于假设目标在大圆上匀速运动。
%
% 【输入参数】
%   T          - 目标插值时刻（秒），类型为 double 标量
%   t_valid    - 有效航迹点的时间数组，类型为 double 行向量，升序
%   valid_meas - 有效航迹点的量测数据，类型为 cell array of struct
%   ref_time   - （可选）参考起始时间，类型为 datetime
%
% 【返回值】
%   result     - 插值结果结构体 struct，包含字段：
%                  time_sec: T（秒）
%                  time_str: 格式化时间字符串
%                  lat:      插值得到的纬度（度）
%                  lon:      插值得到的经度（度）
%                  aligned:  逻辑值 true
% ============================================================================

function result = spherical_interpolate_(T, t_valid, valid_meas, ref_time)
    %% ---- 确定有效航迹点数量 ----
    n_valid = length(t_valid);   % 有效采样点的总数

    %% ---- 二分查找 T 在时间数组中的位置 ----
    idx = 1;
    while idx <= n_valid && t_valid(idx) < T
        idx = idx + 1;
    end

    %% ---- 根据 idx 位置决定插值/外推策略 ----
    if idx == 1
        %% 情况1：T 在所有采样点之前，用第1和第2个点做前向外推
        m0 = valid_meas{1};       m1 = valid_meas{2};
        t0 = t_valid(1);          t1 = t_valid(2);
        ratio = (T - t0) / (t1 - t0);

    elseif idx > n_valid
        %% 情况2：T 在所有采样点之后，用倒数第1和第2个点做后向外推
        m0 = valid_meas{n_valid - 1}; m1 = valid_meas{n_valid};
        t0 = t_valid(n_valid - 1);    t1 = t_valid(n_valid);
        ratio = (T - t0) / (t1 - t0);

    else
        %% 情况3：T 在两个有效采样点之间，正常内插
        m0 = valid_meas{idx - 1};   m1 = valid_meas{idx};
        t0 = t_valid(idx - 1);      t1 = t_valid(idx);
        ratio = (T - t0) / (t1 - t0);
    end

    %% ---- 球面大圆插值核心计算 ----
    [lon, lat] = sphere_utils_interpolate_great_circle( ...
        m0.lon, m0.lat, m1.lon, m1.lat, ratio);

    %% ---- 生成可读的时间字符串 ----
    if ~isempty(ref_time)
        time_str = sphere_utils_seconds_to_datetime_str(T, ref_time);
    else
        time_str = sprintf('%.1fs', T);
    end

    %% ---- 打包返回结果 ----
    result = struct('time_sec', T, 'time_str', time_str, ...
                    'lat', lat, 'lon', lon, 'aligned', true);

end  % 函数 spherical_interpolate_ 结束
