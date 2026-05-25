% ============================================================================
% load_adsb.m
% ADS-B 数据加载模块 —— 解析 CSV 文件，提取航迹并重采样到仿真时间网格
% ============================================================================
%
% 【功能概述】
%   从 ADS-B（Automatic Dependent Surveillance-Broadcast，广播式自动相关监视）
%   的 CSV 数据文件中加载指定飞机的飞行航迹，经过时间过滤、去重、排序后，
%   通过线性插值重采样到与仿真一致的均匀时间网格上，并计算经纬度变化率
%   （用于后续速度计算和状态估计）。
%
%   ADS-B 是民航飞机定期广播的位置/速度/高度等飞行数据，频率通常为 0.5~2 Hz。
%   在本仿真系统中，ADS-B 数据被用作目标的"真实航迹"（ground truth），
%   用于生成雷达模拟量测和评估跟踪性能。
%
% 【数学原理 —— 线性插值重采样】
%   ADS-B 原始数据的时间戳是不均匀的（取决于飞机的实际广播时刻），
%   而仿真需要均匀时间步长 dt_sec 的航迹数据。重采样过程如下：
%
%     1. 提取时间窗口内的有效原始数据 (t_i, lat_i, lon_i)
%        → 数量：numOriginal
%     2. 生成均匀的时间网格 t_grid = [0, dt_sec, 2*dt_sec, ..., duration_sec]
%        → 数量：numResampled = floor(duration_sec / dt_sec) + 1
%     3. 在每个网格点 t_k 上做线性插值：
%          lat(t_k) = interp1(t_raw, lat_raw, t_k, 'linear')
%          lon(t_k) = interp1(t_raw, lon_raw, t_k, 'linear')
%     4. 用中心差分法估计经纬度变化率（角速度，单位：度/秒）：
%          lon_rate(k) = (lon(k+1) - lon(k-1)) / (2*dt_sec)      ← 中心差分（k=2..n-1）
%          lon_rate(1)   = (lon(2) - lon(1)) / dt_sec             ← 前向差分（首点）
%          lon_rate(n)   = (lon(n) - lon(n-1)) / dt_sec           ← 后向差分（末点）
%
%   MATLAB 的 interp1 函数：
%     interp1(x, v, xq, method, extrapolation) 是一维插值函数
%       x:        原始自变量（此处为时间）
%       v:        原始因变量（此处为纬度或经度）
%       xq:       查询点（目标时间网格）
%       method:   'linear' 为线性插值
%       'extrap': 允许在原始数据范围外进行外推（超出仿真窗口边缘时用到）
%
% 【输入参数】
%   csv_path       - ADS-B CSV 文件的完整路径，类型为 char 字符串
%                    该文件应包含 19 列数据，包括航班号、经纬度、速度等字段
%   icao_list      - 要提取的飞机 ICAO 代码列表，类型为 cell array of char
%                    例如：{'780A2B', '780A3C', '780A4D'}
%                    ICAO（International Civil Aviation Organization）代码
%                    是每架飞机的全球唯一标识符（24位十六进制）
%   label_list     - 飞机的可读标签列表，类型为 cell array of char
%                    例如：{'CUA293', 'CSZ9168', 'CES5670'}
%                    与 icao_list 一一对应
%   dt_sec         - 仿真时间步长（秒），即重采样网格的间隔
%                    例如：1.0 表示每秒一个航迹点
%   start_time     - 仿真起始时间，类型为 datetime
%                    用于将 ADS-B 原始时间戳转换为相对秒数
%   duration_sec   - 仿真总时长（秒）
%   time_offset_sec - （可选）时间偏移量（秒），默认值为 0
%                     将 ADS-B 数据的搜索窗口向后平移
%                     用于匹配仿真中可能的时间偏移
%
% 【返回值】
%   true_tracks    - 飞机真实航迹数据，类型为 cell array，长度为 n_ac（飞机数量）
%                    每个 cell 元素是一个 N×5 矩阵：
%                      列1: lon_grid   - 经度（度）
%                      列2: lat_grid   - 纬度（度）
%                      列3: lon_rate   - 经度变化率（度/秒）
%                      列4: lat_rate   - 纬度变化率（度/秒）
%                      列5: t_grid     - 时间（秒，相对于 time_offset_sec）
%                    行数 N = floor(duration_sec / dt_sec) + 1
%   labels         - 飞机的可读标签列表，类型为 cell array of char
%                    与 icao_list 一一对应
%   speeds         - 飞机平均地速数组，类型为 double 列向量（n_ac × 1）
%                    单位：米/秒（已从节转换为米/秒，系数 0.514444）
%
% 【调用关系】
%   本函数调用：
%     detectImportOptions（MATLAB 内置：自动检测 CSV 导入选项）
%     readtable（MATLAB 内置：读取 CSV 为表格）
%     datetime（MATLAB 内置：时间字符串解析）
%     interp1（MATLAB 内置：一维插值/重采样）
%     unique、sort（MATLAB 内置：去重和排序）
%   本函数被调用：main.m 或仿真初始化脚本
%
% 【注意事项】
%   1. ADS-B 文件必须包含指定的飞机 ICAO 代码，否则会报错退出
%   2. 在仿真时间窗口内至少需要 3 个有效 ADS-B 数据点（用于曲线拟合）
%   3. 速度从节（knots）转换为米/秒：1 knot = 0.514444 m/s
%   4. CSV 文件编码和列名需与实际数据文件保持一致
%
% ============================================================================

function [true_tracks, labels, speeds] = load_adsb(csv_path, icao_list, label_list, ...
        dt_sec, start_time, duration_sec, time_offset_sec)
    % 加载 ADS-B CSV 数据，提取飞机航迹，重采样到仿真时间网格
    %
    % 输入:
    %   csv_path:        ADS-B CSV 文件路径
    %   icao_list:       飞机 ICAO 代码列表（cell array of char）
    %   label_list:      飞机标签列表（cell array of char）
    %   dt_sec:          仿真时间步长（秒）
    %   start_time:      仿真起始时间（datetime）
    %   duration_sec:    仿真时长（秒）
    %   time_offset_sec: 时间偏移量（秒，可选，默认0）
    %
    % 返回:
    %   true_tracks: 飞机真实航迹 cell 数组，每个元素为 N×5 矩阵
    %                [lon, lat, lon_rate, lat_rate, time]
    %   labels: 飞机标签 cell 数组
    %   speeds: 平均地速（米/秒）数组

    %% ---- 处理可选参数 ----
    % nargin 是 MATLAB 内置变量，记录函数被调用时传入的实参个数
    % 如果调用时只传了6个参数（nargin < 7），则 time_offset_sec 使用默认值 0
    if nargin < 7, time_offset_sec = 0; end

    %% ---- 配置 CSV 导入选项 ----
    % detectImportOptions 自动检测 CSV 文件的结构（分隔符、列数等）
    % 'NumVariables', 19 显式指定文件有 19 列数据
    opts = detectImportOptions(csv_path, 'NumVariables', 19);
    % 手动设定列名——ADS-B 数据的标准字段定义：
    %   icao:   飞机 ICAO 24 位代码（全球唯一标识符）
    %   lat:    纬度（度，WGS-84）
    %   lon:    经度（度，WGS-84）
    %   heading: 航向角（度，0=北，顺时针）
    %   alt_ft: 气压高度（英尺）
    %   speed_kt: 地速（节，knots = 海里/小时）
    %   ts:     时间戳字符串（格式：yyyy-MM-dd HH:mm:ss）
    %   x7, rx, type, reg, origin, dest, flight: 其他辅助字段
    %   flag1, vr_ft, icao_flt, flag2, airline:  其他标记字段
    opts.VariableNames = {'icao','lat','lon','heading','alt_ft','speed_kt',...
        'x7','rx','type','reg','ts','origin','dest','flight','flag1',...
        'vr_ft','icao_flt','flag2','airline'};

    %% ---- 读取 CSV 文件到 MATLAB 表格 ----
    % readtable 根据 opts 配置将 CSV 文件读入 table 类型变量 T
    % T 是一个 MATLAB 表格（table），可以通过 T.列名 访问各列
    T = readtable(csv_path, opts);

    %% ---- 初始化输出变量 ----
    % n_ac: 要提取的飞机数量
    n_ac = length(icao_list);
    % cell(n_ac, 1) 创建 n_ac×1 的空 cell 数组，用于存储每架飞机的航迹
    true_tracks = cell(n_ac, 1);
    % labels 用于存储飞机的可读标签
    labels = cell(n_ac, 1);
    % zeros(n_ac, 1) 创建 n_ac×1 的全零列向量，用于存储平均速度
    speeds = zeros(n_ac, 1);

    %% ====================================================================
    %% 主循环：逐架飞机提取航迹数据
    %% ====================================================================
    for a = 1:n_ac

        %% -- 获取当前飞机的 ICAO 代码 --
        % icao_list{a} 用花括号读取 cell 数组中的 char 字符串
        icao = icao_list{a};

        %% -- 在 ADS-B 表格中查找该飞机的所有记录 --
        % strcmp(T.icao, icao) 逐行比较表格的 icao 列与目标 icao 是否相等
        % 返回一个逻辑向量（长度 = 表格行数），匹配的行为 true
        % sum(idx) 统计匹配行数（true=1, false=0）
        idx = strcmp(T.icao, icao);
        if sum(idx) == 0
            % 如果该飞机在 ADS-B 数据中完全找不到，抛出错误
            error('Aircraft %s not found in ADS-B data', icao);
        end

        %% -- 提取该飞机的相关列数据 --
        % 用逻辑索引 T.lat(idx) 提取匹配行的指定列
        % 逻辑索引：T.lat(idx) 只返回 idx 为 true 的那些行
        ac_lat = T.lat(idx);       % 纬度向量（度）
        ac_lon = T.lon(idx);       % 经度向量（度）
        ac_spd = T.speed_kt(idx);  % 地速向量（节）
        ts_raw = T.ts(idx);        % 原始时间戳字符串向量

        %% -- 解析时间戳为 datetime 对象 --
        % ADS-B 的时间戳可能是 cell 数组（每行一个字符串）或字符串数组
        % 需要做兼容处理：
        %   iscell(ts_raw):  如果 ts_raw 是 cell 数组
        %     datetime(ts_raw, 'InputFormat', ...): 直接从 cell 数组解析
        %   否则（ts_raw 是 string 数组类型）：
        %     cellstr(string(ts_raw)) 先将 string 转 char 再转 cell
        %     'InputFormat', 'yyyy-MM-dd HH:mm:ss' 指定时间字符串格式
        if iscell(ts_raw)
            ts_dt = datetime(ts_raw, 'InputFormat', 'yyyy-MM-dd HH:mm:ss');
        else
            ts_dt = datetime(cellstr(string(ts_raw)), 'InputFormat', 'yyyy-MM-dd HH:mm:ss');
        end

        %% -- 将 datetime 转换为相对秒数 --
        % seconds(ts_dt - start_time) 计算每个时间戳相对于仿真起始时间的秒数差
        % 结果是一个 double 向量，单位为秒
        t_sec = seconds(ts_dt - start_time);

        %% -- 过滤：只保留仿真时间窗口 [time_offset_sec, time_offset_sec + duration_sec] 内的数据 --
        % & 是逻辑与（逐元素运算）
        % ~isnan(ac_lat) 排除纬度值为 NaN 的无效记录
        % ~isnan(ac_lon) 排除经度值为 NaN 的无效记录
        valid = t_sec >= time_offset_sec & t_sec <= time_offset_sec + duration_sec ...
                & ~isnan(ac_lat) & ~isnan(ac_lon);

        % 用逻辑索引过滤，只保留有效数据
        ac_lat = ac_lat(valid);
        ac_lon = ac_lon(valid);
        ac_spd = ac_spd(valid);
        t_sec = t_sec(valid);

        %% -- 安全性检查：有效数据点不足3个则报错 --
        % 至少需要 3 个点才能进行有意义的插值和差分计算
        if sum(valid) < 3
            error('Aircraft %s has <3 valid points in simulation window', icao);
        end

        %% -- 按时间去重 --
        % ADS-B 数据可能有多个记录共享同一时间戳（数据重复）
        % unique(t_sec, 'stable') 返回 t_sec 去重后的唯一值
        %   'stable' 参数：保持原始顺序不变（不排序），保留第一次出现的值
        %   ui: 被保留的唯一值在原始数组中的索引
        % 然后用 ui 去重所有相关数据列
        [t_sec, ui] = unique(t_sec, 'stable');
        ac_lat = ac_lat(ui);
        ac_lon = ac_lon(ui);
        ac_spd = ac_spd(ui);

        %% -- 按时间排序 --
        % 确保数据按时间升序排列（去重过程中可能打乱了顺序）
        % sort(t_sec) 返回升序排列的值和对应的排序索引 si
        [t_sec, si] = sort(t_sec);
        ac_lat = ac_lat(si);
        ac_lon = ac_lon(si);
        ac_spd = ac_spd(si);

        %% -- 生成均匀仿真时间网格并重采样 --
        % 生成从 0 到 duration_sec、步长为 dt_sec 的均匀网格
        % (0:dt_sec:duration_sec)' 的转置 ' 将其变为列向量
        %
        % 注意：时间网格是相对于 time_offset_sec 的
        % 例如 time_offset_sec = 5, duration_sec = 100
        %   t_grid = [0, 1, 2, ..., 100]（101个点）
        %   对应绝对时间偏移 [5, 6, 7, ..., 105]
        t_grid = (0:dt_sec:duration_sec)';

        % 将原始时间转为相对于 offset 的时间（用于插值）
        t_sec_relative = t_sec - time_offset_sec;

        %% -- 线性插值重采样：经纬度 --
        % interp1(x, v, xq, method, extrapolation) 一维插值
        %   x:             原始时间点（自变量）
        %   v:             原始观测值（因变量，此处为精度或纬度）
        %   xq:            查询时间点（目标网格）
        %   'linear':      线性插值方法（两点之间直线连接）
        %   'extrap':      允许外推（查询点超出原始数据范围时也能给出值）
        %
        % 线性插值假设目标在相邻两点之间匀速运动
        % 对于 ADS-B 重采样场景，这是一个合理且计算量最小的假设
        lat_grid = interp1(t_sec_relative, ac_lat, t_grid, 'linear', 'extrap');
        lon_grid = interp1(t_sec_relative, ac_lon, t_grid, 'linear', 'extrap');

        %% -- 中心差分法估计经纬度变化率（角速度）--
        % 变化率用于后续计算目标的地速和航向
        % 单位：度/秒（degrees per second）
        %
        % 差分方法：
        %   中心差分（k = 2, 3, ..., n-1）：
        %     rate(k) = (x(k+1) - x(k-1)) / (2*dt_sec)
        %     精度：O(dt^2)，二阶精度
        %
        %   前向差分（k = 1，首点）：
        %     rate(1) = (x(2) - x(1)) / dt_sec
        %     精度：O(dt)，一阶精度
        %
        %   后向差分（k = n，末点）：
        %     rate(n) = (x(n) - x(n-1)) / dt_sec
        %     精度：O(dt)，一阶精度

        n = length(t_grid);   % 重采样后的总点数

        % 预分配变化率数组（全零，后面覆盖）
        lon_rate = zeros(n, 1);   % 经度变化率（度/秒）
        lat_rate = zeros(n, 1);   % 纬度变化率（度/秒）

        % 中心差分（内部点，2 <= k <= n-1）
        for k = 2:n-1
            % 用前后两个邻点的值做中心差分
            % 分子：(x_{k+1} - x_{k-1})，分母：(2 * dt_sec)
            lon_rate(k) = (lon_grid(k+1) - lon_grid(k-1)) / (2*dt_sec);
            lat_rate(k) = (lat_grid(k+1) - lat_grid(k-1)) / (2*dt_sec);
        end

        % 边界处理（首点和末点——不能用中心差分，只能用单侧差分）
        if n >= 2                                     % 至少需要2个点才能算差分
            lon_rate(1)   = (lon_grid(2) - lon_grid(1)) / dt_sec;          % 首点：前向差分
            lat_rate(1)   = (lat_grid(2) - lat_grid(1)) / dt_sec;
            lon_rate(end) = (lon_grid(end) - lon_grid(end-1)) / dt_sec;    % 末点：后向差分
            lat_rate(end) = (lat_grid(end) - lat_grid(end-1)) / dt_sec;
        end

        %% -- 打包输出：当前飞机的完整航迹数据 --
        % 将五列数据拼成一个 N×5 矩阵存入 cell 数组
        % [lon_grid, lat_grid, lon_rate, lat_rate, t_grid]
        % 水平拼接（用逗号或空格分隔），每个变量都是 N×1 列向量
        true_tracks{a} = [lon_grid, lat_grid, lon_rate, lat_rate, t_grid];

        %% -- 存储飞机标签 --
        labels{a} = label_list{a};

        %% -- 计算平均地速（米/秒）--
        % mean(ac_spd, 'omitnan') 忽略 NaN 值计算平均值
        % 系数 0.514444：将节（knots，海里/小时）转换为米/秒
        %   1 海里 = 1852 米，1 小时 = 3600 秒
        %   1 knot = 1852 / 3600 ≈ 0.514444 m/s
        speeds(a) = mean(ac_spd, 'omitnan') * 0.514444;

        %% -- 打印加载信息 --
        % sum(idx) 是该飞机在 ADS-B 文件中的总记录数（过滤前）
        % n 是重采样后的点数
        % speeds(a) 是平均地速（米/秒，递进1位小数）
        fprintf('  %s(%s): %d ADS-B pts, resampled %d pts, avg spd %.0f m/s\n', ...
            label_list{a}, icao, sum(idx), n, speeds(a));

    end  % for 循环结束——所有飞机处理完毕

end  % 函数 load_adsb 结束
