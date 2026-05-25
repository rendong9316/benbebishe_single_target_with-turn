% ============================================================================
% extract_measurement_field.m
% 从量测列表中提取指定字段的数值数组
% ============================================================================
%
% 【功能概述】
%   遍历 cell 数组形式的量测序列，提取每个元素（帧）中指定字段的数值，
%   返回一个 double 列向量。对于漏检帧（空数组）或缺少目标字段的帧，
%   自动填充 NaN（Not a Number）作为无效值的占位符。
%   这是一个纯数据清洗/提取工具函数，无数学计算。
%
% 【在空间配准流程中的角色】
%   该函数服务于 save_all.m 中的 MAT 文件保存阶段。在将对齐后的雷达航迹
%   持久化为 MAT 格式时，需要将分散在 struct 中的各个字段（range_meas、
%   azimuth_meas、lat、lon 等）提取出来，整理成规整的数值矩阵以便后续分析。
%   本函数就是完成这个"从 struct 抽取到矩阵"的转换。
%
%   典型使用方式（在 save_all.m 中）：
%     r1_range_meas = extract_measurement_field(r1_meas_list, 'range_meas');
%     r1_az_meas    = extract_measurement_field(r1_meas_list, 'azimuth_meas');
%     r1_lat        = extract_measurement_field(r1_meas_list, 'lat');
%     r1_lon        = extract_measurement_field(r1_meas_list, 'lon');
%
% 【实现原理】
%   遍历 meas_list（一个 cell 数组，每个元素是一帧的 struct），对每个元素：
%     1. 如果该帧为空 []（漏检），该位填 NaN
%     2. 如果 struct 中不包含目标字段，该位填 NaN
%     3. 否则，用动态字段访问语法 m.(key) 读取字段值，填入对应位置
%   最终返回一个 N×1 的列向量，其中 N = length(meas_list)
%
%   MATLAB 动态字段访问：m.(key)
%     当 key 是字符串变量时，m.(key) 等价于 m.字段名。
%     例如 key = 'range_meas' 时，m.(key) 等价于 m.range_meas。
%     这种语法允许在运行时动态决定访问哪个字段。
%
% 【输入参数】
%   meas_list   - 量测序列，类型为 cell array of struct
%                 每个 cell 元素是一帧量测（struct，含多种字段），
%                 漏检帧为空数组 []
%   key         - 要提取的字段名称，类型为 char 字符串
%                 例如 'range_meas'、'azimuth_meas'、'time_sec'、
%                 'lat'、'lon'、'radial_vel_meas' 等
%
% 【返回值】
%   vals        - 提取出的字段值数组，类型为 double 列向量（N×1）
%                 长度与 meas_list 相同
%                 漏检帧或缺失字段的对应位置填 NaN
%                 例如：[50000.1; NaN; 49999.8; 50000.5; ...]
%                       ↑第2帧漏检，填 NaN
%
% 【调用关系】
%   本函数被调用：save_all.m（数据持久化时提取各字段）
%   本函数调用：无（纯工具函数，不依赖其他自定义函数）
%
% ============================================================================

function vals = extract_measurement_field(meas_list, key)
    % 从量测列表中提取指定字段的数值向量
    %
    % 输入:
    %   meas_list: cell array of struct，每帧量测数据
    %   key:       要提取的字段名字符串（如 'range_meas'）
    %
    % 返回:
    %   vals: N×1 的 double 列向量，漏检/缺失字段处填 NaN

    %% ---- 预分配输出数组 ----
    % NaN(length(meas_list), 1) 创建一个 N×1 的全 NaN 列向量
    % N = length(meas_list)，即量测序列的总帧数
    % 先用 NaN 填满，然后在有效帧的位置覆盖为实际值
    % 这样漏检帧自动保留 NaN，无需额外处理
    vals = NaN(length(meas_list), 1);

    %% ---- 遍历所有帧，逐帧提取字段值 ----
    for i = 1:length(meas_list)
        m = meas_list{i};                          % 用花括号 {} 读取 cell 数组的第 i 帧

        % 检查该帧是否有效（非空 且 包含目标字段）
        % isempty(m):    如果 m 是空数组 []，返回 true（漏检帧）
        % isfield(m, key): 如果 struct m 中包含名为 key 的字段，返回 true
        % && 是短路与：如果 isempty(m) 为 true，不再检查 isfield(m, key)
        if ~isempty(m) && isfield(m, key)
            % 使用动态字段访问语法 m.(key) 读取字段值
            % 例如 key = 'range_meas' 时，m.(key) 等价于 m.range_meas
            vals(i) = m.(key);
        end
        % 如果 isempty(m) 或 ~isfield(m, key)，vals(i) 保持 NaN 不变
    end

end  % 函数 extract_measurement_field 结束
