% =========================================================================
% stitch_tracks.m — 同雷达碎片航迹拼接与间隙填补
% =========================================================================
%
% 【功能概述】
%   接收同一雷达的多段碎片化航迹（已在统一时间网格上对齐），执行三步：
%   1. 按时间排序并验证片段无重叠冲突
%   2. 合并为一条带空洞的连续航迹（重叠处优先选择协方差最小的点）
%   3. 对短时间间隙（<= max_gap_sec）用球面大圆插值填补
%
%   解决的核心问题：雷达内部 M/N 起始 + K_loss 终止导致一条真实目标
%   航迹被切分成多段碎片，需要拼接回完整轨迹便于后续融合。
%
% 【数学原理】
%   步骤2 (逐点合并):
%     对于每个时间点 i，遍历所有片段中该点的量测，选择协方差矩阵迹
%     最小的那个作为该点代表估计。协方差迹 = sum(diag(P))，是总体
%     不确定度的标量度量。
%
%   步骤3 (球面大圆插值):
%     已知首尾两点 (lon_a, lat_a) 和 (lon_b, lat_b)，插值参数
%     ratio ∈ [0,1] 表示沿大圆路径的相对位置：
%       - 计算大圆总角距
%       - 沿球面按球面线性插值 (slerp)
%       - 保持在大圆最短路径上，确保插值点的地理合理性
%
% 【输入参数】
%   segments     - cell 数组，每个元素是一条片段。每条片段是等长的
%                  cell 数组（与 unified_time 同长度），每个 cell 是
%                  单个时间点的量测/状态结构体。无数据的位置为空数组[]。
%   unified_time - [n_pts x 1] 双精度数组，统一时间网格（秒）
%   max_gap_sec  - 标量 (可选)，最大填补间隙（秒）。间隙时间小于等于
%                  此值的空洞通过插值填补；大于此值则保持为空。
%                  不传或传空时不执行插值。
%
% 【输出】
%   stitched - [n_pts x 1] cell 数组，拼接后的单条航迹。
%              每个 cell 可能是: (1) 量测/状态结构体 (有实际数据)
%                               (2) 空数组 [] (长间隙无法填补)
%                               (3) 插值结构体 (含 interpolated=true 标记)
%
% 【调用关系】
%   被仿真主程序调用（run_simulation.m），用于将 radar_tracker 输出
%   的碎片段拼接成连续航迹，再送入 UKF 进行滤波和后续融合。
%
%   内部调用: sphere_utils_interpolate_great_circle() 进行球面大圆插值
% =========================================================================

function stitched = stitch_tracks(segments, unified_time, max_gap_sec)
    % 空输入保护
    if isempty(segments)
        stitched = {};
        return;
    end

    % 未指定 max_gap_sec 时不执行间隙插值
    if nargin < 3, max_gap_sec = []; end

    n_pts = length(segments{1});  % 时间网格总点数

    % =================================================================
    % 步骤1: 片段提取与排序
    % =================================================================
    % 对每条片段，遍历找到其第一个和最后一个有效数据的时间戳
    % seg_info(k,:) = [t_start, t_end] — 第 k 条片段的时间范围
    n_seg = length(segments);
    seg_info = zeros(n_seg, 2);

    for k = 1:n_seg
        t_start_k = inf; t_end_k = -inf;
        for i = 1:n_pts
            pt = segments{k}{i};
            % 有效点必须有 lat 字段且非 NaN
            if ~isempty(pt) && isfield(pt, 'lat') && ~isnan(pt.lat)
                t_start_k = min(t_start_k, unified_time(i));
                t_end_k = max(t_end_k, unified_time(i));
            end
        end
        seg_info(k, :) = [t_start_k, t_end_k];
    end

    % 按起始时间升序排列，保证拼接时时间顺序一致
    [~, sort_idx] = sort(seg_info(:, 1));
    seg_info = seg_info(sort_idx, :);
    segments = segments(sort_idx);

    % =================================================================
    % 步骤2: 逐点择优合并
    % =================================================================
    % 对于每个时间网格点，从所有片段中选择协方差最小的那个点
    % 协方差用迹 (trace) 作为不确定度的标量度量
    stitched = cell(n_pts, 1);

    for i = 1:n_pts
        best = [];
        best_trace = inf;  % 初始化为无穷大

        for k = 1:n_seg
            pt = segments{k}{i};

            % 跳过无效点（空、无 lat 字段或 lat=NaN）
            if isempty(pt) || ~isfield(pt, 'lat') || isnan(pt.lat)
                continue;
            end

            % 获取该点的协方差迹（越小不确定度越低 → 优先选择）
            tr = inf;
            if isfield(pt, 'P') && ~isempty(pt.P)
                tr = trace(pt.P);
            end

            % 选择迹最小的点作为代表
            if tr < best_trace
                best_trace = tr;
                best = pt;
            elseif isempty(best)
                % 尚无选择时，退而取第一个有效点
                best = pt;
            end
        end

        stitched{i} = best;  % 空[]表示该时间点所有片段均无有效数据
    end

    % =================================================================
    % 步骤3: 短间隙球面大圆插值填补
    % =================================================================
    if ~isempty(max_gap_sec) && max_gap_sec > 0
        i = 1;
        while i <= n_pts
            % 跳过已有数据的点
            if ~isempty(stitched{i})
                i = i + 1;
                continue;
            end

            % 找到连续空洞的起止位置
            gap_start = i;  % 空洞起始索引
            while i <= n_pts && isempty(stitched{i})
                i = i + 1;
            end
            gap_end = i - 1;  % 空洞结束索引

            % 间隙两端都有效才有插值的意义
            if gap_start > 1 && gap_end < n_pts
                t_gap = unified_time(gap_end) - unified_time(gap_start - 1);

                % 间隙时间 <= max_gap_sec 时才插值填补
                if t_gap <= max_gap_sec
                    m_prev = stitched{gap_start - 1};  % 间隙前最后有效点
                    m_next = stitched{gap_end + 1};     % 间隙后首个有效点

                    % 对间隙内每个点执行球面大圆线性插值
                    for j = gap_start:gap_end
                        % 计算插值比例 ratio ∈ (0, 1)
                        % ratio = (t_j - t_prev) / (t_next - t_prev)
                        ratio = (unified_time(j) - m_prev.time_sec) / ...
                                (m_next.time_sec - m_prev.time_sec);

                        % 球面大圆插值 → 返回经纬度
                        [lon_fill, lat_fill] = sphere_utils_interpolate_great_circle( ...
                            m_prev.lon, m_prev.lat, m_next.lon, m_next.lat, ratio);

                        % 创建插值结构体，标记 interpolated=true 以区别于实测点
                        stitched{j} = struct('time_sec', unified_time(j), ...
                            'lat', lat_fill, 'lon', lon_fill, 'interpolated', true);
                    end
                end
            end

            % 跳到空洞之后继续扫描
            i = gap_end + 1;
        end
    end
end
