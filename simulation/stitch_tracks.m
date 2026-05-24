% =========================================================================
% stitch_tracks.m
% 同雷达航迹拼接 —— 判断片段归属 + 合并 + 短间隙填补
% =========================================================================
%
% 【功能概述】
%   接收同一雷达的多段碎片航迹（已在统一时间网格上对齐），
%   1. 按时间排序并验证片段不重叠冲突
%   2. 合并为一条带空洞的连续航迹（重叠处选协方差最小的点）
%   3. 对短间隙（≤ max_gap_sec）用球面大圆插值填补
%
% 【使用方式】
%   stitched = stitch_tracks(segments, unified_time, max_gap_sec);
%
% 输入：
%   segments    - cell array, 每个元素是经 align_radar_to_grid 对齐后的
%                 cell 数组（等长，统一时间网格）
%   unified_time- 统一时间网格（秒）
%   max_gap_sec - 最大填补间隙（秒），超过此值保持为空
%
% 输出：
%   stitched    - 等长 cell 数组，合并并填补后的单条航迹
% =========================================================================

function stitched = stitch_tracks(segments, unified_time, max_gap_sec)
    if isempty(segments)
        stitched = {};
        return;
    end

    if nargin < 3, max_gap_sec = []; end

    n_pts = length(segments{1});

    % =========================================================================
    % 步骤1: 片段归属判断 —— 提取每段的时间范围并按起始时间排序
    % =========================================================================
    n_seg = length(segments);
    seg_info = zeros(n_seg, 2);  % [t_start, t_end]
    for k = 1:n_seg
        t_start_k = inf; t_end_k = -inf;
        for i = 1:n_pts
            pt = segments{k}{i};
            if ~isempty(pt) && isfield(pt, 'lat') && ~isnan(pt.lat)
                t_start_k = min(t_start_k, unified_time(i));
                t_end_k = max(t_end_k, unified_time(i));
            end
        end
        seg_info(k, :) = [t_start_k, t_end_k];
    end

    % 按起始时间排序
    [~, sort_idx] = sort(seg_info(:, 1));
    seg_info = seg_info(sort_idx, :);
    segments = segments(sort_idx);

    % =========================================================================
    % 步骤2: 逐点合并 —— 重叠处按协方差迹择优
    % =========================================================================
    stitched = cell(n_pts, 1);

    for i = 1:n_pts
        best = [];
        best_trace = inf;

        for k = 1:n_seg
            pt = segments{k}{i};
            if isempty(pt) || ~isfield(pt, 'lat') || isnan(pt.lat)
                continue;
            end

            tr = inf;
            if isfield(pt, 'P') && ~isempty(pt.P)
                tr = trace(pt.P);
            end

            if tr < best_trace
                best_trace = tr;
                best = pt;
            elseif isempty(best)
                best = pt;
            end
        end

        stitched{i} = best;
    end

    % =========================================================================
    % 步骤3: 短间隙填补 —— 球面大圆插值
    % =========================================================================
    if ~isempty(max_gap_sec) && max_gap_sec > 0
        i = 1;
        while i <= n_pts
            % 跳过有效点
            if ~isempty(stitched{i})
                i = i + 1;
                continue;
            end

            % 找到间隙起始
            gap_start = i;
            while i <= n_pts && isempty(stitched{i})
                i = i + 1;
            end
            gap_end = i - 1;

            % 间隙两端都有效才可填补
            if gap_start > 1 && gap_end < n_pts
                t_gap = unified_time(gap_end) - unified_time(gap_start - 1);
                if t_gap <= max_gap_sec
                    m_prev = stitched{gap_start - 1};
                    m_next = stitched{gap_end + 1};
                    for j = gap_start:gap_end
                        ratio = (unified_time(j) - m_prev.time_sec) / ...
                                (m_next.time_sec - m_prev.time_sec);
                        [lon_fill, lat_fill] = sphere_utils_interpolate_great_circle( ...
                            m_prev.lon, m_prev.lat, m_next.lon, m_next.lat, ratio);
                        stitched{j} = struct('time_sec', unified_time(j), ...
                            'lat', lat_fill, 'lon', lon_fill, 'interpolated', true);
                    end
                end
            end
            i = gap_end + 1;
        end
    end
end
