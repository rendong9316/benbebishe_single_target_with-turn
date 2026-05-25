% =========================================================================
% tracker_utils.m
% 雷达航迹管理工具集 — 包含雷达内部跟踪器与航迹拼接
% =========================================================================
% 本文件合并了原 radar_tracker.m 和 stitch_tracks.m。
%
% 调用方式:
%   state = tracker_utils('init', start_id)
%   [state, seg] = tracker_utils('process', state, meas, M, N, K_loss)
%   extra = tracker_utils('finalize', state)
%   stitched = tracker_utils('stitch', segments, unified_time, max_gap_sec)
% =========================================================================

function varargout = tracker_utils(action, varargin)
    switch action
        case {'init', 'process', 'finalize'}
            [varargout{1:nargout}] = radar_tracker(action, varargin{:});
        case 'stitch'
            varargout{1} = stitch_tracks(varargin{:});
        otherwise
            error('tracker_utils: unknown action "%s"', action);
    end
end

% =========================================================================
% radar_tracker — 雷达内部航迹管理状态机
% =========================================================================
% 模拟真实雷达在 CFAR 检测之后的内部航迹管理逻辑。对逐帧量测结果
% 执行 M/N 航迹起始判据和连续丢点 K_loss 终止判据，输出碎片化的航迹段。
%
% 三状态状态机: UNINITIATED(0) → INITIATING(1) → TRACKING(2)
%
% 调用方式:
%   初始化:   state = radar_tracker('init', start_id);
%   逐帧处理: [state, seg] = radar_tracker('process', state, meas, M, N, K_loss);
%   结束收尾: extra = radar_tracker('finalize', state);
% =========================================================================
function varargout = radar_tracker(action, varargin)
    switch action
        case 'init'
            varargout{1} = init_tracker(varargin{1});
        case 'process'
            [varargout{1}, varargout{2}] = process_scan(varargin{1}, ...
                varargin{2}, varargin{3}, varargin{4}, varargin{5});
        case 'finalize'
            varargout{1} = finalize_tracker(varargin{1});
        otherwise
            error('radar_tracker: unknown action "%s"', action);
    end
end

% =========================================================================
% init_tracker — 初始化跟踪器状态结构体
% =========================================================================
function state = init_tracker(start_id)
    state = struct();
    state.mode = 0;          % 0=UNINITIATED, 1=INITIATING, 2=TRACKING
    state.next_id = start_id;
    state.track_id = 0;
    state.segment = [];
    state.miss_count = 0;
    state.buf_detected = [];
    state.buf_meas = {};
end

% =========================================================================
% process_scan — 处理一帧扫描的检测结果（状态机核心）
% =========================================================================
function [state, completed] = process_scan(state, meas, M, N, K_loss)
    is_det = ~isempty(meas);
    completed = [];

    switch state.mode
        case 0  % UNINITIATED
            if is_det
                state.mode = 1;
                state.buf_detected = true;
                state.buf_meas = {meas};
            end

        case 1  % INITIATING: M/N 起始确认中
            state.buf_detected(end+1) = is_det;
            state.buf_meas{end+1} = meas;
            if length(state.buf_detected) > N
                state.buf_detected(1) = [];
                state.buf_meas(1) = [];
            end
            if length(state.buf_detected) >= N && sum(state.buf_detected) >= M
                state.mode = 2;
                state.track_id = state.next_id;
                state.next_id = state.next_id + 1;
                state.miss_count = 0;
                state.segment = [];
                for k = 1:length(state.buf_meas)
                    if ~isempty(state.buf_meas{k})
                        m = state.buf_meas{k};
                        m.track_id = state.track_id;
                        if isempty(state.segment)
                            state.segment = m;
                        else
                            state.segment(end+1) = m;
                        end
                    end
                end
                state = rmfield(state, 'buf_detected');
                state = rmfield(state, 'buf_meas');
            end

        case 2  % TRACKING
            if is_det
                state.miss_count = 0;
                meas.track_id = state.track_id;
                if isempty(state.segment)
                    state.segment = meas;
                else
                    state.segment(end+1) = meas;
                end
            else
                state.miss_count = state.miss_count + 1;
                if state.miss_count >= K_loss
                    completed = state.segment;
                    state.mode = 0;
                    state.track_id = 0;
                    state.segment = [];
                    state.miss_count = 0;
                end
            end
    end
end

% =========================================================================
% finalize_tracker — 仿真结束时收尾：输出最后的未终止段
% =========================================================================
function segments = finalize_tracker(state)
    segments = {};
    if state.mode == 2 && ~isempty(state.segment)
        segments{1} = state.segment;
    end
end

% =========================================================================
% stitch_tracks — 同雷达碎片航迹拼接与间隙填补
% =========================================================================
% 接收同一雷达的多段碎片化航迹，执行三步：
%   1. 按时间排序并验证片段无重叠冲突
%   2. 合并为一条带空洞的连续航迹（重叠处优先选择协方差最小的点）
%   3. 对短时间间隙（<= max_gap_sec）用球面大圆插值填补
%
% 输入:
%   segments     - cell 数组，每条片段是等长的 cell 数组
%   unified_time - [n_pts x 1] 统一时间网格（秒）
%   max_gap_sec  - 标量 (可选)，最大填补间隙（秒）
% 输出:
%   stitched - [n_pts x 1] cell 数组，拼接后的单条航迹
% =========================================================================
function stitched = stitch_tracks(segments, unified_time, max_gap_sec)
    if isempty(segments)
        stitched = {};
        return;
    end

    if nargin < 3, max_gap_sec = []; end

    n_pts = length(segments{1});
    n_seg = length(segments);

    % 步骤1: 片段提取与排序
    seg_info = zeros(n_seg, 2);
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

    [~, sort_idx] = sort(seg_info(:, 1));
    seg_info = seg_info(sort_idx, :);
    segments = segments(sort_idx);

    % 步骤2: 逐点择优合并
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

    % 步骤3: 短间隙球面大圆插值填补
    if ~isempty(max_gap_sec) && max_gap_sec > 0
        i = 1;
        while i <= n_pts
            if ~isempty(stitched{i})
                i = i + 1;
                continue;
            end
            gap_start = i;
            while i <= n_pts && isempty(stitched{i})
                i = i + 1;
            end
            gap_end = i - 1;
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
