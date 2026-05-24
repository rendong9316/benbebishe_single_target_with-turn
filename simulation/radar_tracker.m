% =========================================================================
% radar_tracker.m
% 雷达内部跟踪器 —— M/N航迹起始 + 航迹维持 + K_loss航迹终止
% =========================================================================
%
% 【功能概述】
%   模拟真实雷达在 CFAR 检测之后的内部航迹管理逻辑。对逐帧检测结果
%   执行 M/N 起始判据和连续丢点终止判据，输出碎片化的航迹段。
%
% 【状态机】
%   UNINITIATED (0)  — 无航迹，等待首次检测
%   INITIATING  (1)  — M/N 起始确认中，累积检测历史
%   TRACKING    (2)  — 航迹维持中，累积量测点
%
% 【使用方式】
%   state = radar_tracker('init', 1);                % 从 track_id=1 开始
%   for each scan:
%       [state, seg] = radar_tracker('process', state, meas, M, N, K);
%       if ~isempty(seg) → 保存该航迹段
%   end
%   extra = radar_tracker('finalize', state);        % 输出最后一段
% =========================================================================

function varargout = radar_tracker(action, varargin)
    switch action
        case 'init'
            varargout{1} = init_tracker(varargin{1});
        case 'process'
            [varargout{1}, varargout{2}] = process_scan(varargin{1}, varargin{2}, ...
                varargin{3}, varargin{4}, varargin{5});
        case 'finalize'
            varargout{1} = finalize_tracker(varargin{1});
        otherwise
            error('radar_tracker: unknown action "%s"', action);
    end
end

% =========================================================================
function state = init_tracker(start_id)
    state = struct();
    state.mode = 0;          % 0=uninitiated, 1=initiating, 2=tracking
    state.next_id = start_id;
    state.track_id = 0;
    state.segment = [];
    state.miss_count = 0;
    state.buf_detected = [];
    state.buf_meas = {};
end

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

        case 1  % INITIATING
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
function segments = finalize_tracker(state)
    segments = {};
    if state.mode == 2 && ~isempty(state.segment)
        segments{1} = state.segment;
    end
end
