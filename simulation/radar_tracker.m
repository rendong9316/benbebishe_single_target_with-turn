% =========================================================================
% radar_tracker.m — 雷达内部航迹管理状态机
% =========================================================================
%
% 【功能概述】
%   模拟真实雷达在 CFAR (恒虚警率) 检测之后的内部航迹管理逻辑。对逐帧
%   量测结果执行 M/N 航迹起始判据和连续丢点 K_loss 终止判据，输出碎片
%   化的航迹段（segments）。该模块模拟雷达信号处理后端的目标跟踪功能，
%   以状态机方式实现，通过不同的 action 字符串控制状态转换。
%
% 【状态机设计】
%   一个三状态的状态机，每个雷达独立维护：
%
%   UNINITIATED (mode=0) —— 无航迹状态
%     等待首次检测。收到检测后转入 INITIATING。
%
%   INITIATING (mode=1) —— M/N 起始确认中
%     维护滑动窗口缓冲区 buf_detected[1..N]，记录最近 N 次扫描中
%     是否有检测。当缓冲区中检测次数 >= M 时，确认航迹起始，转入
%     TRACKING 状态，并将缓冲区中的有效量测作为航迹段初始化部分。
%
%   TRACKING (mode=2) —— 航迹维持中
%     连续积累量测点。miss_count 记录连续丢点数。一旦 miss_count
%     达到 K_loss，终止航迹（输出当前段，回到 UNINITIATED）。
%
% 【使用方式（三段式接口）】
%   1. 初始化:
%      state = radar_tracker('init', start_id);
%      创建状态机，指定起始航迹 ID 编号。
%
%   2. 逐帧处理 (在每帧循环中调用):
%      [state, seg] = radar_tracker('process', state, meas, M, N, K_loss);
%      meas 为当前帧的量测结构体，为空表示未检测到。
%      若 seg 非空，表示一条航迹段已完成（被终止），应保存。
%
%   3. 结束收尾:
%      extra = radar_tracker('finalize', state);
%      输出仍在跟踪中的最后一条航迹段（如有）。
%
% 【输入参数】
%   action    - 字符串: 'init' | 'process' | 'finalize'
%   start_id  - (init时) 起始航迹ID编号 (整数)
%   state     - (process/finalize时) 跟踪器状态结构体
%   meas      - (process时) 当前帧量测结构体 (空=未检测到)
%   M, N      - 航迹起始参数: 最近N次扫描中至少M次检测
%   K_loss    - 航迹终止参数: 连续K_loss次无检测即终止
%
% 【输出】
%   state     - 更新后的跟踪器状态结构体
%   completed - 完成/终止的航迹段 (结构体数组)，无完成段时为空
%   segments  - (finalize时) 最后未终止的航迹段
%
% 【调用关系】
%   被仿真主程序调用（run_simulation.m），每个雷达独立一个 tracker 实例。
%   被 stitch_tracks.m 的输入段来源。
%
% 【与 UKF 的关系】
%   本模块仅管理航迹的存在性和量测聚集，不执行滤波。滤波由 UKF 模块
%   在更高层完成。tracker 输出的 segments 是原始量测序列，后续由 UKF
%   进行状态估计后再送入融合流程。
% =========================================================================

function varargout = radar_tracker(action, varargin)
    % ---------------------------------------------------------------
    % 顶层调度开关：根据 action 字符串分发到对应的子函数
    % ---------------------------------------------------------------
    switch action
        case 'init'
            % 初始化: 创建状态机，start_id 作为第2个参数
            varargout{1} = init_tracker(varargin{1});

        case 'process'
            % 逐帧处理: state(1), meas(2), M(3), N(4), K_loss(5)
            [varargout{1}, varargout{2}] = process_scan(varargin{1}, ...
                varargin{2}, varargin{3}, varargin{4}, varargin{5});

        case 'finalize'
            % 结束收尾: state(1), 返回最后的未完成段
            varargout{1} = finalize_tracker(varargin{1});

        otherwise
            error('radar_tracker: unknown action "%s"', action);
    end
end

% =========================================================================
% init_tracker — 初始化跟踪器状态结构体
% =========================================================================
% 输入:
%   start_id - 起始航迹 ID (整数)
% 输出:
%   state    - 初始化后的状态结构体
%     .mode       = 0 (UNINITIATED)
%     .next_id    = start_id  (下一可用航迹ID)
%     .track_id   = 0 (当前无航迹)
%     .segment    = [] (空航迹段)
%     .miss_count = 0 (丢点计数清零)
%     .buf_detected = [] (M/N 起始检测缓冲区)
%     .buf_meas   = {} (M/N 起始量测缓冲区)
% =========================================================================
function state = init_tracker(start_id)
    state = struct();
    state.mode = 0;          % 0=UNINITIATED, 1=INITIATING, 2=TRACKING
    state.next_id = start_id; % 下一条航迹将分配的 ID
    state.track_id = 0;       % 当前航迹 ID (0 表示无)
    state.segment = [];       % 当前航迹段的量测序列
    state.miss_count = 0;     % 连续丢点计数
    state.buf_detected = [];  % 逻辑数组: 最近N次扫描的检测状态
    state.buf_meas = {};      % cell数组: 对应的量测结构体
end

% =========================================================================
% process_scan — 处理一帧扫描的检测结果（状态机核心）
% =========================================================================
% 输入:
%   state  - 当前跟踪器状态
%   meas   - 当前帧量测 (空 = 未检测到)
%   M, N   - M/N 起始参数
%   K_loss - 终止丢点数
% 输出:
%   state     - 更新后的状态
%   completed - 完成的航迹段 (有终止时非空)
% =========================================================================
function [state, completed] = process_scan(state, meas, M, N, K_loss)
    is_det = ~isempty(meas);  % 本帧是否检测到目标
    completed = [];           % 默认为无完成段

    % 根据当前状态机模式执行对应逻辑
    switch state.mode

        case 0  % ================================================
                % UNINITIATED: 无航迹，等待首次检测
                % ================================================
            if is_det
                % 检测到目标 → 转入 INITIATING 状态
                state.mode = 1;
                state.buf_detected = true;   % 首次检测记为 true
                state.buf_meas = {meas};     % 保存首次量测
            end
            % 未检测到则保持 UNINITIATED，不做任何事

        case 1  % ================================================
                % INITIATING: M/N 起始确认中
                % ================================================
            % 将本帧检测结果追加到滑动窗口缓冲区末尾
            state.buf_detected(end+1) = is_det;
            state.buf_meas{end+1} = meas;

            % 保持滑动窗口大小不超过 N
            % 超出部分从头部丢弃（先进先出）
            if length(state.buf_detected) > N
                state.buf_detected(1) = [];
                state.buf_meas(1) = [];
            end

            % 检查 M/N 起始判据: 在最近 N 次扫描中至少有 M 次检测
            if length(state.buf_detected) >= N && sum(state.buf_detected) >= M
                % 起始判据满足 → 确认航迹，转入 TRACKING
                state.mode = 2;

                % 分配新的航迹 ID
                state.track_id = state.next_id;
                state.next_id = state.next_id + 1;

                state.miss_count = 0;  % 丢点计数清零
                state.segment = [];    % 初始化航迹段

                % 将缓冲区中所有有效量测写入航迹段
                % 这样航迹段从 M/N 起始的首个检测点开始
                for k = 1:length(state.buf_meas)
                    if ~isempty(state.buf_meas{k})
                        m = state.buf_meas{k};
                        m.track_id = state.track_id;  % 标记航迹 ID
                        if isempty(state.segment)
                            state.segment = m;  % 首个量测
                        else
                            state.segment(end+1) = m;  % 追加
                        end
                    end
                end
                % 清除起始缓冲区（航迹已确认，不再需要）
                state = rmfield(state, 'buf_detected');
                state = rmfield(state, 'buf_meas');
            end

        case 2  % ================================================
                % TRACKING: 航迹维持中
                % ================================================
            if is_det
                % 检测到 → 重置丢点计数，积累量测
                state.miss_count = 0;
                meas.track_id = state.track_id;

                % 追加量测到航迹段
                if isempty(state.segment)
                    state.segment = meas;
                else
                    state.segment(end+1) = meas;
                end
            else
                % 未检测到 → 丢点计数递增
                state.miss_count = state.miss_count + 1;

                % 检查终止判据: 连续丢点达到 K_loss
                if state.miss_count >= K_loss
                    % 航迹终止 → 输出当前段，回到 UNINITIATED
                    completed = state.segment;  % 输出已完成的航迹段

                    % 重置状态机
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
% 输入:
%   state - 当前跟踪器状态
% 输出:
%   segments - cell 数组，包含最后还在跟踪中的航迹段（如有）
% =========================================================================
function segments = finalize_tracker(state)
    segments = {};
    % 仅在 TRACKING 模式下且有有效数据时才输出
    if state.mode == 2 && ~isempty(state.segment)
        segments{1} = state.segment;
    end
end
