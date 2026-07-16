% =========================================================================
% track_initiation.m
% =========================================================================
% 【功能概述】
%   纯过程化 M/N 滑窗航迹起始器，采用 action dispatcher 模式。
%   从 single_track_runner 中提取的精确算法逻辑。
%
% 【Actions】
%   'init'    — 初始化 M/N 滑窗状态
%   'process' — 逐帧处理点迹，满足 M/N 条件时返回最优起始对
%   'reset'   — 重置状态（等价于 'init'）
%
% 【M/N 起始算法】
%   1. 维护长度为 N 的滑窗，记录每帧点迹
%   2. 当滑窗内 ≥ M 帧有点迹 且 当前帧有点迹时，触发起始尝试
%   3. 多假设配对：当前帧 × 历史帧 所有点迹对
%   4. Haversine 距离 → 速度检验（30-600 m/s）
%   5. 共识评分：其他有检测帧中点迹是否靠近配对轨迹（80km 门限）
%   6. 选择最高评分对，评分 ≥ 1 即成功
%
% 【输入格式】
%   state = track_initiation('init', params)
%   [state, det1, det2, success] = track_initiation('process', state, dets, params, frame_id)
%   state = track_initiation('reset', params)
% =========================================================================

function varargout = track_initiation(action, varargin)
    switch action
        case 'init'
            [varargout{1}] = init_state(varargin{1});

        case 'process'
            [varargout{1:nargout}] = process_frame(varargin{:});

        case 'reset'
            [varargout{1}] = init_state(varargin{1});

        otherwise
            error('track_initiation: unknown action "%s"', action);
    end
end


% =========================================================================
% init_state — 初始化 M/N 滑窗状态
% =========================================================================
function state = init_state(params)
    state.window = {};
    state.has_det = [];
    state.N = params.tracker_N;
    state.M = params.tracker_M;
end


% =========================================================================
% process_frame — 逐帧 M/N 滑窗起始处理
%
% 输入:
%   state    — 当前滑窗状态（struct，含 .window .has_det .N .M）
%   dets     — 当前帧点迹（struct 数组，须含 .lat .lon 字段）
%   params   — 参数结构体（需含 .dt_sec）
%   frame_id — 当前帧编号（保留，供未来扩展使用）
%
% 输出:
%   state   — 更新后的滑窗状态
%   det1    — 配对中的早期点迹（best_prev）
%   det2    — 配对中的当前帧点迹（best_curr）
%   success — 逻辑值，true 表示 M/N 条件满足且找到有效起始对
% =========================================================================
function [state, det1, det2, success] = process_frame(state, dets, params, frame_id) %#ok<INUSD>
    % ---- 默认返回 ----
    det1 = [];
    det2 = [];
    success = false;

    % ---- 1. 将当前帧点迹追加到滑窗 ----
    state.window{end+1} = dets;
    state.has_det(end+1) = ~isempty(dets);

    % ---- 2. 保持窗长不超过 N ----
    if length(state.window) > state.N
        state.window(1) = [];
        state.has_det(1) = [];
    end

    % ---- 3. 检查 M/N 条件 ----
    n_with_det = sum(state.has_det);
    if n_with_det < state.M || isempty(dets)
        return;
    end

    % ---- 4. 多假设配对 + 共识评分 ----
    best_prev = [];
    best_curr_idx = 1;
    best_support = -1;

    for curr_idx = 1:length(dets)
        for i = 1:(length(state.window)-1)
            prev_dets = state.window{i};
            if isempty(prev_dets), continue; end
            for p = 1:length(prev_dets)
                dp = prev_dets(p);
                dc = dets(curr_idx);
                if ~isfield(dp, 'lat') || isnan(dp.lat), continue; end
                if ~isfield(dc, 'lat') || isnan(dc.lat), continue; end

                % Haversine 距离 → 估计速度
                dist = sphere_utils_haversine_distance(dp.lon, dp.lat, dc.lon, dc.lat);
                dt_frames = length(state.window) - i;
                est_speed = dist / (dt_frames * params.dt_sec);
                if est_speed < 30 || est_speed > 600
                    continue;
                end

                % 共识评分：其他含检测帧中点迹是否靠近该轨迹
                support = 0;
                for jj = 1:(length(state.window)-1)
                    if jj == i, continue; end
                    other = state.window{jj};
                    if isempty(other), continue; end
                    for oo = 1:length(other)
                        do = other(oo);
                        if ~isfield(do, 'lat') || isnan(do.lat), continue; end
                        d1 = sphere_utils_haversine_distance(dp.lon, dp.lat, do.lon, do.lat);
                        d2 = sphere_utils_haversine_distance(dc.lon, dc.lat, do.lon, do.lat);
                        if d1 < 80000 && d2 < 80000
                            support = support + 1;
                        end
                    end
                end

                if support > best_support
                    best_support = support;
                    best_prev = dp;
                    best_curr_idx = curr_idx;
                end
            end
        end
    end

    % ---- 5. 判定是否起始 ----
    if best_support >= 1
        det1 = best_prev;
        det2 = dets(best_curr_idx);
        success = true;
    end
end
