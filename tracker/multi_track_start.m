% =========================================================================
% multi_track_start.m — 多目标航迹起始模块
% =========================================================================
% 【功能概述】
%   为 multi_track_manager 提供 M/N 滑窗航迹起始功能。
%   从未关联点迹（unused_dets）中，通过 M/N 滑窗检测新目标并返回
%   起始候选对（det1, det2）。
%
%   与 track_initiation.m 不同，本模块专门服务于多目标场景：
%   - 每帧从 unused_dets 中提取候选起始点
%   - 通过 M/N 滑窗判断是否满足起始条件
%   - 成功后返回 det1/det2，由调用方创建新航迹
%
% 【调用关系】
%   被 multi_track_manager 在 Step 8 后调用
%   内部调用: track_initiation('process', ...)
%
% 【输入】
%   state   — 滑窗状态（由上一帧返回，首次为 []）
%   unused_dets — 当前帧未关联点迹数组
%   params  — 参数结构体
%   frame_id — 当前帧编号
%
% 【输出】
%   new_state — 更新后的滑窗状态
%   det1      — 起始点1（历史帧点迹）
%   det2      — 起始点2（当前帧点迹）
%   success   — 是否成功起始
% =========================================================================

function [new_state, det1, det2, success] = multi_track_start(state, unused_dets, params, frame_id)
    if isempty(state)
        state = track_initiation('init', params);
    end

    [new_state, det1, det2, success] = track_initiation('process', state, unused_dets, params, frame_id);
end
