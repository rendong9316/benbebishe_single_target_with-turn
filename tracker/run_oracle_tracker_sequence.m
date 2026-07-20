function [trackList, tempTrackList, snapshots, diagList] = run_oracle_tracker_sequence(detList, ukf_tpl, params, truth_all, t_grid, verbose)
% RUN_ORACLE_TRACKER_SEQUENCE 逐帧调用 Oracle 跟踪器处理完整序列
%
% 这是 Oracle 跟踪器的最高层入口函数，负责:
%   1. 初始化跟踪状态（空航迹列表、帧计数器、全局航迹ID）
%   2. 逐帧调用 TRACK_MAIN_ORACLE 处理每帧检测
%   3. 可选地打印进度日志
%   4. 最后对航迹列表按 ID 排序
%
% 输入:
%   detList    — cell 数组，每帧的检测点迹列表
%   ukf_tpl    — UKF 滤波器模板
%   params     — 仿真参数结构体
%   truth_all  — 真值航迹数据
%   t_grid     — 时间网格
%   verbose    — 是否打印进度（可选，默认 false）
%
% 输出:
%   trackList  — 最终航迹列表（按 ID 排序）
%   tempTrackList — 航迹起始模块的内部状态
%   snapshots  — 每帧的航迹快照（精简版）
%   diagList   — 每帧的诊断信息

    % 如果未传入 verbose 参数，默认为 false
    if nargin < 6, verbose = false; end

    % 获取总帧数
    n_frames = numel(detList);

    % 预分配输出数组
    snapshots = cell(n_frames, 1);  % 每帧航迹快照
    diagList = cell(n_frames, 1);   % 每帧诊断信息

    % 初始化跟踪状态：
    %   trackList   — 当前所有航迹（活跃 + 历史）
    %   tempTrackList — 航迹起始模块的内部状态（每个目标一个滑窗条目）
    %   next_id     — 全局递增的航迹 ID 计数器，从 1 开始
    trackList = {};
    tempTrackList = struct([]);
    next_id = 1;

    % 逐帧循环：调用 TRACK_MAIN_ORACLE 处理每帧检测
    for k = 1:n_frames
        % TRACK_MAIN_ORACLE 是单帧处理入口，内部依次执行:
        %   1. 航迹生命周期管理（真值终止检测）
        %   2. UKF 预测（所有活跃航迹一步预测）
        %   3. Oracle 点迹-航迹关联（基于真值 ID 匹配）
        %   4. UKF 更新（关联点迹校正 / 未关联纯预测）
        %   5. 航迹质量管理和状态转移
        %   6. 未用点迹送入 trackStarter 进行新航迹起始
        [trackList, tempTrackList, snapshots{k}, next_id, diagList{k}] = TRACK_MAIN_ORACLE( ...
            trackList, tempTrackList, detList{k}, ukf_tpl, params, k, next_id, truth_all, t_grid);

        % 如果启用了详细输出，每隔 10 帧或首尾帧打印当前状态
        if verbose && (k == 1 || mod(k, 10) == 0 || k == n_frames)
            % 计算当前活跃航迹数量（排除 HISTORY 类型的航迹）
            active = sum(cellfun(@(trk) trk.type ~= 7, trackList));
            fprintf('  frame %3d/%3d: active=%d, total=%d%s', k, n_frames, active, numel(trackList), newline);
        end
    end

    % 处理完成后，对航迹列表按 ID 排序，保证顺序与创建顺序一致
    trackList = sortTrackList_oracle(trackList);
end
