function [trackList, tempTrackList, snap, next_id, diagInfo] = TRACK_MAIN_ORACLE( ...
        trackList, tempTrackList, pointList, ukf_tpl, params, frame_id, next_id, truth_all, t_grid)

    % Oracle 模式单帧航迹处理入口
    %
    % 本文件虽薄（仅一层转发），但封装了 Track_Process_for_HighRate_Oracle 的调用签名，
    % 使得上层调用者（run.m 中的 run_oracle_tracker 循环）只需依赖这一个函数名，
    % 方便后续替换处理逻辑而不影响上层代码。
    %
    % 完整处理流程（详见 Track_Process_for_HighRate_Oracle）：
    %   1. 航迹生命周期管理（真值结束 → HISTORY 转态）
    %   2. UKF 预测（每个航迹独立 prepare）
    %   3. Oracle 点迹-航迹关联（基于真值 ID 直接匹配）
    %   4. UKF 更新（关联点迹 → Kalman 更新；未关联 → 纯预测）
    %   5. 航迹质量管理和状态转移
    %   6. 未用点迹送入 trackStarter 进行新航迹起始

    % ---- 委托给高层处理函数 ----
    % Track_Process_for_HighRate_Oracle 封装了完整的单帧处理逻辑：
    % 航迹管理 → UKF预测 → Oracle关联 → UKF更新 → 航迹起始
    [trackList, tempTrackList, snap, next_id, diagInfo] = Track_Process_for_HighRate_Oracle( ...
        trackList, tempTrackList, pointList, ukf_tpl, params, frame_id, next_id, truth_all, t_grid);
end
