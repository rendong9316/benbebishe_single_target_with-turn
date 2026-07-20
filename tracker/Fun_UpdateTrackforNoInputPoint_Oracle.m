% =========================================================================
% Fun_UpdateTrackforNoInputPoint_Oracle.m — 无输入点迹航迹处理
% =========================================================================
% 【功能】
%   当某帧没有任何检测点迹时（全漏检帧），对当前所有活跃航迹执行
%   纯预测更新。每条航迹的 UKF 保留预测状态（不执行 Kalman 校正），
%   同时记录 NaN 到 NIS 历史，并触发质量递减逻辑。
%
%   注意：此函数在 Track_Process_for_HighRate_Oracle 的主循环中
%   并不被直接调用——所有航迹都会进入 Fun_UpdateTrackByAsscResult_Oracle，
%   其中未关联的航迹通过 ukf_dispatch('update', ukf, []) 走纯预测路径。
%   本函数作为"全空帧"的快捷批量处理路径存在。
%
% 【输入】
%   trackList  — 当前航迹列表
%   params     — 仿真参数
%   frame_id   — 当前帧编号
%
% 【输出】
%   trackList  — 更新后的航迹列表（所有航迹执行纯预测 + 质量递减）
% =========================================================================
function trackList = Fun_UpdateTrackforNoInputPoint_Oracle(trackList, params, frame_id)
    % 遍历所有航迹，对每条执行纯预测更新
    % 当全帧无检测时，所有航迹都走纯预测路径
    for i = 1:length(trackList)
        trk = trackList{i};
        % ukf_dispatch('update', [], ...) 内部检测到 innov_w 为空时，
        % 直接将状态设为预测值，不执行 Kalman 校正
        [~, ~, trk.ukf] = ukf_dispatch('update', trk.ukf, []);
        trk.assoc_det = [];  % 清除关联点迹引用

        % 记录 NaN 到 NIS 历史（表示本帧无关联，无法计算 NIS）
        if ~isfield(trk, 'nis_history') || isempty(trk.nis_history)
            trk.nis_history = [];
        end
        trk.nis_history(end+1) = NaN;

        % 调用质量管理系统：无关联 → Quality-1，可能触发状态转移
        trk = fun_track_quality_management_and_info_completion_oracle(trk, [], params, params, frame_id);
        trackList{i} = trk;
    end
end
