% =========================================================================
% fun_remove_assc_pts_from_pointlist_oracle.m — 从点迹列表中移除已关联点迹
% =========================================================================
% 【功能】
%   根据 used_det 掩码，从原始点迹列表中提取未被使用的点迹及其
%   原始索引。used_det 是逻辑数组，used_det(j)=true 表示第 j 个点迹
%   已被航迹关联消耗，used_det(j)=false 表示该点迹未被使用。
%
%   此函数在 Track_Process_for_HighRate_Oracle 的阶段5中被调用，
%   剩余的未用点迹将送入 trackStarter 进行新航迹起始。
%
% 【输入】
%   pointList      — 本帧所有检测点迹数组
%   used_det       — 逻辑数组，used_det(j)=true 表示第 j 个点迹已被使用
%
% 【输出】
%   remainingPointList — 未被使用的点迹数组
%   pointOriginalIndex — 未被使用的点迹在原始列表中的索引
% =========================================================================
function [remainingPointList, pointOriginalIndex] = fun_remove_assc_pts_from_pointlist_oracle(pointList, used_det)
    % 找出所有未被使用的点迹索引（used_det 为 false 的位置）
    % find(~used_det) 返回逻辑取反后为 true 的元素位置
    pointOriginalIndex = find(~used_det);
    % 按索引提取剩余点迹（MATLAB 支持逻辑/索引数组直接索引）
    remainingPointList = pointList(pointOriginalIndex);
end
