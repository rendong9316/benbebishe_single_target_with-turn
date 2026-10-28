% =========================================================================
% pair_key.m — 生成配对（两站局部航迹）的键
% =========================================================================
%
% 【功能概述】
%   BC 的跨帧互协方差 P12 是"某一对局部航迹"的属性。融合中心同一时刻
%   可能维护多条系统航迹，若不同配对共用一份 P12 会互相污染。
%   因此按 (R1 航迹 ID, R2 航迹 ID) 生成键，把 P12 分开存放。
%
% 【输入参数】
%   est1 — R1 侧估计结构体（可含 id 字段）
%   est2 — R2 侧估计结构体（可含 id 字段）
%
% 【输出参数】
%   key — 字符串键
% =========================================================================

function key = pair_key(est1, est2)
    id1 = est_id(est1);
    id2 = est_id(est2);
    if isnan(id1) || isnan(id2)
        key = 'default';
    else
        key = sprintf('%d|%d', id1, id2);
    end
end

% -------------------------------------------------------------------------
function id = est_id(est)
    id = NaN;
    if isstruct(est) && isfield(est, 'id') && ~isempty(est.id) && ...
            isnumeric(est.id) && isscalar(est.id) && isfinite(est.id)
        id = double(est.id);
    end
end
