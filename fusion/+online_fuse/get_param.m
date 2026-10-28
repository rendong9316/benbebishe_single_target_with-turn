% =========================================================================
% get_param.m — 安全读取 params 字段
% =========================================================================
%
% 【功能概述】
%   从参数结构体中读取指定字段，字段缺失或为空时返回默认值。
%   R2023a 不支持 isfield(s, 'a', 'b') 三参数写法，这里只做两参数检查。
%
% 【输入参数】
%   params  — 参数结构体
%   name    — 字段名（字符串）
%   default — 默认值
%
% 【输出参数】
%   value — 字段值或默认值
% =========================================================================

function value = get_param(params, name, default)
    value = default;
    if isstruct(params) && isfield(params, name)
        candidate = params.(name);
        if ~isempty(candidate)
            value = candidate;
        end
    end
end
