% align_radar_to_grid.m
% 雷达航迹时间对齐主函数（球面大圆插值版本）
% ===============================
% 将两部异步采样的雷达航迹统一到相同的时间基准上。
% 全部使用球面公式（大圆航线插值），不经过 ENU 切平面。
%
% 输入:
%   meas_list: 雷达航迹列表（cell array of struct，含 time_sec/lat/lon）
%              漏检帧为 []，将被跳过
%   unified_time: 统一时间网格（秒），double 数组
%   ref_time: 参考起始时间（datetime），用于生成 time_str（可选）
%
% 返回:
%   cell array of struct: 对齐后的航迹，长度 = length(unified_time)
%                         None → NaN 位置填 [] 表示无效

function aligned = align_radar_to_grid(meas_list, unified_time, ref_time)
    % 将一部雷达的航迹对齐到统一时间网格（球面大圆插值）
    % 对统一时间网格的每个时刻 T，在雷达原有采样点之间做球面线性插值，
    % 推算出雷达在 T 时刻"本该看到"的目标经纬度。

    if nargin < 3, ref_time = []; end

    % ---- 过滤有效航迹点 ----
    valid = {};
    t_valid = [];
    for i = 1:length(meas_list)
        m = meas_list{i};                                 % cell 数组用 {} 读取元素
        if isempty(m), continue; end                      % 跳过漏检
        if ~isfield(m, 'lat') || isnan(m.lat), continue; end  % 跳过无效经纬度
        valid{end+1} = m;                                  % cell 数组用 {end+1} 追加元素
        t_valid(end+1) = m.time_sec;                       % 普通数组用 (end+1) 追加
    end

    if length(valid) < 2
        aligned = cell(1, length(unified_time));          % 不足以插值，全部返回空
        return;
    end

    % ---- 逐点插值（仅在片段有效时间范围内） ----
    t_min = t_valid(1);
    t_max = t_valid(end);
    aligned = cell(1, length(unified_time));
    for k = 1:length(unified_time)
        T = unified_time(k);
        if T < t_min || T > t_max
            aligned{k} = [];  % 超出片段范围，不进行外推
        else
            result = spherical_interpolate_(T, t_valid, valid, ref_time);
            aligned{k} = result;
        end
    end
end
