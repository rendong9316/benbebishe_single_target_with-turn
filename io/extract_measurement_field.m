% extract_measurement_field.m
% 从量测列表中提取指定字段
% ===============================
% 输入:
%   meas_list: cell array of struct
%   key: 字段名
% 返回:
%   vals: 提取的字段值数组，漏检填NaN

function vals = extract_measurement_field(meas_list, key)
    vals = NaN(length(meas_list), 1);
    for i = 1:length(meas_list)
        m = meas_list{i};
        if ~isempty(m) && isfield(m, key)
            vals(i) = m.(key);
        end
    end
end
