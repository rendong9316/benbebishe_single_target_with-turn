% spherical_interpolate_.m
% 球面大圆插值函数（带下划线后缀避免命名冲突）
% ===============================
% 在指定时刻T处进行球面大圆插值，找到T前后最近的两个有效航迹点，
% 沿球面大圆按时间比例插值出目标在该时刻的经纬度坐标。
%
% 输入:
%   T: 当前要插值的时间点（秒）
%   t_valid: 有效航迹点的时间数组
%   valid_meas: 有效航迹点的测量数据（cell array of struct）
%   ref_time: 参考起始时间（datetime），用于生成时间字符串
%
% 返回:
%   result: 插值结果结构体，含 time_sec/time_str/lat/lon/aligned 字段

function result = spherical_interpolate_(T, t_valid, valid_meas, ref_time)
    % 在时刻 T 处做球面大圆插值
    % 找到 T 前后最近的两个有效航迹点，沿球面大圆按时间比例插值。

    n_valid = length(t_valid);

    % 找到 t_valid 中最后一个 <= T 的位置（等效于 Java 的 Arrays.binarySearch）
    idx = 1;
    while idx <= n_valid && t_valid(idx) < T
        idx = idx + 1;
    end
    % idx 现在是第一个 > T 的位置（1-based）

    if idx == 1
        % T 在所有点之前 → 外推（用最近两个点）
        m0 = valid_meas{1}; m1 = valid_meas{2};
        t0 = t_valid(1); t1 = t_valid(2);
        ratio = (T - t0) / (t1 - t0);
    elseif idx > n_valid
        % T 在所有点之后 → 外推（用最近两个点）
        m0 = valid_meas{n_valid - 1}; m1 = valid_meas{n_valid};
        t0 = t_valid(n_valid - 1); t1 = t_valid(n_valid);
        ratio = (T - t0) / (t1 - t0);
    else
        % T 在两点之间 → 内插
        m0 = valid_meas{idx - 1}; m1 = valid_meas{idx};
        t0 = t_valid(idx - 1); t1 = t_valid(idx);
        ratio = (T - t0) / (t1 - t0);
    end

    % 球面大圆插值
    [lon, lat] = sphere_utils_interpolate_great_circle( ...
        m0.lon, m0.lat, m1.lon, m1.lat, ratio);

    if ~isempty(ref_time)
        time_str = sphere_utils_seconds_to_datetime_str(T, ref_time);
    else
        time_str = sprintf('%.1fs', T);
    end
    %% struct() 创建结构体，字段名用单引号，多个字段用逗号分隔
    result = struct('time_sec', T, 'time_str', time_str, ...
                    'lat', lat, 'lon', lon, 'aligned', true);
end
