% =========================================================================
% build_target_states_at_time.m — 指定时刻所有目标状态提取
% =========================================================================
% 【功能】
%   在给定仿真时间 time_sec，从 truth_all 中提取所有仍在飞行中的
%   目标状态。每个目标的状态为 [lon, lat, lon_rate, lat_rate, aircraft_id]。
%
%   使用线性插值（interp1）在真值时间序列中查找指定时刻的经纬度
%   和速度。如果目标在该时刻尚未开始或已经结束飞行，则跳过。
%
%   该函数被检测生成模块（generate_frame_detections_multi）调用，
%   用于在每一帧获取当前时刻所有目标的精确位置和速度，作为
%   检测概率判断和量测计算的依据。
%
% 【输入参数】
%   truth_all  — cell 数组，每个元素为 N×5 矩阵
%               [time_sec, lon, lat, lon_rate, lat_rate]
%               由 aircraft_trajectory_interpolate('generate', ...) 生成
%   time_sec   — 标量，需要提取状态的仿真时间（秒），
%               相对于场景起始时间
%
% 【输出】
%   tgt_states — M×5 矩阵，M 为当前时刻仍在飞行的目标数（0 ≤ M ≤ N_targets）
%                每行 [lon, lat, lon_rate, lat_rate, aircraft_id]
%                aircraft_id 为 1-based 的目标编号
%
% 【使用示例】
%   % 在第 500 帧提取所有目标状态
%   states = build_target_states_at_time(scenario.truth_all, 500 * params.dt_sec);
%   % states 为 M×5 矩阵，M 为目标数
%
% =========================================================================
function tgt_states = build_target_states_at_time(truth_all, time_sec)
    % 初始化为空矩阵（0×5），表示当前时刻没有目标在飞行
    tgt_states = zeros(0, 5);

    % 遍历所有目标的真值航迹
    for ac = 1:length(truth_all)
        % 取出第 ac 个目标的 [time, lon, lat, lon_rate, lat_rate] 矩阵
        tt = truth_all{ac};

        % 检查三个条件：
        %   1. 航迹是否为空（无数据）
        %   2. 当前时间是否早于航迹起始时间（目标尚未起飞）
        %   3. 当前时间是否晚于航迹结束时间（目标已离开）
        % 任一条件满足则跳过该目标，continue 进入下一个目标
        if isempty(tt) || time_sec < tt(1, 5) || time_sec > tt(end, 5)
            continue;
        end

        % 使用线性插值（interp1）在真值时间序列中查找 time_sec 时刻的值
        % interp1(x, y, xi, 'linear', 'extrap')：
        %   x: 已知时间序列（tt(:,5)）
        %   y: 对应量（tt(:,1)~tt(:,4)）
        %   xi: 查询时间点（time_sec）
        %   'extrap': 允许在边界附近外推，防止浮点误差导致 NaN
        %
        % 分别插值得到经度、纬度、经度变化率、纬度变化率
        lon = interp1(tt(:, 5), tt(:, 1), time_sec, 'linear', 'extrap');
        lat = interp1(tt(:, 5), tt(:, 2), time_sec, 'linear', 'extrap');
        lon_rate = interp1(tt(:, 5), tt(:, 3), time_sec, 'linear', 'extrap');
        lat_rate = interp1(tt(:, 5), tt(:, 4), time_sec, 'linear', 'extrap');

        % 检查插值结果是否全部有效
        % 如果任意量为 NaN，说明插值失败（可能时间超出范围或数据异常）
        % 此时跳过该目标
        if any(isnan([lon, lat, lon_rate, lat_rate]))
            continue;
        end

        % 将插值得到的目标状态追加到输出矩阵末尾
        % [lon, lat, lon_rate, lat_rate, ac]：
        %   前四列为状态向量（经纬度 + 速度分量）
        %   第五列为目标编号（1-based，与 truth_all 的索引一致）
        tgt_states(end+1, :) = [lon, lat, lon_rate, lat_rate, ac];
    end
end
