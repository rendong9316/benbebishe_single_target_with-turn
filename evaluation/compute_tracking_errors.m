% =========================================================================
% compute_tracking_errors.m
% =========================================================================
% 【功能概述】
%   定量误差评估函数。将UKF滤波轨迹和校准前后点迹分别与真值轨迹
%   比较，计算位置误差的统计分布。输出逐帧误差和汇总统计，支持
%   多飞机场景。
%
% 【数学原理】
%   1. 位置误差计算:
%      使用 Haversine 公式计算球面上两点间的大圆距离:
%         d = 2 * R * atan2(sqrt(a), sqrt(1-a))
%      其中 a = sin^2(dlat/2) + cos(lat1)*cos(lat2)*sin^2(dlon/2)
%           R = 6371 km (地球平均半径)
%
%   2. 真值时间对齐:
%      真值轨迹 time_sec 和雷达帧 frame_times 的时间轴可能不同。
%      使用线性插值 interp1 将真值映射到帧时刻:
%         true_position(k) = interp1(truth.time, truth.pos, frame_time(k))
%
%   3. 航迹-真值匹配:
%      对每帧每条真值飞机，在活跃航迹中寻找距离最近的（200km内），
%      作为该飞机的跟踪结果。避免了硬分配（hard assignment）可能
%      导致的交换错误（swap error）。
%
%   4. 汇总统计指标:
%      - n: 样本数
%      - median: 中位数（对离群值鲁棒）
%      - mean: 均值
%      - std: 标准差
%      - rms: 均方根误差 sqrt(mean(error^2))
%      - min/max: 极值
%      - pct95: 95%分位数
%
% 【输入参数】
%   trackSnapshots - cell数组，每帧的航迹快照
%                    由 single_track_runner 或 multi_track_manager 产生
%   detList       - cell数组，每帧的点迹列表（含 is_clutter, aircraft_id 等字段）
%   truthTrajs    - 结构体数组，每条飞机的真值轨迹，需含:
%                   .label: 飞机标签
%                   .time_sec: 时间向量（秒）
%                   .lat: 纬度向量
%                   .lon: 经度向量
%   n_frames      - 总帧数
%   dt_sec        - 帧时间间隔（秒）
%   radar_label   - 雷达站标签字符串（用于输出标识）
%
% 【输出】
%   errorStats - 结构体，包含:
%                .radar: 雷达标签
%                .n_frames: 帧数
%                .ukf_errors_km: cell数组，每架飞机的UKF误差序列
%                .det_errors_km: cell数组，校准后点迹误差序列
%                .raw_errors_km: cell数组，校准前原始点迹误差序列
%                .summary: 每架飞机的汇总统计
%                .overall: 所有飞机合并的总体统计
%
% 【调用关系】
%   被评估脚本调用
%   子调用: haversine_km, compute_summary, interp1
% =========================================================================

function errorStats = compute_tracking_errors(trackSnapshots, detList, truthTrajs, ...
        n_frames, dt_sec, radar_label)

    n_ac = length(truthTrajs);  % 飞机数量

    % 构建帧时间向量: t_k = k * dt_sec, k = 0,1,...,n_frames-1
    frame_times = (0:n_frames-1) * dt_sec;

    % ---- 逐帧逐飞机误差存储 ----
    ukf_errs  = cell(n_ac, 1);  % UKF滤波后的位置误差 (km)
    det_errs  = cell(n_ac, 1);  % 校准后点迹的位置误差 (km)
    raw_errs  = cell(n_ac, 1);  % 校准前原始点迹误差 (km)
    ukf_lats  = cell(n_ac, 1);  % UKF输出的纬度（供绘图用）
    det_lats  = cell(n_ac, 1);
    raw_lats  = cell(n_ac, 1);

    % ---- 遍历每架飞机 ----
    for a = 1:n_ac
        tt = truthTrajs{a};
        % 初始化误差存储
        ukf_errs{a} = [];  det_errs{a} = [];  raw_errs{a} = [];
        ukf_lats{a} = [];  det_lats{a} = [];  raw_lats{a} = [];

        % ---- 遍历每一帧 ----
        for k = 1:n_frames
            % ---- 真值位置：通过线性插值获取当前帧时刻的位置 ----
            t_true_lat = interp1(tt.time_sec, tt.lat, frame_times(k), 'linear', 'extrap');
            t_true_lon = interp1(tt.time_sec, tt.lon, frame_times(k), 'linear', 'extrap');

            % ---- UKF航迹误差 ----
            % 在所有活跃航迹中找距离真值最近的（最近邻匹配）
            snap = trackSnapshots{k};
            if ~isempty(snap.trackList)
                best_ukf_dist = inf;
                best_ukf_lat = NaN;
                for t = 1:length(snap.trackList)
                    trk = snap.trackList{t};
                    % 跳过历史航迹和无效位置
                    if trk.type == 7 || isnan(trk.lat), continue; end
                    d = haversine_km(trk.lon, trk.lat, t_true_lon, t_true_lat);
                    % 200km上限防止误匹配到其他飞机的航迹
                    if d < best_ukf_dist && d < 200
                        best_ukf_dist = d;
                        best_ukf_lat = trk.lat;
                    end
                end
                if ~isinf(best_ukf_dist)
                    ukf_errs{a}(end+1) = best_ukf_dist;
                    ukf_lats{a}(end+1) = best_ukf_lat;
                end
            end

            % ---- 点迹误差（分校准前后） ----
            dets = detList{k};
            for d = 1:length(dets)
                dp = dets(d);

                % 跳过杂波（不是真实飞机产生的）
                if dp.is_clutter, continue; end

                % 确认点迹属于当前飞机（需要aircraft_id字段）
                if ~isfield(dp, 'aircraft_id') || dp.aircraft_id ~= a, continue; end

                % 校准后点迹误差
                if isfield(dp, 'lat') && ~isnan(dp.lat)
                    d_cal = haversine_km(dp.lon, dp.lat, t_true_lon, t_true_lat);
                    det_errs{a}(end+1) = d_cal;
                    det_lats{a}(end+1) = dp.lat;
                end

                % 校准前原始点迹误差
                if isfield(dp, 'raw_lat') && ~isnan(dp.raw_lat)
                    d_raw = haversine_km(dp.raw_lon, dp.raw_lat, t_true_lon, t_true_lat);
                    raw_errs{a}(end+1) = d_raw;
                    raw_lats{a}(end+1) = dp.raw_lat;
                end
            end
        end
    end

    % ---- 汇总统计：每架飞机单独统计 ----
    summary = struct();
    for a = 1:n_ac
        summary(a).aircraft = a;

        % 计算三类误差的统计指标
        s_ukf = compute_summary(ukf_errs{a});
        s_det = compute_summary(det_errs{a});
        s_raw = compute_summary(raw_errs{a});

        summary(a).ukf = s_ukf;
        summary(a).det_calibrated = s_det;
        summary(a).det_raw = s_raw;

        % ---- 计算UKF相对于检测的改善百分比 ----
        % 改善率 = (1 - median_ukf / median_det) * 100%
        % 正值表示UKF改善了误差（滤波后精度优于直接检测）
        if s_ukf.n > 0 && s_det.n > 0
            summary(a).ukf_vs_det_pct = (1 - s_ukf.median / max(s_det.median, 0.01)) * 100;
        else
            summary(a).ukf_vs_det_pct = 0;
        end
    end

    % ---- 总体统计：所有飞机数据合并 ----
    all_ukf = []; all_det = []; all_raw = [];
    for a = 1:n_ac
        all_ukf = [all_ukf, ukf_errs{a}];
        all_det = [all_det, det_errs{a}];
        all_raw = [all_raw, raw_errs{a}];
    end
    overall.ukf = compute_summary(all_ukf);
    overall.det = compute_summary(all_det);
    overall.raw = compute_summary(all_raw);

    % ---- 打包输出结构体 ----
    errorStats = struct(...
        'radar', radar_label, ...
        'n_frames', n_frames, ...
        'ukf_errors_km', {ukf_errs}, ...
        'det_errors_km', {det_errs}, ...
        'raw_errors_km', {raw_errs}, ...
        'summary', summary, ...
        'overall', overall);
end


% =========================================================================
% 辅助函数: compute_summary
% 计算误差序列的汇总统计指标
%
% 返回字段: n(样本数), median(中位数), mean(均值), std(标准差),
%           rms(均方根), min/max(极值), pct95(95%分位数)
% =========================================================================
function s = compute_summary(errs)
    s.n = length(errs);
    if s.n > 0
        s.median = median(errs);          % 中位数: 对离群值鲁棒的中心趋势
        s.mean   = mean(errs);            % 均值: 算术平均
        s.std    = std(errs);             % 标准差: 误差的散布程度
        s.rms    = sqrt(mean(errs.^2));   % 均方根误差: 综合衡量偏差和散布
        s.min    = min(errs);             % 最小误差
        s.max    = max(errs);             % 最大误差
        s.pct95  = prctile(errs, 95);     % 95%分位数: 排除5%最大离群值后的上界
    else
        s.median = NaN; s.mean = NaN; s.std = NaN; s.rms = NaN;
        s.min = NaN; s.max = NaN; s.pct95 = NaN;
    end
end


% =========================================================================
% 辅助函数: haversine_km
% Haversine公式计算球面两点间大圆距离
%
% 输入: (lon1, lat1), (lon2, lat2) — 两点的经纬度（度）
% 输出: d — 两点间距离（km）
%
% 公式: d = 2 * R * atan2(sqrt(a), sqrt(1-a))
%       a = sin^2(dlat/2) + cos(lat1)*cos(lat2)*sin^2(dlon/2)
%       R = 6371 km
%
% 注: 使用 atan2 而非 arcsin 以获得更好的数值稳定性，
%     尤其在接近对跖点(antipodal)的场景。
% =========================================================================
function d = haversine_km(lon1, lat1, lon2, lat2)
    R = 6371;  % 地球平均半径 (km)
    dlat = deg2rad(lat2 - lat1);
    dlon = deg2rad(lon2 - lon1);
    a = sin(dlat/2)^2 + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * sin(dlon/2)^2;
    % 数值裁剪: 防止浮点误差导致 a 略大于1
    a = max(0, min(1, a));
    d = R * 2 * atan2(sqrt(a), sqrt(1 - a));
end
