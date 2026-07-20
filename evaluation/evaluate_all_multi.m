% =========================================================================
% evaluate_all_multi.m — 评估模块统一调度器（多目标版本）
% =========================================================================
%
% 【功能概述】
%   本文件是整个评估模块的入口调度器。它接收一个 action 字符串，
%   根据 action 的值分发到两个具体的评估函数之一：
%     1. 'tracking_errors' → compute_tracking_errors_multi：
%        计算单站 UKF 跟踪误差、校准点迹误差、原始点迹误差
%     2. 'fusion'          → evaluate_fusion_multi：
%        计算多雷达融合算法的跟踪误差，并与单站基线对比
%
% 【输出数据结构】
%   tracking_errors 返回 errorStats 结构体：
%     .radar        — 雷达标签 ('R1' 或 'R2')
%     .summary(a)   — 第 a 架飞机的误差统计（含 ukf / det_calibrated / det_raw 三种）
%     .overall      — 所有飞机的全局汇总统计
%     每个统计包含：n, median, mean, std, rms, pct95, min, max
%
%   fusion 返回 fusion_eval 结构体：
%     .method_names       — 融合算法名称列表
%     .pair_to_aircraft   — 跨雷达匹配对 → 真值飞机的映射
%     .summary(m,a)       — 第 m 种算法对第 a 架飞机的误差统计
%     .overall(m)         — 第 m 种算法的全局汇总（含 R1_only / R2_only）
%
% =========================================================================

function varargout = evaluate_all_multi(action, varargin)
    % 统一入口：根据 action 字符串分发到具体评估函数
    % action 取值：'tracking_errors'（跟踪误差）或 'fusion'（融合误差）
    % varargin 是可变参数，会原封不动传递给下面的子函数
    % varargout 支持多返回值（虽然目前只返回一个结构体）
    switch action
        case 'tracking_errors'
            % 跟踪误差评估：计算单站 UKF 和检测点迹的 RMSE 统计
            % 调用 compute_tracking_errors_multi 并传递剩余参数
            varargout{1} = compute_tracking_errors_multi(varargin{:});
        case 'fusion'
            % 融合误差评估：对比多种融合算法与单站基线的误差
            % 调用 evaluate_fusion_multi 并传递剩余参数
            varargout{1} = evaluate_fusion_multi(varargin{:});
        otherwise
            % 未知的 action 字符串，抛出错误
            % 错误码格式：evaluate_all_multi:unknownAction
            error('evaluate_all_multi: unknown action ''%s''', action);
    end
end


% =========================================================================
% compute_tracking_errors_multi — 单站跟踪误差统计（多目标版）
% =========================================================================
%
% 【功能】
%   遍历每一帧、每一架飞机，将 UKF 估计位置、校准后点迹、原始点迹
%   分别与真值位置计算 Haversine 球面距离，得到误差序列，然后统计
%   中位数、均值、标准差、RMS、95 百分位数、最小/最大值。
%
% 【关联逻辑】
%   UKF 航迹关联：遍历当前帧所有 UKF 航迹，通过 trk.truth_idx == a
%   找到对应真值飞机的航迹，计算其估计位置与真值位置的距离。
%   检测点迹关联：遍历当前帧所有检测点迹，通过 dp.aircraft_id == a
%   过滤出属于该飞机的点迹，再区分校准点迹（dp.lat）和原始点迹（dp.raw_lat）。
%
% 【输入参数】
%   trackSnapshots — cell 数组，每帧一个 snapshot 结构体，含 trackList
%   detList        — cell 数组，每帧一个检测点迹数组
%   truthTrajs     — cell 数组，每架飞机一条真值轨迹（含 time_sec, lat, lon）
%   snapshot_times — 航迹快照对应的显式时间网格（秒）
%   detection_times — 检测点迹对应的显式时间网格（秒）
%   radar_label    — 雷达标签字符串（'R1' 或 'R2'）
%
% 【输出】
%   errorStats — 包含逐飞机和全局误差统计的结构体
% =========================================================================
function errorStats = compute_tracking_errors_multi(trackSnapshots, detList, truthTrajs, ...
        snapshot_times, detection_times, radar_label)
    % 函数入口：计算单站跟踪误差统计（多目标版）
    % 输入：
    %   trackSnapshots  - cell[k] = 第k帧 UKF 航迹快照（含 trackList）
    %   detList         - cell[k] = 第k帧检测点迹数组
    %   truthTrajs      - cell[a] = 第a架飞机的真值轨迹（含 time_sec, lat, lon）
    %   snapshot_times  - 与 trackSnapshots 等长的时间网格（秒）
    %   detection_times - 与 detList 等长的时间网格（秒）
    %   radar_label     - 雷达标签字符串（'R1' 或 'R2'）
    % 输出：errorStats 结构体，含逐飞机和全局误差统计

    % 计算飞机总数（真值轨迹条数 = 目标架数）
    n_ac = length(truthTrajs);
    % 验证时间网格合法性（长度匹配、严格递增、全有限值）
    snapshot_times = validate_time_grid_eval(snapshot_times, length(trackSnapshots), ...
        'snapshot_times');
    detection_times = validate_time_grid_eval(detection_times, length(detList), ...
        'detection_times');
    % 航迹快照总帧数
    n_frames = length(trackSnapshots);

    % 预分配六组 cell 数组：三组存误差（km），三组存纬度估计值
    % 每组大小为 n_ac×1，对应每架飞机
    ukf_errs  = cell(n_ac, 1);   % 每架飞机的 UKF 误差序列（km）
    det_errs  = cell(n_ac, 1);   % 每架飞机的校准点迹误差序列（km）
    raw_errs  = cell(n_ac, 1);   % 每架飞机的原始点迹误差序列（km）
    ukf_lats  = cell(n_ac, 1);   % 每架飞机的 UKF 估计纬度
    det_lats  = cell(n_ac, 1);   % 每架飞机的校准点迹纬度
    raw_lats  = cell(n_ac, 1);   % 每架飞机的原始点迹纬度

    % ===== 外层循环：遍历每一架飞机 =====
    for a = 1:n_ac
        % 取出第 a 架飞机的真值轨迹结构体
        tt = truthTrajs{a};
        % 清空该飞机的误差和纬度记录（复用 cell 数组）
        ukf_errs{a} = [];  det_errs{a} = [];  raw_errs{a} = [];
        ukf_lats{a} = [];  det_lats{a} = [];  raw_lats{a} = [];

        % ===== UKF 快照：使用快照自身的时间网格 =====
        % 逐帧遍历航迹快照，计算 UKF 估计位置与真值位置的球面距离
        for k = 1:length(trackSnapshots)
            % 当前帧的时间戳（秒）
            tnow = snapshot_times(k);
            % 如果当前时间戳超出真值轨迹的时间范围，跳过
            if tnow < tt.time_sec(1) || tnow > tt.time_sec(end)
                continue;
            end
            % 用线性插值获取当前时刻的真值经纬度
            % 真值轨迹可能不是等间隔采样的，需要插值对齐
            t_true_lat = interp1(tt.time_sec, tt.lat, tnow, 'linear');
            t_true_lon = interp1(tt.time_sec, tt.lon, tnow, 'linear');
            % 如果插值结果为 NaN（如外推越界），跳过
            if isnan(t_true_lat) || isnan(t_true_lon)
                continue;
            end

            % 取出当前帧的 UKF 航迹快照
            snap = trackSnapshots{k};
            % 如果快照为空或没有航迹列表，跳过
            if ~isempty(snap.trackList)
                % 初始化最佳匹配：距离无穷大，纬度 NaN
                best_ukf_dist = inf;
                best_ukf_lat = NaN;
                % 遍历当前帧的所有 UKF 航迹
                for t = 1:length(snap.trackList)
                    trk = snap.trackList{t};
                    % 过滤条件1：type==7 表示非可靠航迹（如临时/历史航迹），不参与评估
                    % 过滤条件2：经度为 NaN 的航迹跳过
                    if trk.type == 7 || isnan(trk.lat), continue; end
                    % 通过 truth_idx 字段将 UKF 航迹与真值飞机关联
                    % truth_idx == a 表示这条航迹跟踪的是第 a 架飞机
                    if isfield(trk, 'truth_idx') && trk.truth_idx == a
                        % 计算 UKF 估计位置与真值位置的 Haversine 球面距离
                        d = haversine_km_eval(trk.lon, trk.lat, t_true_lon, t_true_lat);
                        % 取距离最小的航迹作为最佳匹配（防止一对多）
                        if d < best_ukf_dist
                            best_ukf_dist = d;
                            best_ukf_lat = trk.lat;
                        end
                    end
                end
                % 如果找到了匹配的航迹（距离不是无穷大），记录误差和纬度
                if ~isinf(best_ukf_dist)
                    ukf_errs{a}(end+1) = best_ukf_dist;  % 追加误差值
                    ukf_lats{a}(end+1) = best_ukf_lat;    % 追加估计纬度
                end
            end
        end

        % ===== 检测点迹：使用检测自身的时间网格 =====
        % 逐帧遍历检测点迹，区分校准点迹和原始点迹分别计算误差
        for k = 1:length(detList)
            % 当前帧的时间戳（秒）
            tnow = detection_times(k);
            % 如果当前时间戳超出真值轨迹的时间范围，跳过
            if tnow < tt.time_sec(1) || tnow > tt.time_sec(end)
                continue;
            end
            % 用线性插值获取当前时刻的真值经纬度
            t_true_lat = interp1(tt.time_sec, tt.lat, tnow, 'linear');
            t_true_lon = interp1(tt.time_sec, tt.lon, tnow, 'linear');
            % 如果插值结果为 NaN，跳过
            if isnan(t_true_lat) || isnan(t_true_lon)
                continue;
            end

            % 取出当前帧的所有检测点迹数组
            dets = detList{k};
            % 遍历该帧中的每一个检测点迹
            for d = 1:length(dets)
                dp = dets(d);
                % 过滤条件1：is_clutter=true 表示杂波/虚警，不参与评估
                if dp.is_clutter, continue; end
                % 过滤条件2：通过 aircraft_id 字段将检测点迹与真值飞机关联
                % 只保留属于当前飞机 a 的点迹
                if ~isfield(dp, 'aircraft_id') || dp.aircraft_id ~= a, continue; end
                % 检查是否存在校准后的经纬度字段（dp.lat）
                % 校准点迹是经过偏差校正和坐标变换的检测点
                if isfield(dp, 'lat') && ~isnan(dp.lat)
                    % 计算校准点迹与真值的 Haversine 距离
                    det_errs{a}(end+1) = haversine_km_eval( ...
                        dp.lon, dp.lat, t_true_lon, t_true_lat);
                    % 记录校准点迹的纬度
                    det_lats{a}(end+1) = dp.lat;
                end
                % 检查是否存在原始（未校准）经纬度字段（dp.raw_lat）
                if isfield(dp, 'raw_lat') && ~isnan(dp.raw_lat)
                    % 计算原始点迹与真值的 Haversine 距离
                    raw_errs{a}(end+1) = haversine_km_eval( ...
                        dp.raw_lon, dp.raw_lat, t_true_lon, t_true_lat);
                    % 记录原始点迹的纬度
                    raw_lats{a}(end+1) = dp.raw_lat;
                end
            end
        end
    end

    % ===== 逐飞机汇总统计 =====
    % 初始化 summary 结构体数组，每个元素对应一架飞机的统计
    summary = struct();
    for a = 1:n_ac
        summary(a).aircraft = a;  % 记录飞机编号
        % 计算 UKF、校准点迹、原始点迹各自的统计量
        % compute_summary_eval 返回 n/median/mean/std/rms/pct95/min/max
        s_ukf = compute_summary_eval(ukf_errs{a});
        s_det = compute_summary_eval(det_errs{a});
        s_raw = compute_summary_eval(raw_errs{a});
        summary(a).ukf = s_ukf;
        summary(a).det_calibrated = s_det;
        summary(a).det_raw = s_raw;
        % 计算 UKF 相对于校准点迹的中位数改善百分比
        % 公式：(1 - ukf_median / det_median) * 100
        % 正值表示 UKF 更好（误差更小），负值表示校准点迹更好
        % max(s_det.median, 0.01) 防止除零
        if s_ukf.n > 0 && s_det.n > 0
            summary(a).ukf_vs_det_pct = (1 - s_ukf.median / max(s_det.median, 0.01)) * 100;
        else
            % 数据不足时改善百分比设为 0
            summary(a).ukf_vs_det_pct = 0;
        end
    end

    % ===== 全局汇总：合并所有飞机的误差 =====
    % 将所有飞机的误差序列拼接成一个长向量
    all_ukf = []; all_det = []; all_raw = [];
    for a = 1:n_ac
        all_ukf = [all_ukf, ukf_errs{a}];   % 拼接所有飞机的 UKF 误差
        all_det = [all_det, det_errs{a}];   % 拼接所有飞机的校准点迹误差
        all_raw = [all_raw, raw_errs{a}];   % 拼接所有飞机的原始点迹误差
    end
    % 对全局误差序列计算统计量
    overall.ukf = compute_summary_eval(all_ukf);
    overall.det = compute_summary_eval(all_det);
    overall.raw = compute_summary_eval(all_raw);

    % 组装返回结构体
    errorStats = struct(...
        'radar', radar_label, ...           % 雷达标签（'R1' 或 'R2'）
        'n_frames', n_frames, ...            % 航迹快照总帧数
        'snapshot_times', snapshot_times, ... % 快照时间网格（秒）
        'detection_times', detection_times, ... % 检测时间网格（秒）
        'ukf_errors_km', {ukf_errs}, ...     % 每架飞机的 UKF 误差序列（cell 数组）
        'det_errors_km', {det_errs}, ...     % 每架飞机的校准点迹误差序列
        'raw_errors_km', {raw_errs}, ...     % 每架飞机的原始点迹误差序列
        'summary', summary, ...              % 逐飞机统计（含 ukf/det_raw/改善率）
        'overall', overall);                 % 全局统计（所有飞机合并）
end


function times = validate_time_grid_eval(times, expected_length, name)
    % 验证时间网格：确保长度匹配、严格递增、全有限值
    % 输入：times - 时间向量, expected_length - 期望长度, name - 参数名（用于错误信息）
    % 输出：times - 转置为行向量的 double 数组

    % 强制转为 double 类型，并 reshape 为 1×N 行向量
    times = double(times(:)');
    % 检查长度是否与实际数据帧数一致
    if numel(times) ~= expected_length
        error('evaluate_all_multi:timeGridLength', ...
            '%s 长度必须等于对应数据帧数', name);
    end
    % 检查所有值是否有限（非 NaN/Inf），且严格递增（diff > 0）
    % any(~isfinite(times)) 捕获 NaN 和 Inf
    % any(diff(times) <= 0) 捕获重复或递减的时间戳
    if any(~isfinite(times)) || (numel(times) > 1 && any(diff(times) <= 0))
        error('evaluate_all_multi:invalidTimeGrid', ...
            '%s 必须有限且严格递增', name);
    end
end


% =========================================================================
% evaluate_fusion_multi — 融合误差评估（多目标版）
% =========================================================================
%
% 【功能】
%   对每种融合算法，将融合后的航迹位置与真值位置计算球面距离，
%   得到误差序列。同时计算单站（R1_only、R2_only）作为基线对比。
%
% 【核心步骤】
%   Step 1: 将跨雷达匹配对映射到真值飞机（通过 matcher 预计算或位置猜测）
%   Step 2: 逐帧计算每种融合算法的航迹误差
%   Step 3: 逐帧计算单站基线误差（从 R1/R2 快照中按 ID 提取匹配航迹）
%   Step 4: 汇总所有统计量
%
% 【输入参数】
%   all_fused_snapshots — 融合快照的 cell 数组
%   method_names        — 融合算法名称列表
%   matched_pairs       — 跨雷达匹配对结构体数组
%   trackSnapshots_R1   — R1 的单站航迹快照
%   trackSnapshots_R2   — R2 的单站航迹快照（已对齐）
%   truthTrajs          — 真值轨迹
%   n_frames            — 总帧数
%   dt_sec              — 帧间隔
%   matcher             — 匹配器结构体，含 pair_to_aircraft 等字段
% =========================================================================
function fusion_eval = evaluate_fusion_multi(all_fused_snapshots, method_names, ...
        matched_pairs, trackSnapshots_R1, trackSnapshots_R2, ...
        truthTrajs, n_frames, dt_sec, matcher)
    % 函数入口：融合误差评估（多目标版）
    % 输入：
    %   all_fused_snapshots - cell[n_pairs, n_methods] 融合快照
    %   method_names        - 融合算法名称元胞数组
    %   matched_pairs       - 跨雷达匹配对结构体数组
    %   trackSnapshots_R1   - R1 单站航迹快照
    %   trackSnapshots_R2   - R2 单站航迹快照（已对齐到 R1 时间轴）
    %   truthTrajs          - 真值轨迹
    %   n_frames            - 总帧数
    %   dt_sec              - 帧间隔（秒）
    %   matcher             - 匹配器结构体（含 pair_to_aircraft, aligned_R2 等）
    % 输出：fusion_eval 结构体，含各算法误差统计和单站基线对比

    % 融合算法种类数
    n_methods = length(method_names);
    % 飞机数量
    n_ac = length(truthTrajs);
    % 帧时间戳数组：0, dt_sec, 2*dt_sec, ..., (n_frames-1)*dt_sec
    frame_times = (0:n_frames-1) * dt_sec;

    % ===== 边界情况：没有跨雷达匹配对 =====
    % 此时无法计算融合误差，返回全 NaN 结构体
    if isempty(matched_pairs)
        % 初始化所有误差 cell 数组为空
        fusion_errs = cell(n_methods, n_ac);
        r1_errs = cell(1, n_ac);
        r2_errs = cell(1, n_ac);
        % 嵌套循环初始化所有 cell 为空数组
        for m = 1:n_methods
            for a = 1:n_ac
                fusion_errs{m, a} = [];
            end
        end
        for a = 1:n_ac
            r1_errs{a} = [];
            r2_errs{a} = [];
        end
        % 初始化统计结构体（全 NaN）
        summary = struct();
        for m = 1:n_methods
            for a = 1:n_ac
                summary(m,a).method = method_names{m};
                summary(m,a).aircraft = a;
                summary(m,a).s = compute_err_stats_eval([]);  % 空数组返回全 NaN
            end
        end
        % 初始化全局统计（包含 R1_only 和 R2_only）
        overall = struct();
        all_methods = [method_names, {'R1_only', 'R2_only'}];
        for m = 1:length(all_methods)
            overall(m).method = all_methods{m};
            overall(m).s = compute_err_stats_eval([]);
        end
        % 组装返回结构体
        fusion_eval = struct('method_names', {method_names}, ...
            'pair_to_aircraft', [], ...
            'fusion_errors', {fusion_errs}, ...
            'r1_errors', {r1_errs}, ...
            'r2_errors', {r2_errs}, ...
            'summary', summary, ...
            'overall', overall);
        fprintf('\n无跨雷达匹配对，跳过融合误差评估。\n');
        return;
    end

    % ===== Step 1: 将匹配对映射到真值飞机 =====
    % pair_to_aircraft[p] = 第 p 个匹配对对应的真值飞机编号
    pair_to_aircraft = zeros(length(matched_pairs), 1);

    % 【优先路径】如果 matcher 中已经有预计算的 pair_to_aircraft（多目标场景）
    % 直接使用，避免重新计算（多目标场景下飞机数量多，重新计算成本高）
    if isfield(matcher, 'pair_to_aircraft') && length(matcher.pair_to_aircraft) >= length(matched_pairs)
        % 截取前 N 个匹配对的映射结果（N = matched_pairs 长度）
        pair_to_aircraft = matcher.pair_to_aircraft(1:length(matched_pairs));
    else
        % 【回退路径】单目标场景的位置猜测逻辑
        % 通过 R1 航迹的历史位置与各飞机真值轨迹的平均距离来推测归属
        % 适用于 matcher 中没有预计算 pair_to_aircraft 的情况
        for p = 1:length(matched_pairs)
            mp = matched_pairs(p);
            % 从 matcher 中找到该 R1 航迹的索引
            % 有两种可能的 matcher 格式：unique_r1_ids 或 r1_ids(:,1)
            if isfield(matcher, 'unique_r1_ids')
                r1_idx = find(matcher.unique_r1_ids == mp.R1_track_id, 1);
            else
                r1_idx = find(matcher.r1_ids(:,1) == mp.R1_track_id, 1);
            end
            % 如果找不到对应的 R1 航迹索引，跳过这个匹配对
            if isempty(r1_idx), continue; end
            % 提取该 R1 航迹的历史经度（第1列）和纬度（第2列）
            % squeeze 去除 singleton 维度，' 转置为行向量
            r1_lons = squeeze(matcher.r1_pos(r1_idx, :, 1))';
            r1_lats = squeeze(matcher.r1_pos(r1_idx, :, 2))';
            % 遍历所有飞机，计算 R1 航迹与各飞机真值轨迹的平均距离
            best_ac = 0;   % 最佳匹配飞机编号（0 表示未找到）
            best_dist = inf;  % 最小平均距离
            for a = 1:n_ac
                tt = truthTrajs{a};
                % 用 extrap 外推获取所有帧时间的真值经纬度
                % 'extrap' 允许在时间范围外线性外推（处理边界情况）
                t_lat = interp1(tt.time_sec, tt.lat, frame_times, 'linear', 'extrap');
                t_lon = interp1(tt.time_sec, tt.lon, frame_times, 'linear', 'extrap');
                % 计算 R1 航迹位置与真值轨迹的 Haversine 距离向量，取均值
                mean_dist = nanmean(haversine_km_vec_eval(r1_lons, r1_lats, t_lon, t_lat));
                % 取平均距离最小的飞机作为匹配结果
                if mean_dist < best_dist
                    best_dist = mean_dist;
                    best_ac = a;
                end
            end
            % 记录该匹配对对应的飞机编号
            pair_to_aircraft(p) = best_ac;
        end
    end

    % 打印匹配对到真值飞机的映射关系，方便调试
    % 输出格式：Pair N (R1#xxx <-> R2#yyy) -> 飞机标签 或 未映射
    fprintf('\n匹配对 -> 真值飞机映射:\n');
    for p = 1:length(matched_pairs)
        ac = pair_to_aircraft(p);
        % 如果映射结果不在有效范围内（<1 或 >飞机总数），说明映射失败
        if ac < 1 || ac > length(truthTrajs)
            fprintf('  Pair %d (R1#%d <-> R2#%d) -> 未映射\n', ...
                p, matched_pairs(p).R1_track_id, matched_pairs(p).R2_track_id);
            continue;
        end
        % 打印映射成功的配对信息，含飞机标签
        fprintf('  Pair %d (R1#%d <-> R2#%d) -> 飞机%s\n', ...
            p, matched_pairs(p).R1_track_id, matched_pairs(p).R2_track_id, ...
            truthTrajs{ac}.label);
    end

    % ===== Step 2: 逐帧计算融合航迹误差 =====
    % fusion_errs[m,a] 存储第 m 种算法对第 a 架飞机的误差序列
    fusion_errs = cell(n_methods, n_ac);
    % 融合快照的对数（= matched_pairs 的数量）
    n_pairs = size(all_fused_snapshots, 1);

    % 遍历每种融合算法
    for m = 1:n_methods
        % 初始化该算法对所有飞机的误差为空
        for a = 1:n_ac
            fusion_errs{m, a} = [];
        end
        % 遍历每个融合快照对（即每个跨雷达匹配对）
        for p = 1:n_pairs
            % 取出该匹配对在该算法下的所有帧快照（cell[n_frames, 1]）
            fused_snaps = all_fused_snapshots{p, m};
            % 获取该匹配对对应的真值飞机编号
            ac = 0;
            if p <= length(pair_to_aircraft)
                ac = pair_to_aircraft(p);
            end
            % 如果映射失败（ac==0），跳过该匹配对
            if ac == 0, continue; end
            % 遍历该快照中的所有帧
            for k = 1:n_frames
                % 预先计算所有飞机在当前帧的真值经纬度（避免重复插值）
                % 这样每帧只做一次插值，而不是每架飞机一次
                t_true_lat_all = zeros(n_ac, 1);
                t_true_lon_all = zeros(n_ac, 1);
                for a = 1:n_ac
                    t_true_lat_all(a) = interp1(truthTrajs{a}.time_sec, ...
                        truthTrajs{a}.lat, frame_times(k), 'linear', 'extrap');
                    t_true_lon_all(a) = interp1(truthTrajs{a}.time_sec, ...
                        truthTrajs{a}.lon, frame_times(k), 'linear', 'extrap');
                end
                % 取出当前帧的融合航迹快照
                snap = fused_snaps{k};
                % 如果快照为空或没有航迹，跳过
                if isempty(snap.trackList), continue; end
                % 遍历融合快照中的所有航迹
                for t = 1:length(snap.trackList)
                    ftrk = snap.trackList{t};
                    % 跳过经度为 NaN 的无效航迹
                    if isnan(ftrk.lon), continue; end
                    % 计算融合航迹与对应真值飞机的球面距离
                    % 使用预先计算的真值经纬度（t_true_lat_all(ac), t_true_lon_all(ac)）
                    d = haversine_km_eval(ftrk.lon, ftrk.lat, t_true_lon_all(ac), t_true_lat_all(ac));
                    % 追加到该算法对该飞机的误差序列
                    fusion_errs{m, ac}(end+1) = d;
                end
            end
        end
    end

    % ===== Step 3: 单站基线误差计算 =====
    % 计算 R1_only 和 R2_only 作为融合算法的基线对比
    r1_errs = cell(1, n_ac);
    r2_errs = cell(1, n_ac);
    % 取 R1 快照和已对齐的 R2 快照
    r1_snaps = trackSnapshots_R1;
    r2_snaps_aligned = matcher.aligned_R2;  % R2 已对齐到 R1 时间轴的快照
    % 初始化每架飞机的误差为空
    for a = 1:n_ac
        r1_errs{a} = [];
        r2_errs{a} = [];
    end
    % 逐帧计算单站误差
    for k = 1:n_frames
        % 遍历每架飞机
        for a = 1:n_ac
            % 获取当前帧该飞机的真值经纬度（用 extrap 处理边界）
            t_true_lat = interp1(truthTrajs{a}.time_sec, truthTrajs{a}.lat, ...
                frame_times(k), 'linear', 'extrap');
            t_true_lon = interp1(truthTrajs{a}.time_sec, truthTrajs{a}.lon, ...
                frame_times(k), 'linear', 'extrap');
            % ===== R1 单站误差 =====
            % 取出当前帧的 R1 航迹快照
            snap_r1 = r1_snaps{k};
            if ~isempty(snap_r1.trackList)
                % 遍历匹配对，找到属于当前飞机的 R1 航迹
                for p = 1:length(matched_pairs)
                    % 如果该匹配对对应的飞机是当前飞机
                    if pair_to_aircraft(p) == a
                        % 取出该匹配对中 R1 航迹的 ID
                        r1_id = matched_pairs(p).R1_track_id;
                        % 按 ID 在快照中查找对应的航迹
                        trk1 = find_track_by_id_eval(snap_r1, r1_id);
                        % 如果找到航迹且经度有效，计算与真值的距离
                        if ~isempty(trk1) && ~isnan(trk1.lon)
                            d = haversine_km_eval(trk1.lon, trk1.lat, t_true_lon, t_true_lat);
                            r1_errs{a}(end+1) = d;
                        end
                        % 找到一个就退出（每个飞机在每个匹配对中只有一个 R1 航迹）
                        break;
                    end
                end
            end
            % ===== R2 单站误差 =====
            % 取出当前帧的 R2 对齐快照
            snap_r2 = r2_snaps_aligned{k};
            if ~isempty(snap_r2.trackList)
                % 遍历匹配对，找到属于当前飞机的 R2 航迹
                for p = 1:length(matched_pairs)
                    if pair_to_aircraft(p) == a
                        r2_id = matched_pairs(p).R2_track_id;
                        trk2 = find_track_by_id_eval(snap_r2, r2_id);
                        if ~isempty(trk2) && ~isnan(trk2.lon)
                            d = haversine_km_eval(trk2.lon, trk2.lat, t_true_lon, t_true_lat);
                            r2_errs{a}(end+1) = d;
                        end
                        break;
                    end
                end
            end
        end
    end

    % ===== Step 4: 汇总统计 =====
    % summary(m,a) 存储第 m 种算法对第 a 架飞机的统计
    summary = struct();
    % 融合算法的逐飞机统计（m=1..n_methods）
    for m = 1:n_methods
        for a = 1:n_ac
            summary(m,a).method = method_names{m};  % 算法名称
            summary(m,a).aircraft = a;               % 飞机编号
            % 调用 compute_err_stats_eval 计算该算法对该飞机的误差统计
            summary(m,a).s = compute_err_stats_eval(fusion_errs{m,a});
        end
    end
    % R1_only 的逐飞机统计（m = n_methods+1）
    for a = 1:n_ac
        summary(n_methods+1, a).method = 'R1_only';
        summary(n_methods+1, a).aircraft = a;
        summary(n_methods+1, a).s = compute_err_stats_eval(r1_errs{a});
    end
    % R2_only 的逐飞机统计（m = n_methods+2）
    for a = 1:n_ac
        summary(n_methods+2, a).method = 'R2_only';
        summary(n_methods+2, a).aircraft = a;
        summary(n_methods+2, a).s = compute_err_stats_eval(r2_errs{a});
    end

    % 全局统计（合并所有飞机的误差）
    overall = struct();
    % 所有方法的名称列表（融合算法 + R1_only + R2_only）
    all_methods = [method_names, {'R1_only', 'R2_only'}];
    % 将所有误差 cell 纵向拼接：[fusion_errs; r1_errs; r2_errs]
    % 这样 all_errs[m,a] 对应 all_methods{m} 对飞机 a 的误差序列
    all_errs = [fusion_errs; r1_errs; r2_errs];
    for m = 1:length(all_methods)
        % 合并该算法所有飞机的误差
        combined = [];
        for a = 1:n_ac
            combined = [combined, all_errs{m,a}];  % 拼接第 m 种方法所有飞机的误差
        end
        overall(m).method = all_methods{m};
        overall(m).s = compute_err_stats_eval(combined);
    end

    % 组装返回结构体
    fusion_eval = struct(...
        'method_names', {method_names}, ...   % 融合算法名称列表
        'pair_to_aircraft', pair_to_aircraft, ... % 匹配对 -> 真值飞机映射
        'fusion_errors', {fusion_errs}, ...   % 融合算法误差（cell[n_methods, n_ac]）
        'r1_errors', {r1_errs}, ...           % R1 单站误差（cell[1, n_ac]）
        'r2_errors', {r2_errs}, ...           % R2 单站误差（cell[1, n_ac]）
        'summary', summary, ...               % 逐算法逐飞机统计
        'overall', overall);                  % 全局统计
end


% =========================================================================
% compute_summary_eval — 误差汇总统计
% =========================================================================
% 对一组误差值计算基本统计量：样本数、中位数、均值、标准差、RMS、
% 95 百分位数、最小值、最大值。如果输入为空则全部返回 NaN。
function s = compute_summary_eval(errs)
    % 对一组误差值计算基本统计量
    % 输入：errs - 误差值向量（km）
    % 输出：s - 结构体含 n/median/mean/std/rms/pct95/min/max
    % 如果输入为空则全部返回 NaN

    % 样本数
    s.n = length(errs);
    if s.n > 0
        s.median = median(errs);    % 中位数（对异常值鲁棒，比均值更能代表典型误差）
        s.mean = mean(errs);        % 均值（算术平均）
        s.std = std(errs);          % 标准差（误差波动程度）
        s.rms = sqrt(mean(errs.^2)); % 均方根误差（与 RMSE 等价，对大误差更敏感）
        s.min = min(errs);          % 最小误差
        s.max = max(errs);          % 最大误差
        s.pct95 = prctile(errs, 95); % 95 百分位误差（95% 的误差小于此值）
    else
        % 空数组时全部返回 NaN（下游统计代码需检查 n > 0）
        s.median = NaN; s.mean = NaN; s.std = NaN; s.rms = NaN;
        s.min = NaN; s.max = NaN; s.pct95 = NaN;
    end
end


% =========================================================================
% haversine_km_eval — Haversine 球面距离计算（km）
% =========================================================================
% 输入：两个点的经度、纬度（单位：度）
% 输出：两点间的大圆距离（单位：km）
% 公式：a = sin²(Δlat/2) + cos(lat1)·cos(lat2)·sin²(Δlon/2)
%       d = 2R·atan2(√a, √(1-a))
function d = haversine_km_eval(lon1, lat1, lon2, lat2)
    % Haversine 球面距离计算（km）
    % 输入：lon1, lat1 - 第一个点的经度和纬度（度）
    %       lon2, lat2 - 第二个点的经度和纬度（度）
    % 输出：d - 两点间的大圆距离（km）
    % 公式：a = sin²(Δlat/2) + cos(lat1)·cos(lat2)·sin²(Δlon/2)
    %       d = 2R·atan2(√a, √(1-a))

    % 地球平均半径（km）
    R = 6371;
    % 纬度差转弧度（deg2rad 将角度转换为弧度）
    dlat = deg2rad(lat2 - lat1);
    % 经度差转弧度
    dlon = deg2rad(lon2 - lon1);
    % Haversine 公式核心计算
    % sin²(Δlat/2) 贡献纬度方向距离
    % cos(lat1)·cos(lat2)·sin²(Δlon/2) 贡献经度方向距离（考虑纬度衰减）
    a = sin(dlat/2)^2 + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * sin(dlon/2)^2;
    % 钳制 a 到 [0,1] 防止浮点误差导致 atan2 域外错误
    % 理论上 a ∈ [0,1]，但浮点运算可能产生 1.0000000001 或 -1e-16
    a = max(0, min(1, a));
    % 计算大圆距离（atan2 比 acos 数值稳定性更好）
    d = R * 2 * atan2(sqrt(a), sqrt(1 - a));
end


% =========================================================================
% compute_err_stats_eval — 误差统计（简化版）
% =========================================================================
% 与 compute_summary_eval 几乎相同，但顺序不同（用于 fusion 评估）
function s = compute_err_stats_eval(errs)
    % 误差统计（简化版）
    % 与 compute_summary_eval 几乎相同，但字段顺序不同（用于 fusion 评估）
    % 区别：compute_summary_eval 中 min/max 在 pct95 之前，这里反之
    s.n = length(errs);
    if s.n > 0
        s.median = median(errs);
        s.mean = mean(errs);
        s.std = std(errs);
        s.rms = sqrt(mean(errs.^2));
        s.pct95 = prctile(errs, 95);
        s.min = min(errs);
        s.max = max(errs);
    else
        s.median = NaN; s.mean = NaN; s.std = NaN; s.rms = NaN;
        s.pct95 = NaN; s.min = NaN; s.max = NaN;
    end
end


% =========================================================================
% find_track_by_id_eval — 按航迹 ID 在快照中查找航迹
% =========================================================================
% 遍历快照的 trackList，找到 id == tid 的航迹并返回
% 如果没找到或快照为空，返回空数组 []
function trk = find_track_by_id_eval(snap, tid)
    % 按航迹 ID 在快照中查找航迹
    % 输入：snap - 航迹快照结构体（含 trackList 字段）
    %       tid  - 要查找的航迹 ID
    % 输出：trk - 找到的航迹结构体，未找到则返回空数组 []

    % 初始化返回值为空
    trk = [];
    % 如果快照为空或缺少 trackList 字段，直接返回空
    if isempty(snap) || ~isfield(snap, 'trackList'), return; end
    % 遍历快照中的所有航迹
    for t = 1:length(snap.trackList)
        % 如果航迹 ID 匹配，赋值返回并立即退出
        if snap.trackList{t}.id == tid
            trk = snap.trackList{t};
            return;
        end
    end
end


% =========================================================================
% haversine_km_vec_eval — 向量化 Haversine 距离批量计算
% =========================================================================
% 对多组经纬度坐标批量计算球面距离
% 输入：lon1, lat1, lon2, lat2 为等长向量
% 输出：d_vec 为对应位置的距离向量，含 NaN 输入的位置也返回 NaN
function d_vec = haversine_km_vec_eval(lon1, lat1, lon2, lat2)
    % 向量化 Haversine 距离批量计算
    % 输入：lon1, lat1, lon2, lat2 为等长向量（单位：度）
    % 输出：d_vec 为对应位置的距离向量（km），含 NaN 输入的位置也返回 NaN

    % 预分配输出向量（与输入等长）
    d_vec = zeros(size(lon1));
    % 逐个元素计算（for 循环替代向量化，因为 NaN 传播需要逐元素判断）
    for i = 1:length(lon1)
        % 如果任一输入为 NaN，对应输出也设为 NaN（传播 NaN）
        if isnan(lon1(i)) || isnan(lat1(i)) || isnan(lon2(i)) || isnan(lat2(i))
            d_vec(i) = NaN;
        else
            % 调用标量版本计算单个距离
            d_vec(i) = haversine_km_eval(lon1(i), lat1(i), lon2(i), lat2(i));
        end
    end
end
