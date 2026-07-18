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
    % varargin 是可变参数，会原封不动传递给下面的子函数
    switch action
        case 'tracking_errors'
            % 跟踪误差评估：计算单站 UKF 和检测点迹的 RMSE 统计
            varargout{1} = compute_tracking_errors_multi(varargin{:});
        case 'fusion'
            % 融合误差评估：对比多种融合算法与单站基线的误差
            varargout{1} = evaluate_fusion_multi(varargin{:});
        otherwise
            % 未知的 action 字符串，抛出错误
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
%   n_frames       — 仿真总帧数
%   dt_sec         — 帧间隔（秒）
%   radar_label    — 雷达标签字符串（'R1' 或 'R2'）
%
% 【输出】
%   errorStats — 包含逐飞机和全局误差统计的结构体
% =========================================================================
function errorStats = compute_tracking_errors_multi(trackSnapshots, detList, truthTrajs, ...
        n_frames, dt_sec, radar_label)
    % 获取飞机数量（真值轨迹条数）
    n_ac = length(truthTrajs);
    % 生成所有帧的时间戳数组 [0, dt, 2*dt, ..., (n_frames-1)*dt]
    frame_times = (0:n_frames-1) * dt_sec;

    % 预分配六组 cell 数组：三组存误差（km），三组存纬度
    ukf_errs  = cell(n_ac, 1);   % 每架飞机的 UKF 误差序列
    det_errs  = cell(n_ac, 1);   % 每架飞机的校准点迹误差序列
    raw_errs  = cell(n_ac, 1);   % 每架飞机的原始点迹误差序列
    ukf_lats  = cell(n_ac, 1);   % 每架飞机的 UKF 估计纬度
    det_lats  = cell(n_ac, 1);   % 每架飞机的校准点迹纬度
    raw_lats  = cell(n_ac, 1);   % 每架飞机的原始点迹纬度

    % ===== 外层循环：遍历每一架飞机 =====
    for a = 1:n_ac
        % 取出第 a 架飞机的真值轨迹
        tt = truthTrajs{a};
        % 清空该飞机的误差和纬度记录
        ukf_errs{a} = [];  det_errs{a} = [];  raw_errs{a} = [];
        ukf_lats{a} = [];  det_lats{a} = [];  raw_lats{a} = [];

        % ===== 内层循环：遍历每一帧 =====
        for k = 1:n_frames
            % 当前帧对应的仿真时刻
            tnow = frame_times(k);
            % 如果当前时刻超出真值轨迹的时间范围，跳过（真值已结束或未开始）
            if tnow < tt.time_sec(1) || tnow > tt.time_sec(end)
                continue;
            end
            % 用线性插值从真值轨迹中获取当前时刻的真值经纬度
            t_true_lat = interp1(tt.time_sec, tt.lat, tnow, 'linear');
            t_true_lon = interp1(tt.time_sec, tt.lon, tnow, 'linear');
            % 如果插值结果为 NaN（可能真值数据有间断），跳过
            if isnan(t_true_lat) || isnan(t_true_lon)
                continue;
            end

            % 取出当前帧的航迹快照
            snap = trackSnapshots{k};
            % ===== UKF 误差计算 =====
            if ~isempty(snap.trackList)
                % 初始化：最近距离设为无穷大，最佳航迹纬度暂为空
                best_ukf_dist = inf;
                best_ukf_lat = NaN;
                % 遍历当前帧的所有 UKF 航迹
                for t = 1:length(snap.trackList)
                    trk = snap.trackList{t};
                    % 跳过非 UKF 类型的航迹（type==7 表示 HISTORY 航迹）
                    % 以及纬度为 NaN 的无效航迹
                    if trk.type == 7 || isnan(trk.lat), continue; end
                    % 【关键关联逻辑】优先使用 truth_idx 字段进行航迹-飞机配对
                    % 这是 oracle 模式下最可靠的关联方式，避免用位置猜对的歧义
                    if isfield(trk, 'truth_idx') && trk.truth_idx == a
                        % 计算该航迹估计位置与真值位置的球面距离
                        d = haversine_km_eval(trk.lon, trk.lat, t_true_lon, t_true_lat);
                        % 取距离最小的航迹作为该帧的匹配结果
                        if d < best_ukf_dist
                            best_ukf_dist = d;
                            best_ukf_lat = trk.lat;
                        end
                    end
                end
                % 如果找到了匹配的 UKF 航迹（距离不是无穷大），记录误差
                if ~isinf(best_ukf_dist)
                    ukf_errs{a}(end+1) = best_ukf_dist;
                    ukf_lats{a}(end+1) = best_ukf_lat;
                end
            end

            % ===== 检测点迹误差计算 =====
            dets = detList{k};
            % 遍历当前帧的所有检测点迹
            for d = 1:length(dets)
                dp = dets(d);
                % 跳过杂波（clutter）
                if dp.is_clutter, continue; end
                % 只处理属于当前飞机的点迹（aircraft_id 字段标记归属）
                if ~isfield(dp, 'aircraft_id') || dp.aircraft_id ~= a, continue; end
                % 【校准点迹】如果存在校准后的经纬度字段
                if isfield(dp, 'lat') && ~isnan(dp.lat)
                    % 计算校准点迹与真值的球面距离
                    d_cal = haversine_km_eval(dp.lon, dp.lat, t_true_lon, t_true_lat);
                    det_errs{a}(end+1) = d_cal;
                    det_lats{a}(end+1) = dp.lat;
                end
                % 【原始点迹】如果存在原始（未校准）经纬度字段
                if isfield(dp, 'raw_lat') && ~isnan(dp.raw_lat)
                    % 计算原始点迹与真值的球面距离
                    d_raw = haversine_km_eval(dp.raw_lon, dp.raw_lat, t_true_lon, t_true_lat);
                    raw_errs{a}(end+1) = d_raw;
                    raw_lats{a}(end+1) = dp.raw_lat;
                end
            end
        end
    end

    % ===== 逐飞机汇总统计 =====
    summary = struct();
    for a = 1:n_ac
        summary(a).aircraft = a;
        % 计算 UKF、校准点迹、原始点迹各自的统计量
        s_ukf = compute_summary_eval(ukf_errs{a});
        s_det = compute_summary_eval(det_errs{a});
        s_raw = compute_summary_eval(raw_errs{a});
        summary(a).ukf = s_ukf;
        summary(a).det_calibrated = s_det;
        summary(a).det_raw = s_raw;
        % 计算 UKF 相对于校准点迹的中位数改善百分比
        % 正值表示 UKF 更好（误差更小），负值表示校准点迹更好
        if s_ukf.n > 0 && s_det.n > 0
            summary(a).ukf_vs_det_pct = (1 - s_ukf.median / max(s_det.median, 0.01)) * 100;
        else
            summary(a).ukf_vs_det_pct = 0;
        end
    end

    % ===== 全局汇总：合并所有飞机的误差 =====
    all_ukf = []; all_det = []; all_raw = [];
    for a = 1:n_ac
        all_ukf = [all_ukf, ukf_errs{a}];
        all_det = [all_det, det_errs{a}];
        all_raw = [all_raw, raw_errs{a}];
    end
    overall.ukf = compute_summary_eval(all_ukf);
    overall.det = compute_summary_eval(all_det);
    overall.raw = compute_summary_eval(all_raw);

    % 组装返回结构体
    errorStats = struct(...
        'radar', radar_label, ...           % 雷达标签
        'n_frames', n_frames, ...            % 总帧数
        'ukf_errors_km', {ukf_errs}, ...     % 每架飞机的 UKF 误差序列
        'det_errors_km', {det_errs}, ...     % 每架飞机的校准点迹误差序列
        'raw_errors_km', {raw_errs}, ...     % 每架飞机的原始点迹误差序列
        'summary', summary, ...              % 逐飞机统计
        'overall', overall);                 % 全局统计
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
    n_methods = length(method_names);   % 融合算法种类数
    n_ac = length(truthTrajs);          % 飞机数量
    frame_times = (0:n_frames-1) * dt_sec;  % 帧时间戳数组

    % ===== 边界情况：没有跨雷达匹配对 =====
    if isempty(matched_pairs)
        % 初始化所有误差 cell 数组为空
        fusion_errs = cell(n_methods, n_ac);
        r1_errs = cell(1, n_ac);
        r2_errs = cell(1, n_ac);
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
                summary(m,a).s = compute_err_stats_eval([]);
            end
        end
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
    pair_to_aircraft = zeros(length(matched_pairs), 1);

    % 【优先路径】如果 matcher 中已经有预计算的 pair_to_aircraft（多目标场景）
    % 直接使用，避免重新计算
    if isfield(matcher, 'pair_to_aircraft') && length(matcher.pair_to_aircraft) >= length(matched_pairs)
        pair_to_aircraft = matcher.pair_to_aircraft(1:length(matched_pairs));
    else
        % 【回退路径】单目标场景的位置猜测逻辑
        % 通过 R1 航迹的历史位置与各飞机真值轨迹的平均距离来推测归属
        for p = 1:length(matched_pairs)
            mp = matched_pairs(p);
            % 从 matcher 中找到该 R1 航迹的索引
            if isfield(matcher, 'unique_r1_ids')
                r1_idx = find(matcher.unique_r1_ids == mp.R1_track_id, 1);
            else
                r1_idx = find(matcher.r1_ids(:,1) == mp.R1_track_id, 1);
            end
            if isempty(r1_idx), continue; end
            % 提取该 R1 航迹的历史经纬度
            r1_lons = squeeze(matcher.r1_pos(r1_idx, :, 1))';
            r1_lats = squeeze(matcher.r1_pos(r1_idx, :, 2))';
            % 遍历所有飞机，计算 R1 航迹与各飞机真值轨迹的平均距离
            best_ac = 0;
            best_dist = inf;
            for a = 1:n_ac
                tt = truthTrajs{a};
                % 用 extrap 外推获取所有帧时间的真值经纬度
                t_lat = interp1(tt.time_sec, tt.lat, frame_times, 'linear', 'extrap');
                t_lon = interp1(tt.time_sec, tt.lon, frame_times, 'linear', 'extrap');
                % 计算平均球面距离
                mean_dist = nanmean(haversine_km_vec_eval(r1_lons, r1_lats, t_lon, t_lat));
                % 取平均距离最小的飞机作为匹配结果
                if mean_dist < best_dist
                    best_dist = mean_dist;
                    best_ac = a;
                end
            end
            pair_to_aircraft(p) = best_ac;
        end
    end

    % 打印匹配对到真值飞机的映射关系，方便调试
    fprintf('\n匹配对 -> 真值飞机映射:\n');
    for p = 1:length(matched_pairs)
        ac = pair_to_aircraft(p);
        % 如果映射结果不在有效范围内，说明映射失败
        if ac < 1 || ac > length(truthTrajs)
            fprintf('  Pair %d (R1#%d <-> R2#%d) -> 未映射\n', ...
                p, matched_pairs(p).R1_track_id, matched_pairs(p).R2_track_id);
            continue;
        end
        fprintf('  Pair %d (R1#%d <-> R2#%d) -> 飞机%s\n', ...
            p, matched_pairs(p).R1_track_id, matched_pairs(p).R2_track_id, ...
            truthTrajs{ac}.label);
    end

    % ===== Step 2: 逐帧计算融合航迹误差 =====
    fusion_errs = cell(n_methods, n_ac);
    n_pairs = size(all_fused_snapshots, 1);  % 融合快照的对数

    for m = 1:n_methods
        for a = 1:n_ac
            fusion_errs{m, a} = [];
        end
        % 遍历每个融合快照对
        for p = 1:n_pairs
            fused_snaps = all_fused_snapshots{p, m};  % 该对在该算法下的所有帧快照
            ac = 0;
            if p <= length(pair_to_aircraft)
                ac = pair_to_aircraft(p);  % 获取对应的真值飞机编号
            end
            if ac == 0, continue; end  % 映射失败则跳过
            % 遍历该快照中的所有帧
            for k = 1:n_frames
                % 预先计算所有飞机在当前帧的真值经纬度（避免重复插值）
                t_true_lat_all = zeros(n_ac, 1);
                t_true_lon_all = zeros(n_ac, 1);
                for a = 1:n_ac
                    t_true_lat_all(a) = interp1(truthTrajs{a}.time_sec, ...
                        truthTrajs{a}.lat, frame_times(k), 'linear', 'extrap');
                    t_true_lon_all(a) = interp1(truthTrajs{a}.time_sec, ...
                        truthTrajs{a}.lon, frame_times(k), 'linear', 'extrap');
                end
                snap = fused_snaps{k};  % 当前帧的融合航迹快照
                if isempty(snap.trackList), continue; end
                % 遍历融合快照中的所有航迹
                for t = 1:length(snap.trackList)
                    ftrk = snap.trackList{t};
                    if isnan(ftrk.lon), continue; end  % 跳过无效经度
                    % 计算融合航迹与对应真值飞机的球面距离
                    d = haversine_km_eval(ftrk.lon, ftrk.lat, t_true_lon_all(ac), t_true_lat_all(ac));
                    fusion_errs{m, ac}(end+1) = d;
                end
            end
        end
    end

    % ===== Step 3: 单站基线误差计算 =====
    r1_errs = cell(1, n_ac);
    r2_errs = cell(1, n_ac);
    r1_snaps = trackSnapshots_R1;
    r2_snaps_aligned = matcher.aligned_R2;  % R2 已对齐到 R1 时间轴的快照
    for a = 1:n_ac
        r1_errs{a} = [];
        r2_errs{a} = [];
    end
    % 逐帧计算单站误差
    for k = 1:n_frames
        for a = 1:n_ac
            % 获取当前帧该飞机的真值经纬度
            t_true_lat = interp1(truthTrajs{a}.time_sec, truthTrajs{a}.lat, ...
                frame_times(k), 'linear', 'extrap');
            t_true_lon = interp1(truthTrajs{a}.time_sec, truthTrajs{a}.lon, ...
                frame_times(k), 'linear', 'extrap');
            % ===== R1 单站误差 =====
            snap_r1 = r1_snaps{k};
            if ~isempty(snap_r1.trackList)
                % 遍历匹配对，找到属于当前飞机的 R1 航迹
                for p = 1:length(matched_pairs)
                    if pair_to_aircraft(p) == a
                        r1_id = matched_pairs(p).R1_track_id;
                        trk1 = find_track_by_id_eval(snap_r1, r1_id);  % 按 ID 查找
                        if ~isempty(trk1) && ~isnan(trk1.lon)
                            d = haversine_km_eval(trk1.lon, trk1.lat, t_true_lon, t_true_lat);
                            r1_errs{a}(end+1) = d;
                        end
                        break;  % 找到一个就退出（每个飞机在每个匹配对中只有一个 R1 航迹）
                    end
                end
            end
            % ===== R2 单站误差 =====
            snap_r2 = r2_snaps_aligned{k};
            if ~isempty(snap_r2.trackList)
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
    summary = struct();
    % 融合算法的逐飞机统计
    for m = 1:n_methods
        for a = 1:n_ac
            summary(m,a).method = method_names{m};
            summary(m,a).aircraft = a;
            summary(m,a).s = compute_err_stats_eval(fusion_errs{m,a});
        end
    end
    % R1_only 的逐飞机统计
    for a = 1:n_ac
        summary(n_methods+1, a).method = 'R1_only';
        summary(n_methods+1, a).aircraft = a;
        summary(n_methods+1, a).s = compute_err_stats_eval(r1_errs{a});
    end
    % R2_only 的逐飞机统计
    for a = 1:n_ac
        summary(n_methods+2, a).method = 'R2_only';
        summary(n_methods+2, a).aircraft = a;
        summary(n_methods+2, a).s = compute_err_stats_eval(r2_errs{a});
    end

    % 全局统计（合并所有飞机的误差）
    overall = struct();
    all_methods = [method_names, {'R1_only', 'R2_only'}];
    all_errs = [fusion_errs; r1_errs; r2_errs];  % 拼接所有误差 cell
    for m = 1:length(all_methods)
        combined = [];
        for a = 1:n_ac
            combined = [combined, all_errs{m,a}];  % 合并该算法所有飞机的误差
        end
        overall(m).method = all_methods{m};
        overall(m).s = compute_err_stats_eval(combined);
    end

    % 组装返回结构体
    fusion_eval = struct(...
        'method_names', {method_names}, ...
        'pair_to_aircraft', pair_to_aircraft, ...
        'fusion_errors', {fusion_errs}, ...
        'r1_errors', {r1_errs}, ...
        'r2_errors', {r2_errs}, ...
        'summary', summary, ...
        'overall', overall);
end


% =========================================================================
% compute_summary_eval — 误差汇总统计
% =========================================================================
% 对一组误差值计算基本统计量：样本数、中位数、均值、标准差、RMS、
% 95 百分位数、最小值、最大值。如果输入为空则全部返回 NaN。
function s = compute_summary_eval(errs)
    s.n = length(errs);
    if s.n > 0
        s.median = median(errs);    % 中位数（对异常值鲁棒）
        s.mean = mean(errs);        % 均值
        s.std = std(errs);          % 标准差
        s.rms = sqrt(mean(errs.^2)); % 均方根误差（与 RMSE 等价）
        s.min = min(errs);          % 最小误差
        s.max = max(errs);          % 最大误差
        s.pct95 = prctile(errs, 95); % 95 百分位误差
    else
        % 空数组时全部返回 NaN
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
    R = 6371;  % 地球半径（km）
    dlat = deg2rad(lat2 - lat1);  % 纬度差转弧度
    dlon = deg2rad(lon2 - lon1);  % 经度差转弧度
    % Haversine 公式核心计算
    a = sin(dlat/2)^2 + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * sin(dlon/2)^2;
    % 钳制 a 到 [0,1] 防止浮点误差导致 acos 域外错误
    a = max(0, min(1, a));
    % 计算大圆距离
    d = R * 2 * atan2(sqrt(a), sqrt(1 - a));
end


% =========================================================================
% compute_err_stats_eval — 误差统计（简化版）
% =========================================================================
% 与 compute_summary_eval 几乎相同，但顺序不同（用于 fusion 评估）
function s = compute_err_stats_eval(errs)
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
    trk = [];
    if isempty(snap) || ~isfield(snap, 'trackList'), return; end
    for t = 1:length(snap.trackList)
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
    d_vec = zeros(size(lon1));  % 预分配输出向量
    for i = 1:length(lon1)
        % 如果任一输入为 NaN，对应输出也设为 NaN
        if isnan(lon1(i)) || isnan(lat1(i)) || isnan(lon2(i)) || isnan(lat2(i))
            d_vec(i) = NaN;
        else
            % 调用标量版本计算
            d_vec(i) = haversine_km_eval(lon1(i), lat1(i), lon2(i), lat2(i));
        end
    end
end
