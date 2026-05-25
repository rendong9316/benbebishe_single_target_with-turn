% =========================================================================
% evaluate_fusion.m
% =========================================================================
% 【功能概述】
%   融合航迹误差评估函数。对比多种融合算法的航迹精度，并与单站
%   UKF跟踪结果对比。实现以下评估流程：
%     1. 将匹配对映射到真实飞机身份
%     2. 逐帧计算每种融合方法的航迹位置误差
%     3. 逐帧计算各单站（R1, R2）的航迹位置误差
%     4. 汇总统计并输出对比结果
%
% 【数学原理】
%   1. 匹配对-真值映射:
%      对于每个R1-R2航迹匹配对，收集R1航迹所有帧的位置，计算与
%      每条真实飞机轨迹的平均Haversine距离。选择平均距离最小的
%      飞机作为该匹配对对应的真实目标。
%
%   2. 融合航迹编号约定:
%      融合航迹的 id 字段直接对应 matched_pairs 中的配对索引。
%      因此 ftrk.id = p_idx 意味着该融合航迹对应第 p_idx 个匹配对。
%
%   3. 时序对齐:
%      R2航迹的帧时间可能与R1不同（不同雷达可能有不同的观测时刻）。
%      matcher.aligned_R2 已将R2航迹时间对齐到R1的帧时间轴，确保
%      逐帧比较时时间基准一致。
%
%   4. 误差统计:
%      与 compute_tracking_errors 使用相同的统计指标:
%      median, mean, std, rms, pct95, min, max。
%
% 【输入参数】
%   all_fused_snapshots - cell数组，每个元素是一个融合方法的航迹快照
%                         all_fused_snapshots{m}{k} = 第m个方法第k帧的快照
%   method_names        - cell数组，融合方法名称（如 'CI','IMM','WLS','KF'）
%   matched_pairs       - 结构体数组，R1-R2航迹匹配对，每对含:
%                         .R1_track_id, .R2_track_id
%   trackSnapshots_R1   - cell数组，R1雷达的单站航迹快照
%   trackSnapshots_R2   - cell数组，R2雷达的单站航迹快照（未对齐的原始）
%   truthTrajs          - 结构体数组，真实飞机轨迹
%   n_frames            - 总帧数
%   dt_sec              - 帧时间间隔（秒）
%   matcher             - 结构体，航迹匹配器，含:
%                         .r1_ids: R1航迹ID列表
%                         .r1_pos: R1航迹位置历史 (id x frame x [lon,lat])
%                         .aligned_R2: R2航迹对齐后的快照
%
% 【输出】
%   fusion_eval - 结构体，包含:
%                 .method_names: 融合方法名称列表
%                 .pair_to_aircraft: 匹配对→飞机ID映射
%                 .fusion_errors: cell数组，{method, aircraft}的误差序列
%                 .r1_errors: R1单站误差
%                 .r2_errors: R2单站误差
%                 .summary: 各方法×各飞机的汇总统计
%                 .overall: 各方法的总体统计（所有飞机合并）
%
% 【调用关系】
%   被主评估脚本调用
%   子调用: haversine_km, haversine_km_vec, compute_err_stats,
%           find_track_by_id, interp1
% =========================================================================

function fusion_eval = evaluate_fusion(all_fused_snapshots, method_names, ...
        matched_pairs, trackSnapshots_R1, trackSnapshots_R2, ...
        truthTrajs, n_frames, dt_sec, matcher)

    n_methods = length(method_names);  % 融合方法数量
    n_ac = length(truthTrajs);         % 真实飞机数量
    frame_times = (0:n_frames-1) * dt_sec;

    % =====================================================================
    % 第1步: 将R1-R2匹配对映射到真实飞机
    % =====================================================================
    % 对于每个匹配对，通过R1航迹所有帧的轨迹位置与真值比较，
    % 确定该匹配对对应的是哪架飞机。
    % =====================================================================
    pair_to_aircraft = zeros(length(matched_pairs), 1);
    for p = 1:length(matched_pairs)
        mp = matched_pairs(p);

        % 找到R1航迹在matcher中的索引
        r1_idx = find(matcher.r1_ids == mp.R1_track_id, 1);
        if isempty(r1_idx), continue; end

        % 提取R1航迹所有帧的位置（lon, lat）
        r1_lons = squeeze(matcher.r1_pos(r1_idx, :, 1))';
        r1_lats = squeeze(matcher.r1_pos(r1_idx, :, 2))';

        % 与每条真值飞机比较，选平均距离最小的
        best_ac = 0;
        best_dist = inf;
        for a = 1:n_ac
            tt = truthTrajs{a};
            t_lat = interp1(tt.time_sec, tt.lat, frame_times, 'linear', 'extrap');
            t_lon = interp1(tt.time_sec, tt.lon, frame_times, 'linear', 'extrap');
            mean_dist = nanmean(haversine_km_vec(r1_lons, r1_lats, t_lon, t_lat));
            if mean_dist < best_dist
                best_dist = mean_dist;
                best_ac = a;
            end
        end
        pair_to_aircraft(p) = best_ac;
    end

    % 打印匹配对→飞机映射表（便于人工核对）
    fprintf('\n匹配对 -> 真值飞机映射:\n');
    for p = 1:length(matched_pairs)
        fprintf('  Pair %d (R1#%d <-> R2#%d) -> 飞机%s\n', ...
            p, matched_pairs(p).R1_track_id, matched_pairs(p).R2_track_id, ...
            truthTrajs{pair_to_aircraft(p)}.label);
    end

    % =====================================================================
    % 第2步: 逐帧计算融合航迹误差
    % =====================================================================
    % fusion_errs{m, a} = 第m个融合方法对第a架飞机的误差序列
    % =====================================================================
    fusion_errs = cell(n_methods, n_ac);  % {method, aircraft}

    for m = 1:n_methods
        fused_snaps = all_fused_snapshots{m};

        % 初始化每个飞机的误差数组为空
        for a = 1:n_ac
            fusion_errs{m, a} = [];
        end

        % 逐帧计算
        for k = 1:n_frames
            % ---- 计算当前帧所有飞机的真值位置 ----
            t_true_lat_all = zeros(n_ac, 1);
            t_true_lon_all = zeros(n_ac, 1);
            for a = 1:n_ac
                t_true_lat_all(a) = interp1(truthTrajs{a}.time_sec, ...
                    truthTrajs{a}.lat, frame_times(k), 'linear', 'extrap');
                t_true_lon_all(a) = interp1(truthTrajs{a}.time_sec, ...
                    truthTrajs{a}.lon, frame_times(k), 'linear', 'extrap');
            end

            snap = fused_snaps{k};
            if isempty(snap.trackList), continue; end  % 当前帧无融合航迹

            % 遍历融合航迹
            for t = 1:length(snap.trackList)
                ftrk = snap.trackList{t};
                p_idx = ftrk.id;  % 融合航迹id = 匹配对索引

                % 检查索引有效性
                if p_idx < 1 || p_idx > length(pair_to_aircraft), continue; end
                ac = pair_to_aircraft(p_idx);  % 获取对应的真实飞机
                if ac == 0, continue; end       % 无法映射的跳过

                if isnan(ftrk.lon), continue; end  % 位置无效

                % 计算Haversine误差
                d = haversine_km(ftrk.lon, ftrk.lat, t_true_lon_all(ac), t_true_lat_all(ac));
                fusion_errs{m, ac}(end+1) = d;
            end
        end
    end

    % =====================================================================
    % 第3步: 计算单站航迹误差（用于对比基准）
    % =====================================================================
    r1_errs = cell(1, n_ac);  % R1雷达单站误差
    r2_errs = cell(1, n_ac);  % R2雷达单站误差

    r1_snaps = trackSnapshots_R1;
    r2_snaps_aligned = matcher.aligned_R2;  % 使用已对齐的R2快照

    for a = 1:n_ac
        r1_errs{a} = [];
        r2_errs{a} = [];
    end

    for k = 1:n_frames
        for a = 1:n_ac
            % 真值位置
            t_true_lat = interp1(truthTrajs{a}.time_sec, truthTrajs{a}.lat, ...
                frame_times(k), 'linear', 'extrap');
            t_true_lon = interp1(truthTrajs{a}.time_sec, truthTrajs{a}.lon, ...
                frame_times(k), 'linear', 'extrap');

            % ---- R1单站误差 ----
            % 通过匹配对关系找到该飞机对应的R1航迹
            snap_r1 = r1_snaps{k};
            if ~isempty(snap_r1.trackList)
                for p = 1:length(matched_pairs)
                    if pair_to_aircraft(p) == a
                        r1_id = matched_pairs(p).R1_track_id;
                        trk1 = find_track_by_id(snap_r1, r1_id);
                        if ~isempty(trk1) && ~isnan(trk1.lon)
                            d = haversine_km(trk1.lon, trk1.lat, t_true_lon, t_true_lat);
                            r1_errs{a}(end+1) = d;
                        end
                        break;  % 找到对应的R1航迹，退出配对遍历
                    end
                end
            end

            % ---- R2单站误差（已时间对齐） ----
            snap_r2 = r2_snaps_aligned{k};
            if ~isempty(snap_r2.trackList)
                for p = 1:length(matched_pairs)
                    if pair_to_aircraft(p) == a
                        r2_id = matched_pairs(p).R2_track_id;
                        trk2 = find_track_by_id(snap_r2, r2_id);
                        if ~isempty(trk2) && ~isnan(trk2.lon)
                            d = haversine_km(trk2.lon, trk2.lat, t_true_lon, t_true_lat);
                            r2_errs{a}(end+1) = d;
                        end
                        break;
                    end
                end
            end
        end
    end

    % =====================================================================
    % 第4步: 汇总统计
    % =====================================================================
    summary = struct();

    % 各融合方法 × 各飞机的统计
    for m = 1:n_methods
        for a = 1:n_ac
            summary(m,a).method = method_names{m};
            summary(m,a).aircraft = a;
            summary(m,a).s = compute_err_stats(fusion_errs{m,a});
        end
    end

    % 单站基线统计（追加在summary末尾）
    % R1单站: 第 n_methods+1 行
    for a = 1:n_ac
        summary(n_methods+1, a).method = 'R1_only';
        summary(n_methods+1, a).aircraft = a;
        summary(n_methods+1, a).s = compute_err_stats(r1_errs{a});
    end
    % R2单站: 第 n_methods+2 行
    for a = 1:n_ac
        summary(n_methods+2, a).method = 'R2_only';
        summary(n_methods+2, a).aircraft = a;
        summary(n_methods+2, a).s = compute_err_stats(r2_errs{a});
    end

    % ---- 总体统计（所有飞机数据合并） ----
    overall = struct();
    all_methods = [method_names, {'R1_only', 'R2_only'}];
    all_errs = [fusion_errs; r1_errs; r2_errs];  % 垂直拼接
    for m = 1:length(all_methods)
        combined = [];
        for a = 1:n_ac
            combined = [combined, all_errs{m,a}];
        end
        overall(m).method = all_methods{m};
        overall(m).s = compute_err_stats(combined);
    end

    % ---- 打包输出结构体 ----
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
% 辅助函数: compute_err_stats
% 计算误差序列的汇总统计指标
%
% 返回 7 个关键统计量:
%   n: 样本数 (Number of samples)
%   median: 中位数 (Median) — 鲁棒中心趋势
%   mean: 均值 (Mean) — 算术平均
%   std: 标准差 (Standard Deviation) — 散布度
%   rms: 均方根误差 (Root Mean Square) — √(mean(err^2))
%   pct95: 第95百分位数 (95th Percentile) — 排除极端离群值后的上界
%   min/max: 极值 (Extrema)
% =========================================================================
function s = compute_err_stats(errs)
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
% 辅助函数: find_track_by_id
% 在航迹快照中按ID查找指定航迹
% =========================================================================
function trk = find_track_by_id(snap, tid)
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
% 辅助函数: haversine_km
% Haversine公式计算球面两点间大圆距离（km）
% 注意: atan2 比 arcsin 在接近对跖点时数值更稳定
% =========================================================================
function d = haversine_km(lon1, lat1, lon2, lat2)
    R = 6371;  % 地球平均半径 (km)
    dlat = deg2rad(lat2 - lat1);
    dlon = deg2rad(lon2 - lon1);
    a = sin(dlat/2)^2 + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * sin(dlon/2)^2;
    a = max(0, min(1, a));  % 裁剪到 [0,1] 防止浮点误差
    d = R * 2 * atan2(sqrt(a), sqrt(1 - a));
end


% =========================================================================
% 辅助函数: haversine_km_vec
% Haversine距离的向量化版本
% 对两组等长经纬度向量逐对计算距离，NaN值保持为NaN
% =========================================================================
function d_vec = haversine_km_vec(lon1, lat1, lon2, lat2)
    d_vec = zeros(size(lon1));
    for i = 1:length(lon1)
        if isnan(lon1(i)) || isnan(lat1(i)) || isnan(lon2(i)) || isnan(lat2(i))
            d_vec(i) = NaN;  % 保留NaN，不影响后续 nanmean 统计
        else
            d_vec(i) = haversine_km(lon1(i), lat1(i), lon2(i), lat2(i));
        end
    end
end
