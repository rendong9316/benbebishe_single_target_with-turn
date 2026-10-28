% =========================================================================
% track_matcher_dualgate.m — 三判据航迹关联算法（兼容原双门限）
% =========================================================================
% 算法来源：开题报告《双门限航迹关联算法》 → 升级为三判据关联
%
% 【核心思想】
%   分布式融合中，两个传感器各自输出独立航迹，融合中心需要判断两条航迹
%   是否来自同一真实目标。单次距离判断易受噪声/杂波/偏差干扰，故采用
%   多判据分级判定 + 质量自适应修正。
%
% 【判据 ①：位置距离粗筛 T1（保留）】
%   对每个时刻 i，计算 R1 航迹 r 与 R2 航迹 s 的位置距离 d_i。
%     - d_i < T1·α(life)  → 通过，连续计数 k_count++
%     - d_i >= T1·α(life) → 不通过，k_count 立即归零
%   α(life) 为短航迹质量修正因子（见 compute_quality_adjusted_T1）。
%
% 【判据 ②：马氏距离平稳性检验（升级，替换原"距离方差"）】
%   原判据用距离方差 var(d) < var_thresh 判断"距离是否恒定"，存在致命
%   缺陷：两条平行同向飞行的不同目标，间距恒定 → 方差≈0 → 被误判为匹配。
%
%   新判据用马氏距离平方：
%     D²(k) = Δx' · inv(P1_pos + P2_pos) · Δx
%   其中 Δx 为两航迹位置的平面坐标差（km），P_pos 为位置协方差的 2×2
%   子矩阵（同样换算到 km²）。D² 服从 χ²(2)，门限取 χ²(2,0.95)=5.99。
%
%   语义：连续 M 帧中，D² < 门限的帧占比 ≥ maha_ratio（默认 0.8）则通过。
%   与"方差"相比，马氏距离把滤波器给出的不确定度纳入判据——不确定度大
%   （P 大）时门限自动放宽，不确定度小时自动收紧，统计意义明确。
%
% 【判据 ③：速度一致性（新增）】
%     |v1 - v2| < kappa · sqrt(trace(Pv1) + trace(Pv2))
%   速度从 UKF 状态 x = [lon, v_lon, lat, v_lat] 取（度/秒），换算到 km/s：
%     v_km = [v_lon·111.32·cos(lat); v_lat·110.57]
%   Pv 为速度协方差的 2×2 子矩阵（P 的 [2,2] [2,4] [4,2] [4,4]），同样按
%   上述尺度换算到 (km/s)²。kappa 默认 3。
%
%   为什么有效：平行航迹的位置距离恒定（判据①②都拦不住），但两条航迹
%   对应的是不同航班，速度大小/方向不同，判据③可唯一区分。该判据同时把
%   滤波器与关联串起来——速度估计精度直接决定关联可靠性。
%
% 【开关与向后兼容】
%   params.dualgate_use_maha / dualgate_use_vel 均为 false 时，判据②退化
%   为原来的"距离方差"检验，判据③不启用，行为与升级前完全一致。
%
% 【输入】
%   trackSnapshots_R1, trackSnapshots_R2 — [n_frames x 1] cell，每帧快照
%   params — 含三判据参数
%
% 【输出】
%   matched_pairs — struct 数组，每元素含 R1_track_id, R2_track_id,
%                   match_count, coexist_count, match_ratio, mean_dist_km,
%                   mean_speed_diff, mean_heading_diff, var_dist,
%                   match_frames, quality,
%                   maha_ratio（新增）, vel_diff_kmps（新增）,
%                   vel_tol_kmps（新增）
% =========================================================================

function matched_pairs = track_matcher_dualgate_fast(trackSnapshots_R1, trackSnapshots_R2, params)
    % 获取帧数，用于后续距离序列的长度设定
    n_frames = length(trackSnapshots_R1);
    % 从 params 中读取双门限算法参数，带默认值
    % T1_base: 第一门限距离阈值（km），默认 50m
    T1_base = get_param_local(params, 'dualgate_T1_km', 50);
    % M: 连续通过第一门限的最小帧数，默认 5 帧
    M = get_param_local(params, 'dualgate_M', 5);
    % var_thresh: 方差阈值（km^2），用于第二门限校验
    var_thresh = get_param_local(params, 'dualgate_var_km2', 100);
    % coexist_thresh: 最少共现帧数，关联结果需满足此最小持续时间
    coexist_thresh = get_param_local(params, 'dualgate_coexist_thresh', 3);

    % ---- 判据②③的开关与参数（缺省关闭 → 向后兼容原双门限行为）----
    gates = struct();
    % 是否启用马氏距离判据（替换距离方差）
    gates.use_maha    = logical(get_param_local(params, 'dualgate_use_maha', false));
    % 马氏距离平方门限，chi2(2, 0.95) = 5.99
    gates.maha_thresh = get_param_local(params, 'dualgate_maha_thresh', 5.99);
    % 窗口内通过马氏门限的帧占比下限
    gates.maha_ratio  = get_param_local(params, 'dualgate_maha_ratio', 0.8);
    % 是否启用速度一致性判据
    gates.use_vel     = logical(get_param_local(params, 'dualgate_use_vel', false));
    % 速度一致性倍数（sigma 倍数）
    gates.vel_kappa   = get_param_local(params, 'dualgate_vel_kappa', 3.0);
    % 判据①的帧计数模式（针对低发现概率场景的容错改造）：
    %   'continuous' — 严格连续（原逻辑）：NaN 或距离超限都立即归零
    %   'cumulative' — 累计容错：NaN（漏检/航迹中断）跳过但不归零，距离超限才归零
    gates.gate_mode   = char(get_param_local(params, 'dualgate_gate_mode', 'continuous'));

    % 收集 R1/R2 所有曾经活跃的航迹 ID
    % collect_all_active_ids 遍历所有帧快照，提取非 HISTORY 航迹的 ID
    r1_ids = collect_all_active_ids(trackSnapshots_R1);
    r2_ids = collect_all_active_ids(trackSnapshots_R2);

    % 预建索引（加速：把逐帧线性查找转为矩阵列访问 + 向量化）
    % 与原版唯一区别仅在此数据访问层，判定逻辑完全一致。
    I1 = pairing.build_track_index(trackSnapshots_R1, n_frames);
    I2 = pairing.build_track_index(trackSnapshots_R2, n_frames);

    matched_pairs = {};
    % 任一传感器无活跃航迹，直接返回空结果
    if isempty(r1_ids) || isempty(r2_ids)
        return;
    end

    % 预计算每条航迹的最大 life（用于质量修正）
    % compute_max_life 遍历所有帧快照，找出每条航迹出现过的最大 life 值
    r1_max_life = compute_max_life(trackSnapshots_R1, r1_ids);
    r2_max_life = compute_max_life(trackSnapshots_R2, r2_ids);

    % 对每对 (r1_id, r2_id) 跑三判据判定
    % 这是一个 O(N*M) 的笛卡尔积遍历，N 为 R1 航迹数，M 为 R2 航迹数
    for i = 1:length(r1_ids)
        r1_id = r1_ids(i);
        for j = 1:length(r2_ids)
            r2_id = r2_ids(j);

            % 质量修正后的 T1：取两条航迹中较小的 life 来决定修正因子
            % 短航迹（life 小）的 T1 会放大，降低关联门槛
            min_life = min(r1_max_life(i), r2_max_life(j));
            T1_adj = compute_quality_adjusted_T1(T1_base, min_life);

            % 判据①的逐帧距离序列：计算两条航迹在每一帧的位置距离
            % 如果某帧某条航迹不存在，对应位置为 NaN
            dist_series = compute_per_frame_distance(I1, I2, r1_id, r2_id);

            % 判据②③的逐帧序列（仅在开关打开时计算，避免无谓开销）
            if gates.use_maha || gates.use_vel
                [maha_series, vel_series, vel_tol_series] = compute_state_series(...
                    I1, I2, r1_id, r2_id);
            else
                maha_series = [];
                vel_series = [];
                vel_tol_series = [];
            end

            % 三判据判定核心：①连续计数 + ②马氏/方差校验 + ③速度一致性
            [matched, match_info] = dual_threshold_decide(...
                dist_series, T1_adj, M, var_thresh, coexist_thresh, ...
                gates, maha_series, vel_series, vel_tol_series);

            if matched
                % 构造匹配结果结构体（原字段保持不变，新增三判据诊断字段）
                mp = struct(...
                    'R1_track_id', r1_id, ...          % R1 侧航迹 ID
                    'R2_track_id', r2_id, ...          % R2 侧航迹 ID
                    'match_count', match_info.n_match_frames, ...  % 匹配帧数
                    'coexist_count', match_info.n_match_frames, ... % 共存帧数
                    'match_ratio', match_info.n_match_frames / n_frames, ... % 匹配比例
                    'mean_dist_km', match_info.mean_dist, ...  % 平均距离（km）
                    'mean_speed_diff', 0, ...  % 速度差（保留字段，未参与评分）
                    'mean_heading_diff', 0, ...  % 航向差（保留字段，未参与评分）
                    'var_dist', match_info.var_dist, ...  % 距离方差（km^2）
                    'match_frames', match_info.match_frames, ...  % 匹配帧号列表
                    'quality', compute_quality_score(match_info.mean_dist, ...  % 综合质量评分
                        match_info.n_match_frames, n_frames), ...
                    'maha_ratio', match_info.maha_ratio, ...    % 匹配区间内 D² 通过率
                    'vel_diff_kmps', match_info.vel_diff, ...   % 平均速度差（km/s）
                    'vel_tol_kmps', match_info.vel_tol);        % 速度一致性门限（km/s）
                % 追加到结果列表
                matched_pairs{end+1} = mp;
            end
        end
    end

    % 转为 struct 数组（cell of struct → struct array）
    % struct2table_vertcat 将元胞数组拼接为连续的结构体数组
    if ~isempty(matched_pairs)
        matched_pairs = struct2table_vertcat(matched_pairs);
    else
        % 空结果返回空结构体，保持字段名一致
        matched_pairs = struct('R1_track_id', {}, 'R2_track_id', {});
    end

    % 互斥后处理：每条 R1 航迹只保留得分最高的 R2 匹配，反之亦然
    % 这是 1v1 匹配约束，避免一对多或多对一的歧义
    if get_param_local(params, 'dualgate_mutual_exclusion', true) && ...
       ~isempty(matched_pairs) && isfield(matched_pairs, 'quality')
        matched_pairs = apply_mutual_exclusion(matched_pairs);
    end

    % 打印匹配统计信息（含三判据开关状态）
    fprintf(['[三判据匹配] T1=%.1fkm M=%d var_thresh=%.1fkm^2 | ' ...
        'maha=%d(thr=%.2f,ratio=%.2f) vel=%d(kappa=%.1f) → 匹配 %d 对\n'], ...
        T1_base, M, var_thresh, gates.use_maha, gates.maha_thresh, ...
        gates.maha_ratio, gates.use_vel, gates.vel_kappa, length(matched_pairs));
end


% =========================================================================
% apply_mutual_exclusion — 一对一互斥：每条 R1/R2 航迹只保留最佳匹配
% =========================================================================
function matched_pairs = apply_mutual_exclusion(matched_pairs)
    % 按 quality 降序排，优先保留高质量匹配
    [~, idx] = sort([matched_pairs.quality], 'descend');
    matched_pairs = matched_pairs(idx);

    used_r1 = [];  % 已匹配的 R1 航迹 ID
    used_r2 = [];  % 已匹配的 R2 航迹 ID
    keep = false(length(matched_pairs), 1);  % 保留标记数组
    for i = 1:length(matched_pairs)
        mp = matched_pairs(i);
        % 如果 R1 或 R2 航迹已被更高评分的匹配占用，跳过
        if ismember(mp.R1_track_id, used_r1) || ismember(mp.R2_track_id, used_r2)
            continue;
        end
        % 标记这两个航迹已被使用
        used_r1(end+1) = mp.R1_track_id;
        used_r2(end+1) = mp.R2_track_id;
        keep(i) = true;
    end
    % 只保留未被跳过的匹配对
    matched_pairs = matched_pairs(keep);
end


% =========================================================================
% collect_all_active_ids — 收集快照中所有出现过的活跃航迹 ID
% =========================================================================
function ids = collect_all_active_ids(snaps)
    ids = [];
    % 遍历每一帧快照
    for k = 1:length(snaps)
        % 跳过没有 trackList 或 trackList 为空的帧
        if ~isfield(snaps{k}, 'trackList') || isempty(snaps{k}.trackList)
            continue;
        end
        trks = snaps{k}.trackList;
        % 遍历该帧的所有航迹
        for t = 1:length(trks)
            % 排除 HISTORY 类型（type==7）和无效位置（NaN 经纬度）的航迹
            if trks{t}.type ~= 7 && ~isnan(trks{t}.lat)
        ids(end+1) = trks{t}.id;
            end
        end
    end
    % 去重并排序
    ids = unique(ids);
end


% =========================================================================
% compute_max_life — 每条航迹的最大 life 值
% =========================================================================
function max_life = compute_max_life(snaps, ids)
    max_life = zeros(size(ids));
    % 遍历每一帧快照
    for k = 1:length(snaps)
        if ~isfield(snaps{k}, 'trackList') || isempty(snaps{k}.trackList)
            continue;
        end
        trks = snaps{k}.trackList;
        % 遍历该帧的所有航迹
        for t = 1:length(trks)
            trk = trks{t};
            % 跳过 HISTORY 航迹和无效位置的航迹
            if trk.type == 7 || isnan(trk.lat)
                continue;
            end
            % 查找该航迹 ID 在 ids 数组中的位置
            idx = find(ids == trk.id, 1);
            % 如果找到且当前 life 更大，更新最大值
            if ~isempty(idx) && trk.life > max_life(idx)
                max_life(idx) = trk.life;
            end
        end
    end
end


% =========================================================================
% compute_quality_adjusted_T1 — 关联质量修正：短航迹 T1 放大
% =========================================================================
function T1_adj = compute_quality_adjusted_T1(T1_base, min_life)
    % 根据两条航迹中较小的 life 值决定 T1 修正因子
    % 短航迹（life 小）位置噪声大，放宽距离门限以提高关联召回率
    if min_life < 10
        factor = 1.5;    % 极短航迹：T1 放大 50%
    elseif min_life < 30
        factor = 1.25;   % 中等航迹：T1 放大 25%
    else
        factor = 1.0;    % 长航迹：不修正
    end
    T1_adj = T1_base * factor;
end


% =========================================================================
% compute_per_frame_distance — 计算两条航迹在每帧的位置距离序列（km）
% =========================================================================
function dist_series = compute_per_frame_distance(I1, I2, r1_id, r2_id)
    % 索引化向量实现（严格等价原逐帧 find_track_by_id 版本）
    n_frames = I1.n_frames;
    dist_series = nan(1, n_frames);
    c1 = pairing.idx_col(I1, r1_id);
    c2 = pairing.idx_col(I2, r2_id);
    if c1 == 0 || c2 == 0, return; end
    m = I1.has_pos(:, c1) & I2.has_pos(:, c2);   % 两站均存在且位置有效
    if ~any(m), return; end
    dist_series(m) = pairing.hav_vec(I1.lon_raw(m, c1), I1.lat_raw(m, c1), ...
                                     I2.lon_raw(m, c2), I2.lat_raw(m, c2));
end


% =========================================================================
% compute_state_series — 逐帧马氏距离平方 D² 与速度差序列
% =========================================================================
% 输出（均为 1×n_frames，无数据帧为 NaN）：
%   maha_series    — D²(k) = Δx' inv(P1+P2) Δx，Δx 与 P 均在 km 局部平面
%   vel_series     — |v1 - v2|（km/s）
%   vel_tol_series — sqrt(trace(Pv1)+trace(Pv2))（km/s），未乘 kappa
% =========================================================================
function [maha_series, vel_series, vel_tol_series] = compute_state_series(I1, I2, r1_id, r2_id)

    n_frames = I1.n_frames;
    maha_series    = nan(1, n_frames);
    vel_series     = nan(1, n_frames);
    vel_tol_series = nan(1, n_frames);

    c1 = pairing.idx_col(I1, r1_id);
    c2 = pairing.idx_col(I2, r2_id);
    if c1 == 0 || c2 == 0, return; end
    m = I1.has_state(:, c1) & I2.has_state(:, c2);   % 两站 UKF 状态均有效
    if ~any(m), return; end

    lo1 = I1.lon_ukf(m, c1); la1 = I1.lat_ukf(m, c1);
    lo2 = I2.lon_ukf(m, c2); la2 = I2.lat_ukf(m, c2);
    vlo1 = I1.vlon(m, c1);   vla1 = I1.vlat(m, c1);
    vlo2 = I2.vlon(m, c2);   vla2 = I2.vlat(m, c2);

    % 度 → km 的局部线性化尺度（以两条航迹的平均纬度为准）
    lat_mid = 0.5 * (la1 + la2);
    sx = 111.32 * cosd(lat_mid);
    sy = 110.57;

    % ---- 判据②：马氏距离平方（P 换算到 km²：P_km = S·P·S'，S=diag(sx,sy)）----
    p11 = I1.P11(m, c1) + I2.P11(m, c2);
    p12 = I1.P12(m, c1) + I2.P12(m, c2);
    p22 = I1.P22(m, c1) + I2.P22(m, c2);
    dx = sx .* (lo1 - lo2);
    dy = sy .* (la1 - la2);
    q11 = sx.^2 .* p11;
    q12 = sx .* sy .* p12;
    q22 = sy.^2 .* p22;
    det = q11 .* q22 - q12.^2;
    bad = det < 1e-12;                 % 与原 rcond 保护等价
    if any(bad)
        q11(bad) = q11(bad) + 1e-6;
        q22(bad) = q22(bad) + 1e-6;
        det = q11 .* q22 - q12.^2;
    end
    d2 = (q22 .* dx.^2 - 2 * q12 .* dx .* dy + q11 .* dy.^2) ./ det;

    % ---- 判据③：速度一致性 ----
    dvx = sx .* (vlo1 - vlo2);         % km/s
    dvy = sy .* (vla1 - vla2);
    vdiff = sqrt(dvx.^2 + dvy.^2);
    % 速度协方差换算到 (km/s)²，取迹：trace(S·Pv·S') = sx²·Pv11 + sy²·Pv22
    tr1 = sx.^2 .* I1.Pv11(m, c1) + sy.^2 .* I1.Pv22(m, c1);
    tr2 = sx.^2 .* I2.Pv11(m, c2) + sy.^2 .* I2.Pv22(m, c2);
    vtol = sqrt(max(tr1, 0) + max(tr2, 0));

    idx = find(m);
    maha_series(idx)    = d2(:).';
    vel_series(idx)     = vdiff(:).';
    vel_tol_series(idx) = vtol(:).';
end


% =========================================================================
% extract_state_local — 从航迹中提取位置/速度及其协方差子块
% =========================================================================
% UKF 状态 x = [lon, v_lon, lat, v_lat]（度、度/秒）
% 位置协方差：P([1 3],[1 3])；速度协方差：P([2 4],[2 4])
% =========================================================================
function [s, ok] = extract_state_local(trk)
    s = struct('lon', NaN, 'lat', NaN, 'vel', [NaN; NaN], ...
               'Ppos', zeros(2, 2), 'Pvel', zeros(2, 2));
    ok = false;

    % 字段与维度检查
    if ~isfield(trk, 'ukf') || isempty(trk.ukf)
        return;
    end
    if ~isfield(trk.ukf, 'x') || numel(trk.ukf.x) < 4
        return;
    end
    if ~isfield(trk.ukf, 'P') || isempty(trk.ukf.P)
        return;
    end
    if size(trk.ukf.P, 1) < 4 || size(trk.ukf.P, 2) < 4
        return;
    end

    x = trk.ukf.x(:);
    P = trk.ukf.P;
    % 数值有效性检查
    if any(~isfinite(x(1:4))) || any(~isfinite(P(1:4, 1:4)), 'all')
        return;
    end

    s.lon  = x(1);
    s.lat  = x(3);
    s.vel  = [x(2); x(4)];
    s.Ppos = P([1 3], [1 3]);
    s.Pvel = P([2 4], [2 4]);
    ok = true;
end


% =========================================================================
% dual_threshold_decide — 三判据判定核心逻辑
% =========================================================================
function [matched, info] = dual_threshold_decide(dist_series, T1, M, ...
    var_thresh, coexist_thresh, gates, maha_series, vel_series, vel_tol_series)
    % 判定核心逻辑
    %
    % 判据①（T1）：逐帧距离粗筛
    %   - d_i < T1 → 通过，连续计数 k_count++
    %   - d_i >= T1 → 不通过，k_count 归零
    %
    % 判据②（M + 平稳性）：
    %   - 连续 M 帧通过判据① 后进入校验
    %   - use_maha=true  → 窗口内 D² < maha_thresh 的帧占比 ≥ maha_ratio
    %   - use_maha=false → 退化为原逻辑：窗口内距离方差 < var_thresh
    %   - 校验失败 → k_count 不清零，继续等待（视为短期巧合）
    %
    % 判据③（速度一致性）：
    %   - use_vel=true → 窗口内 |v1-v2| < kappa·sqrt(trace(Pv1)+trace(Pv2))
    %                    的帧占比 ≥ maha_ratio
    %   - use_vel=false → 跳过
    %
    % 关联区间扩展：确认关联后，向前扩展到首次通过帧，向后扩展到
    %   连续通过帧的末端，得到完整的共现区间

    matched = false;  % 初始化判定结果为不匹配
    % 初始化返回信息结构体（新增字段默认 NaN，表示未启用/未计算）
    info = struct('n_match_frames', 0, 'mean_dist', NaN, ...
                  'var_dist', NaN, 'match_frames', [], ...
                  'maha_ratio', NaN, 'vel_diff', NaN, 'vel_tol', NaN);

    n = length(dist_series);
    k_count = 0;           % 连续通过判据①的帧数
    first_pass_idx = 0;    % 第一次通过判据①的帧索引

    for i = 1:n
        if isnan(dist_series(i))
            % 任一航迹在该帧不存在（漏检 / 航迹中断）
            %   continuous：视为中断，计数器归零（原逻辑）
            %   cumulative / leaky：视为「无证据」，跳过但不归零
            if strcmp(gates.gate_mode, 'continuous')
                k_count = 0;
                first_pass_idx = 0;
            end
            continue;
        end

        if dist_series(i) < T1
            % 通过判据①
            if k_count == 0
                first_pass_idx = i;  % 记录首次通过的帧
            end
            k_count = k_count + 1;

            if k_count >= M
                % 取最近 M 帧作为平稳性/速度检验窗口
                win = (i - M + 1):i;
                pass_ok = true;

                % ---- 判据②：马氏距离（或退化为方差）----
                if gates.use_maha
                    pass_ok = maha_pass_ratio(maha_series(win), gates.maha_thresh) ...
                              >= gates.maha_ratio;
                else
                    var_D = var(dist_series(win));
                    pass_ok = (var_D < var_thresh);
                end

                % ---- 判据③：速度一致性 ----
                if pass_ok && gates.use_vel
                    pass_ok = vel_pass_ratio(vel_series(win), ...
                        vel_tol_series(win), gates.vel_kappa) >= gates.maha_ratio;
                end

                if pass_ok
                    % 全部判据通过 → 关联成功
                    % 扩展匹配区间：从 first_pass_idx 延伸到连续通过的末端
                    ext_start = first_pass_idx;
                    ext_end = i;
                    while ext_end < n && ~isnan(dist_series(ext_end+1)) && ...
                          dist_series(ext_end+1) < T1
                        ext_end = ext_end + 1;
                    end
                    match_dists = dist_series(ext_start:ext_end);
                    match_dists = match_dists(~isnan(match_dists));
                    match_frames = (ext_start:ext_end)';

                    info.n_match_frames = length(match_frames);
                    info.mean_dist = mean(match_dists);
                    info.var_dist = var(match_dists);
                    info.match_frames = match_frames;

                    % 在整个匹配区间上统计判据②③的平均表现（供诊断/评估）
                    if gates.use_maha
                        seg = maha_series(ext_start:ext_end);
                        info.maha_ratio = maha_pass_ratio(seg, gates.maha_thresh);
                    end
                    if gates.use_vel
                        v_seg = vel_series(ext_start:ext_end);
                        t_seg = vel_tol_series(ext_start:ext_end);
                        valid = ~isnan(v_seg) & ~isnan(t_seg);
                        if any(valid)
                            info.vel_diff = mean(v_seg(valid));
                            info.vel_tol  = mean(gates.vel_kappa * t_seg(valid));
                        end
                    end

                    matched = true;

                    % 最少共现帧数校验：关联区间必须至少持续 coexist_thresh 帧
                    if info.n_match_frames < coexist_thresh
                        matched = false;
                    end
                    return;
                end
                % 校验未通过 → k_count 不清零，继续等待后续时刻
                % 这意味着可能是短期巧合（两条不同航迹在某几帧偶然靠近）
            end
        else
            % 不通过判据①（有数据但距离 >= T1）
            %   continuous / cumulative：立即归零（严格否决）
            %   leaky：漏桶衰减（k_count-1），容忍偶发超限后仍可继续累积
            if strcmp(gates.gate_mode, 'leaky')
                k_count = max(0, k_count - 1);
                if k_count == 0, first_pass_idx = 0; end
            else
                k_count = 0;
                first_pass_idx = 0;
            end
        end
    end
end


% =========================================================================
% maha_pass_ratio — 序列中 D² < 门限 的帧占比（NaN 视为不通过）
% =========================================================================
function r = maha_pass_ratio(series, thresh)
    if isempty(series)
        r = 0;
        return;
    end
    r = sum(~isnan(series) & series < thresh) / numel(series);
end


% =========================================================================
% vel_pass_ratio — 序列中 |v1-v2| < kappa·tol 的帧占比（NaN 视为不通过）
% =========================================================================
function r = vel_pass_ratio(v, tol, kappa)
    if isempty(v)
        r = 0;
        return;
    end
    r = sum(~isnan(v) & ~isnan(tol) & v < kappa * tol) / numel(v);
end


% =========================================================================
% compute_quality_score — 综合质量评分 0-100
% =========================================================================
function q = compute_quality_score(mean_dist, n_match, n_total)
    % 距离分数：距离越小分数越高，按指数衰减
    dist_score = 100 * exp(-mean_dist / 20);
    % 比例分数：匹配帧占总帧数的比例
    ratio_score = 100 * (n_match / max(n_total, 1));
    % 综合评分：距离和比例各占 50% 权重
    q = 0.5 * dist_score + 0.5 * ratio_score;
end


% =========================================================================
% find_track_by_id — 在快照中按 ID 找航迹
% =========================================================================
function trk = find_track_by_id(snap, tid)
    trk = [];
    % 快照中没有 trackList 或为空，直接返回
    if ~isfield(snap, 'trackList') || isempty(snap.trackList)
        return;
    end
    % 遍历航迹列表，查找 ID 匹配的航迹
    for t = 1:length(snap.trackList)
        if snap.trackList{t}.id == tid
            trk = snap.trackList{t};
            return;
        end
    end
end


% =========================================================================
% haversine_km_local — Haversine 距离（km）
% =========================================================================
function d = haversine_km_local(lon1, lat1, lon2, lat2)
    % 使用 Haversine 公式计算地球表面两点间的大圆距离（km）
    % 适用于经纬度坐标的距离计算
    R = 6371;  % 地球半径（km）
    dlat = deg2rad(lat2 - lat1);  % 纬度差转为弧度
    dlon = deg2rad(lon2 - lon1);  % 经度差转为弧度
    % Haversine 公式核心计算
    a = sin(dlat/2)^2 + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * sin(dlon/2)^2;
    % 钳位到 [0, 1] 防止浮点误差导致 acos 出错
    a = max(0, min(1, a));
    % 计算大圆距离
    d = R * 2 * atan2(sqrt(a), sqrt(1 - a));
end


% =========================================================================
% struct2table_vertcat — cell of struct 转 struct 数组
% =========================================================================
function s = struct2table_vertcat(c)
    if isempty(c)
        s = struct();
        return;
    end
    % 逐个拼接 struct 数组元素
    s = c{1};
    for i = 2:length(c)
        s(end+1) = c{i};
    end
end


% =========================================================================
% get_param_local — 参数读取
% =========================================================================
function v = get_param_local(params, name, default)
    % 从 params 结构体中安全读取字段，不存在时返回默认值
    v = default;
    if isfield(params, name)
        v = params.(name);
    end
end
