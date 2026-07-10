# 代码深度审查历史卷二（第 66—71 章）

> **文档状态（2026-07-10）**：本文件承接 `CODE_REVIEW.md`，是历史阶段稿，不代表 131 个 MATLAB 文件已经完成逐行覆盖。源码基线为 `7c166d41541ccd74f23fd6c3ea0b871d8603950e`；权威覆盖矩阵、问题状态、验证证据和勘误见 [`code-review/README.md`](code-review/README.md)。其中 66.2 与 66.6 对 `radar_coverage_check.m` 存在重复审查，保留原文但不重复计入有效覆盖。

---

## 第 66 章：仿真模块逐行深度审查

### 66.1 generate_frame_detections.m 逐行审查（231行）

第1-51行：文件头注释
注释详细描述了天波双基地量测模型、量测误差模型、虚警杂波模型。
评价：注释质量高，每个公式都有物理含义解释。

第97-100行：函数签名
function [detList, has_target_det] = generate_frame_detections(rx_lon, rx_lat, tx_lon, tx_lat, tgt_lon, tgt_lat, tgt_lon_rate, tgt_lat_rate, frameID, time_sec, range_bias, az_bias, beam_center, params, range_noise, az_noise)
评价：17个参数，函数签名过长。建议将params和radar_config拆分为独立参数。

第102-104行：默认参数
if nargin < 16, range_noise = params.radar1_range_noise_std_m; end
if nargin < 17, az_noise = params.radar1_azimuth_noise_std_deg; end
评价：默认使用R1的噪声水平。如果R2调用此函数但没有传入range_noise和az_noise，会使用R1的参数，导致噪声估计偏低。

第117行：威力覆盖检查
[in_cov, ~, ~] = radar_coverage_check(rx_lon, rx_lat, tgt_lon, tgt_lat, beam_center, params);
评价：调用radar_coverage_check判断目标是否在雷达威力范围内。如果目标不在覆盖区内，不做任何检测尝试。

第119-160行：目标检测
if in_cov
    if rand() <= params.detection_probability
        has_target_det = true;
        Rg_true = skywave_geometry(group_range, ...)
        az_true = skywave_geometry(azimuth, ...)
        vd_true = skywave_geometry(doppler, ...)
        Rg_meas = Rg_true + range_bias + randn() * range_noise
        az_meas = az_true + az_bias + randn() * az_noise
        vd_meas = vd_true + randn() * params.radial_vel_noise_std_ms
        det = struct(frameID, frameID, time_sec, time_sec, prange, Rg_meas, paz, az_meas, pvr, vd_meas, range_meas, Rg_meas, azimuth_meas, az_meas, radial_vel_meas, vd_meas, range_true, Rg_true, azimuth_true, az_true, radial_vel_true, vd_true, lat_true, tgt_lat, lon_true, tgt_lon, lat, NaN, lon, NaN, is_clutter, false);
        detList = [detList, det];
    end
end

评价：
1. 第140行：Rg_meas = Rg_true + range_bias + randn() * range_noise。这是标准的量测模型：真值 + 系统偏差 + 随机噪声
2. 第141行：az_meas = az_true + az_bias + randn() * az_noise。方位角噪声标准差0.35度（R1）或0.6度（R2）
3. 第142行：vd_meas = vd_true + randn() * params.radial_vel_noise_std_ms。多普勒噪声标准差0.5 m/s，两雷达共用
4. 第151-156行：det结构体包含大量字段，内存占用较大

第176-229行：虚警杂波生成
n_false = poissrnd(params.n_resolution_cells * params.false_alarm_rate);
half_beam = params.beam_width_deg / 2;
for f = 1:n_false
    fake_r1 = params.range_min_m + rand() * (params.range_max_m - params.range_min_m);
    fake_az = beam_center - half_beam + rand() * params.beam_width_deg;
    [clut_lon, clut_lat] = sphere_utils_destination_point(rx_lon, rx_lat, fake_r1, fake_az);
    fake_Rg = skywave_geometry(group_range, tx_lon, tx_lat, rx_lon, rx_lat, clut_lon, clut_lat);
    fake_vr = -200 + rand() * 400;
    det = struct(frameID, frameID, time_sec, time_sec, prange, fake_Rg + range_bias, paz, fake_az + az_bias, pvr, fake_vr, range_meas, NaN, azimuth_meas, NaN, radial_vel_meas, fake_vr, range_true, NaN, azimuth_true, NaN, radial_vel_true, NaN, lat_true, clut_lat, lon_true, clut_lon, lat, clut_lat, lon, clut_lon, is_clutter, true);
    detList = [detList, det];
end

评价：
1. 第177行：泊松分布生成虚警数量，lambda = 1500 * 0.001 = 1.5
2. 第185行：fake_r1在[1000km, 2000km]均匀分布
3. 第186行：fake_az在[beam_center-half_beam, beam_center+half_beam]均匀分布
4. 第189-191行：通过sphere_utils_destination_point映射到地理坐标
5. 第195行：fake_Rg = skywave_geometry(group_range, ...)计算杂波的天波群距离
6. 第198行：fake_vr在[-200, +200]均匀分布，模拟电离层杂波的多普勒展宽
7. 第220-221行：杂波的prange和paz也掺入了系统偏差，这是为了保证偏差校正后drange approx fake_Rg的逻辑一致

### 66.2 radar_coverage_check.m 逐行审查（97行）

第65行：r1 = sphere_utils_haversine_distance(rx_lon, rx_lat, tgt_lon, tgt_lat);
评价：计算接收站到目标的地表大圆距离

第71行：az = sphere_utils_azimuth(rx_lon, rx_lat, tgt_lon, tgt_lat);
评价：计算接收站到目标的方位角

第78-85行：az_diff = abs(az - beam_center); if az_diff > 180, az_diff = 360 - az_diff; end
评价：处理0度/360度附近的方位角跳跃

第93-95行：in_coverage = (r1 >= range_min_m) && (r1 <= range_max_m) && (az_diff <= half_beam);
评价：三个条件必须同时满足，逻辑正确

### 66.3 aircraft_trajectory_create.m 逐行审查（666行）

第104-275行：直线航迹创建
traj.speed = speed_ms;
traj.dt_sec = dt_sec;
traj.waypoints = waypoints_lla(:, 1:2);
n_wp = size(traj.waypoints, 1);
traj.segments = cell(n_wp - 1, 1);
t_cum = 0.0;
for i = 1:(n_wp - 1)
    lon0 = traj.waypoints(i, 1); lat0 = traj.waypoints(i, 2);
    lon1 = traj.waypoints(i+1, 1); lat1 = traj.waypoints(i+1, 2);
    dist = sphere_utils_haversine_distance(lon0, lat0, lon1, lat1);
    dur = dist / speed_ms;
    lon_rate = (lon1 - lon0) / dur;
    lat_rate = (lat1 - lat0) / dur;
    traj.segments{i} = struct(start, [lon0, lat0], end, [lon1, lat1], lon_rate, lon_rate, lat_rate, lat_rate, dur, dur, t_start, t_cum);
    t_cum = t_cum + dur;
end
traj.duration_sec = t_cum;
traj.n_segments = length(traj.segments);
traj.time_array = 0:dt_sec:traj.duration_sec;
traj.n_steps = length(traj.time_array);

评价：
1. 第201行：dist = sphere_utils_haversine_distance(lon0, lat0, lon1, lat1)。使用Haversine公式计算球面距离
2. 第206行：dur = dist / speed_ms。航段时长 = 距离 / 速度
3. 第211-215行：lon_rate和lat_rate是经纬度变化率，单位度/秒。这是匀速直线运动的假设

第283-294行：拐弯航迹
waypoints = [126.0, 32.5, 0; 128.5, 33.5, 0; 128.6, 31.7, 0];
speed_ms = 140.0;
traj = aircraft_trajectory_create(waypoints, speed_ms, params.dt_sec);

评价：
1. 三个航路点形成约120度拐角
2. 速度降低到140 m/s（504 km/h），保持帧数

第309-427行：180度回头弯
speed_ms = params.aircraft_speed_ms;  % 230 m/s
omega_deg = 1.0;  % 1度/s标准转弯率
omega_rad = omega_deg * pi / 180.0;
R_turn_m = speed_ms / omega_rad;  % 13184 m
turn_dur_sec = 180.0;  % 180秒
arc_length_m = pi * R_turn_m;  % 41400 m

评价：
1. 转弯半径R = v/omega = 230/(pi/180) = 13184 m
2. 转弯时长 = 180度 / 1度/s = 180秒
3. 弧长 = pi * R = 41400 m
4. 圆心固定为(131.44, 31.75)，硬编码地理坐标

第452-633行：渐进拐弯
W1 = [126.6685, 32.2184];  % 起点
W2 = [128.2501, 31.0887];  % 拐弯顶点
W3 = [132.0502, 31.4379];  % 终点

评价：
1. 三个航路点定义渐进拐弯
2. 转弯提前量d_anticipate = R * tan(theta/2)
3. 弧段用1秒步长采样，然后按dt_sec=30秒分组打包

### 66.4 aircraft_trajectory_interpolate.m 逐行审查（171行）

第58-135行：单点插值
[idx, t_seg] = aircraft_trajectory_locate(traj, t);
seg = traj.segments{idx};
t_seg = min(t_seg, seg.dur);
lon = seg.start(1) + seg.lon_rate * t_seg;
lat = seg.start(2) + seg.lat_rate * t_seg;

评价：分段线性插值，航段内匀速运动假设

第144-156行：批量插值
function out = interpolate_batch_impl(traj, t_array)
    n = length(t_array);
    out = zeros(n, 5);
    for i = 1:n
        t = t_array(i);
        [pos, vel] = aircraft_trajectory_interpolate(traj, t);
        out(i, 1) = pos(1); out(i, 2) = pos(2);
        out(i, 3) = vel(1); out(i, 4) = vel(2);
        out(i, 5) = t;
    end
end

评价：逐点调用单点插值，没有向量化优化

### 66.5 aircraft_trajectory_locate.m 逐行审查

第49-82行：时间定位
function [idx, t_seg] = aircraft_trajectory_locate(traj, t)
    t = max(0, min(t, traj.duration_sec));  % 钳位
    for idx = 1:traj.n_segments
        if t < traj.segments{idx}.t_start + traj.segments{idx}.dur
            t_seg = t - traj.segments{idx}.t_start;
            return;
        end
    end
end

评价：线性搜索O(N_segments)，建议二分查找O(log N)

### 66.6 radar_coverage_check.m 逐行审查（97行）

第65行：r1 = sphere_utils_haversine_distance(rx_lon, rx_lat, tgt_lon, tgt_lat);
第71行：az = sphere_utils_azimuth(rx_lon, rx_lat, tgt_lon, tgt_lat);
第78-85行：az_diff = abs(az - beam_center); if az_diff > 180, az_diff = 360 - az_diff; end
第93-95行：in_coverage = (r1 >= range_min_m) && (r1 <= range_max_m) && (az_diff <= half_beam);

评价：三个条件必须同时满足，逻辑正确

---

## 第 67 章：utils 工具函数完整审查

### 67.1 skywave_geometry.m 逐行审查（180行）

第34-35行：全局常量
R_e = 6371000.0;  % 地球平均半径 6371 km
H   = 300000.0;   % 电离层等效固定虚高 300 km

评价：H=300km是F层典型高度，但实际高度在250-400km之间变化

第55-63行：群距离计算
sigma_tx = geocentric_angle_impl(tx_lon, tx_lat, tgt_lon, tgt_lat);
sigma_rx = geocentric_angle_impl(rx_lon, rx_lat, tgt_lon, tgt_lat);
D_tx = 2.0 * R_e * sin(sigma_tx / 2.0);
D_rx = 2.0 * R_e * sin(sigma_rx / 2.0);
r_tx = sqrt(D_tx^2 + (2.0 * H)^2);
r_rx = sqrt(D_rx^2 + (2.0 * H)^2);
varargout{1} = r_tx + r_rx;

评价：弦长公式D = 2R*sin(sigma/2)正确
斜距公式r = sqrt(D^2 + (2H)^2)基于固定虚高电离层模型

第65-68行：多普勒计算
vd = doppler_impl(tx_lon, tx_lat, rx_lon, rx_lat, tgt_lon, tgt_lat, lon_rate, lat_rate, R_e, H);

第143-168行：doppler_impl
lat_rad = deg2rad(tgt_lat);
v_east  = lon_rate * pi / 180.0 * R_e * cos(lat_rad);
v_north = lat_rate * pi / 180.0 * R_e;
sigma_tx = geocentric_angle_impl(tx_lon, tx_lat, tgt_lon, tgt_lat);
sigma_rx = geocentric_angle_impl(rx_lon, rx_lat, tgt_lon, tgt_lat);
D_tx = 2.0 * R_e * sin(sigma_tx / 2.0);
D_rx = 2.0 * R_e * sin(sigma_rx / 2.0);
r_tx = sqrt(D_tx^2 + (2.0 * H)^2);
r_rx = sqrt(D_rx^2 + (2.0 * H)^2);
az_tx = azimuth_impl(tx_lon, tx_lat, tgt_lon, tgt_lat);
az_rx = azimuth_impl(rx_lon, rx_lat, tgt_lon, tgt_lat);
dr_tx_dt = path_rate_impl(D_tx, r_tx, sigma_tx, az_tx, v_east, v_north);
dr_rx_dt = path_rate_impl(D_rx, r_rx, sigma_rx, az_rx, v_east, v_north);
vd = dr_tx_dt + dr_rx_dt;

评价：多普勒速度推导完全正确（见第21章）

第175-179行：path_rate_impl
az_rad = deg2rad(az);
v_along_gc = v_east * sin(az_rad) + v_north * cos(az_rad);
dr_dt = (D / r) * cos(sigma / 2.0) * v_along_gc;

评价：dr/dt = (D/r)*cos(sigma/2)*v_along_gc，推导正确

### 67.2 sphere_utils_haversine_distance.m 逐行审查（102行）

第52-53行：dlon = deg2rad(lon2 - lon1); dlat = deg2rad(lat2 - lat1);
第59-60行：lat1_rad = deg2rad(lat1); lat2_rad = deg2rad(lat2);
第76行：a = sin(dlat/2)^2 + cos(lat1_rad)*cos(lat2_rad)*sin(dlon/2)^2;
第93行：c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
第101行：dist = 6371000.0 * c;

评价：Haversine公式标准实现，正确

### 67.3 sphere_utils_azimuth.m 逐行审查（107行）

第58行：dlon = deg2rad(lon_to - lon_from);
第64-65行：lat_from_rad = deg2rad(lat_from); lat_to_rad = deg2rad(lat_to);
第82行：y = sin(dlon) * cos(lat_to_rad);
第84-85行：x = cos(lat_from_rad)*sin(lat_to_rad) - sin(lat_from_rad)*cos(lat_to_rad)*cos(dlon);
第106行：az = mod(rad2deg(atan2(y, x)), 360.0);

评价：大圆初始方位角标准公式，正确

### 67.4 sphere_utils_destination_point.m 逐行审查（126行）

第64-65行：R = 6371000.0; arc_len = distance_m / R;
第70-72行：az_rad = deg2rad(az_deg); lat1 = deg2rad(lat_start); lon1 = deg2rad(lon_start);
第93-94行：lat2 = asin(sin(lat1)*cos(arc_len) + cos(lat1)*sin(arc_len)*cos(az_rad));
第118-119行：lon2 = lon1 + atan2(sin(az_rad)*sin(arc_len)*cos(lat1), cos(arc_len)-sin(lat1)*sin(lat2));
第124-125行：lon = rad2deg(lon2); lat = rad2deg(lat2);

评价：大圆目的地点标准公式，正确

---

## 第 68 章：融合模块完整审查

### 68.1 run_track_fusion.m 逐行审查（306行）

第50-51行：函数签名
function fused_snapshots = run_track_fusion(matched_pairs, trackSnapshots_R1, aligned_R2, params, method)

第56-57行：基本参数
n_frames = length(trackSnapshots_R1);
n_pairs = length(matched_pairs);

第63-68行：航迹ID到pair索引的快速查找
r1_to_pair = containers.Map(int32, int32);
r2_to_pair = containers.Map(int32, int32);
for p = 1:n_pairs
    r1_to_pair(matched_pairs(p).R1_track_id) = p;
    r2_to_pair(matched_pairs(p).R2_track_id) = p;
end

评价：使用containers.Map实现O(1)查找，正确

第75-81行：BC方法专用初始化
if strcmp(method, BC)
    P12_cell = cell(n_pairs, 1);
    for p = 1:n_pairs
        P12_cell{p} = zeros(4, 4);
    end
    has_prev = false(n_pairs, 1);
end

评价：BC方法需要维护互协方差P12，初始化为零矩阵

第91-278行：主循环逐帧逐对执行融合
for k = 1:n_frames
    snap_r1 = trackSnapshots_R1{k};
    snap_r2 = aligned_R2{k};
    fused_snap = struct(frameID, k, trackList, {{}});
    for p = 1:n_pairs
        r1_id = matched_pairs(p).R1_track_id;
        r2_id = matched_pairs(p).R2_track_id;
        trk1 = find_track(snap_r1, r1_id);
        trk2 = find_track(snap_r2, r2_id);
        ...
        if ~isempty(trk1) && ~isempty(trk2)
            x1 = trk1.ukf.x; P1 = trk1.ukf.P;
            x2 = trk2.ukf.x; P2 = trk2.ukf.P;
            switch upper(method)
                case SCC: [x_f, P_f] = track_fusion_algorithms(SCC, x1, P1, x2, P2);
                case CI:  [x_f, P_f, w_opt] = track_fusion_algorithms(CI, x1, P1, x2, P2);
                case FCI: [x_f, P_f, w_fci] = track_fusion_algorithms(FCI, x1, P1, x2, P2);
                case BC:
                    if has_prev(p)
                        F_cv_dt = [1, dt, 0, 0; 0, 1, 0, 0; 0, 0, 1, dt; 0, 0, 0, 1];
                        Q_half = trk1.ukf.Q * 0.5;
                        P12_pred = F_cv_dt * P12_cell{p} * F_cv_dt' + Q_half;
                        ...
                    else
                        P12_new = zeros(4, 4);
                    end
                    [x_f, P_f] = track_fusion_algorithms(BC, x1, P1, x2, P2, P12_new);
                    P12_cell{p} = P12_new;
                    has_prev(p) = true;
                end
            fused_trk.lat = x_f(3); fused_trk.lon = x_f(1);
            fused_trk.ukf_x = x_f; fused_trk.ukf_P = P_f;
            fused_trk.source = both;
        elseif ~isempty(trk1)
            fused_trk.lat = trk1.ukf.x(3); fused_trk.lon = trk1.ukf.x(1);
            fused_trk.ukf_x = trk1.ukf.x; fused_trk.ukf_P = trk1.ukf.P;
            fused_trk.source = R1_only;
        else
            fused_trk.lat = trk2.ukf.x(3); fused_trk.lon = trk2.ukf.x(1);
            fused_trk.ukf_x = trk2.ukf.x; fused_trk.ukf_P = trk2.ukf.P;
            fused_trk.source = R2_only;
        end
        fused_snap.trackList{end+1} = fused_trk;
    end
    fused_snapshots{k} = fused_snap;
end

评价：
1. 第171行：F_cv_dt = [1, dt, 0, 0; 0, 1, 0, 0; 0, 0, 1, dt; 0, 0, 0, 1]。这是CV模型的状态转移矩阵，正确
2. 第179行：Q_half = trk1.ukf.Q * 0.5。两雷达共享同一过程噪声，各承担一半。但R1和R2的Q不同（scale 1e5 vs 2e5），简单除以2不合理
3. 第180行：`P12_pred = F_cv_dt * P12_cell{p} * F_cv_dt' + Q_half`。源码包含右侧转置，符合协方差传播的基本矩阵形式；历史稿此前漏抄了该转置，那是文档转录错误，不是源码缺陷。该实现仍是 Bar-Shalom 互协方差传播的经验近似：共享过程噪声被固定写为 R1 的 `Q/2`，更新压缩又以协方差迹之比近似 `(I-KH)`，其统计一致性需要通过联合协方差半正定性和 Monte Carlo 一致性验证。

### 68.2 track_fusion_algorithms.m 逐行审查（422行）

第39-66行：调度函数
function [x_fused, P_fused, varargout] = track_fusion_algorithms(method, x1, P1, x2, P2, varargin)
    switch upper(method)
        case SCC: [x_fused, P_fused] = fuse_scc(x1, P1, x2, P2);
        case BC:  [x_fused, P_fused] = fuse_bc(x1, P1, x2, P2, varargin{:});
        case CI:  [x_fused, P_fused, w] = fuse_ci(x1, P1, x2, P2); varargout{1} = w;
        case FCI: [x_fused, P_fused, w] = fuse_fci(x1, P1, x2, P2); varargout{1} = w;
    end
end

### 68.3 fuse_scc（第111-138行）
P_fused_inv = inv(P1) + inv(P2);
P_fused = inv(P_fused_inv);
x_fused = P_fused * (P1 \\ x1 + P2 \\ x2);

评价：信息形式融合，假设两个估计独立

### 68.4 fuse_bc（第186-231行）
S = P1 + P2 - P12 - P12';
S = regularize_cov(S);
K_bc = (P1 - P12) / S;
x_fused = x1 + K_bc * (x2 - x1);
P_fused = P1 - K_bc * (P1 - P12');

评价：BC融合公式正确（见第25章）

### 68.5 fuse_ci（第273-317行）
obj = @(w) ci_cost(w, P1_inv, P2_inv);
w_opt = fminbnd(obj, 0.01, 0.99, optimset(Display, off, TolX, 1e-4));
P_fused_inv = w_opt * P1_inv + (1 - w_opt) * P2_inv;
P_fused = inv(P_fused_inv);
x_fused = P_fused * (w_opt * P1_inv * x1 + (1 - w_opt) * P2_inv * x2);

评价：CI融合公式正确（见第25章）

### 68.6 fuse_fci（第382-421行）
tr1_inv = 1 / trace(P1);
tr2_inv = 1 / trace(P2);
w_fci = tr1_inv / (tr1_inv + tr2_inv);
P_fused_inv = w_fci * P1_inv + (1 - w_fci) * P2_inv;
P_fused = inv(P_fused_inv);
x_fused = P_fused * (w_fci * P1_inv * x1 + (1 - w_fci) * P2_inv * x2);

评价：FCI融合公式正确（见第25章）

### 68.7 time_align_tracks.m 逐行审查（136行）

第59行：dt_offset = params.time_offset_radar2_sec;  % 默认13秒
第95-99行：dt = -dt_offset;  // 回退
F = [1, dt, 0, 0; 0, 1, 0, 0; 0, 0, 1, dt; 0, 0, 0, 1];
第104行：trk.ukf.x = F * trk.ukf.x;
第114-116行：dt_abs = abs(dt);
Q_dt = trk.ukf.Q * (dt_abs / params.dt_sec);
trk.ukf.P = F * trk.ukf.P * F' + Q_dt;

评价：
1. 第115行：Q_dt = Q_base * (13/30) = 0.43 * Q_base。回退的Q增量仅为前向预测的43%，反直觉
2. 回退应该是确定性的状态转移，不应增加过程噪声
3. 建议：回退时不加Q，或加一个很小的Q

---

## 第 69 章：评估模块完整审查

### 69.1 evaluate_all.m 逐行审查（362行）

第25-118行：compute_tracking_errors
for a = 1:n_ac
    tt = truthTrajs{a};
    for k = 1:n_frames
        t_true_lat = interp1(tt.time_sec, tt.lat, frame_times(k), 'linear', 'extrap');
        t_true_lon = interp1(tt.time_sec, tt.lon, frame_times(k), 'linear', 'extrap');
        snap = trackSnapshots{k};
        if ~isempty(snap.trackList)
            best_ukf_dist = inf;
            for t = 1:length(snap.trackList)
                trk = snap.trackList{t};
                if trk.type == 7 || isnan(trk.lat), continue; end
                d = haversine_km_eval(trk.lon, trk.lat, t_true_lon, t_true_lat);
                if d < best_ukf_dist && d < 200  % d 的单位为 km，因此这里是 200 km 门限
                    best_ukf_dist = d;
                    best_ukf_lat = trk.lat;
                end
            end
            if ~isinf(best_ukf_dist)
                ukf_errs{a}(end+1) = best_ukf_dist;
            end
        end
    end
end

评价：
1. 第53行调用 `haversine_km_eval`，该辅助函数使用地球半径 `R = 6371`，返回值单位为 km。因此第54行 `d < 200` 表示 200 km，而不是 200 m；历史稿据此推导“绝大多数帧为空、RMSE被严重低估”是单位误判，现予以撤销。
2. 200 km 对单目标最近航迹选择是很宽的兜底上限。它不会排除通常为数公里到数十公里的有效估计，却可能让错误航迹在 200 km 内被当作该目标的最佳航迹，从而掩盖关联或身份错误；在多目标场景尤其不能替代一对一匹配。
3. 建议把门限单位写入参数名，例如 `evaluation_match_gate_km`，并依据场景目标间距、量测误差和航迹管理目标设置；同时报告门内匹配率、门外帧数和身份切换，而不是仅报告被接受样本的 RMSE。

### 69.2 evaluate_fusion（第124-278行）

第131-152行：航迹-飞机映射
for p = 1:length(matched_pairs)
    mp = matched_pairs(p);
    r1_idx = find(matcher.r1_ids == mp.R1_track_id, 1);
    r1_lons = squeeze(matcher.r1_pos(r1_idx, :, 1));
    r1_lats = squeeze(matcher.r1_pos(r1_idx, :, 2));
    best_ac = 0; best_dist = inf;
    for a = 1:n_ac
        tt = truthTrajs{a};
        t_lat = interp1(tt.time_sec, tt.lat, frame_times, 'linear', 'extrap');
        t_lon = interp1(tt.time_sec, tt.lon, frame_times, 'linear', 'extrap');
        mean_dist = nanmean(haversine_km_vec_eval(r1_lons, r1_lats, t_lon, t_lat));
        if mean_dist < best_dist
            best_dist = mean_dist;
            best_ac = a;
        end
    end
    pair_to_aircraft(p) = best_ac;
end

评价：用R1航迹的平均大地线距离来匹配真值飞机，合理

---

## 第 70 章：可视化模块完整审查

### 70.1 plot_results.m 逐行审查（1270行）

第28-47行：调度函数
function plot_results(mode, varargin)
    switch mode
        case 'single_track': plot_single_track_result(varargin{:});
        case 'single_fusion': plot_single_fusion_result(varargin{:});
        case 'combined_tracks': plot_combined_tracks(varargin{:});
        case 'tracks_vs_truth': plot_tracks_vs_truth(varargin{:});
        case 'tracker': plot_tracker_result(varargin{:});
        case 'error_timeline': plot_error_timeline(varargin{:});
        case 'error_timeline_turn': plot_error_timeline_turn(varargin{:});
    end
end

评价：7种绘图模式，通过mode字符串调度

第56-62行：geoaxes容错处理
try
    ax = geoaxes('Units', 'normalized', 'Position', [...]);
    ax.Basemap = 'darkwater';
catch
    ax = geoaxes('Units', 'normalized', 'Position', [...]);
end

评价：如果geoaxes失败（Mapping Toolbox未安装），catch块中再次调用geoaxes仍会失败
这个try-catch没有实际意义

---

## 第 71 章：项目阶段性总结

### 71.1 项目成就
1. 完整的天波OTH-SWR仿真系统
2. 三种UKF体制：基础UKF、自适应UKF、IMM UKF
3. 四种融合算法：SCC、BC、CI、FCI
4. 丰富的评估指标：RMSE、MTL、断裂次数、关联率、NIS统计
5. 两套航迹管理框架：主系统（UKF管线）和南阳子系统（Alpha-Beta管线）

### 71.2 主要不足
1. P_d=1.0作弊模式：所有评估结果在完美检测假设下获得
2. ukf_alpha=1e-2导致数值不稳定：UKF退化为近似EKF
3. PDA协方差修正缺失
4. 代码重复严重：Haversine重复4份、正则化重复2份、模糊推理重复2份
5. 无单元测试：所有验证依赖跑仿真看图表
6. 南阳子系统与主系统功能重叠
7. run('header.m')反模式：全局状态污染

### 71.3 修复优先级
P0（历史清单，待台账逐项复核）：P_d=1.0 / Haversine重复 / 200 km 评估门限过宽风险 / run(header) / IMM配置6组重复 / vx=vy=0 / 近距离坐标转换失败
P1（14个）：ukf_alpha / 模糊推理重复 / PDA协方差修正 / NIS历史 / 杂波预筛 / BC融合P12 / 时间对齐Q / 群距离误用 / 权重分配 / Alpha-Beta权重 / robustMinSquareErr分母 / predictNextStep调试代码 / global变量 / 3-5逻辑浮点匹配
P2（10个）：正则化重复 / tracker耦合 / 回退Q / 航迹脆弱 / 排序 / 质量参数 / 报告匹配 / 窗口长度 / iono高度 / 协方差更新
P3（5个）：注释过多 / 缺少测试 / 文档格式 / 性能优化 / 代码风格

### 71.4 修复路线图
Phase 1（Week 1）：清理重复赋值 / 统一Haversine / 明确评估门限单位并校准 / 标注P_d局限 / 改ukf_alpha / 删除run(header)
Phase 2（Week 2-3）：拆分模块 / 添加验证 / 实现PDA修正 / 滑动窗口NIS / 修复时间对齐 / 清理僵尸代码
Phase 3（Week 4-6）：单元测试 / 解耦tracker-ukf / P_d<1.0评估 / 拆分plot_results / Joseph形式 / 合并子系统
Phase 4（Month 3+）：分层架构 / 完整JPDA / 更多融合算法 / 更多运动模型 / 电离层时变 / 完整测试套件 / 统一命名规范 / CI/CD自动化
