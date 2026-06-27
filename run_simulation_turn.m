%% =========================================================================
% run_simulation_turn.m — 拐弯目标仿真主程序：基础UKF vs 机动自适应UKF
% =========================================================================
% 【程序定位】
%   本程序是本仿真系统的第二个主入口。与run_simulation.m（直线航迹）不同，
%   本程序专门验证拐弯机动场景下自适应UKF相对于基础UKF的性能提升。
%   核心对比：基础UKF（模糊自适应Q，连续平滑调节）× 机动自适应UKF
%            （趋势检测+Q离散提升，对转弯响应更快）
%
% 【拐弯航迹设计】
%   三个航路点形成约120°拐角：
%     W1: (126.0°E, 32.5°N) — 起点（西南方向）
%     W2: (128.5°E, 33.5°N) — 拐点（入向~62°, 出向~182°）
%     W3: (128.6°E, 31.7°N) — 终点（南偏东）
%   航速140m/s（较直线场景230m/s降低，确保有足够帧数观察拐弯过程）
%   由aircraft_trajectory_create('turn')内部生成两个航段
%
% 【两组跟踪对比】
%   第5.1节 — 基础UKF：single_track_runner (ukf_jichu + 模糊自适应Q)
%   第5.2节 — 机动自适应：single_track_runner_adaptive (ukf_zishiying + 机动检测+Q提升)
%   两组使用完全相同的点迹输入（通过复位随机种子保证）
%
% 【两组融合】
%   两组跟踪结果分别做4种融合 → 共8种融合结果
%   Phase 8产出一张对比表：基础 vs 自适应 × (SCC/BC/CI/FCI/R1_only/R2_only)
%
% 【输出文件】
%   results/simulation_turn_YYYYMMDD_HHMMSS.mat — 完整仿真数据（含两组跟踪+融合）
%   results/fig* — 7张对比图表（见Phase 9）
%
% 【与run_simulation.m的关键区别】
%   1. 航迹类型：'turn' action → 拐弯航迹 vs 'straight' → 直线航迹
%   2. 跟踪器：额外运行一组自适应UKF（single_track_runner_adaptive）
%   3. UKF参数：gate_sigma=2.5（放宽门限，补偿转弯预测偏差）
%              tracker_K_loss=20（R1延长丢点容忍）
%   4. 评估：产出基础 vs 自适应的6行对比表 + 改善百分比
%   5. 可视化：使用plot_turn_spatial/plot_turn_stats代替通用plot函数
%   6. 航路点：输出turn_waypoints→传给可视化用于标记拐弯位置
%
% 【Phase流程】
%   Phase 0: 场景初始化 (拐弯航迹 + 覆盖检查 + 航路点 + 拐角计算)
%   Phase 1: ADS-B系统偏差离线标定（同run_simulation.m）
%   Phase 2: 原始点迹生成（R1/R2各自独立随机种子，确保两组一致）
%   Phase 3: 时间对齐策略声明（R1:0s/30s/60s, R2:13s/43s/73s）
%   Phase 4: 偏差校正 + 双基地几何反解（同run_simulation.m）
%   Phase 5: 单目标航迹跟踪 ← 核心：基础UKF vs 机动自适应UKF
%   Phase 6: 航迹级时间对齐（两组R2→R1对齐）
%   Phase 7: 航迹融合（两组×4种算法=8种融合结果）
%   Phase 8: 定量误差评估（对比表 + 最佳算法 + 单站对比）
%   Phase 9: 可视化（7张图） + 数据保存
%
% 【依赖关系】
%   入口：aircraft_trajectory_create('turn') → aircraft_trajectory_interpolate
%   跟踪：single_track_runner / single_track_runner_adaptive
%         → ukf_jichu / ukf_zishiying
%         → nn_associate / pda_weight
%   融合：time_align_tracks → run_track_fusion
%         → regularize_cov / align_radar_to_grid
%   评估：evaluate_all
%   可视化：plot_scene_overview / plot_turn_spatial / plot_turn_stats
%
% 【运行方式】直接在MATLAB命令行中执行: run_simulation_turn
%   或通过run_all.m统一调度运行
% =========================================================================

% 清空工作区、关闭图形窗口、清空命令行
clear; close all; clc;
% 将当前目录及所有子目录加入MATLAB搜索路径，确保所有模块函数可被调用
addpath(genpath('.'));

%% ==================== Phase 0: 场景初始化（拐弯目标） ====================
% 【与直线场景的区别】
%   1. 使用aircraft_trajectory_create('turn')代替aircraft_trajectory_create
%      'turn' action内部：预定义3个航路点 + 140m/s航速 → 调用aircraft_trajectory_create
%   2. 输出turn_waypoints矩阵（3×2），记录航路点供后续绘图使用
%   3. 计算拐角角度：bearing_out - bearing_in（入向→出向的方位变化）
%      azimuth(W1→W2) = bearing_in, azimuth(W2→W3) = bearing_out
%      拐角 = |bearing_out - bearing_in|（标准化到0-180°）
%   4. 覆盖率报告增加百分比显示（帮助判断拐弯是否仍在覆盖范围内）
%
% 【航迹结构体字段说明】
%   traj.segments{1} — 第一航段 W1→W2（入弯段）
%     .start=[126.0,32.5], .end=[128.5,33.5]
%     .lon_rate, .lat_rate — 经纬度变化率（deg/s），匀速运动
%   traj.segments{2} — 第二航段 W2→W3（出弯段）
%   traj.duration_sec — 总时长（秒）= seg1.dur + seg2.dur
%   traj.speed — 航速 140 m/s
%
% 【时间网格】与直线场景相同：R1采样0s/30s/60s, R2采样13s/43s/73s
fprintf('========== Phase 0: 场景初始化 (拐弯目标) ==========\n');

% 加载仿真参数（雷达位置、噪声参数、时间偏移等）
params = simulation_params();
% 固定随机种子，确保每次运行结果可复现
rng(params.random_seed);

% ---- 拐弯航迹生成 ----
% aircraft_trajectory_create('turn', params) 内部：
%   1. 定义三个航路点W1(126.0,32.5)、W2(128.5,33.5)、W3(128.6,31.7)
%   2. 计算航段1 (W1→W2) 和航段2 (W2→W3)的距离、方位、时长
%   3. 航速140m/s（比直线场景的230m/s低，以产生更多帧观察转弯过程）
% 返回值：
%   traj — 航迹结构体（含segments、duration_sec、speed、n_segments等字段）
%   turn_waypoints — 航路点矩阵 (3×2)，每行为 [lon, lat]
[traj, turn_waypoints] = aircraft_trajectory_create('turn', params);
% 将航迹结构体展开为逐秒采样点，用于真值对比和绘图
% true_track列：lon, lat, lon_rate, lat_rate, time_sec
true_track = aircraft_trajectory_interpolate('generate', traj);
fprintf('真实航迹 (拐弯): %d 点, 总时长 %.0f s, 速度 %.0f m/s\n', ...
    size(true_track,1), traj.duration_sec, traj.speed);
fprintf('  航路点 (%d个):\n', size(turn_waypoints,1));
for i = 1:size(turn_waypoints,1)
    fprintf('    W%d: (%.1f, %.1f)\n', i, turn_waypoints(i,1), turn_waypoints(i,2));
end

% ---- 验证拐角角度 ----
% 拐角是拐弯场景的核心参数，决定机动检测的难度
% 只有n_segments >= 2（即有至少两个航段）时才计算拐角
if traj.n_segments >= 2
    % 取第一航段和第二航段（W1→W2和W2→W3）
    seg1 = traj.segments{1};
    seg2 = traj.segments{2};
    % 入向方位角：从W1指向W2的方向（球面方位角，真北为0°，顺时针）
    bearing_in = sphere_utils_azimuth(seg1.start(1), seg1.start(2), seg1.end(1), seg1.end(2));
    % 出向方位角：从W2指向W3的方向
    bearing_out = sphere_utils_azimuth(seg2.start(1), seg2.start(2), seg2.end(1), seg2.end(2));
    % 拐角 = 出向方位 - 入向方位，取绝对值
    turn_angle = abs(bearing_out - bearing_in);
    % 标准化到0-180°范围（拐角不区分方向，取较小的一侧）
    if turn_angle > 180, turn_angle = 360 - turn_angle; end
    fprintf('  入向方位: %.1f°, 出向方位: %.1f°, 拐角: %.1f°\n', bearing_in, bearing_out, turn_angle);
end

% ---- 雷达覆盖检查 ----
% 逐点检查真实航迹点是否在R1和R2的覆盖范围内
% 覆盖条件：目标在波束方位角范围内 且 距离在最小/最大检测距离之间
% 拐弯场景下覆盖率可能略低于直线场景（目标可能飞出波束边缘）
n_in_r1 = 0; n_in_r2 = 0;  % 计数器：分别在R1和R2覆盖内的点数
for i = 1:size(true_track, 1)
    % R1覆盖检测：输入雷达位置、目标位置、波束中心方位角
    [in1, ~, ~] = radar_coverage_check(params.radar1_lon, params.radar1_lat, ...
        true_track(i,1), true_track(i,2), params.radar1_beam_center_deg, params);
    % R2覆盖检测
    [in2, ~, ~] = radar_coverage_check(params.radar2_lon, params.radar2_lat, ...
        true_track(i,1), true_track(i,2), params.radar2_beam_center_deg, params);
    if in1, n_in_r1 = n_in_r1 + 1; end
    if in2, n_in_r2 = n_in_r2 + 1; end
end
% 输出覆盖率（百分比），帮助判断拐弯是否仍在双雷达覆盖范围内
fprintf('  R1覆盖: %d/%d点 (%.0f%%), R2覆盖: %d/%d点 (%.0f%%)\n', ...
    n_in_r1, size(true_track,1), n_in_r1/size(true_track,1)*100, ...
    n_in_r2, size(true_track,1), n_in_r2/size(true_track,1)*100);

% ---- 时间网格构建 ----
% R1和R2使用相同的时间步长（params.dt_sec），但起始时间不同
% R1采样时刻：0s, 30s, 60s, 90s, ...
% R2采样时刻：13s, 43s, 73s, 103s, ...（偏移13s）
% n_frames取两者最小帧数，确保两个雷达有相同数量的输出帧
t1_grid = params.time_offset_radar1_sec : params.dt_sec : traj.duration_sec;
t2_grid = params.time_offset_radar2_sec : params.dt_sec : traj.duration_sec;
n_frames = min(length(t1_grid), length(t2_grid));
fprintf('仿真帧数: %d (dt=%.0fs)\n', n_frames, params.dt_sec);

% ---- 真值结构体构建 ----
% truthTraj用于后续误差评估，包含目标的label、速度、时间序列、位置和速度分量
% truthTrajs是truthTraj的cell数组（单目标场景只有一个元素）
tt = true_track;
truthTraj = struct('label', 'A', 'speed_ms', traj.speed, ...
    'time_sec', tt(:,5), 'lat', tt(:,2), 'lon', tt(:,1), ...
    'lon_rate', tt(:,3), 'lat_rate', tt(:,4));

%% ==================== Phase 1: ADS-B系统偏差标定（同run_simulation.m） ====================
% 【同run_simulation.m Phase 1】详见run_simulation.m对应注释
%  使用相同的ADS-B CSV数据、相同的均匀采样策略（最多5000点）、相同的均值估计方法
%  目的：离线估计R1和R2的量测系统偏差（距离偏差dr、方位角偏差da）
%  方法：对ADS-B合作目标（位置精确已知），计算雷达量测值与真值之差，取均值
%
%  关键变量：
%    dr1_list / da1_list — R1雷达的距离/方位角偏差样本（仅覆盖区内有效点）
%    dr2_list / da2_list — R2雷达的距离/方位角偏差样本
%    dr1_est / da1_est — R1的偏差估计值（样本均值）
%    dr2_est / da2_est — R2的偏差估计值（样本均值）
fprintf('\n========== Phase 1: ADS-B系统偏差标定 ==========\n');
% 重新设置随机种子，确保Phase 1的随机数与Phase 0无关
rng(params.random_seed);

fprintf('加载ADS-B合作目标: %s\n', params.adsb_csv_path);
% 读取ADS-B CSV文件（合作目标真值数据）
% Var2=纬度, Var3=经度（CSV文件格式：时间, 纬度, 经度, ...）
T_adsb = readtable(params.adsb_csv_path, 'ReadVariableNames', false);
adsb_lat = T_adsb.Var2;
adsb_lon = T_adsb.Var3;

% 初始化偏差样本列表（用于后续求均值）
dr1_list = []; da1_list = [];  % R1距离偏差、方位角偏差
dr2_list = []; da2_list = [];  % R2距离偏差、方位角偏差

% 均匀采样：最多检查5000个ADS-B点，计算采样步长
n_check = min(5000, height(T_adsb));
cal_step = max(1, floor(height(T_adsb) / n_check));

% 遍历ADS-B数据，收集覆盖区内的量测偏差样本
for idx = 1:cal_step:height(T_adsb)
    t_lon = adsb_lon(idx);  t_lat = adsb_lat(idx);  % ADS-B真值位置
    if isnan(t_lon) || isnan(t_lat), continue; end  % 跳过无效数据

    % ---- R1偏差样本采集 ----
    [in1, ~, ~] = radar_coverage_check(params.radar1_lon, params.radar1_lat, ...
        t_lon, t_lat, params.radar1_beam_center_deg, params);
    if in1
        % 天波双基地群距离真值：弦长+电离层虚高模型（与量测生成一致）
        Rg_true = skywave_geometry('group_range', params.radar1_tx_lon, params.radar1_tx_lat, ...
            params.radar1_lon, params.radar1_lat, t_lon, t_lat);
        % 方位角真值：接收站→目标
        az_true = sphere_utils_azimuth(params.radar1_lon, params.radar1_lat, t_lon, t_lat);
        % 模拟量测 = 真值 + 系统偏差 + 随机噪声
        Rg_meas = Rg_true + params.radar1_range_bias_m + randn() * params.radar1_range_noise_std_m;
        az_meas = az_true + params.radar1_azimuth_bias_deg + randn() * params.radar1_azimuth_noise_std_deg;
        % 偏差 = 量测 - 真值（包含系统偏差和噪声，均值即为系统偏差估计）
        dr1_list(end+1) = Rg_meas - Rg_true;
        daz = az_meas - az_true;
        % 方位角差值标准化到[-180, 180]
        if daz > 180, daz = daz - 360; elseif daz < -180, daz = daz + 360; end
        da1_list(end+1) = daz;
    end

    % ---- R2偏差样本采集（同R1逻辑） ----
    [in2, ~, ~] = radar_coverage_check(params.radar2_lon, params.radar2_lat, ...
        t_lon, t_lat, params.radar2_beam_center_deg, params);
    if in2
        Rg_true = skywave_geometry('group_range', params.radar2_tx_lon, params.radar2_tx_lat, ...
            params.radar2_lon, params.radar2_lat, t_lon, t_lat);
        az_true = sphere_utils_azimuth(params.radar2_lon, params.radar2_lat, t_lon, t_lat);
        Rg_meas = Rg_true + params.radar2_range_bias_m + randn() * params.radar2_range_noise_std_m;
        az_meas = az_true + params.radar2_azimuth_bias_deg + randn() * params.radar2_azimuth_noise_std_deg;
        dr2_list(end+1) = Rg_meas - Rg_true;
        daz = az_meas - az_true;
        if daz > 180, daz = daz - 360; elseif daz < -180, daz = daz + 360; end
        da2_list(end+1) = daz;
    end
end

% 偏差估计 = 样本均值（简单但有效的无偏估计）
dr1_est = mean(dr1_list);  da1_est = mean(da1_list);
dr2_est = mean(dr2_list);  da2_est = mean(da2_list);
fprintf('ADS-B标校点数: R1=%d, R2=%d\n', length(dr1_list), length(dr2_list));
fprintf('R1: dr_est=%.1f (true=%.0f) m, da_est=%.4f (true=%.1f) deg\n', ...
    dr1_est, params.radar1_range_bias_m, da1_est, params.radar1_azimuth_bias_deg);
fprintf('R2: dr_est=%.1f (true=%.0f) m, da_est=%.4f (true=%.1f) deg\n', ...
    dr2_est, params.radar2_range_bias_m, da2_est, params.radar2_azimuth_bias_deg);

%% ==================== Phase 2: 原始点迹生成（同run_simulation.m） ====================
% 【同run_simulation.m Phase 2】详见run_simulation.m对应注释
%  为每帧生成R1和R2的原始量测（含系统偏差+随机噪声+杂波）
%  关键：每个雷达使用独立的随机种子（R1: seed+k, R2: seed+10000+k）
%         确保R1和R2的量测噪声独立，但同一雷达在多次运行中可复现
%  点迹结构体字段：
%    .prange — 原始（未校正）双基地距离量测 (m)
%    .paz    — 原始（未校正）方位角量测 (deg)
%    .aircraft_id — 目标ID（此处固定为1，单目标场景）
fprintf('\n========== Phase 2: 原始点迹生成 ==========\n');

% 预分配点迹cell数组（每帧一个cell，每个cell内含该帧的所有检测点迹）
detRaw_R1 = cell(n_frames, 1);  % R1原始点迹（偏差校正前）
detRaw_R2 = cell(n_frames, 1);  % R2原始点迹（偏差校正前）

for k = 1:n_frames
    % ---- R1第k帧点迹生成 ----
    % 独立随机种子 = params.random_seed + k
    % 不同帧使用不同种子，但相同帧号每次运行结果一致
    rng(params.random_seed + k);
    % 插值获取第k帧R1采样时刻（t1_grid(k)）的真实位置和速度
    [pos, vel] = aircraft_trajectory_interpolate(traj, t1_grid(k));
    % 生成R1的单帧检测点迹（含量测噪声+杂波+检测概率模拟）
    detRaw_R1{k} = generate_frame_detections(params.radar1_lon, params.radar1_lat, ...
        params.radar1_tx_lon, params.radar1_tx_lat, ...
        pos(1), pos(2), vel(1), vel(2), k, t1_grid(k), ...
        params.radar1_range_bias_m, params.radar1_azimuth_bias_deg, ...
        params.radar1_beam_center_deg, params, ...
        params.radar1_range_noise_std_m, params.radar1_azimuth_noise_std_deg);
    % 标记点迹的目标ID
    for d = 1:length(detRaw_R1{k})
        detRaw_R1{k}(d).aircraft_id = 1;
    end

    % ---- R2第k帧点迹生成 ----
    % R2使用独立随机种子 = params.random_seed + 10000 + k
    % +10000确保与R1的随机序列完全独立
    rng(params.random_seed + 10000 + k);
    [pos2, vel2] = aircraft_trajectory_interpolate(traj, t2_grid(k));
    detRaw_R2{k} = generate_frame_detections(params.radar2_lon, params.radar2_lat, ...
        params.radar2_tx_lon, params.radar2_tx_lat, ...
        pos2(1), pos2(2), vel2(1), vel2(2), k, t2_grid(k), ...
        params.radar2_range_bias_m, params.radar2_azimuth_bias_deg, ...
        params.radar2_beam_center_deg, params, ...
        params.radar2_range_noise_std_m, params.radar2_azimuth_noise_std_deg);
    for d = 1:length(detRaw_R2{k})
        detRaw_R2{k}(d).aircraft_id = 1;
    end
end

fprintf('原始点迹生成完成: R1共%d帧, R2共%d帧\n', n_frames, n_frames);

%% ==================== Phase 3: 时间对齐策略（同run_simulation.m） ====================
% 【同run_simulation.m Phase 3】详见run_simulation.m对应注释
%  此处仅声明时间对齐策略，不执行实际对齐操作
%  R1采样时刻：0s, 30s, 60s, 90s, ...（从仿真起始时刻开始，步长=dt_sec）
%  R2采样时刻：13s, 43s, 73s, 103s, ...（从仿真起始+偏移时刻开始）
%  时间对齐在Phase 6执行：将R2航迹外推/内插到R1时间网格上
fprintf('\n========== Phase 3: 时间对齐策略 ==========\n');
fprintf('R1采样: 0s/30s/30s/...  R2采样: 13s/43s/73s/...  偏移=%ds\n', ...
    params.time_offset_radar2_sec);

%% ==================== Phase 4: 偏差校正 + 几何反解（同run_simulation.m） ====================
% 【同run_simulation.m Phase 4】详见run_simulation.m对应注释
%  对原始点迹进行两步处理：
%   1. 偏差校正：prange - dr_est, paz - da_est（减去Phase 1估计的系统偏差）
%      → 得到校正后的距离(drange)和方位角(daz)
%   2. 双基地几何反解：从(range, azimuth)求解(lon, lat)
%      调用bistatic_inverse_solver → 双基地椭球与方位线相交求解
%  同时保存原始量测的定位结果(raw_lat, raw_lon)用于对比
%
%  输出：
%    detList_R1/R2 — 校正后的点迹（每帧一个cell），用于后续跟踪
%    每个点迹新增字段：
%      .drange / .daz — 校正后的距离/方位角
%      .range_meas / .azimuth_meas — 同drange/daz（跟踪器使用的量测字段名）
%      .lat / .lon — 校正后的定位结果（经纬度）
%      .raw_lat / .raw_lon — 原始（未校正）的定位结果
fprintf('\n========== Phase 4: 偏差校正 ==========\n');

% 预分配校正后点迹cell数组
detList_R1 = cell(n_frames, 1);
detList_R2 = cell(n_frames, 1);

for k = 1:n_frames
    % ---- R1第k帧偏差校正 + 几何反解 ----
    dets_r1 = detRaw_R1{k};
    for d = 1:length(dets_r1)
        % 距离校正：原始量测 - 估计偏差
        Rgc = dets_r1(d).prange - dr1_est;
        % 方位角校正：原始量测 - 估计偏差
        azc = dets_r1(d).paz - da1_est;
        dets_r1(d).drange = Rgc;
        dets_r1(d).daz = azc;
        % 同步更新跟踪器使用的量测字段
        dets_r1(d).range_meas = Rgc;
        dets_r1(d).azimuth_meas = azc;
        % 双基地几何反解：将校正后的(range, az)转换为(lat, lon)
        % 如果点迹已有lat/lon且有效（非NaN），则保留；否则重新解算
        if ~(isfield(dets_r1(d), 'lat') && ~isnan(dets_r1(d).lat))
            [~, lat_e, lon_e] = bistatic_inverse_solver(Rgc, azc, ...
                params.radar1_tx_lon, params.radar1_tx_lat, ...
                params.radar1_lon, params.radar1_lat);
            dets_r1(d).lat = lat_e;
            dets_r1(d).lon = lon_e;
        end
        % 同时计算原始量测的定位结果（用于对比偏差校正效果）
        [~, raw_lat, raw_lon] = bistatic_inverse_solver(dets_r1(d).prange, dets_r1(d).paz, ...
            params.radar1_tx_lon, params.radar1_tx_lat, ...
            params.radar1_lon, params.radar1_lat);
        dets_r1(d).raw_lat = raw_lat;
        dets_r1(d).raw_lon = raw_lon;
    end
    detList_R1{k} = dets_r1;

    % ---- R2第k帧偏差校正 + 几何反解 ----
    dets_r2 = detRaw_R2{k};
    for d = 1:length(dets_r2)
        Rgc = dets_r2(d).prange - dr2_est;
        azc = dets_r2(d).paz - da2_est;
        dets_r2(d).drange = Rgc;
        dets_r2(d).daz = azc;
        dets_r2(d).range_meas = Rgc;
        dets_r2(d).azimuth_meas = azc;
        if ~(isfield(dets_r2(d), 'lat') && ~isnan(dets_r2(d).lat))
            [~, lat_e, lon_e] = bistatic_inverse_solver(Rgc, azc, ...
                params.radar2_tx_lon, params.radar2_tx_lat, ...
                params.radar2_lon, params.radar2_lat);
            dets_r2(d).lat = lat_e;
            dets_r2(d).lon = lon_e;
        end
        [~, raw_lat, raw_lon] = bistatic_inverse_solver(dets_r2(d).prange, dets_r2(d).paz, ...
            params.radar2_tx_lon, params.radar2_tx_lat, ...
            params.radar2_lon, params.radar2_lat);
        dets_r2(d).raw_lat = raw_lat;
        dets_r2(d).raw_lon = raw_lon;
    end
    detList_R2{k} = dets_r2;
end

fprintf('偏差校正完成: R1=%d帧, R2=%d帧\n', n_frames, n_frames);

%% ---- 点迹定位RMSE统计 ----
fprintf('\n--- 点迹定位RMSE ---\n');

% R1 原始点迹（含偏差）
errs = [];
for k = 1:n_frames
    tl = interp1(true_track(:,5), true_track(:,1), t1_grid(k), 'linear', 'extrap');
    tb = interp1(true_track(:,5), true_track(:,2), t1_grid(k), 'linear', 'extrap');
    for d = 1:length(detList_R1{k})
        dp = detList_R1{k}(d);
        if ~dp.is_clutter && isfield(dp,'raw_lat') && ~isnan(dp.raw_lat)
            errs(end+1) = sphere_utils_haversine_distance(dp.raw_lon, dp.raw_lat, tl, tb) / 1000;
        end
    end
end
fprintf('R1 原始点迹(含偏差)    RMSE: %6.1f km (n=%d)\n', rms_km(errs), length(errs));

% R2 原始点迹
errs = [];
for k = 1:n_frames
    tl = interp1(true_track(:,5), true_track(:,1), t2_grid(k), 'linear', 'extrap');
    tb = interp1(true_track(:,5), true_track(:,2), t2_grid(k), 'linear', 'extrap');
    for d = 1:length(detList_R2{k})
        dp = detList_R2{k}(d);
        if ~dp.is_clutter && isfield(dp,'raw_lat') && ~isnan(dp.raw_lat)
            errs(end+1) = sphere_utils_haversine_distance(dp.raw_lon, dp.raw_lat, tl, tb) / 1000;
        end
    end
end
fprintf('R2 原始点迹(含偏差)    RMSE: %6.1f km (n=%d)\n', rms_km(errs), length(errs));

% R1 校准后点迹
errs = [];
for k = 1:n_frames
    tl = interp1(true_track(:,5), true_track(:,1), t1_grid(k), 'linear', 'extrap');
    tb = interp1(true_track(:,5), true_track(:,2), t1_grid(k), 'linear', 'extrap');
    for d = 1:length(detList_R1{k})
        dp = detList_R1{k}(d);
        if ~dp.is_clutter && isfield(dp,'lat') && ~isnan(dp.lat)
            errs(end+1) = sphere_utils_haversine_distance(dp.lon, dp.lat, tl, tb) / 1000;
        end
    end
end
fprintf('R1 校准后点迹          RMSE: %6.1f km (n=%d)\n', rms_km(errs), length(errs));

% R2 校准后点迹
errs = [];
for k = 1:n_frames
    tl = interp1(true_track(:,5), true_track(:,1), t2_grid(k), 'linear', 'extrap');
    tb = interp1(true_track(:,5), true_track(:,2), t2_grid(k), 'linear', 'extrap');
    for d = 1:length(detList_R2{k})
        dp = detList_R2{k}(d);
        if ~dp.is_clutter && isfield(dp,'lat') && ~isnan(dp.lat)
            errs(end+1) = sphere_utils_haversine_distance(dp.lon, dp.lat, tl, tb) / 1000;
        end
    end
end
fprintf('R2 校准后点迹          RMSE: %6.1f km (n=%d)\n', rms_km(errs), length(errs));

%% ==================== Phase 5: 航迹跟踪（基础UKF + 机动自适应UKF） ====================
% 【目的】这是本程序的核心对比实验：两种UKF策略在相同点迹输入下的表现
%
% 【5.1 基础UKF（模糊自适应Q）】
%   使用 single_track_runner（同run_simulation.m Phase 5）
%   内部调用链：
%     ukf_jichu('prepare') → 初始化Sigma点、预测状态和协方差
%     nn_associate → 最近邻关联（选择门内最近点迹）
%     pda_weight → 概率数据关联（计算关联权重）
%     ukf_jichu('update') → Kalman更新（状态+协方差）
%     apply_fuzzy_adapt → 模糊自适应Q（NIS滑动平均→模糊推理→连续Q缩放）
%   模糊自适应特点：Q连续平滑调节，对渐变机动有效，对突变转弯响应较慢
%
% 【5.2 机动自适应UKF（新息序列机动检测+Q提升）】
%   使用 single_track_runner_adaptive ← 这是与基础版的关键区别
%   内部调用链：
%     ukf_jichu('prepare') → nn_associate → pda_weight
%        → ukf_zishiying('update') ← 机动自适应UKF
%   ukf_zishiying内部流程：
%     1. 调用ukf_jichu('update')执行标准Kalman更新（状态预测+量测更新）
%     2. 执行机动检测+自适应Q：
%        a. 维护短时NIS窗口(3帧)和长时NIS窗口(5帧)
%        b. 机动判定条件（同时满足）：
%           - 短时NIS均值 > 长时NIS均值 × 1.25（新息显著增大）
%           - 短时NIS均值 > 2.8（超过2自由度chi-square 75%分位点）
%        c. Q渐进提升策略（非瞬间跳跃，避免协方差突变导致发散）：
%           Q_scale × 1.5 → Q_scale × 2.3 → Q_scale × 3.1 → Q_scale × 3.5
%           每次提升需持续2帧确认，逐级递进
%        d. 机动结束条件：连续4帧NIS恢复正常 → Q恢复至Q_scale × 1.0
%     3. 与模糊自适应的区别：模糊自适应是连续平滑调节，机动自适应是
%        离散级别提升 → 对转弯等突变机动响应更快、更大幅度
%
% 【随机种子复位】
%   在5.2节开始前复位rng到与5.1节相同的初始状态
%   目的：确保两组使用完全相同的点迹输入（detList在Phase 2-4已生成完毕）
%   注意：detList的生成使用了rng，但跟踪器内部不消耗全局随机数流
%   （UKF的Sigma点生成使用确定性算法，NN关联/PDA是确定性的）
%
% 【UKF模板重建】
%   重新调用ukf_jichu('create')创建全新的UKF模板
%   原因：虽然MATLAB struct在函数调用中是值传递（不会修改调用者副本），
%   但5.1节运行后ukf1_tpl内部的params可能被single_track_runner修改，
%   显式重建更安全，避免任何潜在的参数污染
%
% 【R1 UKF参数（精密站，拐弯场景参数调整）】
%   ukf_Q_scale = 5e4
%     — 过程噪声缩放因子，取值较大因为拐弯场景模型失配更严重
%   ukf_P_pos_std = 0.2°
%     — 初始位置不确定度（经纬度标准差），约22km@equator
%   ukf_P_vel_std = 0.004°/s
%     — 初始速度不确定度（经纬度速率标准差）
%   gate_sigma = 2.5
%     — 关联门限因子（标准差倍数），从直线场景的2.0放宽到2.5
%       原因是拐弯时预测位置偏差更大，需要更宽的门限避免漏关联
%       但门太宽会增加杂波关联风险，2.5是一个折中值
%   tracker_K_loss = 20
%     — 连续丢点容忍帧数，从默认值提升到20
%       拐弯期间目标可能短暂飞出波束覆盖（覆盖盲区），
%       延长丢点容忍避免航迹过早终止
%
% 【R2 UKF参数（普通站）】
%   ukf_Q_scale = 1e5
%     — R2的量测精度低于R1，需要更大的过程噪声补偿
%   ukf_P_pos_std = 0.3°, ukf_P_vel_std = 0.005°/s
%     — R2初始不确定度大于R1（量测精度差导致初始化精度低）
%   gate_sigma = 2.5 — 与R1相同
%   tracker_M = 4, tracker_N = 8 — 航迹起始逻辑（M/N检测）
%   tracker_K_loss = 12 — R2丢点容忍略低于R1（R2覆盖可能更稳定）
%
% 【机动诊断统计】
%   遍历自适应UKF的快照，统计：
%   - 起始帧 (init_frame)：航迹从TEMPORARY(6)转为RELIABLE(1)的帧号
%   - 关联帧数 (n_assoc)：有有效量测关联的帧数
%   - 预测帧数 (n_predict)：无量测、仅靠状态预测外推的帧数
%   - 关联率 = 关联帧 / (关联帧+预测帧)
%   - 机动帧数 (n_maneuver)：maneuver_active=true的帧数
%   - 机动持续区间 [起始帧, 结束帧]
%   - NIS均值 和 门内比例（chi-square门限=4，2自由度95%分位点）
%     NIS(归一化新息平方) = ν' * S⁻¹ * ν，反映量测与预测的一致性
%     NIS值过大 → 模型失配（机动发生），NIS值过小 → 量测噪声估计偏大
fprintf('\n========== Phase 5: 航迹跟踪（基础UKF + 机动自适应UKF） ==========\n');

% ======================================================================
% R1 UKF模板配置（精密站，V2调优后）
% ======================================================================
% 将R1的噪声参数赋给params的UKF字段
params.ukf_range_std_m    = params.radar1_range_noise_std_m;
params.ukf_azimuth_std_deg = params.radar1_azimuth_noise_std_deg;
params.ukf_Q_scale     = params.radar1_ukf_Q_scale;
params.ukf_P_pos_std   = params.radar1_ukf_P_pos_std;
params.ukf_P_vel_std   = params.radar1_ukf_P_vel_std;
params.gate_sigma      = params.radar1_gate_sigma;
params.tracker_K_loss  = params.radar1_tracker_K_loss;

% 创建R1 UKF模板（含状态维度、Sigma点权重、量测函数句柄等）
% ukf_jichu('create') 返回的模板包含：状态转移矩阵F、过程噪声Q结构、
%   量测函数h_func（双基地几何）、初始化函数等
ukf1_tpl = ukf_jichu('create', params, params.radar1_lon, params.radar1_lat, ...
    params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);

% ======================================================================
% R2 UKF模板配置（普通站，V2调优后）
% ======================================================================
% 使用params_r2作为R2的独立参数副本（避免与R1参数混淆）
params_r2 = params;
params_r2.ukf_range_std_m    = params.radar2_range_noise_std_m;
params_r2.ukf_azimuth_std_deg = params.radar2_azimuth_noise_std_deg;
params_r2.gate_sigma      = params.radar2_gate_sigma;
params_r2.ukf_Q_scale     = params.radar2_ukf_Q_scale;
params_r2.ukf_P_pos_std   = params.radar2_ukf_P_pos_std;
params_r2.ukf_P_vel_std   = params.radar2_ukf_P_vel_std;
params_r2.tracker_M       = 4;
params_r2.tracker_N       = 8;
params_r2.tracker_K_loss  = params.radar2_tracker_K_loss;

% 创建R2 UKF模板
ukf2_tpl = ukf_jichu('create', params_r2, params.radar2_lon, params.radar2_lat, ...
    params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);

% ======================================================================
% 第5.1节：基础UKF跟踪（模糊自适应Q）
% ======================================================================
% single_track_runner 内部流程：
%   1. 逐帧处理detList中的点迹
%   2. 航迹起始：M/N逻辑 → TEMPORARY → RELIABLE
%   3. 每帧：ukf预测 → 最近邻关联 → PDA加权 → ukf更新 → 模糊自适应Q
%   4. 航迹终止：连续K_loss帧无关联 → HISTORY → TERMINATED
% 返回值：
%   trackSnapshots — cell数组(1×n_frames)，每帧的快照含trackList
%   finalTrk — 最终航迹结构体(.type, .quality, .life)
fprintf('--- 5.1 基础UKF (模糊自适应Q) ---\n');
[trackSnapshots_R1, finalTrk1] = single_track_runner(detList_R1, ukf1_tpl, params, n_frames);
[trackSnapshots_R2, finalTrk2] = single_track_runner(detList_R2, ukf2_tpl, params_r2, n_frames);

% 打印R1/R2基础UKF的航迹状态
% type: 1=RELIABLE(稳定), 2=MAINTAIN(维持), 6=TEMPORARY(临时), 7=HISTORY
% quality: 航迹质量评分（基于关联率和连续性）
% life: 航迹存活帧数
fprintf('R1基础UKF: type=%s quality=%d life=%d\n', ...
    get_type_str(finalTrk1.type), finalTrk1.quality, finalTrk1.life);
fprintf('R2基础UKF: type=%s quality=%d life=%d\n', ...
    get_type_str(finalTrk2.type), finalTrk2.quality, finalTrk2.life);

%% ---- 基础UKF滤波RMSE统计 ----
fprintf('\n--- 基础UKF滤波RMSE ---\n');

% R1 基础UKF航迹
errs = [];
for k = 1:n_frames
    tl = interp1(true_track(:,5), true_track(:,1), t1_grid(k), 'linear', 'extrap');
    tb = interp1(true_track(:,5), true_track(:,2), t1_grid(k), 'linear', 'extrap');
    snap = trackSnapshots_R1{k};
    if ~isempty(snap.trackList)
        trk = snap.trackList{1};
        if trk.type ~= 7 && ~isnan(trk.lat)
            errs(end+1) = sphere_utils_haversine_distance(trk.lon, trk.lat, tl, tb) / 1000;
        end
    end
end
fprintf('R1 基础UKF滤波         RMSE: %6.1f km (n=%d)\n', rms_km(errs), length(errs));

% R2 基础UKF航迹（R2时间网格）
errs = [];
for k = 1:n_frames
    tl = interp1(true_track(:,5), true_track(:,1), t2_grid(k), 'linear', 'extrap');
    tb = interp1(true_track(:,5), true_track(:,2), t2_grid(k), 'linear', 'extrap');
    snap = trackSnapshots_R2{k};
    if ~isempty(snap.trackList)
        trk = snap.trackList{1};
        if trk.type ~= 7 && ~isnan(trk.lat)
            errs(end+1) = sphere_utils_haversine_distance(trk.lon, trk.lat, tl, tb) / 1000;
        end
    end
end
fprintf('R2 基础UKF滤波         RMSE: %6.1f km (n=%d)\n', rms_km(errs), length(errs));

% ======================================================================
% 第5.2节：机动自适应UKF跟踪（新息序列机动检测 + Q离散提升）
% ======================================================================
fprintf('\n--- 5.2 机动自适应UKF (新息序列机动检测+Q提升) ---\n');

% ---- 复位随机种子 + 重建UKF模板 ----
% 复位到与5.1节初始状态相同的随机种子
% 目的：确保detList_R1/R2的生成过程在两组中完全相同
% （虽然detList已在Phase 2-4生成完毕，但此处保守复位）
rng(params.random_seed);
% 重建UKF模板（避免5.1节运行后params可能被修改）
% 注意：虽然MATLAB函数调用是值传递，但params结构体在传递过程中
% 可能被接收函数修改其内部字段，显式重建可确保5.2节使用的是干净模板
ukf1_tpl_ad = ukf_jichu('create', params, params.radar1_lon, params.radar1_lat, ...
    params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
ukf2_tpl_ad = ukf_jichu('create', params_r2, params.radar2_lon, params.radar2_lat, ...
    params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);

% ---- 运行机动自适应UKF ----
% single_track_runner_adaptive 与 single_track_runner 的区别：
%   在ukf更新步骤中调用 ukf_zishiying('update') 而不是 ukf_jichu('update')
%   ukf_zishiying 内部会额外执行机动检测和Q缩放
% 其他流程（航迹起始、NN关联、PDA加权、航迹终止）完全相同
[trackSnapshots_R1_ad, finalTrk1_ad] = single_track_runner_adaptive(detList_R1, ukf1_tpl_ad, params, n_frames);
[trackSnapshots_R2_ad, finalTrk2_ad] = single_track_runner_adaptive(detList_R2, ukf2_tpl_ad, params_r2, n_frames);

fprintf('R1自适应UKF: type=%s quality=%d life=%d\n', ...
    get_type_str(finalTrk1_ad.type), finalTrk1_ad.quality, finalTrk1_ad.life);
fprintf('R2自适应UKF: type=%s quality=%d life=%d\n', ...
    get_type_str(finalTrk2_ad.type), finalTrk2_ad.quality, finalTrk2_ad.life);

% ======================================================================
% 机动检测诊断统计
% ======================================================================
% 遍历自适应UKF的每帧快照，统计机动检测相关的诊断信息
% 这些统计帮助理解：
%   1. 机动检测的触发时机（是否在拐弯附近触发）
%   2. 机动检测持续时间（是否在拐弯结束后及时恢复）
%   3. NIS一致性（检测门限设置是否合理）
fprintf('\n--- 机动自适应UKF诊断 ---\n');
% 分别处理R1和R2
for radar_label = {'R1', 'R2'}
    snaps = trackSnapshots_R1_ad;
    rname = 'R1';
    if strcmp(radar_label{1}, 'R2'), snaps = trackSnapshots_R2_ad; rname = 'R2'; end

    % 初始化统计变量
    n_maneuver = 0;      % 机动激活帧计数
    n_assoc = 0;         % 成功关联帧计数
    n_predict = 0;       % 纯预测帧计数（无关联量测）
    nis_vals = [];       % NIS值列表（用于计算均值和门内比例）
    init_frame = 0;      % 航迹起始帧号（首次type==1的帧）
    maneuver_frames = [];% 机动帧号列表

    for k = 1:length(snaps)
        % 跳过无航迹的帧（仿真早期，航迹尚未起始）
        if isempty(snaps{k}.trackList), continue; end
        trk = snaps{k}.trackList{1};  % 单目标场景，取第一个（也是唯一的）航迹
        % 仅统计RELIABLE(1)状态的帧
        if trk.type == 1
            % 检查是否有有效量测关联
            % assoc_det非空且含prange字段 → 有关联量测
            if ~isempty(trk.assoc_det) && isstruct(trk.assoc_det) && ...
                    isfield(trk.assoc_det, 'prange') && ~isempty(trk.assoc_det.prange)
                n_assoc = n_assoc + 1;   % 有关联量测
            else
                n_predict = n_predict + 1; % 纯预测（无量测）
            end
            % 收集NIS历史值（归一化新息平方）
            if isfield(trk.ukf, 'nis_history')
                nis_vals = [nis_vals, trk.ukf.nis_history];
            end
            % 检查机动激活标志
            if isfield(trk.ukf, 'maneuver_active') && trk.ukf.maneuver_active
                n_maneuver = n_maneuver + 1;
                maneuver_frames(end+1) = k;  % 记录机动帧号
            end
        elseif trk.type == 6 && init_frame == 0
            % type==6是TEMPORARY状态，航迹正在初始化
            % 不参与统计
        end
        % 记录首次转为RELIABLE的帧号（航迹起始帧）
        if init_frame == 0 && trk.type == 1, init_frame = k; end
    end

    % 输出诊断统计结果
    n_tracked = n_assoc + n_predict;  % 总跟踪帧数
    fprintf('%s自适应: 起始帧=%d | 关联=%d (%.0f%%) | 机动帧=%d', ...
        rname, init_frame, n_assoc, n_assoc/max(1,n_tracked)*100, n_maneuver);
    % 如果有机动帧，输出机动持续区间
    if ~isempty(maneuver_frames)
        fprintf(' [%d-%d]', maneuver_frames(1), maneuver_frames(end));
    end
    fprintf('\n');
    % NIS统计：均值反映模型匹配程度，门内比例反映滤波器一致性
    % chi-square门限=4（2自由度，约86%分位点）
    % 门内比例理论上应接近86%，过低说明模型失配严重
    if ~isempty(nis_vals)
        nis_in_gate = sum(nis_vals < 4*2);  % 乘2考虑双雷达量测维度
        fprintf('  NIS: 均值=%.2f 门内=%.0f%% (%d/%d)\n', ...
            mean(nis_vals), nis_in_gate/length(nis_vals)*100, nis_in_gate, length(nis_vals));
    end
end

%% ---- 机动自适应UKF滤波RMSE统计 ----
fprintf('\n--- 机动自适应UKF滤波RMSE ---\n');

% R1 自适应UKF航迹
errs = [];
for k = 1:n_frames
    tl = interp1(true_track(:,5), true_track(:,1), t1_grid(k), 'linear', 'extrap');
    tb = interp1(true_track(:,5), true_track(:,2), t1_grid(k), 'linear', 'extrap');
    snap = trackSnapshots_R1_ad{k};
    if ~isempty(snap.trackList)
        trk = snap.trackList{1};
        if trk.type ~= 7 && ~isnan(trk.lat)
            errs(end+1) = sphere_utils_haversine_distance(trk.lon, trk.lat, tl, tb) / 1000;
        end
    end
end
fprintf('R1 自适应UKF滤波       RMSE: %6.1f km (n=%d)\n', rms_km(errs), length(errs));

% R2 自适应UKF航迹（R2时间网格）
errs = [];
for k = 1:n_frames
    tl = interp1(true_track(:,5), true_track(:,1), t2_grid(k), 'linear', 'extrap');
    tb = interp1(true_track(:,5), true_track(:,2), t2_grid(k), 'linear', 'extrap');
    snap = trackSnapshots_R2_ad{k};
    if ~isempty(snap.trackList)
        trk = snap.trackList{1};
        if trk.type ~= 7 && ~isnan(trk.lat)
            errs(end+1) = sphere_utils_haversine_distance(trk.lon, trk.lat, tl, tb) / 1000;
        end
    end
end
fprintf('R2 自适应UKF滤波       RMSE: %6.1f km (n=%d)\n', rms_km(errs), length(errs));

%% ==================== Phase 6: 航迹级时间对齐（两组R2→R1） ====================
% 【注意】此处有两组R2航迹需要对齐：基础版 + 自适应版
%   aligned_R2    — 基础UKF的R2航迹对齐到R1时间网格
%   aligned_R2_ad — 自适应UKF的R2航迹对齐到R1时间网格
%
%   time_align_tracks 内部逻辑：
%     1. 从trackSnapshots中提取R2的航迹状态（位置+速度+协方差）
%     2. 使用CV模型（匀速直线运动）将状态外推到R1的采样时刻
%        R2采样13s, 43s, 73s → 外推到R1的0s, 30s, 60s（外推量=13s偏移的反向）
%     3. 外推后的状态保留协方差矩阵（考虑外推期间的协方差膨胀）
%     4. 注意：外推使用了从校正后点迹反解得到的地理位置(lat/lon)，
%        而非双基地距离/方位角，因为对齐需要在统一的笛卡尔/地理坐标系下进行
%  两组使用相同的偏移量(13s)和相同的CV模型外推方法
fprintf('\n========== Phase 6: 航迹级时间对齐 ==========\n');

% 基础UKF的R2航迹对齐
aligned_R2 = time_align_tracks(trackSnapshots_R2, params);
% 自适应UKF的R2航迹对齐（参数相同，时间偏移相同）
aligned_R2_ad = time_align_tracks(trackSnapshots_R2_ad, params);
fprintf('R2航迹时间对齐完成 (基础+自适应)\n');

%% ==================== Phase 7: 航迹融合（两组 × 四种算法） ====================
% 【两组融合】
%   基础UKF融合：trackSnapshots_R1 + aligned_R2 → 4种算法
%   自适应UKF融合：trackSnapshots_R1_ad + aligned_R2_ad → 4种算法
%   共8种融合结果
%
% 【融合对结构】
%   matched_pair直接指定R1#1↔R2#1（单目标1对1匹配）
%   字段说明：
%     .R1_track_id / .R2_track_id — 关联的航迹ID
%     .match_count — 两航迹在相同帧共同出现的帧数
%     .coexist_count — 两航迹共存的总帧数
%     .match_ratio — 匹配率 = match_count/coexist_count（此处1.0表示完全匹配）
%     .mean_dist_km — 两航迹在共存帧的平均位置距离
%     .quality — 匹配质量评分（100为最高）
%
% 【四种融合算法说明】
%   SCC  (Simple Convex Combination) — 简单凸组合：协方差加权平均
%   BC   (Bar-Shalom Campo)         — 考虑互协方差的融合
%   CI   (Covariance Intersection)  — 协方差交叉：避免互协方差低估，融合结果保守
%   FCI  (Fast Covariance Intersection) — 快速协方差交叉：计算效率更高的CI变体
%
%   run_track_fusion 内部：
%     1. 逐帧遍历：找到matched_pair中指定ID的航迹对
%     2. 调用align_radar_to_grid：将R1/R2的协方差对齐到统一网格
%     3. 调用regularize_cov：协方差正则化（保证正定性、数值稳定）
%     4. 执行对应算法的融合公式
%     5. 返回融合后的快照cell数组
fprintf('\n========== Phase 7: 航迹融合 ==========\n');

% 构建匹配对：R1的第一个航迹 ↔ R2的第一个航迹
% 单目标场景下，R1只有一条航迹(id=1)，R2也只有一条航迹(id=1)
matched_pair = struct('R1_track_id', 1, 'R2_track_id', 1, ...
    'match_count', 0, 'coexist_count', 0, 'match_ratio', 1.0, ...
    'mean_dist_km', 0, 'quality', 100);

% 四种融合算法名称
method_names = {'SCC', 'BC', 'CI', 'FCI'};

% ---- 基础UKF融合（4种算法） ----
% all_fused_snapshots是一个cell数组，长度为4
% all_fused_snapshots{m}是第m种融合算法的融合快照cell数组(1×n_frames)
% 每个融合快照含 .trackList{1}.lon/.lat（融合后的位置）
all_fused_snapshots = cell(length(method_names), 1);
for m = 1:length(method_names)
    all_fused_snapshots{m} = run_track_fusion(matched_pair, ...
        trackSnapshots_R1, aligned_R2, params, method_names{m});
end

% ---- 机动自适应UKF融合（4种算法） ----
% all_fused_snapshots_ad的结构与all_fused_snapshots相同
% 区别在于输入的自适应UKF航迹质量可能更好（转弯处误差更小）
all_fused_snapshots_ad = cell(length(method_names), 1);
for m = 1:length(method_names)
    all_fused_snapshots_ad{m} = run_track_fusion(matched_pair, ...
        trackSnapshots_R1_ad, aligned_R2_ad, params, method_names{m});
end

fprintf('融合完成: 基础UKF 4种 + 自适应UKF 4种\n');

%% ---- 融合RMSE统计 ----
fprintf('\n--- 基础UKF融合RMSE ---\n');
for m = 1:length(method_names)
    errs = [];
    snaps = all_fused_snapshots{m};
    for k = 1:n_frames
        tl = interp1(true_track(:,5), true_track(:,1), t1_grid(k), 'linear', 'extrap');
        tb = interp1(true_track(:,5), true_track(:,2), t1_grid(k), 'linear', 'extrap');
        if ~isempty(snaps{k}) && ~isempty(snaps{k}.trackList)
            trk = snaps{k}.trackList{1};
            if ~isnan(trk.lat)
                errs(end+1) = sphere_utils_haversine_distance(trk.lon, trk.lat, tl, tb) / 1000;
            end
        end
    end
    fprintf('基础 %s 融合           RMSE: %6.1f km (n=%d)\n', method_names{m}, rms_km(errs), length(errs));
end

fprintf('\n--- 自适应UKF融合RMSE ---\n');
for m = 1:length(method_names)
    errs = [];
    snaps = all_fused_snapshots_ad{m};
    for k = 1:n_frames
        tl = interp1(true_track(:,5), true_track(:,1), t1_grid(k), 'linear', 'extrap');
        tb = interp1(true_track(:,5), true_track(:,2), t1_grid(k), 'linear', 'extrap');
        if ~isempty(snaps{k}) && ~isempty(snaps{k}.trackList)
            trk = snaps{k}.trackList{1};
            if ~isnan(trk.lat)
                errs(end+1) = sphere_utils_haversine_distance(trk.lon, trk.lat, tl, tb) / 1000;
            end
        end
    end
    fprintf('自适应 %s 融合         RMSE: %6.1f km (n=%d)\n', method_names{m}, rms_km(errs), length(errs));
end

% R1/R2 单站（对齐后，用于对比融合增益）
fprintf('\n--- 单站RMSE（对齐后） ---\n');
errs = [];
for k = 1:n_frames
    tl = interp1(true_track(:,5), true_track(:,1), t1_grid(k), 'linear', 'extrap');
    tb = interp1(true_track(:,5), true_track(:,2), t1_grid(k), 'linear', 'extrap');
    snap = trackSnapshots_R1{k};
    if ~isempty(snap.trackList)
        trk = snap.trackList{1};
        if trk.type ~= 7 && ~isnan(trk.lat)
            errs(end+1) = sphere_utils_haversine_distance(trk.lon, trk.lat, tl, tb) / 1000;
        end
    end
end
fprintf('R1 基础UKF单站        RMSE: %6.1f km (n=%d)\n', rms_km(errs), length(errs));

errs = [];
for k = 1:n_frames
    tl = interp1(true_track(:,5), true_track(:,1), t1_grid(k), 'linear', 'extrap');
    tb = interp1(true_track(:,5), true_track(:,2), t1_grid(k), 'linear', 'extrap');
    snap = trackSnapshots_R1_ad{k};
    if ~isempty(snap.trackList)
        trk = snap.trackList{1};
        if trk.type ~= 7 && ~isnan(trk.lat)
            errs(end+1) = sphere_utils_haversine_distance(trk.lon, trk.lat, tl, tb) / 1000;
        end
    end
end
fprintf('R1 自适应UKF单站      RMSE: %6.1f km (n=%d)\n', rms_km(errs), length(errs));

errs = [];
for k = 1:n_frames
    tl = interp1(true_track(:,5), true_track(:,1), t1_grid(k), 'linear', 'extrap');
    tb = interp1(true_track(:,5), true_track(:,2), t1_grid(k), 'linear', 'extrap');
    snap = aligned_R2{k};
    if ~isempty(snap.trackList)
        trk = snap.trackList{1};
        if trk.type ~= 7 && ~isnan(trk.lat)
            errs(end+1) = sphere_utils_haversine_distance(trk.lon, trk.lat, tl, tb) / 1000;
        end
    end
end
fprintf('R2 基础UKF单站        RMSE: %6.1f km (n=%d)\n', rms_km(errs), length(errs));

errs = [];
for k = 1:n_frames
    tl = interp1(true_track(:,5), true_track(:,1), t1_grid(k), 'linear', 'extrap');
    tb = interp1(true_track(:,5), true_track(:,2), t1_grid(k), 'linear', 'extrap');
    snap = aligned_R2_ad{k};
    if ~isempty(snap.trackList)
        trk = snap.trackList{1};
        if trk.type ~= 7 && ~isnan(trk.lat)
            errs(end+1) = sphere_utils_haversine_distance(trk.lon, trk.lat, tl, tb) / 1000;
        end
    end
end
fprintf('R2 自适应UKF单站      RMSE: %6.1f km (n=%d)\n', rms_km(errs), length(errs));

%% ==================== Phase 8: 定量误差评估（基础 vs 自适应 对比） ====================
% 【评估流程】
%   8.1 构建基础UKF的matcher → evaluate_all('fusion') → fusion_eval_base
%   8.2 构建自适应UKF的matcher → evaluate_all('fusion') → fusion_eval_ad
%   8.3 打印6行×(基础/自适应/改善%)对比表
%   8.4 单站误差对比（4组：R1基础/R2基础/R1自适应/R2自适应）
%
% 【matcher结构体】
%   matcher用于将R1和R2的航迹与真值进行关联，计算匹配关系
%   字段：
%     .r1_pos — R1航迹位置历史（1×n_frames×2数组，[:,:,1]=lon, [:,:,2]=lat）
%     .r2_pos — R2航迹位置历史
%     .matched_pairs — 航迹匹配对（此处固定R1#1↔R2#1）
%     .aligned_R2 — 对齐后的R2快照
%     .r1_ids / .r2_ids — 航迹ID列表
%
% 【对比表格式】
%   算法      基础UKF   自适应    改善
%   ------    ------   ------   ------
%   SCC        xx.x     xx.x    +x.x%
%   BC         xx.x     xx.x    +x.x%
%   CI         xx.x     xx.x    +x.x%
%   FCI        xx.x     xx.x    +x.x%
%   R1_only    xx.x     xx.x    +x.x%
%   R2_only    xx.x     xx.x    +x.x%
%
% 【改善率计算】
%   improvement = (1 - rmse_ad/rmse_base) × 100%
%   正值(+x.x%)：自适应UKF优于基础UKF（RMSE更低，改善正数）
%   负值(-x.x%)：基础UKF优于自适应UKF（极少见，通常发生在非机动段）
%   零值(0.0%)：两者相同（或基础RMSE=0，退化情况）
%
% 【评估指标】evaluate_all返回的overall数组字段：
%   .s.rms — 均方根误差 (km)
%   .s.mean — 平均误差 (km)
%   .s.std — 误差标准差 (km)
%   .s.max — 最大误差 (km)
%   .s.median — 中值误差 (km)
%   .label — 算法名称 ('SCC', 'BC', 'CI', 'FCI', 'R1_only', 'R2_only')
fprintf('\n========== Phase 8: 定量误差评估（基础 vs 自适应 对比） ==========\n');

% 将单目标真值包装为cell数组（evaluate_all期望cell数组输入）
truthTrajs = {truthTraj};

% ---- 8.1 基础UKF评估 ----
% build_pos_history：从快照cell中提取指定track_id的位置序列
%   返回1×n_frames×2数组，未出现的帧填NaN
r1_pos = build_pos_history(trackSnapshots_R1, 1, n_frames);
r2_pos = build_pos_history(aligned_R2, 1, n_frames);
% make_matcher：构建单目标1对1的匹配器
matcher_base = make_matcher(r1_pos, r2_pos, aligned_R2);

% evaluate_all('fusion', ...) 评估融合结果
%   1. 逐帧比较融合位置与真值位置，计算Haversine距离误差(km)
%   2. 汇总统计：RMS/mean/std/max/median
%   3. 额外评估R1_only（仅R1航迹）和R2_only（仅R2航迹）的误差
fusion_eval_base = evaluate_all('fusion', all_fused_snapshots, method_names, ...
    matched_pair, trackSnapshots_R1, trackSnapshots_R2, ...
    truthTrajs, n_frames, params.dt_sec, matcher_base);

% ---- 8.2 自适应UKF评估 ----
r1_pos_ad = build_pos_history(trackSnapshots_R1_ad, 1, n_frames);
r2_pos_ad = build_pos_history(aligned_R2_ad, 1, n_frames);
matcher_ad = make_matcher(r1_pos_ad, r2_pos_ad, aligned_R2_ad);

fusion_eval_ad = evaluate_all('fusion', all_fused_snapshots_ad, method_names, ...
    matched_pair, trackSnapshots_R1_ad, trackSnapshots_R2_ad, ...
    truthTrajs, n_frames, params.dt_sec, matcher_ad);

% ---- 8.3 打印对比表 ----
fprintf('\n--- 误差对比: 基础UKF vs 机动自适应UKF (RMSE km) ---\n');
% 表头
fprintf('%-20s %8s %8s %8s\n', '算法', '基础UKF', '自适应', '改善');
fprintf('%-20s %8s %8s %8s\n', '------', '------', '------', '------');

% 遍历所有评估标签：前4个是融合算法，后2个是单站
% all_labels: {'SCC', 'BC', 'CI', 'FCI', 'R1_only', 'R2_only'}
all_labels = [method_names, {'R1_only', 'R2_only'}];
for m = 1:length(all_labels)
    % 从评估结果中提取RMSE值
    rmse_base = fusion_eval_base.overall(m).s.rms;
    rmse_ad = fusion_eval_ad.overall(m).s.rms;
    % 计算改善率（正值=自适应更好，负值=基础更好）
    if rmse_base > 0
        improvement = (1 - rmse_ad/rmse_base) * 100;
    else
        improvement = 0;  % 退化情况：基础RMSE=0时的安全处理
    end
    fprintf('%-20s %8.1f %8.1f %+7.1f%%\n', all_labels{m}, rmse_base, rmse_ad, improvement);
end

% ---- 找出最佳融合算法 ----
% 从四种融合算法的RMSE中找出最小值
% overall(1:4)对应SCC/BC/CI/FCI
rms_vals_base = arrayfun(@(x) x.s.rms, fusion_eval_base.overall(1:4));
rms_vals_ad = arrayfun(@(x) x.s.rms, fusion_eval_ad.overall(1:4));
[~, best_m_base] = min(rms_vals_base);
[~, best_m_ad] = min(rms_vals_ad);

fprintf('\n基础UKF最佳融合: %s (%.1f km)\n', method_names{best_m_base}, rms_vals_base(best_m_base));
fprintf('自适应UKF最佳融合: %s (%.1f km)\n', method_names{best_m_ad}, rms_vals_ad(best_m_ad));

% ---- 8.4 单站UKF误差对比 ----
% 分别评估R1和R2单站（不融合）的跟踪误差
% 用于判断：融合是否确实改善了单站性能？哪个站对融合贡献更大？
fprintf('\n--- 单站UKF误差对比 ---\n');
% 重新对齐R2（确保评估时时间基准一致）
aligned_R2_eval = time_align_tracks(trackSnapshots_R2, params);
aligned_R2_ad_eval = time_align_tracks(trackSnapshots_R2_ad, params);

% evaluate_all('tracking_errors', ...) 评估单站跟踪误差
%   'tracking_errors'模式：直接比较跟踪位置与真值位置（不经融合）
errorStats_R1 = evaluate_all('tracking_errors', trackSnapshots_R1, detList_R1, truthTrajs, n_frames, params.dt_sec, 'R1');
errorStats_R2 = evaluate_all('tracking_errors', aligned_R2_eval, detList_R2, truthTrajs, n_frames, params.dt_sec, 'R2');
errorStats_R1_ad = evaluate_all('tracking_errors', trackSnapshots_R1_ad, detList_R1, truthTrajs, n_frames, params.dt_sec, 'R1-ad');
errorStats_R2_ad = evaluate_all('tracking_errors', aligned_R2_ad_eval, detList_R2, truthTrajs, n_frames, params.dt_sec, 'R2-ad');

% 对比打印：基础 vs 自适应 的单站误差
% 第一行：R1基础 vs R1自适应
% 第二行：R2基础 vs R2自适应
for pair = {errorStats_R1, errorStats_R1_ad; errorStats_R2, errorStats_R2_ad}
    e_base = pair{1}; e_ad = pair{2};
    % 提取UKF的汇总统计（summary字段包含ukf和raw两种统计）
    s_b = e_base.summary(1).ukf;
    s_a = e_ad.summary(1).ukf;
    if s_b.rms > 0
        imp = (1 - s_a.rms/s_b.rms)*100;
    else
        imp = 0;
    end
    fprintf('%s: 基础UKF RMSE=%.1fkm → 自适应 UKF RMSE=%.1fkm (%+.1f%%)\n', ...
        e_base.radar, s_b.rms, s_a.rms, imp);
end

%% ==================== Phase 9: 可视化 + 数据保存 ====================
% 【7张图表的用途】
%   图1 plot_scene_overview — 拐弯航迹+双雷达覆盖（场景定位）
%       内容：真实航迹（彩色渐变线）+ 拐弯航路点标记 +
%             R1/R2雷达站位置 + 覆盖扇区（波束宽度范围） +
%             覆盖边缘虚线圆圈
%       目的：整体把握场景几何关系，验证拐弯是否在双雷达覆盖内
%
%   图2 plot_turn_spatial('point_clouds') — R1/R2并排：原始点云+两条UKF轨迹
%       左侧子图(R1)：灰色散点=原始点云(detList_R1每帧校正后位置)
%                     蓝色虚线=基础UKF轨迹
%                     红色实线=自适应UKF轨迹
%                     黑色实线=真值
%       右侧子图(R2)：同上（R2视角）
%       目的：直观对比基础UKF与自适应UKF在拐弯处的轨迹差异
%
%   图3 plot_turn_spatial('radar_compare') R1 — R1单站综合对比
%       内容：地图轨迹（真值+基础虚线+自适应实线+拐弯放大子图）+
%             误差时间线（基础蓝线vs自适应红线，拐弯区域灰色高亮）+
%             RMSE柱状图（基础灰vs自适应绿）
%       目的：R1视角下自适应UKF相对基础UKF的改进量化
%
%   图4 plot_turn_spatial('radar_compare') R2 — R2单站综合对比
%       内容同上，R2视角
%       目的：R2视角下两种UKF的对比（R2量测精度低，差异可能更显著）
%
%   图5 plot_turn_spatial('fusion_map') — 融合地图对比
%       内容：基础融合（虚线）+ 自适应融合（实线）+ 拐弯放大子图 +
%             信息面板（标注最佳融合算法+RMSE值）
%       目的：融合层面两种UKF的对比，验证自适应UKF是否能
%             通过更好的单站跟踪质量提升融合精度
%
%   图6 plot_turn_stats('rmse_bars') — RMSE柱状图总览
%       内容：6组柱状图（SCC/BC/CI/FCI/R1/R2），每组两根柱子
%             灰色=基础UKF，绿色=自适应UKF
%             上方标注数值和改善百分比
%             底部附详细数值汇总文本框
%       目的：一图纵览所有算法的基础vs自适应表现差异
%
%   图7 plot_turn_spatial('comprehensive') — 全图层综合对比
%       内容：11条轨迹曲线（真值/R1基础/R2基础/R1自适应/R2自适应/
%             SCC基础/SCC自适应/BC基础/BC自适应/CI基础/CI自适应）+
%             R1/R2点云散点 + 雷达站 + UI按钮控制各轨道显隐
%       目的：可交互的全方位对比视图，方便逐一检查各轨道关系
%
% 【保存内容】（比直线场景多两倍数据）
%   基础版：trackSnapshots_R1/R2, finalTrk1/2, all_fused_snapshots
%   自适应版：trackSnapshots_R1/R2_ad, finalTrk1/2_ad, all_fused_snapshots_ad
%   评估：fusion_eval_base/ad, errorStats_R1/R2/R1_ad/R2_ad
%   标定：dr1/2_est, da1/2_est
%   其他：params, truthTraj, true_track, turn_waypoints, method_names
fprintf('\n========== Phase 9: 可视化 ==========\n');
% 确保results目录存在
if ~exist('results', 'dir'), mkdir('results'); end

% 临时关闭所有警告（图表生成过程中可能产生非关键的MATLAB警告）
warn_state = warning('off', 'all');
% 
% % ---- 图1: 场景总览（拐弯航迹 + 双雷达覆盖扇区） ----
% % 使用plot_scene_overview（通用函数，与直线场景相同）
% % 展示拐弯航迹在双雷达覆盖扇区中的位置关系
% plot_scene_overview(true_track, params, 'results');
% 
% % ---- 图2: 点云 + 基础UKF(虚线) + 自适应UKF(实线) 并排对比 ----
% % 'point_clouds'模式：左右并排显示R1和R2的点云和两种UKF轨迹
% % 输入：true_track(真值), detList(点云), trackSnapshots(基础UKF), trackSnapshots_ad(自适应UKF)
% plot_turn_spatial('point_clouds', true_track, detList_R1, detList_R2, ...
%     trackSnapshots_R1, trackSnapshots_R2, ...
%     trackSnapshots_R1_ad, trackSnapshots_R2_ad, params, 'results');
% 
% % ---- 图3: R1单站对比（地图+拐弯放大+误差时间线+RMSE柱状图） ----
% % 雷达位置用 params.radar1_lat, params.radar1_lon（用于地图标注雷达站位置）
% % 最后一个参数3是图形编号，确保MATLAB新开figure窗口
% plot_turn_spatial('radar_compare', true_track, trackSnapshots_R1, trackSnapshots_R1_ad, ...
%     'R1', params.radar1_lat, params.radar1_lon, params, 'results', 3);
% 
% % ---- 图4: R2单站对比（地图+拐弯放大+误差时间线+RMSE柱状图） ----
% plot_turn_spatial('radar_compare', true_track, trackSnapshots_R2, trackSnapshots_R2_ad, ...
%     'R2', params.radar2_lat, params.radar2_lon, params, 'results', 4);
% 
% % ---- 图5: 融合地图对比（基础融合虚线 + 自适应融合实线 + 拐弯放大 + 信息面板） ----
% % 输入两组融合结果和各自的最佳算法索引
% % best_m_base/best_m_ad用于在信息面板中标注和突出显示最佳融合
% plot_turn_spatial('fusion_map', true_track, ...
%     all_fused_snapshots, method_names, best_m_base, ...
%     all_fused_snapshots_ad, method_names, best_m_ad, params, 'results');
% 
% % ---- 图6: RMSE柱状图总览（全部方法 基础灰 vs 自适应绿 + 数值汇总） ----
% % 使用plot_turn_stats（专门为拐弯场景设计的统计图函数）
% % 对比所有6种评估标签的基础vs自适应RMSE
% plot_turn_stats('rmse_bars', fusion_eval_base, fusion_eval_ad, ...
%     method_names, best_m_base, best_m_ad, params, 'results');

% ---- 图7: 全图层综合对比（地图 + 按钮控制显隐） ----
% 'comprehensive'模式：绘制所有可用的轨迹图层
% 使用SCC融合结果作为融合代表（all_fused_snapshots{1}和{1}_ad）
% 提供UI交互按钮让用户可以打开/关闭任意图层
plot_turn_spatial('comprehensive', true_track, detList_R1, detList_R2, ...
    trackSnapshots_R1, trackSnapshots_R2, ...
    trackSnapshots_R1_ad, trackSnapshots_R2_ad, ...
    all_fused_snapshots{1}, all_fused_snapshots_ad{1}, params, 'results');

% 恢复警告状态
warning(warn_state);

% ---- 数据保存 ----
% 将所有仿真数据保存到MAT文件中，便于后续离线分析
% 文件名含时间戳，避免覆盖历史运行结果
fprintf('\n========== Phase 9: 数据保存 ==========\n');
% 构造输出文件路径：results/simulation_turn_YYYYMMDD_HHMMSS.mat
outf = fullfile('results', sprintf('simulation_turn_%s.mat', datestr(now, 'yyyymmdd_HHMMSS')));
% 保存所有关键变量
% 注意：包含基础版和自适应版两套完整数据（跟踪+融合+评估）
save(outf, 'params', 'truthTraj', 'true_track', 'turn_waypoints', ...
    'trackSnapshots_R1', 'trackSnapshots_R2', 'finalTrk1', 'finalTrk2', ...
    'trackSnapshots_R1_ad', 'trackSnapshots_R2_ad', 'finalTrk1_ad', 'finalTrk2_ad', ...
    'all_fused_snapshots', 'all_fused_snapshots_ad', 'method_names', ...
    'fusion_eval_base', 'fusion_eval_ad', ...
    'errorStats_R1', 'errorStats_R2', 'errorStats_R1_ad', 'errorStats_R2_ad', ...
    'dr1_est', 'dr2_est', 'da1_est', 'da2_est');
fprintf('数据已保存: %s\n', outf);
fprintf('\nDone.\n');

% =========================================================================
% 内部辅助函数
% =========================================================================
% 【说明】以下三个函数定义在脚本末尾，均为局部函数（作用域限于本文件）
%   get_type_str      — 航迹类型枚举值→可读字符串
%   build_pos_history  — 从航迹快照cell中提取指定航迹ID的位置历史
%   make_matcher       — 构建单目标场景的简化匹配器结构体
% =========================================================================

% -------------------------------------------------------------------------
% get_type_str — 航迹类型枚举值→可读字符串映射
% -------------------------------------------------------------------------
% 输入：t — 航迹类型枚举值（整数）
% 输出：s — 可读字符串
% 航迹类型定义：
%   1 = RELIABLE  — 稳定跟踪（航迹建立完成，可靠输出）
%   2 = MAINTAIN  — 维持跟踪（航迹曾经可靠，当前纯预测维持）
%   6 = TEMPORARY — 临时航迹（正在初始化，M/N逻辑检测中）
%   7 = HISTORY   — 历史航迹（已终止，保留在记录中）
function s = get_type_str(t)
    switch t
        case 1, s = 'RELIABLE';
        case 2, s = 'MAINTAIN';
        case 6, s = 'TEMPORARY';
        case 7, s = 'HISTORY';
        otherwise, s = 'UNKNOWN';
    end
end

% -------------------------------------------------------------------------
% build_pos_history — 从航迹快照cell数组中提取指定track_id的位置历史
% -------------------------------------------------------------------------
% 输入：
%   snapshots — cell数组(1×n_frames)，每个元素是一个结构体，含 .trackList
%               .trackList{i} 是第i条航迹的结构体，含 .id, .lon, .lat 等字段
%   track_id  — 目标航迹的ID号（单目标场景为1）
%   n_frames  — 总帧数
% 输出：
%   pos — 1×n_frames×2 数组
%         pos(1, k, 1) = 第k帧的经度（lon），未出现该航迹的帧填NaN
%         pos(1, k, 2) = 第k帧的纬度（lat），未出现该航迹的帧填NaN
% 算法：
%   逐帧遍历快照，在每帧的trackList中查找匹配track_id的航迹，
%   提取其(lon, lat)。未找到的帧保持NaN值。
% 用途：
%   为evaluate_all提供航迹位置历史，用于与真值比较计算误差
function pos = build_pos_history(snapshots, track_id, n_frames)
    % 预分配为NaN数组（未出现的帧自然为NaN，便于后续处理）
    pos = nan(1, n_frames, 2);
    for k = 1:n_frames
        snap = snapshots{k};
        % 跳过空快照（该帧无任何航迹）
        if ~isempty(snap.trackList)
            % 在trackList中查找匹配track_id的航迹
            for t = 1:length(snap.trackList)
                if snap.trackList{t}.id == track_id
                    % 提取经纬度（第三维：1=lon, 2=lat）
                    pos(1, k, 1) = snap.trackList{t}.lon;
                    pos(1, k, 2) = snap.trackList{t}.lat;
                    break;  % 找到后跳出内层循环
                end
            end
        end
    end
end

% -------------------------------------------------------------------------
% make_matcher — 构建单目标场景的简化匹配器结构体
% -------------------------------------------------------------------------
% 输入：
%   r1_pos     — R1航迹位置历史（1×n_frames×2数组，[:,:,1]=lon, [:,:,2]=lat）
%   r2_pos     — R2航迹位置历史（1×n_frames×2数组）
%   aligned_r2 — 对齐后的R2快照cell数组（时间对齐到R1网格）
% 输出：
%   m — matcher结构体，用于传入evaluate_all
%       字段：
%         .r1_pos         — R1位置历史（来自build_pos_history）
%         .r2_pos         — R2位置历史（来自build_pos_history）
%         .matched_pairs   — 航迹关联对（单目标场景固定R1#1↔R2#1）
%         .aligned_R2      — 对齐后的R2快照
%         .r1_ids          — R1航迹ID列表（单目标场景固定为1）
%         .r2_ids          — R2航迹ID列表（单目标场景固定为1）
% 说明：
%   单目标场景下，无需运行完整的track_association（航迹关联算法），
%   直接指定R1的第一个航迹与R2的第一个航迹为匹配对。
%   如果场景扩展到多目标，则需要替换为实际的航迹关联逻辑。
function m = make_matcher(r1_pos, r2_pos, aligned_r2)
    m = struct();
    m.r1_pos = r1_pos;
    m.r2_pos = r2_pos;
    % 构建固定匹配对：R1航迹1 ↔ R2航迹1
    % match_ratio=1.0 和 quality=100 表示完全确定的匹配
    m.matched_pairs = struct('R1_track_id', 1, 'R2_track_id', 1, ...
        'match_count', 0, 'coexist_count', 0, 'match_ratio', 1.0, ...
        'mean_dist_km', 0, 'quality', 100);
    m.aligned_R2 = aligned_r2;
    % 单目标场景下只有一个航迹ID
    m.r1_ids = 1;
    m.r2_ids = 1;
end

% rms_km — 计算误差向量的RMSE（km）
%   输入: e — 误差值向量（km），可为空
%   输出: v — RMSE值（km），空向量返回NaN
function v = rms_km(e)
    if isempty(e), v = NaN; else, v = sqrt(mean(e.^2)); end
end
