% =========================================================================
% run_simulation.m — 双基地OTH-SWR单目标直线航迹仿真主程序
% =========================================================================
% 【程序定位】
%   本程序是本仿真系统的两个主入口之一（另一个为 multi_target_single_radar_sim.m，
%   面向多目标场景）。它演示了完整的双基地雷达单目标跟踪与融合流水线：
%   从场景生成、系统标定、点迹仿真、航迹跟踪、航迹融合到误差评估的全流程。
%
% 【双基地雷达基本概念】
%   双基地雷达的发射站(Tx)和接收站(Rx)地理上分离：
%     - 群距离 Rg = distance(Tx,目标) + distance(目标,Rx)
%         物理含义：电磁波从Tx发出，经目标散射后到达Rx的总路径长度
%         与单基地雷达的斜距不同，Rg不满足"以Rx为圆心"的简单几何关系
%     - 方位角 az = Rx观测目标的真北方位角（与单基地雷达相同，纯接收属性）
%     - 双基地径向速度 = 目标速度在 ①Tx→目标方向 和 ②目标→Rx方向 的投影和
%         即：目标靠近Tx和靠近Rx都会使Rg减小（正的双基地径向速度）
%   本场景用两部雷达协同跟踪同一目标：
%     R1（精密站）：Rx@(113°E, 33.5°N), Tx@(109°E, 33.5°N)
%                  σ_range=7km, σ_az=0.35°, 基线~370km
%     R2（普通站）：Rx@(115°E, 33°N), Tx@(111°E, 33°N)
%                  σ_range=14km, σ_az=0.6°, 基线~370km
%     R2的量测噪声约为R1的2倍，用于对比不同精度雷达的融合效果
%
% 【9-Phase 流水线总览】
%   Phase 0: 场景初始化（航迹生成 + 覆盖检查 + 时间网格）
%       ↓
%   Phase 1: ADS-B系统偏差标定（样本均值估计, 5000点均匀采样）
%       ↓
%   Phase 2: 原始点迹生成（逐帧生成含偏差+噪声的点迹, 两部雷达独立）
%       ↓
%   Phase 3: 时间对齐策略（纯声明, 无计算, 延后至Phase 6）
%       ↓
%   Phase 4: 偏差校正 + 几何反解（用Phase 1偏差校正 → bistatic_inverse_solver）
%       ↓
%   Phase 5: 单目标航迹跟踪（UKF + PDA + 模糊自适应Q, 两部雷达独立）
%       ↓
%   Phase 6: 航迹级时间对齐（CV模型全状态外推, R2→R1时间网格）
%       ↓
%   Phase 7: 航迹融合（SCC / BC / CI / FCI 四种算法, 直接1对1）
%       ↓
%   Phase 8: 定量误差评估（RMSE, 中位误差, 融合 vs 单站对比）
%       ↓
%   Phase 9: 可视化 + 数据保存（5张图 + 完整.mat文件）
%
% 【调用链（按Phase展开, 箭头表示数据流向）】
%   Phase 0:
%     simulation_params() → params (13模块默认配置, 含雷达/UKF/关联所有参数)
%     params → aircraft_trajectory_create() → traj (航段结构体)
%     traj   → aircraft_trajectory_interpolate('generate') → true_track (N×5矩阵)
%     true_track → radar_coverage_check() (逐点) → 覆盖率统计
%     params.dt_sec, params.time_offset_* → t1_grid, t2_grid (异步时间数组)
%     true_track → truthTraj (真值结构体, 供Phase 8误差评估用)
%
%   Phase 1:
%     readtable(ADS-B CSV) → T_adsb (约24万行, 纬度+经度两列)
%     T_adsb → radar_coverage_check() (筛选威力范围内点)
%     威力内点 → sphere_utils_haversine_distance() + sphere_utils_azimuth()
%              → 真值极坐标(Rg_true, az_true)
%     添加系统偏差+噪声 → 模拟量测 → 偏差样本(dr_list, da_list)
%     mean(dr_list), mean(da_list) → dr_est, da_est (偏差估计值)
%
%   Phase 2:
%     aircraft_trajectory_interpolate(traj, t) (×n_frames×2站)
%       → (pos, vel) 当前帧目标真值位置和速度
%     generate_frame_detections() → detRaw_R*{k} (点迹结构体数组)
%       (内部: 覆盖检查→Pd判断→真值+偏差+噪声→虚警生成)
%
%   Phase 3:
%     纯策略声明，不执行任何计算
%
%   Phase 4:
%     detRaw_R*{k} → prange - dr_est, paz - da_est (偏差校正)
%     bistatic_inverse_solver(Rg_corrected, az_corrected, Tx, Rx) → (lon, lat)
%     bistatic_inverse_solver(Rg_raw, az_raw, Tx, Rx) → (raw_lon, raw_lat)
%       (同时保留原始偏差下的反解经纬度, 供对比评估)
%
%   Phase 5:
%     ukf_jichu('create', params) → ukf_tpl (UKF模板, 含状态维数/噪声矩阵等)
%     ukf_tpl, detList_R* → single_track_runner() → trackSnapshots_R*, finalTrk*
%       (single_track_runner内部每帧调用 ukf_jichu/nn_associate/pda_weight/
%        apply_fuzzy_adapt 等模块)
%
%   Phase 6:
%     trackSnapshots_R2 → time_align_tracks() → aligned_R2
%       (CV模型外推: F(-offset)×x, F(-offset)×P×F(-offset)' + Q(|offset|))
%
%   Phase 7:
%     trackSnapshots_R1, aligned_R2, matched_pair
%       → run_track_fusion() ×4 → all_fused_snapshots{1..4}
%
%   Phase 8:
%     all_fused_snapshots, truthTraj → evaluate_all('fusion') → fusion_eval
%     trackSnapshots_R1/R2, detList_R1/R2 → evaluate_all('tracking_errors') → errorStats
%
%   Phase 9:
%     true_track → plot_scene_overview() → results/fig1_scene_overview.png
%     detList_R1/R2 → plot_point_cloud_3d() → results/fig2a/b_point_cloud.png
%     综合数据 → plot_results('single_track') → results/fig3_single_track.png
%     融合数据 → plot_results('single_fusion') → results/fig4_fusion.png
%     save() → results/simulation_YYYYMMDD_HHMMSS.mat
%
% 【前置依赖】
%   本程序依赖项目根目录下的所有模块（通过 addpath(genpath('.')) 引入）：
%     config/       — simulation_params.m（仿真参数配置）
%     simulation/   — aircraft_trajectory_create.m, aircraft_trajectory_interpolate.m,
%                      generate_frame_detections.m, time_align_tracks.m
%     registration/ — radar_coverage_check.m, bistatic_inverse_solver.m
%     geometry/     — sphere_utils_haversine_distance.m, sphere_utils_azimuth.m（球面计算）
%     ukf/          — ukf_jichu.m（基础UKF）, ukf_zishiying.m（自适应UKF）
%     tracker/      — single_track_runner.m, nn_associate.m, pda_weight.m,
%                      apply_fuzzy_adapt.m, multi_track_manager.m
%     fusion/       — run_track_fusion.m（SCC/BC/CI/FCI四种融合算法）
%     evaluation/   — evaluate_all.m（误差评估调度器）
%     visualization/— plot_scene_overview.m, plot_point_cloud_3d.m, plot_results.m,
%                      plot_turn_spatial.m, plot_turn_stats.m
%     io/           — save_all.m（数据保存工具）
%
% 【输出文件】
%   MATLAB Workspace — 所有中间变量（detRaw, detList, trackSnapshots等）
%   results/simulation_YYYYMMDD_HHMMSS.mat — 完整仿真数据打包
%   results/fig1_scene_overview.png         — 场景总览图
%   results/fig2a_R1_point_cloud.png        — R1原始点迹3D点云
%   results/fig2b_R2_point_cloud.png        — R2原始点迹3D点云
%   results/fig3_single_track.png           — 单目标跟踪综合分析图
%   results/fig4_fusion*.png                — 融合结果综合对比图
% =========================================================================

clear; close all; clc;
% addpath(genpath('.'))：将当前目录及所有子目录加入MATLAB搜索路径
% 这使得 config/、simulation/、ukf/、tracker/、fusion/ 等子目录下的
% 所有 .m 文件都可以直接调用而无需手动逐目录添加
addpath(genpath('.'));

%% ==================== Phase 0: 场景初始化 ====================
% 【目的】生成目标真实轨迹，验证覆盖条件，建立仿真时间基准
% 【步骤】
%   0.1 加载仿真参数（simulation_params.m，包含13个模块的默认配置：
%       雷达位置/噪声/偏差、UKF参数、关联波门、融合算法选择等）
%   0.2 创建航迹结构体（aircraft_trajectory_create: 基于航段模型的轨迹生成器，
%       内部使用Haversine公式计算大圆距离，匀速运动假设）
%   0.3 批量采样生成完整轨迹（aircraft_trajectory_interpolate('generate'):
%       从 traj.segments 按 dt_sec 步长逐点插值，输出 N×5 矩阵
%       列：[lon, lat, lon_rate(deg/s), lat_rate(deg/s), time_sec]）
%   0.4 覆盖率检查（radar_coverage_check: 距离1000-2000km + 波束±7.5°扇区）
%       统计飞机在R1和R2各自威力范围内的采样点数量
%   0.5 建立两部雷达的异步时间网格
%       R1: 0s, 30s, 60s, 90s, ...（从time_offset_radar1_sec=0开始）
%       R2: 13s, 43s, 73s, 103s, ...（从time_offset_radar2_sec=13开始）
%   0.6 构造真值结构体 truthTraj（供Phase 8误差评估时作为Ground Truth使用）
%       .label: 目标标签 'A'（单目标场景固定）
%       .speed_ms: 飞机速度（m/s，来自 params.aircraft_speed_ms）
%       .time_sec: 时间戳向量（来自 true_track 第5列）
%       .lat/.lon: 经纬度向量（来自 true_track 第2/1列）
%       .lon_rate/.lat_rate: 经纬度变化率（deg/s，来自 true_track 第3/4列）
% 【输出】
%   params     — 完整的仿真参数结构体（所有13个模块的默认配置）
%   traj       — 航迹结构体（.segments: 航段信息, .duration_sec: 总时长,
%                .n_steps: 总采样点数, .start_time: 起始时间）
%   true_track — N×5矩阵 [lon, lat, lon_rate, lat_rate, time_sec]
%   t1_grid    — R1的时间采样数组（秒），从 offset1 到 duration_sec
%   t2_grid    — R2的时间采样数组（秒），从 offset2 到 duration_sec
%   n_frames   — 仿真总帧数（取两者最小，保证逐帧一一对应的循环次数）
%   truthTraj  — 真值结构体，供Phase 8误差评估用
% =========================================================================

fprintf('========== Phase 0: 场景初始化 ==========\n');

% 加载仿真参数：包含雷达位置、噪声特性、UKF参数、关联波门等所有配置
params = simulation_params();
% 固定随机种子：确保每次运行得到完全相同的随机数序列，实现可复现性
rng(params.random_seed);

% 单机航迹生成
% aircraft_trajectory_create: 将航点(waypoints)列表 + 速度 + 时间步长
% 转换为内部航段模型（每个segment存储: 起点经纬度、终点经纬度、
% 航向角heading、航段长度、航段时间）
% aircraft_waypoints 格式: N×2矩阵 [lon, lat]（度），按顺序经过的航点
traj = aircraft_trajectory_create(params.aircraft_waypoints, ...
    params.aircraft_speed_ms, params.dt_sec);
% aircraft_trajectory_interpolate('generate'): 根据航段模型，从起点开始
% 按 dt_sec 步长进行时间采样，用 Haversine 球面插值计算每个采样时刻的
% 精确经纬度和速度分量。输出 size(true_track) = [n_steps, 5]
true_track = aircraft_trajectory_interpolate('generate', traj);
fprintf('真实航迹: %d 点, 总时长 %.0f s, 速度 %.0f m/s\n', ...
    size(true_track,1), traj.duration_sec, params.aircraft_speed_ms);

% 覆盖检查：逐个采样点检查目标是否在雷达威力范围内
% radar_coverage_check 返回三个值：
%   inCoverage — 布尔值，是否在距离+角度范围内
%   range_km   — 目标到Rx的地表距离（km）
%   angle_diff_deg — 目标方位与波束中心的角度差（度）
n_in_r1 = 0; n_in_r2 = 0;
for i = 1:size(true_track, 1)
    [in1, ~, ~] = radar_coverage_check(params.radar1_lon, params.radar1_lat, ...
        true_track(i,1), true_track(i,2), params.radar1_beam_center_deg, params);
    [in2, ~, ~] = radar_coverage_check(params.radar2_lon, params.radar2_lat, ...
        true_track(i,1), true_track(i,2), params.radar2_beam_center_deg, params);
    if in1, n_in_r1 = n_in_r1 + 1; end
    if in2, n_in_r2 = n_in_r2 + 1; end
end
fprintf('  在R1威力内: %d 点, 在R2威力内: %d 点 (共%d点)\n', ...
    n_in_r1, n_in_r2, size(true_track,1));

% 时间网格：两部雷达有不同的起始时间偏移，形成异步采样
% R1从offset1(=0s)开始，R2从offset2(=13s)开始，步长均为dt_sec(=30s)
% 冒号运算符: start : step : end（含end），生成等差数列
t1_grid = params.time_offset_radar1_sec : params.dt_sec : traj.duration_sec;
t2_grid = params.time_offset_radar2_sec : params.dt_sec : traj.duration_sec;
% 仿真帧数取两者中较小者——当一部雷达时间网格用完, 仿真即结束
n_frames = min(length(t1_grid), length(t2_grid));
fprintf('仿真帧数: %d (dt=%.0fs)\n', n_frames, params.dt_sec);

% 真值结构体 (用于误差评估)
% 将true_track的五列分别映射为结构体的五个字段
% true_track列对应: [1]lon [2]lat [3]lon_rate [4]lat_rate [5]time_sec
tt = true_track;
truthTraj = struct('label', 'A', 'speed_ms', params.aircraft_speed_ms, ...
    'time_sec', tt(:,5), 'lat', tt(:,2), 'lon', tt(:,1), ...
    'lon_rate', tt(:,3), 'lat_rate', tt(:,4));

%% ==================== Phase 1: ADS-B系统偏差标定 ====================
% 【目的】利用ADS-B合作目标数据统计估计两部雷达的系统偏差
% 【为什么需要标定？】
%   OTH-SWR（天波超视距雷达）由于电离层传播路径的不确定性，存在显著的
%   系统性偏差：距离偏置~20km、方位偏置~3°。这些偏差如果不校正，会直接
%   导致定位误差累积到数十公里量级，使后续跟踪和融合完全失效。
%   ADS-B（广播式自动相关监视）数据提供目标的"真实"GPS位置（民用航空器
%   自发广播的经纬度+高度信息），精度约10-20米级。通过比较
%   "雷达测量的极坐标"与"ADS-B位置反算的极坐标"之间的差异，
%   可以统计估计出系统偏差。
% 【方法】样本均值估计
%   对ADS-B数据中落在雷达威力范围内的每个点，计算：
%     dr = Rg_meas - Rg_true      （群距离偏差，米）
%         其中 Rg_meas = Rg_true + bias + noise
%              Rg_true = distance(Tx,ADS-B) + distance(ADS-B,Rx)（由ADS-B经纬度反算）
%     da = az_meas - az_true      （方位角偏差，度，含360°包裹处理）
%         其中 az_meas = az_true + bias + noise
%              az_true = azimuth(Rx → ADS-B目标)（由ADS-B经纬度计算）
%   最终估计：dr_est = mean(dr_list), da_est = mean(da_list)
%   这是一个简单的偏差估计方法：假设噪声为零均值高斯分布，
%   通过大量样本（5000点）的算术平均消除随机噪声，保留系统性偏置。
%   （理论上，若噪声严格零均值，mean()是无偏估计；实际中噪声不完全对称，
%    但5000样本量足以保证估计精度在10%以内）
% 【采样策略】均匀采样5000个ADS-B点，步长 = floor(height(T_adsb) / 5000)
%   这避免了连续采样带来的时间相关性，使得偏差估计在不同的时间区段
%   都具有代表性（覆盖不同电离层状态下的偏差特征）
% 【ADS-B数据格式】
%   CSV文件，~24万行，每行格式：索引,纬度,经度（无表头）
%   Var1: 行号索引, Var2: 纬度(deg), Var3: 经度(deg)
%   使用 readtable() 读取，ReadVariableNames=false 表示第一行也当数据读入
% 【输出】
%   dr1_est, da1_est — R1（精密站）的距离偏差和方位偏差估计值
%   dr2_est, da2_est — R2（普通站）的距离偏差和方位偏差估计值
%   dr1_list, da1_list — R1偏差样本列表（用于后续分析偏差分布）
%   dr2_list, da2_list — R2偏差样本列表
%   这些估计值将在Phase 4中用于校正原始点迹
% =========================================================================

fprintf('\n========== Phase 1: ADS-B系统偏差标定 ==========\n');

% 重新固定随机种子：Phase 1虽然不直接依赖随机数（ADS-B数据是外部文件），
% 但在模拟"量测"时加入了randn()噪声来模拟雷达测量的随机性
rng(params.random_seed);

fprintf('加载ADS-B合作目标: %s\n', params.adsb_csv_path);
% readtable: 读取CSV文件为MATLAB table类型
% 'ReadVariableNames', false: 第一行不当作变量名，全部作为数据行读入
% T_adsb 的列: Var1=索引, Var2=纬度(deg), Var3=经度(deg)
T_adsb = readtable(params.adsb_csv_path, 'ReadVariableNames', false);
% 将table列提取为向量，方便后续索引访问
adsb_lat = T_adsb.Var2;   % 纬度向量（度），北纬为正
adsb_lon = T_adsb.Var3;   % 经度向量（度），东经为正

% 初始化偏差样本列表（动态增长数组）
dr1_list = []; da1_list = [];  % R1的距离偏差(米)和方位偏差(度)样本
dr2_list = []; da2_list = [];  % R2的距离偏差(米)和方位偏差(度)样本

% 采样参数：最多5000个点，均匀采样避免时间相关性
n_check = min(5000, height(T_adsb));
cal_step = max(1, floor(height(T_adsb) / n_check));
% cal_step: 采样步长，如 height=240000, n_check=5000 → cal_step=48
% 每48行取一个点，共约5000个样本分布在全部数据中

for idx = 1:cal_step:height(T_adsb)
    % 提取当前ADS-B点的经纬度
    t_lon = adsb_lon(idx);  t_lat = adsb_lat(idx);
    % 跳过NaN值（数据文件中可能有缺失行）
    if isnan(t_lon) || isnan(t_lat), continue; end

    % ---- R1（精密站）偏差计算 ----
    % 首先检查ADS-B点是否在R1的覆盖范围内（距离+角度）
    [in1, ~, ~] = radar_coverage_check(params.radar1_lon, params.radar1_lat, ...
        t_lon, t_lat, params.radar1_beam_center_deg, params);
    if in1
        % 从ADS-B真实位置反算"真值极坐标"（天波模型，与量测生成一致）
        % Rg_true: 天波双基地群距离 = r_tx + r_rx（弦长+电离层虚高模型）
        Rg_true = skywave_geometry('group_range', params.radar1_tx_lon, params.radar1_tx_lat, ...
            params.radar1_lon, params.radar1_lat, t_lon, t_lat);
        % az_true: Rx到目标的真北方位角（度，0°=正北，顺时针增加）
        az_true = sphere_utils_azimuth(params.radar1_lon, params.radar1_lat, t_lon, t_lat);
        % 模拟"雷达测量值"=真值+系统偏差+随机噪声
        % 这里用 randn() 模拟实际的雷达测量，使偏差估计更贴近真实场景
        Rg_meas = Rg_true + params.radar1_range_bias_m + randn() * params.radar1_range_noise_std_m;
        az_meas = az_true + params.radar1_azimuth_bias_deg + randn() * params.radar1_azimuth_noise_std_deg;
        % 偏差样本：量测值 - 真值（理想情况下=0+噪声，实际=系统偏差+噪声）
        dr1_list(end+1) = Rg_meas - Rg_true;
        % 方位角偏差：需要做360°包裹处理
        % 例：az_meas=359°, az_true=1° → 原始差=358°，应包裹为-2°
        daz = az_meas - az_true;
        if daz > 180, daz = daz - 360; elseif daz < -180, daz = daz + 360; end
        da1_list(end+1) = daz;
    end

    % ---- R2（普通站）偏差计算 ----
    % 与R1完全相同的流程，但使用R2的雷达参数和ADS-B数据
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

% 样本均值估计：通过大量样本平均消除零均值随机噪声，保留系统性偏置
% mean() 对向量取算术平均
dr1_est = mean(dr1_list);  da1_est = mean(da1_list);  % R1偏差估计值
dr2_est = mean(dr2_list);  da2_est = mean(da2_list);  % R2偏差估计值
fprintf('ADS-B标校点数: R1=%d, R2=%d\n', length(dr1_list), length(dr2_list));
fprintf('R1: dr_est=%.1f (true=%.0f) m, da_est=%.4f (true=%.1f) deg\n', ...
    dr1_est, params.radar1_range_bias_m, da1_est, params.radar1_azimuth_bias_deg);
fprintf('R2: dr_est=%.1f (true=%.0f) m, da_est=%.4f (true=%.1f) deg\n', ...
    dr2_est, params.radar2_range_bias_m, da2_est, params.radar2_azimuth_bias_deg);

%% ==================== Phase 2: 原始点迹生成 ====================
% 【目的】逐帧生成两部雷达的原始点迹（含系统偏差+随机噪声，不做校正）
% 【点迹结构体字段（detRaw_R*{k}数组中每个元素的字段）】
%   .prange      — 原始群距离（=真值+系统偏差+随机噪声，单位：米）
%                   注意：这里的"原始"意味着Phase 1估计的偏差尚未被减去
%   .paz         — 原始方位角（=真值+系统偏差+随机噪声，单位：度，0°=正北）
%   .pvr         — 原始伪径向速度（=真值+随机噪声，单位：m/s）
%                   （伪）是因为双基地径向速度并非目标在某单一方向的速度投影
%   .is_clutter  — 是否为虚警/杂波点迹（逻辑值: true=杂波, false=真实目标检测）
%   .lat / .lon  — 目标真实经纬度（ground truth，用于后续评估对比）
%   .aircraft_id — 目标编号（单目标场景固定为1，本Phase中由外部赋值）
% 【generate_frame_detections 内部流程（每帧每站调用一次）】
%   1. radar_coverage_check() → 判断目标是否在雷达威力范围内
%      （距离: 1000-2000km, 角度: beam_center ± beam_width/2）
%   2. 若在覆盖内: rand() <= Pd(0.6) → 按检测概率判断是否检测到
%      Pd=0.6表示目标在覆盖范围内时有60%的概率被检测到（考虑RCS闪烁、干扰等）
%   3. 若检测到: 生成含噪点迹
%      prange = Rg_true + range_bias + randn()×σ_range  （群距离）
%      paz    = az_true + az_bias + randn()×σ_az         （方位角）
%      pvr    = vr_true + randn()×σ_vr                   （径向速度）
%   4. poissrnd(分辨率单元数 × 虚警率) → 杂波/虚警数量
%      （泊松分布模拟单位时间内随机出现的虚假检测）
%   5. 虚警/杂波点迹在 (range, az) 空间均匀随机采样
%      → is_clutter=true, aircraft_id=NaN
% 【随机数种子管理（确保两部雷达噪声独立）】
%   R1: random_seed + k （k=帧号, 1到n_frames）
%   R2: random_seed + 10000 + k （偏移10000确保与R1种子空间不重叠）
%   每帧使用不同的种子，使得各帧的噪声/检测/杂波随机数独立
%   偏移10000的设计确保: 即使n_frames远小于10000, R1和R2的种子也不交叉
% 【输出】
%   detRaw_R1 — n_frames×1 cell数组，每个cell包含一个点迹结构体数组
%               结构体含 .prange/.paz/.pvr/.is_clutter/.lat/.lon/.aircraft_id
%   detRaw_R2 — 同上，R2的原始点迹
%   注意：detRaw_R* 中各点迹的 .lat/.lon 尚未包含在原始输出中，
%   本Phase在生成后为每个点迹补充 .aircraft_id = 1（单目标标记）
% =========================================================================

fprintf('\n========== Phase 2: 原始点迹生成 ==========\n');

% cell数组: 每个cell存一帧的点迹（每个点迹是一个结构体）
% n_frames行×1列，第k个cell对应第k帧
detRaw_R1 = cell(n_frames, 1);  % R1原始点迹（含系统偏差+随机噪声）
detRaw_R2 = cell(n_frames, 1);  % R2原始点迹（含系统偏差+随机噪声）

% RNG策略: seed+1e7/2e7大偏移连续推进，与run_mc_straight.m完全一致
% 每部雷达仅调用rng一次，帧间随机流连续推进，打破旧rng(seed+k)的Toeplitz对角线相关性
rng(params.random_seed + 1e7);  % R1: 独立随机流
for k = 1:n_frames
    [pos, vel] = aircraft_trajectory_interpolate(traj, t1_grid(k));
    detRaw_R1{k} = generate_frame_detections(params.radar1_lon, params.radar1_lat, ...
        params.radar1_tx_lon, params.radar1_tx_lat, ...
        pos(1), pos(2), vel(1), vel(2), k, t1_grid(k), ...
        params.radar1_range_bias_m, params.radar1_azimuth_bias_deg, ...
        params.radar1_beam_center_deg, params, ...
        params.radar1_range_noise_std_m, params.radar1_azimuth_noise_std_deg);
    for d = 1:length(detRaw_R1{k})
        detRaw_R1{k}(d).aircraft_id = 1;
    end
end

rng(params.random_seed + 2e7);  % R2: 独立随机流
for k = 1:n_frames
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

%% ==================== Phase 3: 时间对齐策略 ====================
% 【目的】声明时间对齐方案，本Phase不执行任何计算
% 【为什么两部雷达时间不同步？】
%   R1在 t=0s, 30s, 60s, 90s, ... 采样（从 offset=0s 开始）
%   R2在 t=13s, 43s, 73s, 103s, ... 采样（从 offset=13s 开始）
%   偏移量 = params.time_offset_radar2_sec = 13秒
%   这意味着两部雷达看到的"同一帧序号k"对应的目标实际时间相差了13秒。
%   对亚音速飞机（~230m/s），13秒的时间偏移导致实际位置差约3km。
%   对于OTH-SWR的定位精度（通常km级），3km的误差不可忽略。
%   因此，在融合之前必须进行时间对齐。
% 【为何不在点迹级对齐？】
%   如果直接对原始点迹做时间插值，会引入以下问题：
%   ① 插值误差（原始点迹本身就含噪声，再插值会叠加误差）
%   ② 破坏量测噪声的统计特性（人为改变噪声方差）
%   ③ 如果目标在某一帧未被检测到（Pd=0.6, 有40%概率漏检），无法插值
% 【更好的方案：航迹级对齐】
%   两部雷达各自在自己的时间网格上独立进行UKF跟踪（Phase 5），
%   得到完整的航迹（包含位置+速度+协方差的完整状态估计）后，
%   再在Phase 6用 time_align_tracks() 将R2航迹外推到R1时间网格。
%   航迹级对齐的优势：
%   ① UKF已滤除了大量噪声，对齐时数据更"干净"
%   ② 有完整的状态向量（lon, lon_rate, lat, lat_rate），
%      可以用CV模型精确外推/回退
%   ③ 有协方差矩阵，外推后可以正确传播不确定性
%   ④ 漏检帧的航迹状态由UKF预测填补，不受影响
% 【策略总结】
%   Phase 3（当前）：纯策略声明，不改变数据
%   Phase 4: 偏差校正（仍然在各自时间网格上）
%   Phase 5: 独立跟踪（各自时间网格上UKF+PDA）
%   Phase 6: 航迹级时间对齐（用CV模型将R2航迹外推到R1时间网格）
% =========================================================================

fprintf('\n========== Phase 3: 时间对齐策略 ==========\n');
fprintf('R1采样: 0s/30s/60s/...  R2采样: 13s/43s/73s/...  偏移=%ds\n', ...
    params.time_offset_radar2_sec);
fprintf('策略: 点迹不做对齐, 两部雷达各自在原时间网格上滤波跟踪\n');
fprintf('      航迹级对齐延后到 Phase 6 融合前, 用 CV 模型全状态外推\n');

%% ==================== Phase 4: 偏差校正 + 几何反解 ====================
% 【目的】用Phase 1估计的偏差校正原始点迹，然后双基地反解出经纬度
% 【步骤详解】
%   4.1 偏差校正（每个点迹）：
%       drange = prange - dr_est   （校正后群距离，单位：米）
%       daz    = paz - da_est      （校正后方位角，单位：度）
%       注意：dr_est 是 Phase 1 从ADS-B统计得到的估计值，并非真实偏差值
%       如果 Phase 1 估计准确（dr_est ≈ range_bias），则 drange ≈ Rg_true + noise
%       如果 Phase 1 估计有误差，校正后仍有残留偏差
%   4.2 双基地几何反解（bistatic_inverse_solver）：
%       已知条件：群距离 Rg（校正后）、方位角 az（校正后）、Tx位置、Rx位置
%       求解目标：目标到Rx的地表距离 r1、目标经纬度 (lon, lat)
%       几何公式推导：
%         设基线 d = distance(Tx, Rx)  （Tx和Rx之间的Haversine距离）
%         设 φ = az - azimuth(Rx→Tx)   （目标方位与Tx方向之间的夹角）
%         对于双基地三角形：
%           Rg = r0 + r1  （群距离 = Tx→目标 + Rx→目标）
%           余弦定理：r0² = d² + r1² - 2×d×r1×cos(φ)
%           联立消去 r0，得：
%           r1 = 0.5 × (Rg² - d²) / (Rg - d × cos(φ))
%         得到 r1 后，用球面正算（Haversine forward）从 Rx 沿 az 方向推 r1 距离
%         得到目标的 (lon, lat)
%   4.3 同时保留原始偏差下的反解经纬度：
%       [~, raw_lat, raw_lon] = bistatic_inverse_solver(prange(原始含偏差), paz(原始含偏差), ...)
%       raw_lat/raw_lon 用于与校正后 lat/lon 对比，可视化偏差校正的效果
%       对比 raw_lat vs lat 可以直观看到：偏差校正使定位精度提升了多少
% 【is_clutter 点的处理】
%   杂波点迹的(.lat, .lon)可能为NaN或随机值，同样经历偏差校正和反解过程，
%   但校正后的坐标没有物理意义。Phase 5 跟踪器通过 is_clutter 字段过滤。
% 【输出】
%   detList_R1{k} — R1第k帧校正后点迹数组，每个点迹新增以下字段：
%     .drange:      校正后群距离（米）= prange - dr1_est
%     .daz:         校正后方位角（度）= paz - da1_est
%     .range_meas:  校正后群距离的副本（与 .drange 相同，命名方便下游使用）
%     .azimuth_meas: 校正后方位角的副本（同上）
%     .lat:         校正后反解纬度（度）（或prange中的原始真值lat）
%     .lon:         校正后反解经度（度）（或prange中的原始真值lon）
%     .raw_lat:     原始偏差下的反解纬度（度，用于对比）
%     .raw_lon:     原始偏差下的反解经度（度，用于对比）
%   detList_R2{k} — 同上，R2第k帧校正后点迹
% =========================================================================

fprintf('\n========== Phase 4: 偏差校正 ==========\n');

% cell数组: 存校正后的点迹
detList_R1 = cell(n_frames, 1);  % R1校正后点迹（含反解经纬度+原始偏差反解经纬度）
detList_R2 = cell(n_frames, 1);  % R2校正后点迹

for k = 1:n_frames
    % ---- R1: 偏差校正 + 几何反解 ----
    dets_r1 = detRaw_R1{k};  % 第k帧原始点迹（含偏差）
    for d = 1:length(dets_r1)
        % 4.1 偏差校正：减去Phase 1估计的系统偏差
        Rgc = dets_r1(d).prange - dr1_est;  % 校正后群距离（米）
        azc = dets_r1(d).paz - da1_est;     % 校正后方位角（度）
        % 将校正值存入点迹结构体
        dets_r1(d).drange = Rgc;
        dets_r1(d).daz = azc;
        dets_r1(d).range_meas = Rgc;    % 副本，兼容下游模块的字段名称约定
        dets_r1(d).azimuth_meas = azc;   % 副本
        % 4.2 双基地几何反解
        % 如果点迹已有有效经纬度（来自generate_frame_detections的真值），则跳过反解
        % 否则用校正后的极坐标(Rgc, azc)反解出经纬度
        % isfield 检查字段是否存在，isnan 检查是否为NaN
        if ~(isfield(dets_r1(d), 'lat') && ~isnan(dets_r1(d).lat))
            [~, lat_e, lon_e] = bistatic_inverse_solver(Rgc, azc, ...
                params.radar1_tx_lon, params.radar1_tx_lat, ...
                params.radar1_lon, params.radar1_lat);
            dets_r1(d).lat = lat_e;
            dets_r1(d).lon = lon_e;
        end
        % 4.3 保留原始偏差下的反解经纬度（用于对比评估偏差校正效果）
        % 使用原始含偏差的 prange 和 paz 做反解
        [~, raw_lat, raw_lon] = bistatic_inverse_solver(dets_r1(d).prange, dets_r1(d).paz, ...
            params.radar1_tx_lon, params.radar1_tx_lat, ...
            params.radar1_lon, params.radar1_lat);
        dets_r1(d).raw_lat = raw_lat;  % 原始偏差反解纬度（可能有大的定位误差）
        dets_r1(d).raw_lon = raw_lon;  % 原始偏差反解经度
    end
    detList_R1{k} = dets_r1;

    % ---- R2: 偏差校正 + 几何反解（与R1完全相同的流程） ----
    dets_r2 = detRaw_R2{k};
    for d = 1:length(dets_r2)
        % 4.1 偏差校正（R2使用自己的偏差估计值 dr2_est, da2_est）
        Rgc = dets_r2(d).prange - dr2_est;
        azc = dets_r2(d).paz - da2_est;
        dets_r2(d).drange = Rgc;
        dets_r2(d).daz = azc;
        dets_r2(d).range_meas = Rgc;
        dets_r2(d).azimuth_meas = azc;
        % 4.2 双基地几何反解（R2的Tx/Rx位置不同）
        if ~(isfield(dets_r2(d), 'lat') && ~isnan(dets_r2(d).lat))
            [~, lat_e, lon_e] = bistatic_inverse_solver(Rgc, azc, ...
                params.radar2_tx_lon, params.radar2_tx_lat, ...
                params.radar2_lon, params.radar2_lat);
            dets_r2(d).lat = lat_e;
            dets_r2(d).lon = lon_e;
        end
        % 4.3 保留原始偏差下的反解经纬度
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

% R1 原始点迹（含偏差，raw_lat/raw_lon 来自未校正量测的反解）
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

% R1 校准后点迹（偏差校正后反解）
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

%% ==================== Phase 5: 单目标航迹跟踪 ====================
% 【目的】对两部雷达的校正后点迹分别执行 UKF + PDA 单目标跟踪
% 【跟踪器架构（single_track_runner 内部流水线，逐帧处理）】
%   状态机: INITIATING(航迹起始) → TRACKING(稳定跟踪) → LOST(丢失) → INITIATING（循环）
%   每个状态的含义：
%     INITIATING: 用前M帧连续关联到的点迹做两点差分初始化，确认后转入TRACKING
%     TRACKING:  每帧执行 UKF 预测+更新循环，失联K帧后转为LOST
%     LOST:      航迹已终止，不再更新（type=7=HISTORY）
%   TRACKING状态下每帧执行的核心循环：
%     1. ukf_jichu('prepare')  → 状态预测 + 量测预测统计（预测新息方差S）
%     2. nn_associate()        → 最近邻关联（地理预筛 + 马氏距离最小）
%     3. pda_weight()          → 概率数据关联（PDA）β权重→加权新息
%     4. ukf_jichu('update')   → Kalman增益 + 状态更新 + 协方差更新
%     5. apply_fuzzy_adapt()   → 模糊自适应Q（航迹成熟后，防止滤波器发散）
% 【单目标特殊处理（single_track_runner vs multi_track_manager）】
%   多目标场景使用 multi_track_manager（含M/N逻辑、航迹起始、航迹终止等），
%   单目标场景直接用 single_track_runner 简化流程：
%     - 跳过M/N逻辑（不需要多假设确认）
%     - 跳过航迹匹配（只有一条航迹，不存在关联歧义）
%     - 直接从第一帧可用点迹初始化（两点差分法）
% 【R1 vs R2 UKF参数差异（反映两部雷达的精度差异）】
%   | 参数            | R1（精密站）  | R2（普通站）  | 含义                     |
%   |-----------------|--------------|--------------|--------------------------|
%   | ukf_Q_scale     | 5e4          | 1e5          | 过程噪声Q缩放因子（2×）   |
%   | ukf_P_pos_std   | 0.2°         | 0.3°         | 初始位置不确定度（1.5×）  |
%   | ukf_P_vel_std   | 0.004°/s     | 0.005°/s     | 初始速度不确定度（1.25×） |
%   | gate_sigma      | 2.0          | 2.5          | 关联波门σ数（更宽）       |
%   | tracker_M       | 默认(4)      | 4            | M/N逻辑的M参数            |
%   | tracker_N       | 默认(8)      | 8            | M/N逻辑的N参数            |
%   | tracker_K_loss  | 默认(8)      | 12           | 允许连续丢失帧数（更宽松）|
%   R2的噪声约为R1的2倍：
%   - Q_scale更大：模型不确定性更高（需容纳更大的预测误差）
%   - gate_sigma更大：关联门放宽（因为量测噪声更大，新息方差更大）
%   - K_loss更大：允许更长的丢失容忍（普通站更容易漏检）
%   - 初始P更大：起始时对目标位置/速度的置信度更低
% 【关联诊断（Phase 5输出统计）】
%   1. 关联率 = 关联帧数 / 跟踪帧数（排除起始和丢失帧）
%      关联率越高说明UKF预测越准确，量测与航迹匹配越好
%      理想情况下关联率应>70%（Pd=0.6 + 部分杂波可能被关联）
%   2. NIS（归一化新息平方）= ν' × S⁻¹ × ν
%      其中 ν = z_meas - z_pred（新息/残差）
%           S = H×P_pred×H' + R（新息协方差）
%      理论上 NIS ~ χ²(dim_z)，dim_z=2（群距离+方位角），理论均值=2
%      门内比例：NIS < 4×2 = 8 的比例，理论值≈86.5%（2σ对应χ²(2)的临界值≈4）
%      若门内比例远低于86%，说明模型不匹配或噪声被低估
%      若NIS均值远大于2，说明系统存在未建模的动态或偏差估计不准
% 【航迹状态编码（type字段）】
%   type=1: RELIABLE(稳定跟踪) — 跟踪质量好，可用于融合
%   type=2: MAINTAIN(维持) — 关联质量略有下降但仍可维护
%   type=6: TEMPORARY(临时/起始中) — 尚未完成M/N确认
%   type=7: HISTORY(历史/死亡) — 航迹已终止，不再更新
% =========================================================================

fprintf('\n========== Phase 5: 单目标航迹跟踪 ==========\n');

% ---- R1 UKF 参数配置（精密站，V2调优后） ----
% 将R1的量测噪声参数注入params，供ukf_jichu('create')构建R矩阵
params.ukf_range_std_m = params.radar1_range_noise_std_m;
params.ukf_azimuth_std_deg = params.radar1_azimuth_noise_std_deg;
params.ukf_Q_scale     = params.radar1_ukf_Q_scale;
params.ukf_P_pos_std   = params.radar1_ukf_P_pos_std;
params.ukf_P_vel_std   = params.radar1_ukf_P_vel_std;
params.gate_sigma      = params.radar1_gate_sigma;
params.gate_vr_ms      = params.radar1_gate_vr_ms;
params.tracker_K_loss  = params.radar1_tracker_K_loss;
% ukf_jichu('create'): 创建UKF模板结构体
%   内部初始化：state_dim(4) + sigma_points + weights + 坐标转换初始化
%   R矩阵 = diag([σ_range², σ_az²]) （量测噪声协方差）
%   radar_lon/lat, tx_lon/lat: 用于极坐标↔经纬度转换
ukf1_tpl = ukf_zishiying('create', params, params.radar1_lon, params.radar1_lat, ...
    params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);

% ---- R2 UKF 参数配置（普通站，V2调优后） ----
% 复制params为params_r2，然后覆写R2特有参数
params_r2 = params;
params_r2.ukf_range_std_m = params.radar2_range_noise_std_m;
params_r2.ukf_azimuth_std_deg = params.radar2_azimuth_noise_std_deg;
params_r2.gate_sigma      = params.radar2_gate_sigma;
params_r2.gate_vr_ms      = params.radar2_gate_vr_ms;
params_r2.ukf_Q_scale     = params.radar2_ukf_Q_scale;
params_r2.ukf_P_pos_std   = params.radar2_ukf_P_pos_std;
params_r2.ukf_P_vel_std   = params.radar2_ukf_P_vel_std;
params_r2.tracker_M       = 4;
params_r2.tracker_N       = 8;
params_r2.tracker_K_loss  = params.radar2_tracker_K_loss;
% 创建R2的UKF模板
ukf2_tpl = ukf_zishiying('create', params_r2, params.radar2_lon, params.radar2_lat, ...
    params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);

% 航迹存储变量初始化
% trackList_R1/R2: cell数组，存储每部雷达的最终航迹列表（单目标各1条）
% trackSnapshots_R1/R2: cell数组(n_frames×1)，每帧的快照含当前所有活跃航迹
trackList_R1 = {};  trackList_R2 = {};
trackSnapshots_R1 = cell(n_frames, 1);  % 每帧的R1航迹状态快照
trackSnapshots_R2 = cell(n_frames, 1);  % 每帧的R2航迹状态快照

% 统计非杂波（真实目标）点迹总数
ac_det_count_r1 = 0;  ac_det_count_r2 = 0;

% 遍历所有帧，统计真实目标检出数（is_clutter==false的点迹）
for k = 1:n_frames
    for d = 1:length(detList_R1{k})
        if ~detList_R1{k}(d).is_clutter
            ac_det_count_r1 = ac_det_count_r1 + 1;
        end
    end
    for d = 1:length(detList_R2{k})
        if ~detList_R2{k}(d).is_clutter
            ac_det_count_r2 = ac_det_count_r2 + 1;
        end
    end
end

% 单目标简化跟踪（single_track_runner 内部无M/N, 直接初始化）
% single_track_runner 的输入：
%   detList: 逐帧点迹cell数组
%   ukf_tpl: UKF模板结构体
%   params:  参数结构体
% single_track_runner 的输出：
%   trackSnapshots: n_frames×1 cell，每帧的快照结构体
%     .trackList: 当前活跃的航迹列表（cell数组）
%       每条航迹结构体含: .id, .type, .life, .lon, .lat, .ukf, .assoc_det 等
%   finalTrk: 最终的航迹结构体
[trackSnapshots_R1, finalTrk1] = single_track_runner(detList_R1, ukf1_tpl, params, n_frames, true_track, t1_grid);
[trackSnapshots_R2, finalTrk2] = single_track_runner(detList_R2, ukf2_tpl, params_r2, n_frames, true_track, t2_grid);
trackList_R1 = {finalTrk1};  % 单目标只有一条航迹
trackList_R2 = {finalTrk2};

fprintf('跟踪完成: %d 帧\n', n_frames);
% 理论真实检出数 ≈ n_frames × Pd ≈ n_frames × 0.6 (全部在覆盖内)
% 实际因覆盖外和杂波干扰会略有偏差
fprintf('  R1目标检出=%d, R2目标检出=%d\n', ac_det_count_r1, ac_det_count_r2);

fprintf('\n--- 航迹统计 ---\n');
% type=1(RELIABLE): 跟踪稳定可靠, quality=航迹质量评分(0-100), life=存活帧数
fprintf('R1: type=%s quality=%d life=%d\n', ...
    get_type_str(finalTrk1.type), finalTrk1.quality, finalTrk1.life);
fprintf('R2: type=%s quality=%d life=%d\n', ...
    get_type_str(finalTrk2.type), finalTrk2.quality, finalTrk2.life);

% ---- 关联诊断（统计关联率、NIS等跟踪质量指标） ----
for radar_label = {'R1', 'R2'}
    % 选择对应的航迹快照
    snaps = trackSnapshots_R1;
    if strcmp(radar_label{1}, 'R2'), snaps = trackSnapshots_R2; end
    n_assoc = 0; n_predict = 0; n_init = 0; n_lost = 0;
    init_frame = 0; nis_vals = [];  % NIS值列表，用于计算统计量
    for k = 1:length(snaps)
        if isempty(snaps{k}.trackList), continue; end
        trk = snaps{k}.trackList{1};  % 单目标取第一条（也是唯一一条）
        % 分类统计：根据航迹type判断当前状态
        if trk.type == 6
            n_init = n_init + 1;       % 起始中（TEMPORARY）
        elseif trk.type == 1
            % TRACKING状态：判断是否有关联到点迹
            % assoc_det非空→关联到了点迹；空→只有预测无更新
            if ~isempty(trk.assoc_det) && isstruct(trk.assoc_det) && isfield(trk.assoc_det, 'prange') && ~isempty(trk.assoc_det.prange)
                n_assoc = n_assoc + 1;   % 有关联
            else
                n_predict = n_predict + 1; % 纯预测（missed detection）
            end
            % 收集NIS历史值（用于计算均值和门内比例）
            if isfield(trk.ukf, 'nis_history')
                nis_vals = [nis_vals, trk.ukf.nis_history];
            end
        elseif trk.type == 7
            n_lost = n_lost + 1;        % 死亡/历史
        end
        % 记录进入TRACKING的第一帧（航迹起始帧号）
        if init_frame == 0 && trk.type == 1, init_frame = k; end
    end
    n_tracked = n_assoc + n_predict;  % 总跟踪帧数
    fprintf('%s: 起始帧=%d | 关联=%d 纯预测=%d (关联率=%.0f%%) | 起始中=%d 丢失=%d\n', ...
        radar_label{1}, init_frame, n_assoc, n_predict, ...
        n_assoc/max(1,n_tracked)*100, n_init, n_lost);
    if ~isempty(nis_vals)
        % NIS门限：χ²(2)在2σ下的临界值约4, 对应NIS<4×dim=4×2=8
        % 即当dim_z=2时，门内范围是(0, 8)
        nis_in_gate = sum(nis_vals < 4*2);
        fprintf('  NIS: 均值=%.2f 门内=%.0f%% (%d/%d)\n', ...
            mean(nis_vals), nis_in_gate/length(nis_vals)*100, nis_in_gate, length(nis_vals));
    end
end
fprintf('\n');

%% ---- 基础UKF滤波RMSE统计 ----
fprintf('\n--- 基础UKF滤波RMSE ---\n');

% R1 UKF航迹
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

% R2 UKF航迹（R2时间网格）
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

%% ==================== Phase 6: 航迹级时间对齐 ====================
% 【目的】将R2航迹从 t2_grid 外推到 t1_grid，实现两部雷达航迹的时间同步
% 【方法】CV（匀速）模型全状态外推
%   对R2的每帧航迹状态 x(t2)，用CV模型外推到R1对应时刻 t1 = t2 - offset：
%     状态向量 x = [lon, lon_rate, lat, lat_rate]'（4维）
%     状态转移矩阵 F(dt) = [1 dt 0 0; 0 1 0 0; 0 0 1 dt; 0 0 0 1]
%     外推（负时间步长 = 逆向预测）：
%       x(t1) = F(-offset) × x(t2)                     （位置: 往回推 offset·vel）
%       P(t1) = F(-offset) × P(t2) × F(-offset)' + Q(|offset|)
%     其中 F(-offset) = [1 -offset 0 0; 0 1 0 0; 0 0 1 -offset; 0 0 0 1]
%     即：位置往回退 offset × 速度，速度不变（CV假设）
%   逆向预测会增大协方差（体现"外推"带来的额外不确定性）：
%     Q(|offset|) 是与偏移时间成正比的附加过程噪声
%     意味着：外推越远（offset越大），协方差越大，权重越低
% 【为什么用CV模型而不是更复杂模型？】
%   offset=13秒相对较短，CV模型近似足够：
%   - 更高阶模型（CA、CT）在短时间外推上改善有限
%   - CV模型更简单、数值更稳定（不会因高阶项累积误差）
% 【调用】time_align_tracks(trackSnapshots_R2, params)
%   time_align_tracks 内部逐帧处理，将R2每帧的航迹状态外推到R1对应帧号的时间
%   外推后的航迹快照附着在对应的t1_grid帧号上
% 【输出】
%   aligned_R2 — n_frames×1 cell，对齐到R1时间网格的R2航迹快照
%   每个cell的结构与 trackSnapshots_R2 相同（.trackList 等字段），
%   但状态值和协方差已调整到R1的时间戳
% =========================================================================

fprintf('\n========== Phase 6: 航迹级时间对齐 ==========\n');
fprintf('将R2航迹 (t2_grid) 用CV模型全状态外推到R1时间网格 (t1_grid)\n');

aligned_R2 = time_align_tracks(trackSnapshots_R2, params);
fprintf('R2航迹时间对齐完成\n');

%% ==================== Phase 7: 航迹融合（四种算法） ====================
% 【目的】将R1和R2的对齐后航迹进行融合，比较四种融合算法的性能
% 【单目标特殊处理】直接1对1融合（R1航迹ID=1 与 R2航迹ID=1）
%   多目标场景需要先执行 track_management（航迹关联/匹配），找到 R1 和 R2
%   中属于同一目标的航迹对。单目标场景只有一条航迹，跳过匹配直接融合。
% 【matched_pair 结构体】
%   用于告知 run_track_fusion 哪条R1航迹与哪条R2航迹配对融合
%   .R1_track_id:  R1侧的航迹ID（单目标固定为1）
%   .R2_track_id:  R2侧的航迹ID（单目标固定为1）
%   .match_count:  两部雷达共同跟踪到该目标的帧数
%   .coexist_count: 两部雷达同时有跟踪数据的帧数
%   .match_ratio:  match_count / coexist_count（匹配成功率, 单目标=100%）
%   .mean_dist_km: 两雷达航迹位置的平均距离（km）
%   .quality:      匹配质量评分（0-100）
% 【四种融合算法】输入都是 (x1, P1, x2, P2) → (x_fused, P_fused)
%   SCC (Simple Convex Combination, 简单凸组合):
%     假设R1和R2的估计误差完全不相关（互协方差P12=0）
%     公式: P⁻¹ = P₁⁻¹ + P₂⁻¹
%           x = P × (P₁⁻¹×x₁ + P₂⁻¹×x₂)
%     优点: 计算简单, 稳定
%     缺点: 若实际存在相关性, 会过于乐观（高估精度, 低估协方差）
%
%   BC (Bar-Shalom-Campo, 精确融合):
%     显式计算互协方差 P12=(I-K1×H)×F×P12_prev×F'×(I-K2×H)' + (I-K1×H)×Q×(I-K2×H)'
%     公式: P = P₁ + P₂ - P12 - P12'  (全信息矩阵公式)
%           x = x₁ + (P₁ - P12)×(P₁ + P₂ - P12 - P12')⁻¹×(x₂ - x₁)
%     优点: 考虑互协方差, 理论上最优（当P12准确时）
%     缺点: 需要维护和传播P12矩阵, 计算量大
%
%   CI (Covariance Intersection, 协方差交叉):
%     保守融合策略, 不假设误差独立也不计算P12
%     公式: P⁻¹ = w×P₁⁻¹ + (1-w)×P₂⁻¹         (0 < w < 1)
%           x = P × (w×P₁⁻¹×x₁ + (1-w)×P₂⁻¹×x₂)
%           w* = argmin_w det(P)                 (fminbnd一维搜索)
%     优点: 保守但安全（协方差不会低估, 保证一致性）
%     缺点: 需要迭代优化w（计算量较SCC大）
%
%   FCI (Fast CI, 快速协方差交叉):
%     CI的闭式近似解, 无需迭代优化
%     公式: w = tr(P₁)⁻¹ / (tr(P₁)⁻¹ + tr(P₂)⁻¹)  （迹加权）
%     优点: 计算量与SCC相当（无需迭代）, 结果接近CI
%     缺点: w的解析近似并非精确最优（但通常足够好）
%
%   算法预期表现（按RMSE从小到大）: BC ≈ CI < FCI < SCC
%     但BC的协方差可能过于乐观（互协方差估计不准时）
%     SCC在独立性假设成立时也很好, 但双基地场景通常不独立
%     CI/FCI 提供保守但不至于发散的估计
% 【输出】
%   all_fused_snapshots — 4×1 cell数组
%     {1}: SCC融合航迹快照
%     {2}: BC融合航迹快照
%     {3}: CI融合航迹快照
%     {4}: FCI融合航迹快照
%   每个cell是 n_frames×1 cell，结构与 trackSnapshots 相同
% =========================================================================

fprintf('\n========== Phase 7: 航迹融合 (四种算法) ==========\n');

% 单目标: 直接1对1融合, 各站严格1条航迹 (ID=1)
r1_id = 1; r2_id = 1;
fprintf('融合对: R1#1 <-> R2#1 (直接1对1)\n');

% 构建单对匹配结构体（多目标场景由 track_management 输出配对列表）
matched_pair = struct('R1_track_id', r1_id, 'R2_track_id', r2_id, ...
    'match_count', 0, 'coexist_count', 0, 'match_ratio', 1.0, ...
    'mean_dist_km', 0, 'quality', 100);

% 四种融合算法的简称（用于日志输出和图表标注）
method_names = {'SCC', 'BC', 'CI', 'FCI'};
all_fused_snapshots = cell(length(method_names), 1);  % 4×1 cell, 存每种算法的融合结果

for m = 1:length(method_names)
    method = method_names{m};
    fprintf('  运行 %s 融合...\n', method);
    % run_track_fusion: 对匹配对执行指定算法的融合
    % 输入: matched_pair(配对信息), trackSnapshots_R1(R1快照), aligned_R2(对齐后R2快照),
    %       params(参数), method(算法名)
    % 输出: 融合后的航迹快照（n_frames×1 cell）
    all_fused_snapshots{m} = run_track_fusion(matched_pair, ...
        trackSnapshots_R1, aligned_R2, params, method);
end
fprintf('融合完成: %d 种算法\n', length(method_names));

%% ---- 融合RMSE统计 ----
fprintf('\n--- 融合RMSE ---\n');
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
    fprintf('%s 融合                 RMSE: %6.1f km (n=%d)\n', method_names{m}, rms_km(errs), length(errs));
end
% R1/R2单站（对齐后，用于对比）
errs_r1 = [];
for k = 1:n_frames
    tl = interp1(true_track(:,5), true_track(:,1), t1_grid(k), 'linear', 'extrap');
    tb = interp1(true_track(:,5), true_track(:,2), t1_grid(k), 'linear', 'extrap');
    snap = trackSnapshots_R1{k};
    if ~isempty(snap.trackList)
        trk = snap.trackList{1};
        if trk.type ~= 7 && ~isnan(trk.lat)
            errs_r1(end+1) = sphere_utils_haversine_distance(trk.lon, trk.lat, tl, tb) / 1000;
        end
    end
end
fprintf('R1 单站(对齐后)        RMSE: %6.1f km (n=%d)\n', rms_km(errs_r1), length(errs_r1));

errs_r2 = [];
for k = 1:n_frames
    tl = interp1(true_track(:,5), true_track(:,1), t1_grid(k), 'linear', 'extrap');
    tb = interp1(true_track(:,5), true_track(:,2), t1_grid(k), 'linear', 'extrap');
    snap = aligned_R2{k};
    if ~isempty(snap.trackList)
        trk = snap.trackList{1};
        if trk.type ~= 7 && ~isnan(trk.lat)
            errs_r2(end+1) = sphere_utils_haversine_distance(trk.lon, trk.lat, tl, tb) / 1000;
        end
    end
end
fprintf('R2 单站(对齐后)        RMSE: %6.1f km (n=%d)\n', rms_km(errs_r2), length(errs_r2));

%% ==================== Phase 8: 定量误差评估 ====================
% 【目的】计算并对比融合航迹与单站航迹的位置误差，量化融合增益
% 【评估指标】
%   RMSE（均方根误差，km）:
%     RMSE = sqrt(mean((est_lon - true_lon)² + (est_lat - true_lat)²))
%     转换为km: 用 Haversine 公式计算每点的大圆距离误差
%     RMSE 是主要评价指标，对大的误差点更敏感（平方加权）
%   中位误差（km）:
%     median(distance_vector)
%     对异常值更鲁棒（不受个别大误差点影响）
%   UKF vs 检测改善（%）:
%     改善率 = (原始点迹RMSE - UKF滤波后RMSE) / 原始点迹RMSE × 100
%     正数表示滤波改善了精度，负数表示滤波反而恶化（通常不应出现）
%     衡量 UKF 滤波器相对于原始点迹精度的提升百分比
% 【评估流程】
%   8.1 构建 matcher_simple 结构体（单目标简化版 matcher）
%       包含匹配对信息、对齐后R2航迹、R1/R2航迹位置历史
%       r1_pos / r2_pos: 3D数组 [1, n_frames, 2]，第三维为(lon, lat)
%       这个matcher是 evaluate_all('fusion') 需要的输入格式
%   8.2 evaluate_all('fusion') → 融合航迹 vs 真值
%       为每种融合算法 + R1_only + R2_only 计算统计量
%       输出 fusion_eval.overall(1..6) 每个的 .s 子结构含:
%         .rms: RMSE (km)
%         .median: 中位误差 (km)
%         .mean: 平均误差 (km)
%         .pct95: 95%分位数误差 (km)
%   8.3 evaluate_all('tracking_errors') → 单站UKF误差
%       计算 UKF 滤波后航迹位置相对于真值的误差
%       同时计算 UKF 相对于原始点迹的改善率
%   8.4 打印对比表：融合算法(4种) + R1_only + R2_only = 6行对比
%       找出最优融合算法（RMSE最小者）
%       计算融合相对于各单站的改善百分比
% 【输出】
%   fusion_eval — 融合评估结构体
%     .overall: 1×6 结构体数组（4种融合+2个单站）
%       每个 .s 含 .rms, .median, .mean, .pct95
%     .summary: 6×1 结构体数组
%   errorStats_R1 — R1单站UKF跟踪误差统计结构体
%     .radar: 'R1'
%     .summary(1).ukf: UKF航迹误差统计（.n, .median, .mean, .rms, .pct95）
%     .summary(1).ukf_vs_det_pct: UKF相对原始点迹的改善百分比
%   errorStats_R2 — R2单站UKF跟踪误差统计结构体
%     .summary(1).ukf: UKF航迹误差统计
%     .summary(1).ukf_vs_det_pct: 改善百分比
% =========================================================================

fprintf('\n========== Phase 8: 定量误差评估 ==========\n');

% 构建 matcher 结构体（用于 evaluate_fusion，单目标简化版）
% 多目标场景需要完整的 track_management 输出；单目标手动构建
n_frames_val = n_frames;
matcher_simple = struct();
matcher_simple.matched_pairs = matched_pair;  % 匹配对（R1#1 ↔ R2#1）
matcher_simple.aligned_R2 = aligned_R2;       % 对齐后的R2快照
matcher_simple.r1_ids = r1_id;                % R1侧航迹ID
matcher_simple.r2_ids = r2_id;                % R2侧航迹ID

% 提取R1航迹位置历史（用于 evaluate_fusion 中计算位置偏差）
% r1_pos: [1, n_frames, 2] 3D数组，单目标维=1, 帧维=n_frames, 坐标维=2(lon,lat)
% NaN表示该帧无航迹（在覆盖外或未检出导致的缺失帧）
r1_pos = nan(1, n_frames, 2);
for k = 1:n_frames
    snap = trackSnapshots_R1{k};  % 第k帧快照
    if ~isempty(snap.trackList)
        for t = 1:length(snap.trackList)
            if snap.trackList{t}.id == r1_id  % 找到ID=1的航迹
                r1_pos(1, k, 1) = snap.trackList{t}.lon;  % 经度
                r1_pos(1, k, 2) = snap.trackList{t}.lat;  % 纬度
                break;
            end
        end
    end
end
matcher_simple.r1_pos = r1_pos;

% 提取R2航迹位置历史（从对齐后的R2快照，已在R1时间网格上）
r2_pos = nan(1, n_frames, 2);
for k = 1:n_frames
    snap = aligned_R2{k};  % 对齐后快照（时间已同步到R1网格）
    if ~isempty(snap.trackList)
        for t = 1:length(snap.trackList)
            if snap.trackList{t}.id == r2_id  % 找到ID=1的航迹
                r2_pos(1, k, 1) = snap.trackList{t}.lon;
                r2_pos(1, k, 2) = snap.trackList{t}.lat;
                break;
            end
        end
    end
end
matcher_simple.r2_pos = r2_pos;

% 融合误差评估（用 evaluate_all('fusion') 统一接口）
% truthTrajs: cell数组含真值结构体（单目标场景只有1个）
truthTrajs = {truthTraj};
fusion_eval = evaluate_all('fusion', all_fused_snapshots, method_names, ...
    matched_pair, trackSnapshots_R1, trackSnapshots_R2, ...
    truthTrajs, n_frames, params.dt_sec, matcher_simple);

% 打印融合 vs 单站对比表
fprintf('\n--- 融合误差对比 (RMSE km) ---\n');
fprintf('%-8s %8s %8s\n', '算法', 'RMSE', '中位');
fprintf('%-8s %8s %8s\n', '------', '------', '------');
% 所有6种方案: 4种融合算法 + R1单站 + R2单站
all_method_labels = [method_names, {'R1_only', 'R2_only'}];
for m = 1:length(all_method_labels)
    s = fusion_eval.overall(m).s;  % .s 子结构含 .rms, .median, .mean, .pct95
    fprintf('%-8s %8.1f %8.1f\n', all_method_labels{m}, s.rms, s.median);
end

% 找出最佳融合算法（RMSE最小者）
% arrayfun 对 fusion_eval.overall(1:4) 的每个元素提取 .s.rms
rms_vals = arrayfun(@(x) x.s.rms, fusion_eval.overall(1:4));
[best_fusion_rmse, best_m] = min(rms_vals);
r1_rmse = fusion_eval.overall(5).s.rms;  % R1单站RMSE
r2_rmse = fusion_eval.overall(6).s.rms;  % R2单站RMSE
fprintf('\n最佳融合算法: %s (RMSE=%.1fkm)\n', method_names{best_m}, best_fusion_rmse);
% 改善百分比计算公式: (单站RMSE - 融合RMSE) / 单站RMSE × 100
% 正值表示融合改善了精度，负值表示融合反而变差
fprintf('融合 vs R1(精密站): %+.1f%%\n', (1 - best_fusion_rmse/r1_rmse)*100);
fprintf('融合 vs R2(普通站): %+.1f%% 改善\n', (1 - best_fusion_rmse/r2_rmse)*100);

% 单站跟踪误差评估（UKF航迹 vs 真值，评估UKF滤波本身的效果）
% 使用对齐后的R2航迹（与R1时间一致，便于对比）
aligned_R2_eval = time_align_tracks(trackSnapshots_R2, params);
errorStats_R1 = evaluate_all('tracking_errors', trackSnapshots_R1, detList_R1, ...
    truthTrajs, n_frames, params.dt_sec, 'R1');
errorStats_R2 = evaluate_all('tracking_errors', aligned_R2_eval, detList_R2, ...
    truthTrajs, n_frames, params.dt_sec, 'R2');

% 打印各站的UKF跟踪误差统计
for es = {errorStats_R1, errorStats_R2}
    e = es{1};  % e 是 errorStats 结构体
    fprintf('\n--- %s UKF滤波误差 ---\n', e.radar);
    fprintf('%-6s %6s %8s %8s %8s %8s %8s\n', ...
        '飞机', '点数', '中位(km)', '均值(km)', 'RMSE(km)', '95%(km)', 'vs检测');
    s_u = e.summary(1).ukf;  % .ukf 子结构含 UKF 误差的统计量
    fprintf('飞机A   %6d %8.1f %8.1f %8.1f %8.1f %7.0f%%\n', ...
        s_u.n, s_u.median, s_u.mean, s_u.rms, s_u.pct95, ...
        e.summary(1).ukf_vs_det_pct);  % UKF相对于原始点迹的改善百分比
end

%% ==================== Phase 9: 可视化 + 数据保存 ====================
% 【目的】生成所有分析图表并保存完整仿真数据
% 【图表清单】（共生成约5-6张图）
%   fig1: 场景总览图 (plot_scene_overview)
%     - 真值航迹（黑色实线）+ 雷达位置标注（R1红色★，R2蓝色★）
%     - 两部雷达的覆盖扇形区域（半透明色块）
%     - 经度/纬度坐标网格 + 比例尺
%     保存为: results/fig1_scene_overview.png
%
%   fig2a: R1原始点迹3D点云 (plot_point_cloud_3d)
%     - X轴: 群距离(km), Y轴: 方位角(deg), Z轴: 伪径向速度(m/s)
%     - 真实目标点迹: 红色, 杂波点迹: 灰色
%     - 保存为: results/fig2a_R1_point_cloud.png
%
%   fig2b: R2原始点迹3D点云 (plot_point_cloud_3d)
%     - 与R1相同格式
%     - 保存为: results/fig2b_R2_point_cloud.png
%
%   fig3: 单目标跟踪综合分析图 (plot_results('single_track'))
%     - 6子图布局（可能的布局：地图上的航迹+点云+误差时序+诊断指标）
%     - 同时展示R1和R2的跟踪结果
%     - 保存为: results/fig3_single_track*.png
%
%   fig4: 融合结果综合图 (plot_results('single_fusion'))
%     - 8子图布局（4种算法×2个视角，或类似划分）
%     - 比较SCC/BC/CI/FCI四种算法的融合效果
%     - 标注最优算法和相应的RMSE值
%     - 保存为: results/fig4_fusion*.png
%
% 【保存内容】（results/simulation_YYYYMMDD_HHMMSS.mat）
%   sysPara:
%     系统参数快照，包含: 雷达位置(经纬度+Tx/Rx)、时间步长、帧数、
%     雷达噪声参数(σ_range, σ_az, σ_vr)、覆盖范围(距离/角度)、
%     检测概率Pd、虚警率Pfa、随机种子等
%     用途: 存档仿真配置，便于后续复现或对比不同参数组合的结果
%
%   calibResult:
%     标定结果结构体，包含:
%       dr1/2_est: 估计的距离偏差（米）和方位偏差（度）
%       dr1/2_true: 真实的距离偏差和方位偏差（从params中提取）
%       n_cal_R1/R2: 用于标定的ADS-B样本点数
%     用途: 评估标定精度（估计值 vs 真实值的误差）
%
%   truthTraj:
%     目标真值轨迹结构体（.label, .speed_ms, .time_sec, .lat, .lon等）
%
%   R1:
%     .detRaw:        原始点迹cell（n_frames×1）
%     .detList:       校正后点迹cell（n_frames×1，含反解经纬度）
%     .trackSnapshots: UKF跟踪快照cell（n_frames×1，每帧的航迹状态）
%     .finalTrack:    最终航迹结构体（单目标）
%     .targetDetCount: 真实目标检出总数（用于统计Pd实际值）
%
%   R2: 同上，R2的对应数据
%
%   params: 完整参数结构体（所有配置项的当前值）
%
%   errorStats_R1/R2: 单站UKF跟踪误差统计
%     .radar: 'R1'或'R2'
%     .summary(1).ukf:  .n(点数) .median .mean .rms .pct95(km)
%     .summary(1).det:  原始点迹的对应统计量
%     .summary(1).ukf_vs_det_pct: UKF相对于原始点迹的改善百分比
%
%   fusion_eval: 融合评估结果
%     .overall: 1×6结构体数组（4种融合+2个单站），每个.s含统计量
%     .summary: 6×1结构体数组
%
%   all_fused_snapshots: 4×1 cell（SCC/BC/CI/FCI的融合航迹快照）
%
%   method_names: {'SCC','BC','CI','FCI'}
% =========================================================================

fprintf('\n========== Phase 9: 可视化 ==========\n');
% 确保 results/ 目录存在（不存在则创建）
if ~exist('results', 'dir'), mkdir('results'); end

% 暂禁MATLAB 2026a内部UI尺寸警告（不影响实际出图质量和内容）
% 某些MATLAB版本在figure窗口尺寸变化时会产生内部警告信息
% 这些警告与用户代码无关，关闭以避免控制台干扰
warn_state = warning('off', 'all');

% 图1: 场景总览 — 真值航迹+雷达位置+覆盖扇形
plot_scene_overview(true_track, params, 'results');

% 图2a/2b: R1和R2的原始点迹3D点云（距离×方位×径向速度空间）
% 用于直观检查点迹质量：目标点迹应聚为一簇，杂波应随机散布
plot_point_cloud_3d(detList_R1, 'R1', 'results/fig2a_R1_point_cloud.png');
plot_point_cloud_3d(detList_R2, 'R2', 'results/fig2b_R2_point_cloud.png');

% 图3: 单目标跟踪综合图
% 'single_track' 模式: 同时展示R1和R2的UKF跟踪结果
% 包含航迹地图、误差时序、关联诊断等子图
plot_results('single_track', true_track, detList_R1, detList_R2, ...
    trackSnapshots_R1, trackSnapshots_R2, params, 'results');

% 图4: 融合可视化
% 'single_fusion' 模式: 展示四种融合算法的对比
% 传入 best_m 以高亮标注最优算法
% 传入 fusion_eval 以显示各算法的RMSE数据
plot_results('single_fusion', true_track, trackSnapshots_R1, trackSnapshots_R2, ...
    all_fused_snapshots, method_names, best_m, fusion_eval, truthTraj, params, 'results');

warning(warn_state);  % 恢复警告状态到绘图前的设置

fprintf('\n========== Phase 9: 数据保存 ==========\n');
% ---- 构建 sysPara 结构体：系统参数快照 ----
% 将所有仿真配置参数打包为一个结构体，与.mat数据一起存档
% 这样即使原始 simulation_params.m 文件被修改，存档的参数依然可读
sysPara = struct(...
    'dt_sec', params.dt_sec, 'n_frames', n_frames, ...          % 时间和帧数
    'R1_lon', params.radar1_lon, 'R1_lat', params.radar1_lat, ... % R1位置
    'R1_tx_lon', params.radar1_tx_lon, 'R1_tx_lat', params.radar1_tx_lat, ... % R1发射站
    'R1_beam_center_deg', params.radar1_beam_center_deg, ...    % R1波束中心
    'R1_range_bias_m', params.radar1_range_bias_m, ...          % R1真实距离偏差(用于对比标定结果)
    'R1_azimuth_bias_deg', params.radar1_azimuth_bias_deg, ...  % R1真实方位偏差
    'R2_lon', params.radar2_lon, 'R2_lat', params.radar2_lat, ... % R2位置
    'R2_tx_lon', params.radar2_tx_lon, 'R2_tx_lat', params.radar2_tx_lat, ... % R2发射站
    'R2_beam_center_deg', params.radar2_beam_center_deg, ...    % R2波束中心
    'R2_range_bias_m', params.radar2_range_bias_m, ...          % R2真实距离偏差
    'R2_azimuth_bias_deg', params.radar2_azimuth_bias_deg, ...  % R2真实方位偏差
    'beam_width_deg', params.beam_width_deg, ...                % 波束宽度(度)
    'range_km', [params.range_min_km, params.range_max_km], ... % 覆盖距离范围
    'detection_probability', params.detection_probability, ...   % Pd(检测概率)
    'false_alarm_rate', params.false_alarm_rate, ...             % Pfa(虚警率)
    'radar1_range_noise_m', params.radar1_range_noise_std_m, ... % R1距离噪声σ
    'radar1_az_noise_deg', params.radar1_azimuth_noise_std_deg, ... % R1方位噪声σ
    'radar2_range_noise_m', params.radar2_range_noise_std_m, ... % R2距离噪声σ
    'radar2_az_noise_deg', params.radar2_azimuth_noise_std_deg, ... % R2方位噪声σ
    'radial_vel_noise_std_ms', params.radial_vel_noise_std_ms, ... % 径向速度噪声σ
    'random_seed', params.random_seed);                            % 随机种子

% ---- 构建 calibResult 结构体：标定结果存档 ----
% 同时保存估计值和真实值，便于后续分析标定精度
calibResult = struct(...
    'dr1_est', dr1_est, 'da1_est', da1_est, ...              % R1估计偏差
    'dr2_est', dr2_est, 'da2_est', da2_est, ...              % R2估计偏差
    'dr1_true', params.radar1_range_bias_m, 'da1_true', params.radar1_azimuth_bias_deg, ... % R1真实偏差
    'dr2_true', params.radar2_range_bias_m, 'da2_true', params.radar2_azimuth_bias_deg, ... % R2真实偏差
    'n_cal_R1', length(dr1_list), 'n_cal_R2', length(dr2_list));  % 标定样本数

% ---- 构建 R1/R2 结构体：各雷达的完整数据流水线 ----
% 使用 cell 包装确保 struct() 将 detRaw 整个 cell 数组作为单个字段值存入
R1 = struct('detRaw', {detRaw_R1}, 'detList', {detList_R1}, ...          % 点迹数据
    'trackSnapshots', {trackSnapshots_R1}, 'finalTrack', finalTrk1, ...  % 跟踪结果
    'targetDetCount', ac_det_count_r1);                                    % 检出统计
R2 = struct('detRaw', {detRaw_R2}, 'detList', {detList_R2}, ...
    'trackSnapshots', {trackSnapshots_R2}, 'finalTrack', finalTrk2, ...
    'targetDetCount', ac_det_count_r2);

% 构造带时间戳的文件名，避免覆盖之前的仿真结果
% datestr(now, 'yyyymmdd_HHMMSS') 生成如 '20260525_143052' 的字符串
outf = fullfile('results', sprintf('simulation_%s.mat', datestr(now, 'yyyymmdd_HHMMSS')));
% save 将工作区变量保存到 .mat 文件（MATLAB 二进制数据格式）
save(outf, 'sysPara', 'calibResult', 'truthTraj', 'R1', 'R2', 'params', ...
    'errorStats_R1', 'errorStats_R2', 'fusion_eval', ...
    'all_fused_snapshots', 'method_names');
fprintf('数据已保存: %s\n', outf);
fprintf('\nDone.\n');

% =========================================================================
% 内部函数
% =========================================================================
% 以下三个辅助函数定义在同一个 .m 文件内（MATLAB允许脚本文件包含局部函数）。
% 它们仅在本文件的上下文中可见，外部模块无法直接调用。
% =========================================================================

% get_type_str — 将航迹类型数字编码转换为可读字符串
%   输入: t — 整数类型编码
%   输出: s — 可读的航迹类型名称字符串
%   映射关系:
%     1 → 'RELIABLE'  (稳定跟踪，跟踪质量好，可用于融合)
%     2 → 'MAINTAIN'  (维持状态，质量略有下降但仍可追踪)
%     6 → 'TEMPORARY' (临时航迹，处于M/N起始确认过程中)
%     7 → 'HISTORY'   (历史/死亡航迹，已不再更新)
%     其他 → 'UNKNOWN' (异常状态)
function s = get_type_str(t)
    switch t
        case 1, s = 'RELIABLE';
        case 2, s = 'MAINTAIN';
        case 6, s = 'TEMPORARY';
        case 7, s = 'HISTORY';
        otherwise, s = 'UNKNOWN';
    end
end

% find_active_tracks — 获取航迹列表中所有非死亡(type~=7)航迹的索引
%   输入: trackList — 航迹结构体cell数组
%   输出: idx — 活跃航迹的索引数组（type不等于7的航迹）
%   用途: 过滤掉已终止(HISTORY)的航迹，只保留仍在跟踪或起始中的航迹
%   注意: 本函数在目前的单目标简化流程中未直接调用，但保留作为
%         多目标场景下 track_management 模块的备用工具
function idx = find_active_tracks(trackList)
    idx = [];
    for t = 1:length(trackList)
        if trackList{t}.type ~= 7  % type=7 表示已死亡的HISTORY航迹
            idx(end+1) = t;        % 追加到索引列表末尾
        end
    end
end

% find_reliable — 获取航迹列表中所有可靠(type==1)航迹的索引
%   输入: trackList — 航迹结构体cell数组
%   输出: idx — 可靠航迹的索引数组（type等于1的航迹）
%   用途: 筛选出稳定跟踪的航迹，用于融合等高质量数据需求场景
%   type=1(RELIABLE) 表示跟踪质量最好、最稳定的航迹
%   注意: 本函数在目前的单目标简化流程中未直接调用，但保留作为
%         多目标场景下的备用工具
function idx = find_reliable(trackList)
    idx = [];
    for t = 1:length(trackList)
        if trackList{t}.type == 1  % type=1 表示RELIABLE稳定跟踪
            idx(end+1) = t;
        end
    end
end

% rms_km — 计算误差向量的RMSE（km）
%   输入: e — 误差值向量（km），可为空
%   输出: v — RMSE值（km），空向量返回NaN
function v = rms_km(e)
    if isempty(e), v = NaN; else, v = sqrt(mean(e.^2)); end
end
