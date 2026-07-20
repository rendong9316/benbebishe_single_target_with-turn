function run_filter_math_tests()
% RUN_FILTER_MATH_TESTS 滤波器数学验证测试套件。
%
% 测试覆盖：
%   1. UKF Sigma 点传播和量测均值计算（含方位角绕环处理）
%   2. IMM 模型混合权重和组合预测
%   3. IMM 双向转弯模型（左转/右转）
%   4. 跟踪误差时间网格对齐

    % 测试入口：依次运行三个数学验证测试
    test_ukf_sigma_measurement_mean();   % 验证 UKF Sigma 点传播和量测均值计算的正确性
    test_imm_prediction_weights();       % 验证 IMM 模型混合权重和组合预测的正确性
    test_imm_bidirectional_turn_models();  % 验证双向转弯模型
    test_physical_process_noise();       % 验证物理 Q 和跨雷达一致性
    test_tracking_error_time_grids();    % 验证时间网格对齐和误差计算的正确性
    disp('filter math tests ok');        % 全部通过则打印成功消息
end


function test_physical_process_noise()
    params1 = radar_params(simulation_params_oracle(), 1);
    params2 = radar_params(simulation_params_oracle(), 2);
    ukf1 = ukf_jichu('create', params1, params1.radar1_lon, params1.radar1_lat, ...
        params1.radar1_tx_lon, params1.radar1_tx_lat, params1.dt_sec);
    ukf2 = ukf_jichu('create', params2, params2.radar2_lon, params2.radar2_lat, ...
        params2.radar2_tx_lon, params2.radar2_tx_lat, params2.dt_sec);
    state = [129; 0.0015; 31.5; 0.0008];
    covariance = diag([1e-4, 1e-8, 1e-4, 1e-8]);
    ukf1.x = state; ukf1.P = covariance; ukf1.initialized = true;
    ukf2.x = state; ukf2.P = covariance; ukf2.initialized = true;
    [~, ~, ~, ukf1] = ukf_jichu('predict', ukf1);
    [~, ~, ~, ukf2] = ukf_jichu('predict', ukf2);
    assert(ukf1.Q(1, 2) > 0 && ukf1.Q(3, 4) > 0);
    assert(all(eig((ukf1.Q + ukf1.Q') / 2) > 0));
    assert(norm(ukf1.Q - ukf2.Q, 'fro') < 1e-12, ...
        'Target process noise must not depend on radar measurement quality.');
end

% =========================================================================
% test_ukf_sigma_measurement_mean — 验证 UKF 量测均值计算的绕环处理
% =========================================================================
% 测试内容：
%   1. 线性量测（距离、多普勒）的均值应与 Sigma 点加权平均一致
%   2. 方位角的均值计算正确处理 359.9°/0.1° 的绕环问题
%   3. 方位角偏移 360° 后均值不变（平移不变性）
%   4. 残差都在 [-180, 180] 范围内（无绕环错误）
function test_ukf_sigma_measurement_mean()
    % 创建 R1 雷达的 UKF 参数（从全局参数中提取雷达专属参数）
    params = radar_params(simulation_params_oracle(), 1);
    % 创建 UKF 模板：传入雷达位置、发射机位置和采样间隔
    ukf = ukf_jichu('create', params, params.radar1_lon, params.radar1_lat, ...
        params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
    % 构造两个检测点迹用于初始化（帧1和帧3，不同位置）
    det1 = fixture_detection(1, 0, 128.0, 31.0, ukf);
    det2 = fixture_detection(3, 60, 128.1, 31.05, ukf);
    % 两点初始化 UKF 状态
    ukf = ukf_jichu('init', ukf, det1, det2);

    % 执行 UKF 预测步，获取预测量测 z_pred、Sigma 量测 Z_pred 等
    [~, ~, ~, z_pred, Z_pred, P_zz, ukf] = ukf_jichu('prepare', ukf);
    % 验证线性量测（距离=第1行，多普勒=第3行）的均值计算正确
    expected_linear = Z_pred([1, 3], :) * ukf.Wm;
    assert(abs(z_pred(1) - expected_linear(1)) < 1e-6);   % 距离均值误差 < 1mm
    assert(abs(z_pred(3) - expected_linear(2)) < 1e-9);   % 多普勒均值误差 < 1nm/s
    assert(all(isfinite(P_zz(:))));                        % P_zz 所有元素有限
    % 验证 P_zz 对称性（浮点误差应在容忍范围内）
    assert(norm(P_zz - P_zz', 'fro') < 1e-8 * max(1, norm(P_zz, 'fro')));

    % ---- 方位角绕环测试 ----
    % 构造一组跨越 0/360 边界的方位角值
    Wm = ukf.Wm;
    angles = repmat(359.9, size(Wm));   % 大部分点在 359.9°
    angles(2:5) = 0.1;                  % 少数点在 0.1°（跨越 360° 边界）
    angles(6:9) = 359.7;                % 更多点在 359.7°
    % 计算加权均值（应接近 360° 而非 180°）
    mean_a = local_angle_mean(angles, Wm);
    % 验证均值在 360° 附近（绕环后残差 < 0.2°）
    assert(abs(local_wrap(mean_a)) < 0.2);

    % 将所有角度平移 360°，验证均值具有平移不变性
    shifted = angles;
    shifted(2:5) = shifted(2:5) + 360;
    mean_shifted = local_angle_mean(shifted, Wm);
    assert(abs(local_wrap(mean_a - mean_shifted)) < 1e-9);

    % 验证所有角度残差都在 [-180, 180] 范围内
    residuals = arrayfun(@(a) local_wrap(a - mean_a), angles);
    assert(all(abs(residuals) <= 180));
end

% =========================================================================
% test_imm_prediction_weights — 验证 IMM 混合权重计算
% =========================================================================
% 测试内容：
%   1. c_bar = Pi' * mu 正确（Markov 传播后的先验概率）
%   2. 组合预测 x_pred 使用 c_bar 而非 mu（关键区别）
%   3. 量测预测 z_pred 组合正确
%   4. Identity 转移矩阵（Pi=I）时 c_bar = mu 的特例
function test_imm_prediction_weights()
    % 创建 IMM 滤波器模板
    params = radar_params(simulation_params_oracle(), 1);
    imm = ukf_imm('create', params, params.radar1_lon, params.radar1_lat, ...
        params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
    % 初始化
    det1 = fixture_detection(1, 0, 128.0, 31.0, imm.ukf_cv);
    det2 = fixture_detection(3, 60, 128.1, 31.05, imm.ukf_cv);
    imm = ukf_imm('init', imm, det1, det2);

    % 设置非平凡的模型概率和转移矩阵
    imm.mu = [0.75; 0.15; 0.10];
    imm.Pi = [0.60, 0.25, 0.15;
              0.20, 0.75, 0.05;
              0.10, 0.10, 0.80];
    % 计算 Markov 传播后的先验概率
    c_bar = imm.Pi' * imm.mu;
    % 验证 c_bar 与 mu 不同（转移矩阵改变了模型概率分布）
    assert(norm(c_bar - imm.mu) > 0.1);

    % 执行 IMM 预测
    [x_pred, P_pred, ~, z_pred, ~, P_zz, imm] = ukf_imm('prepare', imm);
    cache = imm.cache;
    % 验证组合状态使用 c_bar 加权（而非 mu）
    expected_x = c_bar(1) * cache.x_pred_cv + ...
        c_bar(2) * cache.x_pred_ct + c_bar(3) * cache.x_pred_ct_right;
    old_x = imm.mu(1) * cache.x_pred_cv + ...
        imm.mu(2) * cache.x_pred_ct + imm.mu(3) * cache.x_pred_ct_right;
    expected_z = c_bar(1) * cache.z_pred_cv + ...
        c_bar(2) * cache.z_pred_ct + c_bar(3) * cache.z_pred_ct_right;
    assert(norm(x_pred - expected_x) < 1e-12);   % 状态组合误差 < 1pm
    assert(norm(x_pred - old_x) > 1e-10);        % 确认使用了 c_bar 而非 mu
    assert(norm(z_pred - expected_z) < 1e-8);    % 量测组合误差 < 10nm
    assert(abs(sum(cache.c_bar) - 1) < 1e-12);   % c_bar 归一化和为 1
    assert(all(isfinite(P_pred(:))) && all(isfinite(P_zz(:))));  % 协方差有限
    % 验证协方差矩阵对称性
    assert(norm(P_pred - P_pred', 'fro') < 1e-8 * max(1, norm(P_pred, 'fro')));
    assert(norm(P_zz - P_zz', 'fro') < 1e-8 * max(1, norm(P_zz, 'fro')));

    % Identity 转移矩阵特例：Pi=I 时 c_bar = mu
    imm_identity = ukf_imm('create', params, params.radar1_lon, params.radar1_lat, ...
        params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
    imm_identity = ukf_imm('init', imm_identity, det1, det2);
    imm_identity.mu = [0.75; 0.15; 0.10];
    imm_identity.Pi = eye(3);
    [x_identity, ~, ~, ~, ~, ~, imm_identity] = ukf_imm('prepare', imm_identity);
    expected_identity = imm_identity.mu(1) * imm_identity.cache.x_pred_cv + ...
        imm_identity.mu(2) * imm_identity.cache.x_pred_ct + ...
        imm_identity.mu(3) * imm_identity.cache.x_pred_ct_right;
    assert(norm(x_identity - expected_identity) < 1e-12);
end

function test_imm_bidirectional_turn_models()
    params = radar_params(simulation_params_oracle(), 1);
    imm = ukf_imm('create', params, params.radar1_lon, params.radar1_lat, ...
        params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
    assert(imm.M == 3 && isequal(size(imm.Pi), [3, 3]));
    assert(all(abs(sum(imm.Pi, 2) - 1) < 1e-12));
    assert(imm.ukf_ct.turn_rate_rad_per_sec > 0);
    assert(imm.ukf_ct_right.turn_rate_rad_per_sec < 0);
    assert(abs(imm.ukf_ct.turn_rate_rad_per_sec + ...
        imm.ukf_ct_right.turn_rate_rad_per_sec) < eps);
    assert(abs(imm.ukf_ct.turn_rate_rad_per_sec - ...
        params.imm_turn_rate_rad_per_sec) < eps);

    det1 = fixture_detection(1, 0, 128.0, 31.0, imm.ukf_cv);
    det2 = fixture_detection(3, 60, 128.1, 31.0, imm.ukf_cv);
    imm = ukf_imm('init', imm, det1, det2);
    x0 = [128; 1e-3; 31; 0];
    P0 = diag([1e-4, 1e-8, 1e-4, 1e-8]);
    imm.ukf_cv.x = x0; imm.ukf_cv.P = P0;
    imm.ukf_ct.x = x0; imm.ukf_ct.P = P0;
    imm.ukf_ct_right.x = x0; imm.ukf_ct_right.P = P0;
    imm.mu = [0.02; 0.49; 0.49];
    imm.Pi = eye(3);

    [~, ~, ~, ~, ~, ~, imm] = ukf_imm('prepare', imm);
    left_delta = imm.cache.x_pred_ct - x0;
    right_delta = imm.cache.x_pred_ct_right - x0;
    assert(left_delta(4) > 0 && right_delta(4) < 0);
    assert(abs(left_delta(4) + right_delta(4)) < 1e-12);
    earth_radius = 6371000.0;
    speed_left = hypot(imm.cache.x_pred_ct(2) * earth_radius * pi / 180 * ...
        cosd(imm.cache.x_pred_ct(3)), ...
        imm.cache.x_pred_ct(4) * earth_radius * pi / 180);
    speed_right = hypot(imm.cache.x_pred_ct_right(2) * earth_radius * pi / 180 * ...
        cosd(imm.cache.x_pred_ct_right(3)), ...
        imm.cache.x_pred_ct_right(4) * earth_radius * pi / 180);
    speed_initial = x0(2) * earth_radius * pi / 180 * cosd(x0(3));
    assert(abs(speed_left - speed_initial) < 0.5);
    assert(abs(speed_right - speed_initial) < 0.5);
    assert(abs(speed_left - speed_right) < 0.05);

    left_innov = imm.cache.z_pred_ct - imm.cache.z_pred_comb;
    right_innov = imm.cache.z_pred_ct_right - imm.cache.z_pred_comb;
    [~, ~, imm_left] = ukf_imm('update', imm, left_innov);
    [~, ~, imm_right] = ukf_imm('update', imm, right_innov);
    assert(imm_left.mu(2) > imm_left.mu(3));
    assert(imm_right.mu(3) > imm_right.mu(2));

    [~, ~, imm] = ukf_imm('update', imm, []);
    assert(numel(imm.mu) == 3 && all(isfinite(imm.mu)));
    assert(abs(sum(imm.mu) - 1) < 1e-12);
    assert(size(imm.mu_history, 2) == 3);
end

% =========================================================================
% test_tracking_error_time_grids — 验证时间网格对齐和误差计算
% =========================================================================
% 测试内容：
%   1. 快照时间和检测时间对齐时的误差为 0（理想情况）
%   2. 混合时间网格（对齐+不对齐）的误差计算正确
%   3. 非法时间网格（长度不匹配、含 NaN、有重复值）应抛出异常
function test_tracking_error_time_grids()
    % 构造真值航迹：线性运动，lon 和 lat 随时间线性变化
    truth_times = [0, 13, 30, 43, 60, 73];
    truth_lon = 128 + 0.001 * truth_times;
    truth_lat = 31 + 0.0005 * truth_times;
    truth = {struct('time_sec', truth_times, 'lon', truth_lon, ...
        'lat', truth_lat, 'label', 'A')};

    % 构造快照时间（与真值时间不完全对齐）
    snapshot_times = [13, 43, 73];
    detection_times = [13, 43, 73];
    snapshots = cell(3, 1);
    detList = cell(3, 1);
    for k = 1:3
        lon = 128 + 0.001 * snapshot_times(k);
        lat = 31 + 0.0005 * snapshot_times(k);
        trk = struct('id', 1, 'type', 1, 'truth_idx', 1, ...
            'lon', lon, 'lat', lat);
        snapshots{k} = struct('frameID', k, 'trackList', {{trk}});
        detList{k} = struct('aircraft_id', int32(1), 'is_clutter', false, ...
            'lon', lon, 'lat', lat, 'raw_lon', lon, 'raw_lat', lat);
    end

    % 测试 1：快照时间 = 检测时间，误差应为 0
    stats = evaluate_all_multi('tracking_errors', snapshots, detList, truth, ...
        snapshot_times, detection_times, 'R2');
    assert(stats.overall.ukf.rms < 1e-9);  % UKF 误差 < 1nm
    assert(stats.overall.det.rms < 1e-9);  % 检测误差 < 1nm
    assert(isequal(stats.snapshot_times, snapshot_times));  % 时间网格正确传递
    assert(isequal(stats.detection_times, detection_times));

    % 测试 2：混合时间网格（对齐的快照 + 不对齐的检测）
    aligned_times = [0, 30, 60];
    aligned_snapshots = cell(3, 1);
    for k = 1:3
        lon = 128 + 0.001 * aligned_times(k);
        lat = 31 + 0.0005 * aligned_times(k);
        trk = struct('id', 1, 'type', 1, 'truth_idx', 1, ...
            'lon', lon, 'lat', lat);
        aligned_snapshots{k} = struct('frameID', k, 'trackList', {{trk}});
    end
    mixed_stats = evaluate_all_multi('tracking_errors', aligned_snapshots, ...
        detList, truth, aligned_times, detection_times, 'R2_aligned');
    assert(mixed_stats.overall.ukf.rms < 1e-9);
    assert(mixed_stats.overall.det.rms < 1e-9);

    % 测试 3：非法时间网格应抛出异常
    assert_throws_time_grid(@() evaluate_all_multi('tracking_errors', ...
        snapshots, detList, truth, [13, 43], detection_times, 'bad'));  % 长度不匹配
    assert_throws_time_grid(@() evaluate_all_multi('tracking_errors', ...
        snapshots, detList, truth, [13, NaN, 73], detection_times, 'bad'));  % 含 NaN
    assert_throws_time_grid(@() evaluate_all_multi('tracking_errors', ...
        snapshots, detList, truth, [13, 13, 73], detection_times, 'bad'));  % 有重复值
end

% =========================================================================
% assert_throws_time_grid — 辅助函数：验证特定异常被抛出
% =========================================================================
% 验证 evaluate_all_multi 在非法时间网格时抛出正确的异常标识符
function assert_throws_time_grid(fn)
    failed = false;
    try
        fn();
    catch exception
        % 期望的异常标识符：timeGridLength 或 invalidTimeGrid
        failed = strcmp(exception.identifier, 'evaluate_all_multi:timeGridLength') || ...
            strcmp(exception.identifier, 'evaluate_all_multi:invalidTimeGrid');
    end
    assert(failed);  % 必须捕获到预期异常，否则测试失败
end

% =========================================================================
% fixture_detection — 测试用检测点迹构造器
% =========================================================================
% 构造一个结构化的检测点迹，用于测试中避免依赖完整仿真流程
function det = fixture_detection(frame_id, time_sec, lon, lat, ukf)
    % 构造状态向量 [lon, lon_rate, lat, lat_rate]
    x = [lon; 0.001; lat; 0.0005];
    % 通过 UKF 量测模型计算预测量测 [range, azimuth, radial_vel]
    z = ukf_jichu('measurement', ukf, x);
    % 组装检测结构体
    det = struct('frameID', frame_id, 'time_sec', time_sec, ...
        'range_meas', z(1), 'azimuth_meas', z(2), ...
        'drange', z(1), 'daz', z(2), 'radial_vel_meas', z(3), ...
        'pvr', z(3), 'lat', lat, 'lon', lon, ...
        'aircraft_id', int32(1), 'is_clutter', false);
end

% =========================================================================
% local_angle_mean — 局部方位角加权均值计算
% =========================================================================
% 处理方位角绕环问题的均值计算
function mean_angle = local_angle_mean(angles, weights)
    ref = angles(1);  % 以第一个角度为参考
    % 计算所有角度相对于参考的有符号差值（-180° 到 180°）
    delta = arrayfun(@(a) local_wrap(a - ref), angles);
    % 加权平均后取模 360°
    mean_angle = mod(ref + delta' * weights, 360);
end

% =========================================================================
% local_wrap — 角度环绕归一化
% =========================================================================
% 将角度差值归一化到 [-180, 180] 范围
function angle = local_wrap(angle)
    angle = mod(angle + 180, 360) - 180;
end
