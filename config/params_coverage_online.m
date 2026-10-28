function params = params_coverage_online()
%PARAMS_COVERAGE_ONLINE 覆盖互补布设 + 在线配对融合的参数集
%
%   在 simulation_params_oracle() 的基础上做覆盖式修改，不改动原配置文件。
%   对应文档：双站航迹配对与融合在线处理模型_20261028.md 第 7 节。
%
%   布设：北站 R1 与南站 R2 同经度、南北错开 100 km，同时向东观测，
%         各 15 度视场，目标区保留约 7 度重叠。
%         一条南北向航迹依次穿过：仅南站覆盖 -> 两站重叠 -> 仅北站覆盖。
%
%   与默认配置的差异：
%     1. 站点与波束改为覆盖互补布设（原为近乎共视）
%     2. 波束宽度 15 度（原 45 度）
%     3. 电离层虚高的标称值等于真值，系统偏差置零
%        —— 本实验只考察时序与航迹长度，不考察配准精度，先把配准误差归零
%     4. 关闭真值终止，航迹只由 K_loss 连续漏检因果终止
%     5. 新增在线配对参数：门限、确认帧数、融合算法

    params = simulation_params_oracle();

    % ==================== 站点几何 ====================
    % 北站 R1、南站 R2 同经度，纬度相差 0.90 度，约 100 km
    params.radar1_lon = 92.50;  params.radar1_lat = 31.45;
    params.radar2_lon = 92.50;  params.radar2_lat = 30.55;
    % 发射站仍在接收站正西 4 度，保持原有的双基地几何
    params.radar1_tx_lon = 88.50;  params.radar1_tx_lat = 31.45;
    params.radar2_tx_lon = 88.50;  params.radar2_tx_lat = 30.55;

    % ==================== 波束 ====================
    % 单站视场 15 度（半角 7.5 度）
    params.beam_width_deg = 15.0;
    % 走廊参考点：川渝华中走廊，目标距离约 1400 km
    params.coverage_ref_lon = 107.40;
    params.coverage_ref_lat = 31.00;
    % 两束各自向背离方向偏转的角度。偏转角越大，两站覆盖带错开越多、
    % 共视段越短、并集越长。4.6 度由 check_coverage_layout 扫出来的：
    % 37 条真实航迹下，单站约 59 帧、共视约 21 帧、并集约 75 帧，
    % 共视段扣掉起始 3 帧和确认 8 帧后仍有 10 帧余量。
    params.coverage_tilt_deg = 4.6;
    [c1, c2] = compute_beam_centers(params);
    params.radar1_beam_center_deg = c1;
    params.radar2_beam_center_deg = c2;

    % ==================== 配准误差归零 ====================
    % 标称虚高与真值一致，反演不再有距离相关偏差
    params.ionosphere_assumed_h_km = params.ionosphere_true_h_km;
    params.radar1_assumed_h_km = params.ionosphere_true_h_km;
    params.radar2_assumed_h_km = params.ionosphere_true_h_km;
    % 系统偏差置零（在线入口不调用 ADS-B 标定）
    params.radar1_range_bias_m = 0.0;  params.radar1_azimuth_bias_deg = 0.0;
    params.radar2_range_bias_m = 0.0;  params.radar2_azimuth_bias_deg = 0.0;

    % ==================== 航迹生命周期 ====================
    % 关闭真值终止，只保留 K_loss 因果终止
    params.oracle_truth_terminate_enable = false;
    params.tracker_K_loss = 8;
    params.radar1_tracker_K_loss = 8;
    params.radar2_tracker_K_loss = 8;

    % ==================== 在线配对与融合 ====================
    % 候选门限：马氏距离平方，chi2(2, 0.95) = 5.99
    params.online_gate_chi2 = 5.99;
    % 确认帧数：最近 N 帧内累计 M 帧落在门限内，才把第二个源并入系统航迹
    % 用 M/N 而不是"连续 M 帧"，避免个别帧越门限就把计数清零
    params.online_M = 8;
    params.online_N = 10;
    % 系统航迹连续无源多少帧后删除
    params.online_K_loss = 8;
    % 融合算法：SCC / CI / BC
    params.online_fusion_method = 'SCC';

    % ==================== 复现 ====================
    params.random_seed = 42;

    fprintf('[布设] R1(北) %.2fE %.2fN 波束中心 %.2f 度%s', ...
        params.radar1_lon, params.radar1_lat, c1, newline);
    fprintf('[布设] R2(南) %.2fE %.2fN 波束中心 %.2f 度%s', ...
        params.radar2_lon, params.radar2_lat, c2, newline);
    fprintf('[布设] 视场 %.1f 度，偏转 %.2f 度，门限 chi2=%.2f，确认 %d 帧%s', ...
        params.beam_width_deg, params.coverage_tilt_deg, ...
        params.online_gate_chi2, params.online_M, newline);
end

function [c1, c2] = compute_beam_centers(params)
%COMPUTE_BEAM_CENTERS 由走廊参考点算出两站的波束中心方位
%
%   两站各自指向同一走廊点时，方位本身就差一个视差角（约 4 度）。
%   在此基础上让北站再向北偏、南站再向南偏，才能把两站覆盖带错开 200 km。

    az1 = skywave_geometry('azimuth', params.radar1_lon, params.radar1_lat, ...
        params.coverage_ref_lon, params.coverage_ref_lat);
    az2 = skywave_geometry('azimuth', params.radar2_lon, params.radar2_lat, ...
        params.coverage_ref_lon, params.coverage_ref_lat);

    % 方位角以正北顺时针为正。北站要覆盖更北的区域，方位角减小；
    % 南站要覆盖更南的区域，方位角增大。
    c1 = az1 - params.coverage_tilt_deg;
    c2 = az2 + params.coverage_tilt_deg;
end
