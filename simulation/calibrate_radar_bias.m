function [dr1_est, da1_est, dr2_est, da2_est, reg] = calibrate_radar_bias(params)
%CALIBRATE_RADAR_BIAS 时空配准：由 ADS-B 标校集估计两站的距离/方位系统偏差
%
%   [dr1, da1, dr2, da2, reg] = calibrate_radar_bias(params)
%
%   三种模式（params.registration_mode）：
%     'none' — 不做任何校正（下界基线）
%     'mean' — 两参数均值标定：ΔR/Δθ 取均值，电离层虚高固定为标称值
%     'mle3' — 三参数联合最大似然 [h, ΔR, Δθ]，接入 dsv41 专利方法，
%              并把估计出的虚高反馈给反演与滤波模型
%
%   关键设计：标定侧的正演模型使用"标称虚高"，而量测生成使用"真实虚高"。
%   二者的失配使群距离偏差随目标距离变化（近距约 20 km、远距约 11 km），
%   均值标定只能扣掉常数部分，无法扣掉距离相关部分；三参数 MLE 通过
%   同时估计 h 来消除这一项。
%
%   本函数从 run.m 的局部函数抽出，成为独立可复用模块：
%   主链路（run.m）与在线链路（run_online.m）共用同一份配准实现，
%   避免两条链路出现口径漂移。

    mode = 'mean';
    if isfield(params, 'registration_mode') && ~isempty(params.registration_mode)
        mode = lower(char(params.registration_mode));
    end
    reg = struct('mode', mode, 'h1_km', NaN, 'h2_km', NaN, ...
                 'eta1', [], 'eta2', [], 'crlb1', [], 'crlb2', [], ...
                 'n1', 0, 'n2', 0);
    if strcmp(mode, 'none')
        dr1_est = 0; da1_est = 0; dr2_est = 0; da2_est = 0;
        fprintf('  配准模式: none（不做偏差校正）%s', newline);
        return;
    end

    h_true_m    = iono_h(params, 'true');      % 真实虚高（量测生成用）
    h_assumed_m = iono_h(params, 'assumed');   % 标称虚高（标定侧正演用）

    rng(params.random_seed);                  % 重置随机种子以保证可复现
    % 解析 CSV 路径：允许相对路径（相对仓库根目录），与调用方的当前目录无关
    csv_path = params.adsb_csv_path;
    if ~isfile(csv_path)
        repo_root = fileparts(fileparts(mfilename('fullpath')));
        cand = fullfile(repo_root, csv_path);
        if isfile(cand), csv_path = cand; end
    end
    if ~isfile(csv_path)
        error('calibrate_radar_bias:calibCsvMissing', ...
            '标校数据文件不存在: %s', params.adsb_csv_path);
    end
    T_adsb = readtable(csv_path, 'ReadVariableNames', false);  % 读取 ADS-B CSV
    adsb_lat = T_adsb.Var2;                               % 第二列：纬度
    adsb_lon = T_adsb.Var3;                               % 第三列：经度
    if isfield(params, 'adsb_lon_offset_deg')
        adsb_lon = adsb_lon + params.adsb_lon_offset_deg;
    end
    % 仅使用探测区域内的 ADS-B 航迹（向量化预筛选，避免遍历全表）
    adsb_track_id = string(T_adsb.Var1);
    lon_lim = [117.0, 124.0]; lat_lim = [24.0, 34.0];
    if isfield(params, 'adsb_region_lon'), lon_lim = params.adsb_region_lon; end
    if isfield(params, 'adsb_region_lat'), lat_lim = params.adsb_region_lat; end
    region_mask = isfinite(adsb_lon) & isfinite(adsb_lat) & adsb_lon >= lon_lim(1) & adsb_lon <= lon_lim(2) & ...
        adsb_lat >= lat_lim(1) & adsb_lat <= lat_lim(2) & strlength(adsb_track_id) > 0;
    selected_ids = unique(adsb_track_id(region_mask));
    if isempty(selected_ids)
        error('calibrate_radar_bias:adsbNoInCoverageTrack', ...
            'ADS-B 数据中没有找到同时满足两部雷达覆盖和最小点数要求的航迹');
    end

    dr1_list = []; da1_list = [];                         % R1 的距离/方位偏差采样列表
    dr2_list = []; da2_list = [];                         % R2 的距离/方位偏差采样列表
    % mle3 模式额外收集：位置与原始量测（供三参数联合估计）
    lon1 = []; lat1 = []; Rg1 = []; az1 = [];
    lon2 = []; lat2 = []; Rg2 = []; az2 = [];

    % 使用全部标校行：样本量决定 (h, dR) 的可分离性，抽稀会破坏可辨识性
    need_model = strcmp(mode, 'mean');   % 只有均值标定需要正演模型残差
    for idx = 1:height(T_adsb)
        t_lon = adsb_lon(idx);                            % 当前采样点的经度
        t_lat = adsb_lat(idx);                            % 当前采样点的纬度
        if ~ismember(adsb_track_id(idx), selected_ids)
            continue;
        end
        if isnan(t_lon) || isnan(t_lat)                   % 跳过无效数据
            continue;
        end
        % --- R1 偏差采样 ---
        [in1, ~, ~] = radar_coverage_check(params.radar1_lon, params.radar1_lat, ...
            t_lon, t_lat, params.radar1_beam_center_deg, params);
        if in1
            Rg_phys = skywave_geometry('group_range', params.radar1_tx_lon, params.radar1_tx_lat, ...
                params.radar1_lon, params.radar1_lat, t_lon, t_lat, h_true_m);
            az_true = sphere_utils_azimuth(params.radar1_lon, params.radar1_lat, t_lon, t_lat);
            Rg_meas = Rg_phys + params.radar1_range_bias_m + randn() * params.radar1_range_noise_std_m;
            az_meas = az_true + params.radar1_azimuth_bias_deg + randn() * params.radar1_azimuth_noise_std_deg;
            if need_model
                Rg_model = skywave_geometry('group_range', params.radar1_tx_lon, params.radar1_tx_lat, ...
                    params.radar1_lon, params.radar1_lat, t_lon, t_lat, h_assumed_m);
                dr1_list(end+1) = Rg_meas - Rg_model;
                da1_list(end+1) = wrap_angle_deg(az_meas - az_true);
            end
            lon1(end+1) = t_lon; lat1(end+1) = t_lat;
            Rg1(end+1)  = Rg_meas; az1(end+1) = az_meas;
        end
        % --- R2 偏差采样 ---
        [in2, ~, ~] = radar_coverage_check(params.radar2_lon, params.radar2_lat, ...
            t_lon, t_lat, params.radar2_beam_center_deg, params);
        if in2
            Rg_phys = skywave_geometry('group_range', params.radar2_tx_lon, params.radar2_tx_lat, ...
                params.radar2_lon, params.radar2_lat, t_lon, t_lat, h_true_m);
            az_true = sphere_utils_azimuth(params.radar2_lon, params.radar2_lat, t_lon, t_lat);
            Rg_meas = Rg_phys + params.radar2_range_bias_m + randn() * params.radar2_range_noise_std_m;
            az_meas = az_true + params.radar2_azimuth_bias_deg + randn() * params.radar2_azimuth_noise_std_deg;
            if need_model
                Rg_model = skywave_geometry('group_range', params.radar2_tx_lon, params.radar2_tx_lat, ...
                    params.radar2_lon, params.radar2_lat, t_lon, t_lat, h_assumed_m);
                dr2_list(end+1) = Rg_meas - Rg_model;
                da2_list(end+1) = wrap_angle_deg(az_meas - az_true);
            end
            lon2(end+1) = t_lon; lat2(end+1) = t_lat;
            Rg2(end+1)  = Rg_meas; az2(end+1) = az_meas;
        end
    end
    if need_model
        if isempty(dr1_list) || isempty(da1_list)
            error('calibrate_radar_bias:calibrationNoSamples', 'R1 ADS-B 标定没有雷达覆盖内的有效样本');
        end
        if isempty(dr2_list) || isempty(da2_list)
            error('calibrate_radar_bias:calibrationNoSamples', 'R2 ADS-B 标定没有雷达覆盖内的有效样本');
        end
    else
        if isempty(Rg1)
            error('calibrate_radar_bias:calibrationNoSamples', 'R1 ADS-B 标定没有雷达覆盖内的有效样本');
        end
        if isempty(Rg2)
            error('calibrate_radar_bias:calibrationNoSamples', 'R2 ADS-B 标定没有雷达覆盖内的有效样本');
        end
    end

    if strcmp(mode, 'mle3')
        % ---- 三参数联合最大似然：[h, ΔR, Δθ] ----
        % 分别对两站估计（站点几何不同，误差源独立）
        p1 = params;
        p1.radar_index = 1;
        p1.tx_lon = params.radar1_tx_lon; p1.tx_lat = params.radar1_tx_lat;
        p1.rx_lon = params.radar1_lon;    p1.rx_lat = params.radar1_lat;
        p1.sigma_R = params.radar1_range_noise_std_m;
        p1.sigma_A = params.radar1_azimuth_noise_std_deg;
        p1.eta0 = [h_assumed_m/1000, 0, 0];
        % 边界必须容纳真实偏差（否则 dtheta 被截断会把 h 与 dR 一起拉偏）
        p1.lb = [200, -100, -10];  p1.ub = [500, 100, 10];
        p1.verbose = false;
        calib1 = struct('lon', lon1, 'lat', lat1, 'Rg_meas', Rg1, 'az_meas', az1, ...
                        'sigma_R', p1.sigma_R, 'sigma_A', p1.sigma_A);
        est1 = estimate_system_bias(calib1, p1);

        p2 = params;
        p2.radar_index = 2;
        p2.tx_lon = params.radar2_tx_lon; p2.tx_lat = params.radar2_tx_lat;
        p2.rx_lon = params.radar2_lon;    p2.rx_lat = params.radar2_lat;
        p2.sigma_R = params.radar2_range_noise_std_m;
        p2.sigma_A = params.radar2_azimuth_noise_std_deg;
        p2.eta0 = [h_assumed_m/1000, 0, 0];
        p2.lb = [200, -100, -10];  p2.ub = [500, 100, 10];
        p2.verbose = false;
        calib2 = struct('lon', lon2, 'lat', lat2, 'Rg_meas', Rg2, 'az_meas', az2, ...
                        'sigma_R', p2.sigma_R, 'sigma_A', p2.sigma_A);
        est2 = estimate_system_bias(calib2, p2);

        dr1_est = est1.eta_hat(2) * 1000;   % km → m
        da1_est = est1.eta_hat(3);          % deg
        dr2_est = est2.eta_hat(2) * 1000;
        da2_est = est2.eta_hat(3);

        reg.h1_km = est1.eta_hat(1);  reg.h2_km = est2.eta_hat(1);
        reg.eta1  = est1.eta_hat;     reg.eta2 = est2.eta_hat;
        reg.crlb1 = est1.crlb;        reg.crlb2 = est2.crlb;
        reg.n1    = est1.n_obs;       reg.n2    = est2.n_obs;
        reg.converged1 = est1.converged;  reg.converged2 = est2.converged;
        reg.rho_h_dR_1 = est1.diag.rho_h_dR;
        reg.rho_h_dR_2 = est2.diag.rho_h_dR;

        fprintf('  配准[mle3] R1: h=%.1fkm dR=%.2fkm dth=%.3fdeg (真值 h=%.0fkm dR=%.1fkm dth=%.2fdeg)%s', ...
            reg.h1_km, est1.eta_hat(2), est1.eta_hat(3), ...
            h_true_m/1000, params.radar1_range_bias_m/1000, params.radar1_azimuth_bias_deg, newline);
        fprintf('  配准[mle3] R2: h=%.1fkm dR=%.2fkm dth=%.3fdeg (真值 h=%.0fkm dR=%.1fkm dth=%.2fdeg)%s', ...
            reg.h2_km, est2.eta_hat(2), est2.eta_hat(3), ...
            h_true_m/1000, params.radar2_range_bias_m/1000, params.radar2_azimuth_bias_deg, newline);
    else
        % ---- 两参数均值标定 ----
        dr1_est = mean(dr1_list);
        da1_est = mean(da1_list);
        dr2_est = mean(dr2_list);
        da2_est = mean(da2_list);
        reg.n1 = numel(dr1_list);  reg.n2 = numel(dr2_list);
    end

    if any(~isfinite([dr1_est, da1_est, dr2_est, da2_est]))
        error('calibrate_radar_bias:calibrationInvalid', 'ADS-B 标定结果包含非有限值');
    end
end

function a = wrap_angle_deg(a)
% 角度归一化到 [-180, 180] 区间
    a = mod(a + 180, 360) - 180;
end
