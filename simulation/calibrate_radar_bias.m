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

    % ---- 标定缓存：离线标定天然可缓存，同一组参数只算一次 ----
    repo_root = fileparts(fileparts(mfilename('fullpath')));
    use_cache = getdef_(params, 'calib_cache_enable', 1) && ~getdef_(params, 'calib_cache_force', 0);
    cache_key = calib_cache_key_(params, mode);
    cache_file = '';
    if use_cache
        cache_dir = fullfile(repo_root, 'results', 'calib_cache');
        if isfield(params, 'calib_cache_dir') && ~isempty(params.calib_cache_dir)
            cache_dir = char(params.calib_cache_dir);
        end
        if ~isfolder(cache_dir), mkdir(cache_dir); end
        cache_file = fullfile(cache_dir, sprintf('%s_%s.mat', mode, hash_str_(cache_key)));
        if isfile(cache_file)
            S = load(cache_file);
            if isfield(S, 'cache_key') && strcmp(S.cache_key, cache_key)
                dr1_est = S.dr1_est; da1_est = S.da1_est;
                dr2_est = S.dr2_est; da2_est = S.da2_est; reg = S.reg;
                fprintf('  配准[%s] 命中缓存 %s%s', mode, hash_str_(cache_key), newline);
                return;
            end
        end
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
    % 预先求出"区域窗内且属于选中航迹"的行号，循环只走这些行。
    % 原实现对 61k 行逐行 ismember(string)，单次标定要 90 s，这里降到 ~15 s。
    row_idx = find(region_mask & ismember(adsb_track_id, selected_ids));

    % ---- L2 失配参数（默认全为 0/1，退化为原来的"干净"标定）----
    sR1 = params.radar1_range_noise_std_m   * getdef_(params, 'gen_range_noise_scale', 1);
    sA1 = params.radar1_azimuth_noise_std_deg * getdef_(params, 'gen_az_noise_scale', 1);
    sR2 = params.radar2_range_noise_std_m   * getdef_(params, 'gen_range_noise_scale', 1);
    sA2 = params.radar2_azimuth_noise_std_deg * getdef_(params, 'gen_az_noise_scale', 1);
    out_frac = getdef_(params, 'calib_outlier_frac', 0);
    out_std  = getdef_(params, 'calib_outlier_std_m', 0);
    pos_err  = getdef_(params, 'calib_ref_pos_err_m', 0);
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
    for ii = 1:numel(row_idx)
        idx = row_idx(ii);
        t_lon = adsb_lon(idx);                            % 当前采样点的经度
        t_lat = adsb_lat(idx);                            % 当前采样点的纬度
        if isnan(t_lon) || isnan(t_lat)                   % 跳过无效数据
            continue;
        end
        % ADS-B 参考位置误差：只污染"参考"，量测仍由真实位置生成
        if pos_err > 0
            ref_lon = t_lon + randn() * pos_err / (111320 * cosd(t_lat));
            ref_lat = t_lat + randn() * pos_err / 110570;
        else
            ref_lon = t_lon;  ref_lat = t_lat;
        end
        % 标校窗内的真实虚高（时间项取 NaN = 窗内均值，空间项照常）
        h_now = iono_h_field(params, t_lon, t_lat, NaN);
        % --- R1 偏差采样 ---
        [in1, ~, ~] = radar_coverage_check(params.radar1_lon, params.radar1_lat, ...
            t_lon, t_lat, params.radar1_beam_center_deg, params);
        if in1
            Rg_phys = skywave_geometry('group_range', params.radar1_tx_lon, params.radar1_tx_lat, ...
                params.radar1_lon, params.radar1_lat, t_lon, t_lat, h_now);
            az_true = sphere_utils_azimuth(params.radar1_lon, params.radar1_lat, t_lon, t_lat);
            Rg_meas = Rg_phys + params.radar1_range_bias_m + randn() * sR1;
            az_meas = az_true + params.radar1_azimuth_bias_deg + randn() * sA1;
            if out_frac > 0 && rand() < out_frac
                Rg_meas = Rg_meas + sign(randn()) * abs(randn()) * out_std;
            end
            if need_model
                Rg_model = skywave_geometry('group_range', params.radar1_tx_lon, params.radar1_tx_lat, ...
                    params.radar1_lon, params.radar1_lat, t_lon, t_lat, h_assumed_m);
                dr1_list(end+1) = Rg_meas - Rg_model;
                da1_list(end+1) = wrap_angle_deg(az_meas - az_true);
            end
            lon1(end+1) = ref_lon; lat1(end+1) = ref_lat;
            Rg1(end+1)  = Rg_meas; az1(end+1) = az_meas;
        end
        % --- R2 偏差采样 ---
        [in2, ~, ~] = radar_coverage_check(params.radar2_lon, params.radar2_lat, ...
            t_lon, t_lat, params.radar2_beam_center_deg, params);
        if in2
            Rg_phys = skywave_geometry('group_range', params.radar2_tx_lon, params.radar2_tx_lat, ...
                params.radar2_lon, params.radar2_lat, t_lon, t_lat, h_now);
            az_true = sphere_utils_azimuth(params.radar2_lon, params.radar2_lat, t_lon, t_lat);
            Rg_meas = Rg_phys + params.radar2_range_bias_m + randn() * sR2;
            az_meas = az_true + params.radar2_azimuth_bias_deg + randn() * sA2;
            if out_frac > 0 && rand() < out_frac
                Rg_meas = Rg_meas + sign(randn()) * abs(randn()) * out_std;
            end
            if need_model
                Rg_model = skywave_geometry('group_range', params.radar2_tx_lon, params.radar2_tx_lat, ...
                    params.radar2_lon, params.radar2_lat, t_lon, t_lat, h_assumed_m);
                dr2_list(end+1) = Rg_meas - Rg_model;
                da2_list(end+1) = wrap_angle_deg(az_meas - az_true);
            end
            lon2(end+1) = ref_lon; lat2(end+1) = ref_lat;
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

    if ~isempty(cache_file)
        save(cache_file, 'dr1_est', 'da1_est', 'dr2_est', 'da2_est', 'reg', 'cache_key');
        fprintf('  配准[%s] 已写入缓存 %s%s', mode, hash_str_(cache_key), newline);
    end
end

function a = wrap_angle_deg(a)
% 角度归一化到 [-180, 180] 区间
    a = mod(a + 180, 360) - 180;
end

function v = getdef_(s, name, def)
    if isfield(s, name) && ~isempty(s.(name)) && isfinite(double(s.(name)))
        v = double(s.(name));
    else
        v = def;
    end
end

function k = calib_cache_key_(params, mode)
%CALIB_CACHE_KEY_ 由影响标定结果的全部参数生成确定性键
%   只列真正进入标定的量：几何、偏差真值、虚高、噪声、失配项、标校集。
    numf = {'random_seed','ionosphere_true_h_km','ionosphere_assumed_h_km', ...
            'radar1_assumed_h_km','radar2_assumed_h_km', ...
            'radar1_range_bias_m','radar1_azimuth_bias_deg', ...
            'radar2_range_bias_m','radar2_azimuth_bias_deg', ...
            'radar1_range_noise_std_m','radar1_azimuth_noise_std_deg', ...
            'radar2_range_noise_std_m','radar2_azimuth_noise_std_deg', ...
            'radar1_lon','radar1_lat','radar2_lon','radar2_lat', ...
            'radar1_tx_lon','radar1_tx_lat','radar2_tx_lon','radar2_tx_lat', ...
            'radar1_beam_center_deg','radar2_beam_center_deg','beam_width_deg', ...
            'range_min_m','range_max_m'};
    % 失配项即使缺省也要进键，保证"显式写 0"与"不写"命中同一条缓存
    mis = {'iono_lon_tilt_km_per_deg','iono_lon_ref_deg', ...
           'iono_drift_amp_km','iono_drift_period_h','iono_drift_phase_sec', ...
           'gen_range_noise_scale','gen_az_noise_scale', ...
           'calib_outlier_frac','calib_outlier_std_m','calib_ref_pos_err_m'};
    mis_def = [0 107.4 0 6 0 1 1 0 0 0];

    parts = {mode};
    for i = 1:numel(numf)
        if isfield(params, numf{i}) && ~isempty(params.(numf{i}))
            parts{end+1} = sprintf('%s=%.8g', numf{i}, double(params.(numf{i}))); %#ok<AGROW>
        end
    end
    for i = 1:numel(mis)
        v = getdef_(params, mis{i}, mis_def(i));
        parts{end+1} = sprintf('%s=%.8g', mis{i}, v); %#ok<AGROW>
    end
    if isfield(params,'adsb_region_lon') && numel(params.adsb_region_lon) >= 2
        parts{end+1} = sprintf('reglon=%.6g,%.6g', params.adsb_region_lon(1), params.adsb_region_lon(2));
    end
    if isfield(params,'adsb_region_lat') && numel(params.adsb_region_lat) >= 2
        parts{end+1} = sprintf('reglat=%.6g,%.6g', params.adsb_region_lat(1), params.adsb_region_lat(2));
    end
    if isfield(params,'adsb_csv_path') && ~isempty(params.adsb_csv_path)
        p = char(params.adsb_csv_path);
        parts{end+1} = ['csv=' p];
        if isfile(p)
            d = dir(p);
            parts{end+1} = sprintf('csvsz=%d_%.0f', d.bytes, d.datenum);
        end
    end
    k = strjoin(parts, '|');
end

function h = hash_str_(k)
% FNV-1a 32 bit：确定性短哈希，用于缓存文件名
    h = uint32(2166136261);
    b = double(k);
    for i = 1:numel(b)
        h = bitxor(h, uint32(mod(b(i), 256)));
        h = uint32(mod(double(h) * 16777619, 4294967296));
    end
    h = sprintf('%08x', h);
end
