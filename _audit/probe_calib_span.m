function probe_calib_span()
% 诊断：两站各自能拿到多少标校样本、距离跨度多大
% 可分离性判据：∂Rg/∂h ≈ 2h/sqrt(D^2+4h^2) 随 D 增大而衰减，
% 只有距离跨度足够大，h 与 dR 才能被分开（否则 corr → -1）

root = fullfile(pwd,'..');
addpath(root);
for d = {'config','simulation','ukf','tracker','fusion','evaluation','utils','visualization'}
    addpath(fullfile(root, d{1}));
end

p = simulation_params_oracle();
h_true = iono_h(p,'true');
h_ass  = iono_h(p,'assumed');

csv = fullfile(root, p.adsb_csv_path);
if ~isfile(csv), csv = p.adsb_csv_path; end
T = readtable(csv, 'ReadVariableNames', false);
lon = T.Var3 + p.adsb_lon_offset_deg;
lat = T.Var2;
id  = string(T.Var1);
mask = isfinite(lon) & isfinite(lat) & ...
       lon >= p.adsb_region_lon(1) & lon <= p.adsb_region_lon(2) & ...
       lat >= p.adsb_region_lat(1) & lat <= p.adsb_region_lat(2) & strlength(id) > 0;
sel = unique(id(mask));

n_check  = min(5000, height(T));
cal_step = max(1, floor(height(T)/n_check));

D1 = []; S1 = []; D2 = []; S2 = [];
for idx = 1:cal_step:height(T)
    if ~ismember(id(idx), sel), continue; end
    t_lon = lon(idx); t_lat = lat(idx);
    if isnan(t_lon) || isnan(t_lat), continue; end
    [in1,~,~] = radar_coverage_check(p.radar1_lon, p.radar1_lat, t_lon, t_lat, p.radar1_beam_center_deg, p);
    if in1
        d1 = sphere_utils_haversine_distance(p.radar1_lon, p.radar1_lat, t_lon, t_lat);
        d0 = sphere_utils_haversine_distance(p.radar1_tx_lon, p.radar1_tx_lat, t_lon, t_lat);
        Rg_t = skywave_geometry('group_range', p.radar1_tx_lon, p.radar1_tx_lat, ...
            p.radar1_lon, p.radar1_lat, t_lon, t_lat, h_true);
        Rg_a = skywave_geometry('group_range', p.radar1_tx_lon, p.radar1_tx_lat, ...
            p.radar1_lon, p.radar1_lat, t_lon, t_lat, h_ass);
        D1(end+1) = d1/1000;  S1(end+1) = (Rg_t - Rg_a)/1000;  % km
        clear d0
    end
    [in2,~,~] = radar_coverage_check(p.radar2_lon, p.radar2_lat, t_lon, t_lat, p.radar2_beam_center_deg, p);
    if in2
        d2 = sphere_utils_haversine_distance(p.radar2_lon, p.radar2_lat, t_lon, t_lat);
        Rg_t = skywave_geometry('group_range', p.radar2_tx_lon, p.radar2_tx_lat, ...
            p.radar2_lon, p.radar2_lat, t_lon, t_lat, h_true);
        Rg_a = skywave_geometry('group_range', p.radar2_tx_lon, p.radar2_tx_lat, ...
            p.radar2_lon, p.radar2_lat, t_lon, t_lat, h_ass);
        D2(end+1) = d2/1000;  S2(end+1) = (Rg_t - Rg_a)/1000;
    end
end

fid = fopen('calib_span.txt','w','n','UTF-8');
pr = @(varargin) fprintf(fid, varargin{:});
pr('################ 标校样本与距离跨度诊断 ################\n');
pr('真值虚高 %.0f km，标称 %.0f km（失配 %.0f km）\n\n', h_true/1000, h_ass/1000, (h_true-h_ass)/1000);

report(pr, 'R1', D1, S1, p.radar1_range_noise_std_m);
report(pr, 'R2', D2, S2, p.radar2_range_noise_std_m);

fclose(fid);
fprintf('WROTE calib_span.txt\n');
end

function report(pr, tag, D, S, sigR_m)
    if isempty(D)
        pr('  %s: 无样本\n', tag); return;
    end
    D = D(:); S = S(:);
    n = numel(D);
    Ds = sort(D); Ss = sort(S);
    pr('  %s: n=%d\n', tag, n);
    pr('    距离: min=%.0f p5=%.0f 中位=%.0f p95=%.0f max=%.0f km，跨度(p5~p95)=%.0f km\n', ...
       Ds(1), Ds(max(1,round(n*0.05))), Ds(round(n/2)), Ds(round(n*0.95)), Ds(end), ...
       Ds(round(n*0.95))-Ds(max(1,round(n*0.05))));
    pr('    h 失配引起的群距离偏差: min=%.1f 中位=%.1f max=%.1f km（变化范围 %.1f km）\n', ...
       Ss(1), Ss(round(n/2)), Ss(end), Ss(end)-Ss(1));
    pr('    量测噪声 sigma_R = %.1f km\n', sigR_m/1000);
    % 可分离性指标：h 偏差的变化范围 / 噪声 —— 越大越可辨识
    snr_like = (Ss(end)-Ss(1)) / (sigR_m/1000);
    pr('    可分离性指标 (h偏差变化范围/噪声) = %.2f  → %s\n', snr_like, ...
       ternary(snr_like > 1, '可分离', '接近不可分离'));
    % 相关系数估计：h 与 dR 的近似相关（用灵敏度向量）
    pr('\n');
end

function s = ternary(c, a, b)
    if c, s = a; else, s = b; end
end
