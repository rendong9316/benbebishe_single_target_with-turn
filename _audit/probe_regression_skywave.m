% =========================================================================
% probe_regression_skywave.m — skywave_geometry 修改前后的数值等价性回归测试
% =========================================================================
% 用 attic/bak_20261025/skywave_geometry.m.bak（改名 skywave_geometry_legacy）
% 与当前版本逐个 action 对比，验证"不传 h 时行为完全不变"。
% =========================================================================

clear; clc;

thisFile = mfilename('fullpath');
rootDir  = fileparts(fileparts(thisFile));
addpath(fullfile(rootDir, 'utils'));
addpath(fileparts(thisFile));        % 让 skywave_geometry_legacy 可见

rng(7, 'twister');
N = 500;

tx_lon = 100.0; tx_lat = 30.7;
rx_lon = 104.0; rx_lat = 30.7;

tgt_lon = 95 + 50 * rand(N, 1);      % 95~145E
tgt_lat = 5  + 45 * rand(N, 1);      % 5~50N

fprintf('=== skywave_geometry regression (N = %d) ===\n', N);

% ---- constants ----
[Re1, H1] = skywave_geometry('constants');
[Re2, H2] = skywave_geometry_legacy('constants');
fprintf('constants        : max|d| = %.3e / %.3e\n', abs(Re1-Re2), abs(H1-H2));

% ---- scalar calls (逐个点，模拟旧调用方式) ----
d_sig = 0; d_D = 0; d_Rg = 0; d_az = 0; d_vd = 0; d_full = 0;
for k = 1:N
    s1 = skywave_geometry('geocentric_angle', tx_lon, tx_lat, tgt_lon(k), tgt_lat(k));
    s2 = skywave_geometry_legacy('geocentric_angle', tx_lon, tx_lat, tgt_lon(k), tgt_lat(k));
    d_sig = max(d_sig, abs(s1 - s2));

    D1 = skywave_geometry('chord_length', tx_lon, tx_lat, tgt_lon(k), tgt_lat(k));
    D2 = skywave_geometry_legacy('chord_length', tx_lon, tx_lat, tgt_lon(k), tgt_lat(k));
    d_D = max(d_D, abs(D1 - D2));

    R1 = skywave_geometry('group_range', tx_lon, tx_lat, rx_lon, rx_lat, tgt_lon(k), tgt_lat(k));
    R2 = skywave_geometry_legacy('group_range', tx_lon, tx_lat, rx_lon, rx_lat, tgt_lon(k), tgt_lat(k));
    d_Rg = max(d_Rg, abs(R1 - R2));

    A1 = skywave_geometry('azimuth', rx_lon, rx_lat, tgt_lon(k), tgt_lat(k));
    A2 = skywave_geometry_legacy('azimuth', rx_lon, rx_lat, tgt_lon(k), tgt_lat(k));
    d_az = max(d_az, abs(A1 - A2));

    lr = 0.02 * randn();  br = 0.02 * randn();
    V1 = skywave_geometry('doppler', tx_lon, tx_lat, rx_lon, rx_lat, ...
                          tgt_lon(k), tgt_lat(k), lr, br);
    V2 = skywave_geometry_legacy('doppler', tx_lon, tx_lat, rx_lon, rx_lat, ...
                          tgt_lon(k), tgt_lat(k), lr, br);
    d_vd = max(d_vd, abs(V1 - V2));

    F1 = cell(1, 8);  F2 = cell(1, 8);
    [F1{:}] = skywave_geometry('full', tx_lon, tx_lat, rx_lon, rx_lat, tgt_lon(k), tgt_lat(k));
    [F2{:}] = skywave_geometry_legacy('full', tx_lon, tx_lat, rx_lon, rx_lat, tgt_lon(k), tgt_lat(k));
    for j = 1:8
        d_full = max(d_full, abs(F1{j} - F2{j}));
    end
end
fprintf('scalar  geocentric: max|d| = %.3e rad\n', d_sig);
fprintf('scalar  chord     : max|d| = %.3e m\n',   d_D);
fprintf('scalar  group_rng : max|d| = %.3e m\n',   d_Rg);
fprintf('scalar  azimuth   : max|d| = %.3e deg\n', d_az);
fprintf('scalar  doppler   : max|d| = %.3e m/s\n', d_vd);
fprintf('scalar  full(8)   : max|d| = %.3e\n',     d_full);

% ---- vectorized call（新增能力，与逐点结果比对）----
Rg_vec = skywave_geometry('group_range', tx_lon, tx_lat, rx_lon, rx_lat, tgt_lon, tgt_lat);
az_vec = skywave_geometry('azimuth', rx_lon, rx_lat, tgt_lon, tgt_lat);
Rg_sc  = zeros(N, 1);  az_sc = zeros(N, 1);
for k = 1:N
    Rg_sc(k) = skywave_geometry_legacy('group_range', tx_lon, tx_lat, rx_lon, rx_lat, tgt_lon(k), tgt_lat(k));
    az_sc(k) = skywave_geometry_legacy('azimuth', rx_lon, rx_lat, tgt_lon(k), tgt_lat(k));
end
fprintf('vector  group_rng : max|d| vs scalar = %.3e m\n', max(abs(Rg_vec - Rg_sc)));
fprintf('vector  azimuth   : max|d| vs scalar = %.3e deg\n', max(abs(az_vec - az_sc)));

% ---- 可选 h 的单调性/量级自检 ----
h_list = [200 250 300 320 400 450] * 1000;
Rg_h = zeros(numel(h_list), 1);
for j = 1:numel(h_list)
    Rg_h(j) = skywave_geometry('group_range', tx_lon, tx_lat, rx_lon, rx_lat, ...
                               tgt_lon(1), tgt_lat(1), h_list(j));
end
fprintf('h sensitivity     : Rg(200km)=%.1f km, Rg(320km)=%.1f km, Rg(450km)=%.1f km\n', ...
    Rg_h(1)/1e3, Rg_h(4)/1e3, Rg_h(6)/1e3);
fprintf('monotone in h     : %d\n', all(diff(Rg_h) > 0));

tol = 1e-9;
ok = max([d_sig, d_D, d_Rg, d_az, d_vd, d_full]) < tol ...
     && max(abs(Rg_vec - Rg_sc)) < tol && max(abs(az_vec - az_sc)) < tol;
fprintf('RESULT: %s\n', ternary(ok, 'PASS', 'FAIL'));


function s = ternary(c, a, b)
    if c, s = a; else, s = b; end
end
