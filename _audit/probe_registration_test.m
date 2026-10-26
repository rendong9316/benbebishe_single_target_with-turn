% =========================================================================
% probe_registration_test.m  —  时空配准模块验证脚本（_audit 探针）
% =========================================================================
% 目的：
%   1) 验证 utils/skywave_geometry.m 的可选 h 参数是向后兼容的；
%   2) 验证 simulation/estimate_system_bias.m 能否从单条标校航迹反演
%      eta = [h, dR, dtheta]；
%   3) 用蒙特卡洛检验估计量无偏性 / 是否达到 CRLB；
%   4) 用多航迹联合估计检验"距离跨度 → 可辨识性"的理论预期。
%
% 运行：cd _audit 后  matlab -batch "probe_registration_test"
% 说明：控制台只输出英文，避免 Windows 中文路径下的乱码。
% =========================================================================

clear; clc;

thisFile = mfilename('fullpath');
rootDir  = fileparts(fileparts(thisFile));
addpath(fullfile(rootDir, 'utils'));
addpath(fullfile(rootDir, 'simulation'));
addpath(fullfile(rootDir, 'config'));

fprintf('=================================================================\n');
fprintf(' Registration probe: 3-parameter ML (h, dR, dtheta)\n');
fprintf(' MATLAB %s\n', version);
fprintf('=================================================================\n');

% -------------------------------------------------------------------------
% 场景常量（R1：Tx 100.0E/30.7N，Rx 104.0E/30.7N）
% -------------------------------------------------------------------------
SITE.tx_lon = 100.0;  SITE.tx_lat = 30.7;
SITE.rx_lon = 104.0;  SITE.rx_lat = 30.7;

TRUTH.h      = 320.0;    % km
TRUTH.dR     =  10.0;    % km
TRUTH.dtheta =   0.3;    % deg

SIG.sigma_R = 10000.0;   % m
SIG.sigma_A = 0.35;      % deg

% =========================================================================
% [0] 向后兼容性回归测试
% =========================================================================
fprintf('\n[0] BACKWARD COMPATIBILITY of skywave_geometry\n');

lon_t = 118.0;  lat_t = 31.0;
Rg_old = skywave_geometry('group_range', SITE.tx_lon, SITE.tx_lat, ...
                          SITE.rx_lon, SITE.rx_lat, lon_t, lat_t);
Rg_h300 = skywave_geometry('group_range', SITE.tx_lon, SITE.tx_lat, ...
                           SITE.rx_lon, SITE.rx_lat, lon_t, lat_t, 300000.0);
Rg_empty = skywave_geometry('group_range', SITE.tx_lon, SITE.tx_lat, ...
                            SITE.rx_lon, SITE.rx_lat, lon_t, lat_t, []);
Rg_h320 = skywave_geometry('group_range', SITE.tx_lon, SITE.tx_lat, ...
                           SITE.rx_lon, SITE.rx_lat, lon_t, lat_t, 320000.0);
az_old = skywave_geometry('azimuth', SITE.rx_lon, SITE.rx_lat, lon_t, lat_t);
az_new = skywave_geometry('azimuth', SITE.rx_lon, SITE.rx_lat, lon_t, lat_t, 320000.0);

fprintf('  Rg(no h)        = %.6f m\n', Rg_old);
fprintf('  Rg(h=300km)     = %.6f m   diff = %.3e m\n', Rg_h300, abs(Rg_h300 - Rg_old));
fprintf('  Rg(h=[])        = %.6f m   diff = %.3e m\n', Rg_empty, abs(Rg_empty - Rg_old));
fprintf('  Rg(h=320km)     = %.6f m   diff vs 300km = %.3f m\n', Rg_h320, Rg_h320 - Rg_old);
fprintf('  az(no h) - az(h)= %.3e deg\n', az_new - az_old);
if abs(Rg_h300 - Rg_old) < 1e-9 && abs(Rg_empty - Rg_old) < 1e-9 && abs(az_new - az_old) < 1e-12
    fprintf('  RESULT: PASS (default behaviour unchanged)\n');
else
    fprintf('  RESULT: FAIL (default behaviour changed!)\n');
end

% =========================================================================
% [1] 单条标校航迹（题设几何）：(117.0,29.5) -> (120.0,32.0)，50 帧 / 30 s
% =========================================================================
fprintf('\n[1] SINGLE CALIBRATION TRACK\n');

trkA = make_track([117.0 29.5], [120.0 32.0], 50, 30.0);
trkA = add_measurements(trkA, SITE, TRUTH, SIG, 20261025);

fprintf('  N = %d, dt = %.0f s, span %.0f s\n', trkA.N, trkA.dt, trkA.t(end));
fprintf('  chord D_tx = %.1f ~ %.1f km, D_rx = %.1f ~ %.1f km\n', ...
    min(trkA.D_tx)/1e3, max(trkA.D_tx)/1e3, min(trkA.D_rx)/1e3, max(trkA.D_rx)/1e3);
fprintf('  sensitivity s = dRg/dh: %.4f ~ %.4f (CV = %.4f)\n', ...
    min(trkA.s), max(trkA.s), std(trkA.s)/mean(trkA.s));

estA = estimate_system_bias(trkA.calib, make_params(SITE, SIG));

report_single('A', TRUTH, estA);

% =========================================================================
% [2] 蒙特卡洛（同一几何，50 次独立噪声实现）
% =========================================================================
fprintf('\n[2] MONTE CARLO (50 runs, same geometry)\n');

MC = 50;
E  = zeros(MC, 3);
SD = zeros(MC, 3);
for k = 1:MC
    tk = add_measurements(trkA, SITE, TRUTH, SIG, 20261025 + 101*k);
    ek = estimate_system_bias(tk.calib, make_params(SITE, SIG));
    E(k, :)  = ek.eta_hat - [TRUTH.h, TRUTH.dR, TRUTH.dtheta];
    SD(k, :) = ek.sigma;
end
names = {'h [km]', 'dR [km]', 'dtheta [deg]'};
fprintf('  %-14s %10s %10s %10s %10s\n', 'param', 'bias', 'std', 'RMSE', 'CRLB_sd');
for j = 1:3
    fprintf('  %-14s %+10.3f %10.3f %10.3f %10.3f\n', names{j}, ...
        mean(E(:, j)), std(E(:, j)), sqrt(mean(E(:, j).^2)), mean(SD(:, j)));
end
fprintf('  (std vs CRLB ratio: h %.2f, dR %.2f, dtheta %.2f)\n', ...
    std(E(:,1))/mean(SD(:,1)), std(E(:,2))/mean(SD(:,2)), std(E(:,3))/mean(SD(:,3)));

% =========================================================================
% [3] 多航迹联合估计（拉宽距离跨度）
% =========================================================================
fprintf('\n[3] MULTI-TRACK JOINT ESTIMATION (3 tracks, wider range span)\n');

trkB = make_track([115.0 28.5], [118.5 31.5], 50, 30.0);
trkC = make_track([121.0 30.5], [124.0 33.0], 50, 30.0);
trkB = add_measurements(trkB, SITE, TRUTH, SIG, 20261101);
trkC = add_measurements(trkC, SITE, TRUTH, SIG, 20261102);

fprintf('  track ranges D_rx: A %.0f-%.0f km | B %.0f-%.0f km | C %.0f-%.0f km\n', ...
    min(trkA.D_rx)/1e3, max(trkA.D_rx)/1e3, min(trkB.D_rx)/1e3, max(trkB.D_rx)/1e3, ...
    min(trkC.D_rx)/1e3, max(trkC.D_rx)/1e3);

estJ = estimate_system_bias([trkA.calib, trkB.calib, trkC.calib], make_params(SITE, SIG));
report_single('J', TRUTH, estJ);

% 联合灵敏度跨度
sAll = [trkA.s; trkB.s; trkC.s];
fprintf('  joint sensitivity s: %.4f ~ %.4f (CV = %.4f), N = %d\n', ...
    min(sAll), max(sAll), std(sAll)/mean(sAll), numel(sAll));

% =========================================================================
% [4] 最优性与配置回退路径自检
% =========================================================================
fprintf('\n[4] OPTIMALITY / CONFIG FALLBACK CHECKS\n');

% ---- 4.1 eta_hat 是否贴边界（贴边界说明几何不可辨识） ----
names = {'h', 'dR', 'dtheta'};
tolB  = [1.0, 0.5, 0.05];
for j = 1:3
    onB = (estA.eta_hat(j) - estA.lb(j) < tolB(j)) || (estA.ub(j) - estA.eta_hat(j) < tolB(j));
    fprintf('  bound check %-7s: est %10.3f in [%8.2f, %8.2f]  on_bound = %d\n', ...
        names{j}, estA.eta_hat(j), estA.lb(j), estA.ub(j), onB);
end

% ---- 4.2 数值局部最优性：三个方向 ± 扰动后代价都应上升 ----
step = [0.5, 0.5, 0.01];
c0 = cost_of(estA.eta_hat, trkA.calib, SITE, SIG);
fprintf('  cost(eta_hat) = %.6f\n', c0);
for j = 1:3
    ep = estA.eta_hat;  ep(j) = ep(j) + step(j);
    em = estA.eta_hat;  em(j) = em(j) - step(j);
    cp = cost_of(ep, trkA.calib, SITE, SIG);
    cm = cost_of(em, trkA.calib, SITE, SIG);
    fprintf('  perturb %-7s +%.2f: dJ = %+9.5f | -%.2f: dJ = %+9.5f  (local_min = %d)\n', ...
        names{j}, step(j), cp - c0, step(j), cm - c0, (cp >= c0) && (cm >= c0));
end

% ---- 4.3 不传站点坐标时，应按 radar_index 从 config 回退 ----
estCfg = estimate_system_bias(trkA.calib, struct('radar_index', 1, ...
    'sigma_R', SIG.sigma_R, 'sigma_A', SIG.sigma_A, 'verbose', false));
fprintf('  config fallback: eta_hat diff vs explicit-site = %.3e km (max abs)\n', ...
    max(abs(estCfg.eta_hat - estA.eta_hat)));

% =========================================================================
% [5] 结论摘要
% =========================================================================
fprintf('\n=================================================================\n');
fprintf('[5] SUMMARY\n');
fprintf('  single-track  : dh = %+8.3f km (2sig %6.2f)  dR err = %+8.3f km (2sig %6.2f)\n', ...
    estA.eta_hat(1)-TRUTH.h, 2*estA.sigma(1), estA.eta_hat(2)-TRUTH.dR, 2*estA.sigma(2));
fprintf('                  dtheta err = %+7.4f deg (2sig %6.4f)\n', ...
    estA.eta_hat(3)-TRUTH.dtheta, 2*estA.sigma(3));
fprintf('  multi-track   : dh = %+8.3f km (2sig %6.2f)  dR err = %+8.3f km (2sig %6.2f)\n', ...
    estJ.eta_hat(1)-TRUTH.h, 2*estJ.sigma(1), estJ.eta_hat(2)-TRUTH.dR, 2*estJ.sigma(2));
fprintf('                  dtheta err = %+7.4f deg (2sig %6.4f)\n', ...
    estJ.eta_hat(3)-TRUTH.dtheta, 2*estJ.sigma(3));
fprintf('  CRLB ratio dR (single/joint) = %.2f\n', estA.sigma(2)/estJ.sigma(2));
fprintf('=================================================================\n');


% =========================================================================
% 局部函数
% =========================================================================
function p = make_params(site, sig)
    p.tx_lon  = site.tx_lon;  p.tx_lat = site.tx_lat;
    p.rx_lon  = site.rx_lon;  p.rx_lat = site.rx_lat;
    p.sigma_R = sig.sigma_R;
    p.sigma_A = sig.sigma_A;
    p.eta0    = [300.0, 0.0, 0.0];
    p.lb      = [200.0, -50.0, -3.0];
    p.ub      = [450.0,  50.0,  3.0];
    p.verbose = false;
end

function trk = make_track(p0, p1, N, dt)
% 匀速直线航迹：经纬度线性插值（与项目 build_track 的口径一致）
    u = (0:N-1).' / (N-1);
    trk.N  = N;
    trk.dt = dt;
    trk.t  = (0:N-1).' * dt;
    trk.lon = p0(1) + (p1(1) - p0(1)) * u;
    trk.lat = p0(2) + (p1(2) - p0(2)) * u;
end

function trk = add_measurements(trk, site, truth, sig, seed)
% 用量测模型生成观测：Rg = group_range(h_true) + dR_true + n_R，
%                        az = azimuth            + dtheta_true + n_A
    rng(seed, 'twister');

    trk.D_tx = skywave_geometry('chord_length', site.tx_lon, site.tx_lat, trk.lon, trk.lat);
    trk.D_rx = skywave_geometry('chord_length', site.rx_lon, site.rx_lat, trk.lon, trk.lat);

    h_m = truth.h * 1000.0;
    trk.s = 4*h_m ./ sqrt(trk.D_tx.^2 + 4*h_m^2) + 4*h_m ./ sqrt(trk.D_rx.^2 + 4*h_m^2);

    Rg_true = skywave_geometry('group_range', site.tx_lon, site.tx_lat, ...
                               site.rx_lon, site.rx_lat, trk.lon, trk.lat, h_m);
    az_true = skywave_geometry('azimuth', site.rx_lon, site.rx_lat, trk.lon, trk.lat);

    trk.calib.lon     = trk.lon;
    trk.calib.lat     = trk.lat;
    trk.calib.t_sec   = trk.t;
    trk.calib.Rg_meas = Rg_true + truth.dR * 1000.0 + sig.sigma_R * randn(trk.N, 1);
    trk.calib.az_meas = mod(az_true + truth.dtheta + sig.sigma_A * randn(trk.N, 1), 360.0);
    trk.calib.sigma_R = sig.sigma_R;
    trk.calib.sigma_A = sig.sigma_A;
end

function report_single(tag, truth, est)
    tv = [truth.h, truth.dR, truth.dtheta];
    nm = {'h [km]', 'dR [km]', 'dtheta [deg]'};
    fprintf('  --- case %s ---\n', tag);
    fprintf('  %-14s %10s %10s %10s %10s %8s\n', 'param', 'truth', 'est', 'error', 'CRLB_sd', '2sig?');
    for j = 1:3
        e  = est.eta_hat(j) - tv(j);
        ok = abs(e) <= 2 * est.sigma(j);
        fprintf('  %-14s %10.3f %10.3f %+10.3f %10.3f %8s\n', ...
            nm{j}, tv(j), est.eta_hat(j), e, est.sigma(j), ternary(ok, 'YES', 'NO'));
    end
    fprintf('  CRLB matrix:\n');
    fprintf('    [%10.3f %10.3f %10.3f]\n', est.crlb(1,:));
    fprintf('    [%10.3f %10.3f %10.3f]\n', est.crlb(2,:));
    fprintf('    [%10.3f %10.3f %10.3f]\n', est.crlb(3,:));
    fprintf('  corr(h,dR) = %+.4f, CV(s) = %.4f, cond(FIM) = %.3e\n', ...
        est.corr(1,2), est.diag.s_cv, est.diag.cond_fim);
    fprintf('  residual: range mean %+7.3f rms %6.3f max %6.3f km | az mean %+7.4f rms %6.4f max %6.4f deg\n', ...
        est.residual.mean(1), est.residual.rms(1), est.residual.max(1), ...
        est.residual.mean(2), est.residual.rms(2), est.residual.max(2));
    fprintf('  exitflag = %d, iterations = %d, cost = %.4f, converged = %d, N = %d\n', ...
        est.exitflag, est.iterations, est.cost, est.converged, est.n_obs);
end

function s = ternary(c, a, b)
    if c, s = a; else, s = b; end
end

function c = cost_of(eta, calib, site, sig)
% 加权残差平方和（与 estimate_system_bias 内部目标函数同口径）
    h_m  = eta(1) * 1000.0;
    dR_m = eta(2) * 1000.0;
    dth  = eta(3);

    Rg_pred = skywave_geometry('group_range', site.tx_lon, site.tx_lat, ...
                               site.rx_lon, site.rx_lat, calib.lon, calib.lat, h_m);
    az_pred = skywave_geometry('azimuth', site.rx_lon, site.rx_lat, calib.lon, calib.lat);

    rR = (calib.Rg_meas - Rg_pred - dR_m) / sig.sigma_R;
    rA = (mod(calib.az_meas - az_pred - dth + 180.0, 360.0) - 180.0) / sig.sigma_A;

    c = sum(rR.^2) + sum(rA.^2);
end
