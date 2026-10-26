% 检查 estimate_system_bias 的 verbose 打印与雷达2回退路径
clear; clc;
rootDir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(rootDir,'utils')); addpath(fullfile(rootDir,'simulation')); addpath(fullfile(rootDir,'config'));
rng(5,'twister');
N = 60; u = (0:N-1)'/(N-1);
lon = 117 + 3*u; lat = 29.5 + 2.5*u;
h_m = 320000; dR = 10e3; dth = 0.3;
cal.lon = lon; cal.lat = lat; cal.t_sec = (0:N-1)'*30;
Rg = skywave_geometry('group_range',102.5,29.6,106.5,29.6,lon,lat,h_m);
az = skywave_geometry('azimuth',106.5,29.6,lon,lat);
cal.Rg_meas = Rg + dR + 20000*randn(N,1);
cal.az_meas = mod(az + dth + 0.60*randn(N,1),360);
cal.sigma_R = 20000;  % 逐航迹噪声覆盖（R2 量级）
cal.sigma_A = 0.60;
e = estimate_system_bias(cal, struct('radar_index',2,'verbose',true));
fprintf('CHK eta = [%.3f %.3f %.4f]\n', e.eta_hat);
fprintf('CHK sigma = [%.3f %.3f %.4f] conv=%d\n', e.sigma, e.converged);
