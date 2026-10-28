% =========================================================================
% process_noise_geo.m — CV 过程噪声协方差（地理坐标，单位：度 / 度每秒）
% =========================================================================
%
% 【功能概述】
%   当局部航迹估计里没有携带过程噪声 Q（融合中心只传 x / P）时，
%   用本函数为 BC 的互协方差递推提供一个过程噪声基准。
%
%   算法与 ukf/ukf_jichu.m 的 process_noise_geo_ukf 保持一致：
%     1. 以加速度谱密度 accel_psd 构造连续白噪声加速度下的 CV 过程噪声
%        block = accel_psd * [dt^3/3, dt^2/2; dt^2/2, dt]
%     2. 经度/纬度方向按 1/(R*cos(lat))、1/R 从米换算到度
%
%   状态向量顺序：[lon; v_lon; lat; v_lat]（与本项目 UKF 一致）。
%
% 【输入参数】
%   params — 参数结构体，可用 ukf_process_accel_psd_m2_s3（默认 0.5）
%   lat    — 目标纬度（deg）
%   dt     — 帧间隔（s）
%
% 【输出参数】
%   Q — [4x4] 过程噪声协方差
% =========================================================================

function Q = process_noise_geo(params, lat, dt)
    earth_radius = 6371000.0;

    accel_psd = 0.5;
    if isstruct(params) && isfield(params, 'ukf_process_accel_psd_m2_s3')
        accel_psd = params.ukf_process_accel_psd_m2_s3;
    end
    accel_psd = max(double(accel_psd), 1e-9);

    block = accel_psd * [dt^3 / 3, dt^2 / 2; dt^2 / 2, dt];
    Q_metric = blkdiag(block, block);

    scale_lon = rad2deg(1 / (earth_radius * max(abs(cosd(lat)), 1e-6)));
    scale_lat = rad2deg(1 / earth_radius);
    transform = diag([scale_lon, scale_lon, scale_lat, scale_lat]);

    Q = transform * Q_metric * transform';
    Q = (Q + Q') / 2;
end
