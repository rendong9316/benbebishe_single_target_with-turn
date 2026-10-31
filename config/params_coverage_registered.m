function params = params_coverage_registered()
%PARAMS_COVERAGE_REGISTERED 覆盖互补布设 + 注入系统偏差 + 专利三参数标定（mle3）
%
%   主链路的"带配准"版本。与 params_coverage_online 的几何完全一致，
%   只改两件事：
%     1. 注入非零系统偏差（真值 h=320 km、dR/dθ 非零），标称仍为 300 km；
%     2. 打开 Stage 1 的三参数 MLE 标定，估计值反馈给反演模型。
%
%   这样主链路变成：注入偏差 -> 离线标定（两站各自）-> 校正 -> 逐帧跟踪 -> 融合。
%   与 params_coverage_online（零偏差、不标定）配对使用，构成配准的 A/B 对照。
%
%   标校数据：全国 ADS-B（UTC 12-18，与评估集零重叠），
%             逐点用 radar_coverage_check 筛"落入本站波束"的样本。

    params = params_coverage_online();

    % ==================== 注入系统偏差（真值，仅量测生成使用）====================
    params.ionosphere_true_h_km    = 320.0;   % 真实等效虚高
    params.ionosphere_assumed_h_km = 300.0;   % 反演/滤波的标称值（失配 20 km）
    params.radar1_assumed_h_km     = 300.0;
    params.radar2_assumed_h_km     = 300.0;
    params.radar1_range_bias_m     =  20000.0;
    params.radar1_azimuth_bias_deg =     -3.0;
    params.radar2_range_bias_m     = -15000.0;
    params.radar2_azimuth_bias_deg =      3.5;

    % ==================== 标定（主流程 Stage 1，逐帧循环之前，离线）====================
    params.registration_mode    = 'mle3';
    params.registration_apply_h = true;   % 估计出的虚高反馈给反演/滤波
    % 2026-10-31 决策 B：主流程必须接入配准。run.m 见此标志即强制检查，
    % registration_mode='none' 会直接报错，防止误用零偏差配置出论文数字。
    params.registration_required = true;
    params.adsb_csv_path        = 'data/adsb/calib_utc12_18.csv';
    % 区域窗放到标校集全域，由 radar_coverage_check 逐点筛"落入本站波束"的样本。
    % 既满足"只用探测范围内的数据"，又保留波束内的距离跨度——这是 h 与 dR 可分离的前提。
    params.adsb_region_lon = [105.0, 125.0];
    params.adsb_region_lat = [20.0, 40.0];

    % ==================== L2 模型失配（默认全关 = 干净标定）====================
    % 用于检验"标定模型 = 生成模型"这一理想假设的影响。全为 0 时退化为原来的干净标定，
    % 结果与 2026-10-31 的 A/B 完全一致。
    params.iono_lon_tilt_km_per_deg = 0.0;    % 虚高经向梯度（空间失配，M2）
    params.iono_lon_ref_deg         = 107.4;
    params.iono_drift_amp_km        = 0.0;    % 虚高时间漂移幅度（时间失配，M1）
    params.iono_drift_period_h      = 6.0;
    params.iono_drift_phase_sec     = 0.0;
    params.gen_range_noise_scale    = 1.0;    % 生成侧噪声放大（估计侧仍按标称，M3）
    params.gen_az_noise_scale       = 1.0;
    params.calib_outlier_frac       = 0.0;    % 标校集粗差比例（M4）
    params.calib_outlier_std_m      = 30000.0;
    params.calib_ref_pos_err_m      = 0.0;    % ADS-B 参考位置误差（M5）

    fprintf('[配准] 已注入偏差：R1 dR=%.0fkm dth=%.1fdeg，R2 dR=%.0fkm dth=%.1fdeg，h_true=%.0fkm / h_nominal=%.0fkm%s', ...
        params.radar1_range_bias_m/1000, params.radar1_azimuth_bias_deg, ...
        params.radar2_range_bias_m/1000, params.radar2_azimuth_bias_deg, ...
        params.ionosphere_true_h_km, params.ionosphere_assumed_h_km, newline);
end
