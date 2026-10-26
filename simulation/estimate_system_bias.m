% =========================================================================
% estimate_system_bias.m
% =========================================================================
% 功能概要：
%   时空配准（spatio-temporal registration）模块 —— 三参数联合最大似然估计。
%   用一条或多条"位置已知"的标校航迹（如 ADS-B 参考航迹），反演双照射源
%   天波雷达的公共系统误差：
%       eta = [ h , dR , dtheta ]
%       h      — 电离层等效虚高            [km]（真值约 320，标称 300）
%       dR     — 群距离（时间同步）系统偏差 [km]
%       dtheta — 方位系统偏差              [deg]
%
% 观测模型（球面几何，严格复用 utils/skywave_geometry.m）：
%   Rg_meas(k) = group_range(tx, rx, tgt_k, h) + dR + eps_R
%   az_meas(k) = azimuth(rx, tgt_k)             + dtheta + eps_A
%   其中 eps_R ~ N(0, sigma_R^2)，eps_A ~ N(0, sigma_A^2)，相互独立。
%   注：group_range 内部为 Rg = sqrt(D_tx^2+4h^2) + sqrt(D_rx^2+4h^2)，
%       D 为球面弦长 D = 2*R_e*sin(sigma/2)（与本项目其余模块完全一致）。
%
% 估计器：
%   加权非线性最小二乘（高斯噪声下等价于最大似然）：
%       eta_hat = argmin  sum_k [ rR_k^2/sigma_R^2 + rA_k^2/sigma_A^2 ]
%   求解器 lsqnonlin + Levenberg-Marquardt。
%
%   【为什么用 logistic 重参数化】
%   MATLAB 的 lsqnonlin 在 'levenberg-marquardt' 算法下**不支持边界约束**
%   （实测：传 lb/ub 时不报错，但迭代 1 次即以 exitflag=4 停在初值）。
%   为同时满足"用 LM"和"物理上下界"，这里把参数映射到无约束空间：
%       eta = lb + (ub-lb) .* sigmoid(z),  z in R^3
%   LM 在 z 空间无约束迭代，eta 天然落在 [lb, ub] 内。
%   注意：CRLB 仍在物理参数 eta 空间解析计算，与求解器内部参数化无关。
%
% 用法：
%   est = estimate_system_bias(calib, params)
%
% 输入 calib（结构体或结构体数组，数组即多航迹联合估计）：
%   .lon, .lat    — 目标经纬度序列（度），Nx1
%   .Rg_meas      — 对应时刻的群距离量测（米），Nx1
%   .az_meas      — 对应时刻的方位量测（度，0~360），Nx1
%   .t_sec        — 对应时刻（秒），Nx1；可选，仅用于排序/对齐，缺失时按 1:N
%   .sigma_R      — 可选，该航迹距离噪声标准差（米），缺省取 params.sigma_R
%   .sigma_A      — 可选，该航迹方位噪声标准差（度），缺省取 params.sigma_A
%
% 输入 params（均可缺省）：
%   .tx_lon/.tx_lat/.rx_lon/.rx_lat — 站点经纬度（度）；缺省时按 .radar_index
%                                      （默认 1）从 simulation_params_oracle 读取
%   .radar_index  — 1 或 2，缺省 1
%   .sigma_R      — 距离噪声标准差（米），缺省 10000
%   .sigma_A      — 方位噪声标准差（度），缺省 0.35
%   .eta0         — 初值 [h dR dtheta]，缺省 [300 0 0]
%   .lb / .ub     — 上下界，缺省 [200 -50 -3] / [450 50 3]
%   .verbose      — 是否打印，缺省 true
%
% 输出 est：
%   .eta_hat    — [h_km, dR_km, dtheta_deg]（1x3）
%   .crlb       — 3x3 协方差下界（单位 km^2 / km^2 / deg^2）
%   .sigma      — sqrt(diag(crlb))，1x3 标准差
%   .corr       — 3x3 参数相关系数矩阵（由 crlb 归一化得到）
%   .residual   — .mean/.rms/.max 为 1x2 [距离(km), 方位(deg)]；
%                 .rR（米）、.rA（度）为逐点残差
%   .converged  — 逻辑标量
%   .exitflag   — lsqnonlin 退出标志
%   .iterations — 迭代次数
%   .cost       — 目标函数终值（加权残差平方和）
%   .n_obs      — 参与估计的有效观测数
%   .diag       — 可辨识性诊断：灵敏度 s_k 的均值/标准差/变异系数、条件数
%   .sigma_R/.sigma_A/.eta0/.lb/.ub — 本次估计实际使用的配置（便于复现）
%
% 依赖：
%   utils/skywave_geometry.m（group_range 的可选 h 参数、azimuth、chord_length）
%   config/simulation_params_oracle.m（仅在未显式给出站点坐标时调用）
%
% 备注：
%   h 与 dR 在群距离方程中近乎同向作用（相关系数约 -0.99），单条航迹若距离
%   跨度小，二者无法分离 —— 这不是求解器问题，而是几何可辨识性问题，CRLB
%   会如实反映（见 crlb(1,1)、crlb(2,2) 与 corr(1,2)）。
% =========================================================================

function est = estimate_system_bias(calib, params)

    % ---------------------------------------------------------------------
    % 0. 入参与默认配置
    % ---------------------------------------------------------------------
    if nargin < 2 || isempty(params)
        params = struct();
    end
    if nargin < 1 || isempty(calib)
        error('estimate_system_bias:missing_input', 'calib 不能为空');
    end

    cfg = local_config(params);

    % ---------------------------------------------------------------------
    % 1. 标校数据展平（支持结构体数组 = 多航迹联合估计）
    % ---------------------------------------------------------------------
    [lon, lat, Rg_meas, az_meas, sR, sA] = flatten_calib(calib, cfg);

    % 有效样本筛选：任一分量为 NaN/Inf 的点直接剔除
    valid = isfinite(lon) & isfinite(lat) & isfinite(Rg_meas) & isfinite(az_meas);
    lon = lon(valid);  lat = lat(valid);
    Rg_meas = Rg_meas(valid);  az_meas = az_meas(valid);
    sR = sR(valid);  sA = sA(valid);

    n = numel(lon);
    if n < 3
        error('estimate_system_bias:too_few_points', ...
            '有效标校点仅 %d 个，至少需要 3 个（3 参数 / 每点 2 个方程）', n);
    end

    % ---------------------------------------------------------------------
    % 2. 边界与初值（logistic 重参数化，见文件头说明）
    % ---------------------------------------------------------------------
    lb   = cfg.lb(:);
    ub   = cfg.ub(:);
    span = ub - lb;
    eta0 = min(max(cfg.eta0(:), lb), ub);

    frac0 = (eta0 - lb) ./ span;
    frac0 = min(max(frac0, 1e-6), 1 - 1e-6);      % 防止 logit 发散
    z0 = log(frac0 ./ (1 - frac0));

    % ---------------------------------------------------------------------
    % 3. LM 求解（无约束 z 空间）
    % ---------------------------------------------------------------------
    obj = @(z) weighted_residuals(z, lb, span, lon, lat, Rg_meas, az_meas, ...
                                  sR, sA, cfg);

    opts = optimoptions('lsqnonlin', ...
        'Algorithm',              'levenberg-marquardt', ...
        'Display',                'off', ...
        'FunctionTolerance',      1e-12, ...
        'StepTolerance',          1e-12, ...
        'MaxIterations',          400, ...
        'MaxFunctionEvaluations', 20000);

    [z_hat, resnorm, ~, exitflag, output] = lsqnonlin(obj, z0, [], [], opts);

    frac_hat = 1.0 ./ (1.0 + exp(-z_hat));
    eta_hat  = (lb + span .* frac_hat).';         % 1x3: [h_km, dR_km, dtheta_deg]

    % ---------------------------------------------------------------------
    % 4. 残差统计（用估计参数重算正演）
    % ---------------------------------------------------------------------
    rR = Rg_meas - forward_range(lon, lat, eta_hat(1) * 1000, cfg) - eta_hat(2) * 1000;
    rA = wrap180(az_meas - forward_azimuth(lon, lat, cfg) - eta_hat(3));

    % ---------------------------------------------------------------------
    % 5. CRLB（物理参数空间的解析 Jacobian）
    % ---------------------------------------------------------------------
    [J, s_k] = jacobian_phys(lon, lat, eta_hat, sR, sA, cfg);
    F    = J.' * J;
    crlb = inv(F);
    crlb = (crlb + crlb.') / 2;                   % 强制对称，消除数值不对称
    sig  = sqrt(diag(crlb)).';
    d    = sqrt(diag(crlb));
    corr = crlb ./ (d * d.');

    % ---------------------------------------------------------------------
    % 6. 组装输出
    % ---------------------------------------------------------------------
    est.eta_hat    = eta_hat;
    est.crlb       = crlb;
    est.sigma      = sig;
    est.corr       = corr;
    est.residual.mean = [mean(rR) / 1000, mean(rA)];
    est.residual.rms  = [sqrt(mean(rR.^2)) / 1000, sqrt(mean(rA.^2))];
    est.residual.max  = [max(abs(rR)) / 1000, max(abs(rA))];
    est.residual.rR   = rR;                       % 米
    est.residual.rA   = rA;                       % 度
    est.residual.unit = {'km', 'deg'};

    est.converged  = (exitflag > 0);
    est.exitflag   = exitflag;
    est.iterations = output.iterations;
    est.cost       = resnorm;
    est.n_obs      = n;

    est.diag.s        = s_k;                      % dRg/dh 灵敏度（无量纲，米/米）
    est.diag.s_mean   = mean(s_k);
    est.diag.s_std    = std(s_k);
    est.diag.s_cv     = std(s_k) / mean(s_k);     % 变异系数：可分离性的直接度量
    est.diag.cond_fim = cond(F);
    est.diag.rho_h_dR = corr(1, 2);

    est.sigma_R = cfg.sigma_R;
    est.sigma_A = cfg.sigma_A;
    est.eta0    = cfg.eta0(:).';
    est.lb      = lb(:).';
    est.ub      = ub(:).';

    % ---------------------------------------------------------------------
    % 7. 打印（英文，避免中文控制台乱码）
    % ---------------------------------------------------------------------
    if cfg.verbose
        fprintf('--- estimate_system_bias -------------------------------------\n');
        fprintf('  radar %d  tx=(%.2f,%.2f)  rx=(%.2f,%.2f)\n', ...
            cfg.radar_index, cfg.tx_lon, cfg.tx_lat, cfg.rx_lon, cfg.rx_lat);
        fprintf('  N = %d   sigma_R = %s km   sigma_A = %s deg\n', ...
            n, fmt_span(sR / 1000), fmt_span(sA));
        fprintf('  eta_hat  h = %+9.3f km   dR = %+9.3f km   dtheta = %+8.4f deg\n', ...
            eta_hat(1), eta_hat(2), eta_hat(3));
        fprintf('  CRLB sd  h = %9.3f km   dR = %9.3f km   dtheta = %8.4f deg\n', ...
            sig(1), sig(2), sig(3));
        fprintf('  corr(h,dR) = %+.4f    CV(s) = %.4f    cond(FIM) = %.3e\n', ...
            corr(1, 2), est.diag.s_cv, est.diag.cond_fim);
        fprintf('  residual  range: mean %+7.3f km  rms %6.3f km  max %6.3f km\n', ...
            est.residual.mean(1), est.residual.rms(1), est.residual.max(1));
        fprintf('            azim : mean %+7.4f deg rms %6.4f deg max %6.4f deg\n', ...
            est.residual.mean(2), est.residual.rms(2), est.residual.max(2));
        fprintf('  exitflag = %d   iterations = %d   cost = %.4f   converged = %d\n', ...
            exitflag, output.iterations, resnorm, est.converged);
        fprintf('-------------------------------------------------------------\n');
    end
end


% =========================================================================
% local_config — 默认配置与站点几何
% =========================================================================
function cfg = local_config(params)

    cfg.verbose     = getfield_def(params, 'verbose', true);
    cfg.radar_index = getfield_def(params, 'radar_index', 1);

    % ---- 站点几何：优先用显式坐标，否则从统一参数配置读取 ----
    if isfield(params, 'tx_lon') && isfield(params, 'tx_lat') && ...
       isfield(params, 'rx_lon') && isfield(params, 'rx_lat')
        cfg.tx_lon = params.tx_lon;  cfg.tx_lat = params.tx_lat;
        cfg.rx_lon = params.rx_lon;  cfg.rx_lat = params.rx_lat;
    else
        idx = cfg.radar_index;
        if ~ismember(idx, [1 2])
            error('estimate_system_bias:bad_radar_index', 'radar_index 只能是 1 或 2');
        end
        p0 = simulation_params_oracle();
        cfg.tx_lon = p0.(sprintf('radar%d_tx_lon', idx));
        cfg.tx_lat = p0.(sprintf('radar%d_tx_lat', idx));
        cfg.rx_lon = p0.(sprintf('radar%d_lon',    idx));
        cfg.rx_lat = p0.(sprintf('radar%d_lat',    idx));
    end

    % ---- 噪声水平（距离：米；方位：度） ----
    cfg.sigma_R = getfield_def(params, 'sigma_R', 10000.0);
    cfg.sigma_A = getfield_def(params, 'sigma_A', 0.35);

    % ---- 初值与物理上下界 ----
    cfg.eta0 = getfield_def(params, 'eta0', [300.0, 0.0, 0.0]);
    cfg.lb   = getfield_def(params, 'lb',   [200.0, -50.0, -3.0]);
    cfg.ub   = getfield_def(params, 'ub',   [450.0,  50.0,  3.0]);

    if numel(cfg.eta0) ~= 3 || numel(cfg.lb) ~= 3 || numel(cfg.ub) ~= 3
        error('estimate_system_bias:bad_bounds', 'eta0 / lb / ub 必须是长度 3 的向量');
    end
    if any(cfg.lb >= cfg.ub)
        error('estimate_system_bias:bad_bounds', 'lb 必须严格小于 ub');
    end
    if cfg.sigma_R <= 0 || cfg.sigma_A <= 0
        error('estimate_system_bias:bad_sigma', 'sigma_R / sigma_A 必须为正');
    end
end


function v = getfield_def(s, name, def)
    if isfield(s, name) && ~isempty(s.(name))
        v = s.(name);
    else
        v = def;
    end
end


% =========================================================================
% flatten_calib — 结构体（数组）→ 列向量，逐航迹可覆盖噪声水平
% =========================================================================
function [lon, lat, Rg, az, sR, sA] = flatten_calib(calib, cfg)

    lon = []; lat = []; Rg = []; az = []; sR = []; sA = [];

    for m = 1:numel(calib)
        c = calib(m);

        need = {'lon', 'lat', 'Rg_meas', 'az_meas'};
        for j = 1:numel(need)
            if ~isfield(c, need{j})
                error('estimate_system_bias:missing_field', ...
                    'calib(%d) 缺少字段 %s', m, need{j});
            end
        end

        n_m = numel(c.lon);
        if numel(c.lat) ~= n_m || numel(c.Rg_meas) ~= n_m || numel(c.az_meas) ~= n_m
            error('estimate_system_bias:size_mismatch', ...
                'calib(%d) 的 lon/lat/Rg_meas/az_meas 长度不一致', m);
        end

        sR_m = getfield_def(c, 'sigma_R', cfg.sigma_R);
        sA_m = getfield_def(c, 'sigma_A', cfg.sigma_A);

        lon = [lon; c.lon(:)];            %#ok<AGROW>
        lat = [lat; c.lat(:)];            %#ok<AGROW>
        Rg  = [Rg;  c.Rg_meas(:)];        %#ok<AGROW>
        az  = [az;  c.az_meas(:)];        %#ok<AGROW>
        sR  = [sR;  repmat(sR_m, n_m, 1)]; %#ok<AGROW>
        sA  = [sA;  repmat(sA_m, n_m, 1)]; %#ok<AGROW>
    end
end


% =========================================================================
% 正演模型（统一走 skywave_geometry，保证与仿真/UKF 端严格一致）
% =========================================================================
function Rg = forward_range(lon, lat, h_m, cfg)
    Rg = skywave_geometry('group_range', cfg.tx_lon, cfg.tx_lat, ...
                          cfg.rx_lon, cfg.rx_lat, lon, lat, h_m);
end

function az = forward_azimuth(lon, lat, cfg)
    az = skywave_geometry('azimuth', cfg.rx_lon, cfg.rx_lat, lon, lat);
end


% =========================================================================
% weighted_residuals — LM 目标（z 空间），返回 2N x 1 加权残差
% =========================================================================
function r = weighted_residuals(z, lb, span, lon, lat, Rg_meas, az_meas, sR, sA, cfg)

    eta = lb + span .* (1.0 ./ (1.0 + exp(-z(:))));

    h_m  = eta(1) * 1000.0;
    dR_m = eta(2) * 1000.0;
    dth  = eta(3);

    Rg_pred = forward_range(lon, lat, h_m, cfg);
    az_pred = forward_azimuth(lon, lat, cfg);

    rR = (Rg_meas - Rg_pred - dR_m) ./ sR;
    rA = wrap180(az_meas - az_pred - dth) ./ sA;

    r = [rR; rA];
end


% =========================================================================
% jacobian_phys — 物理参数 eta 空间的加权 Jacobian（用于 CRLB）
%
%   rR_k = (Rg_meas - Rg(h) - dR) / sigma_R
%   rA_k = (az_meas - az - dtheta) / sigma_A
%
%   dRg/dh = 4h/sqrt(D_tx^2+4h^2) + 4h/sqrt(D_rx^2+4h^2)   （D 为球面弦长）
%   d rR/d h[km]      = -dRg/dh * 1000 / sigma_R
%   d rR/d dR[km]     = -1000 / sigma_R
%   d rA/d dth[deg]   = -1 / sigma_A
% =========================================================================
function [J, s_k] = jacobian_phys(lon, lat, eta, sR, sA, cfg)

    n   = numel(lon);
    h_m = eta(1) * 1000.0;

    D_tx = skywave_geometry('chord_length', cfg.tx_lon, cfg.tx_lat, lon, lat);
    D_rx = skywave_geometry('chord_length', cfg.rx_lon, cfg.rx_lat, lon, lat);

    s_k = 4.0 * h_m ./ sqrt(D_tx.^2 + 4.0 * h_m^2) ...
        + 4.0 * h_m ./ sqrt(D_rx.^2 + 4.0 * h_m^2);      % dRg/dh（米/米）

    J = zeros(2 * n, 3);

    % 距离方程行
    J(1:n, 1) = -s_k * 1000.0 ./ sR;                     % 对 h [km]
    J(1:n, 2) = -1000.0 ./ sR;                           % 对 dR [km]

    % 方位方程行
    J(n+1:end, 3) = -1.0 ./ sA;                          % 对 dtheta [deg]
end


% =========================================================================
% wrap180 — 角度差归一化到 (-180, 180]
% =========================================================================
function d = wrap180(d)
    d = mod(d + 180.0, 360.0) - 180.0;
end


% =========================================================================
% fmt_span — 把逐点噪声水平格式化成 "值" 或 "最小值~最大值"（用于打印）
% =========================================================================
function s = fmt_span(v)
    lo = min(v);  hi = max(v);
    if hi - lo <= 1e-9 * max(1, abs(hi))
        s = sprintf('%.2f', lo);
    else
        s = sprintf('%.2f~%.2f', lo, hi);
    end
end
