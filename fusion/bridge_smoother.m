function result = bridge_smoother(snapshots, params)
% BRIDGE_SMOOTHER 用 RTS 平滑器重建融合航迹中的有界空洞。
%
% 【问题】
%   融合航迹在 gap 帧（两帧之间有空洞，没有数据）是空的。
%   本函数用 RTS（Rauch-Tung-Striebel）平滑器，基于 gap 两侧的航迹端点，
%   插值生成 gap 内的"虚拟航迹"。
%
% 【两种模式】
%   RTS 模式：单模型 CV（常速），适合直线飞行
%   IMM 模式：3 模型 IMM（CV + 左转弯 + 右转弯），适合机动飞行
%   默认使用 IMM 模式。
%
% 【限制】
%   只重建"两头有数据、中间有空洞"的 gap（bounded gaps）。
%   开头的空洞和结尾的空洞保持不变（因为缺少一侧锚点）。

cfg = bridge_config(params);  % 加载桥接配置参数
result = empty_result(snapshots);  % 初始化空结果
if ~cfg.enabled || isempty(snapshots)  % 未启用或无数据，直接返回
    return;
end

anchor_frames = nonempty_frames(snapshots);  % 找出所有非空帧号
if numel(anchor_frames) < 2  % 至少需要两个锚点帧才能填洞
    return;
end

rts_bridge = cell(size(snapshots));  % RTS 模式填洞结果
imm_bridge = cell(size(snapshots));  % IMM 模式填洞结果
rts_diag = empty_diag();  % RTS 诊断信息
imm_diag = empty_diag();  % IMM 诊断信息

for q = 1:numel(anchor_frames)-1  % 遍历每对相邻锚点帧
    left_frame = anchor_frames(q);       % 左侧锚点帧号
    right_frame = anchor_frames(q+1);    % 右侧锚点帧号
    if right_frame <= left_frame + 1  % 相邻帧之间没有空洞，跳过
        continue;
    end

    % 提取左右锚点帧的航迹状态
    left_track = snapshots{left_frame}.trackList{1};
    right_track = snapshots{right_frame}.trackList{1};
    [x_left, P_left] = track_state(left_track);  % 左锚点状态和协方差
    [x_right, P_right] = track_state(right_track);  % 右锚点状态和协方差
    % 原点取两锚点经纬度均值（用于 ENU 局部坐标转换）
    origin = [mean([x_left(1), x_right(1)]), mean([x_left(3), x_right(3)])];
    % 经纬度 → ENU（东北天）局部直角坐标
    [xe_left, Pe_left, J] = geodetic_to_enu_state(x_left, P_left, origin, cfg);
    [xe_right, Pe_right] = geodetic_to_enu_state(x_right, P_right, origin, cfg);

    gap_steps = right_frame - left_frame;  % gap 步数
    % 分别用 RTS 和 IMM 模式填洞
    rts = smooth_gap(xe_left, Pe_left, xe_right, Pe_right, gap_steps, cfg, 'RTS');
    imm = smooth_gap(xe_left, Pe_left, xe_right, Pe_right, gap_steps, cfg, 'IMM');

    rts_diag(end+1) = gap_diag(left_frame, right_frame, rts, cfg); %#ok<AGROW>
    imm_diag(end+1) = gap_diag(left_frame, right_frame, imm, cfg); %#ok<AGROW>

    % 将填洞结果写回桥接快照
    for step = 1:gap_steps-1
        frame = left_frame + step;  % 空洞内的帧号
        % RTS 模式：ENU → 经纬度
        [x_rts, P_rts] = enu_to_geodetic_state(rts.states(:, step+1), ...
            rts.covariances(:, :, step+1), origin, J);
        % IMM 模式：ENU → 经纬度
        [x_imm, P_imm] = enu_to_geodetic_state(imm.states(:, step+1), ...
            imm.covariances(:, :, step+1), origin, J);
        % 构造桥接航迹快照
        rts_bridge{frame} = bridge_snapshot(frame, left_track, x_rts, P_rts, ...
            'RTS', rts.confidence, rts.mahalanobis, [1 0 0]);
        imm_bridge{frame} = bridge_snapshot(frame, left_track, x_imm, P_imm, ...
            'IMM', imm.confidence, imm.mahalanobis, imm.mode_probabilities(step+1, :));
    end
end

result.rts = package_method('RTS', snapshots, rts_bridge, rts_diag);  % 封装 RTS 结果
result.imm = package_method('IMM', snapshots, imm_bridge, imm_diag);  % 封装 IMM 结果
result.default_method = 'IMM';  % 默认使用 IMM 模式
result.bridge_snapshots = result.imm.bridge_snapshots;  % 桥接快照
result.reconstructed_snapshots = result.imm.reconstructed_snapshots;  % 重建快照
result.diagnostics = result.imm.diagnostics;  % 诊断信息
end

function cfg = bridge_config(params)
% bridge_config 加载桥接配置参数，从默认值开始，被 params.bridge 覆盖。
defaults = struct('enabled', true, 'earth_radius_m', 6371000, ...
    'turn_rate_rad_per_sec', pi/180, 'accel_std_mps2', 0.35, ...
    'mode_transition', [0.96 0.02 0.02; 0.08 0.90 0.02; 0.08 0.02 0.90], ...
    'mode_probability_init', [0.80 0.10 0.10], ...
    'confidence_mahal_gate', 13.2767);  % 95% 置信度，3 自由度卡方门限
cfg = defaults;
if isfield(params, 'bridge')  % 如果用户传入了 params.bridge，覆盖默认值
    names = fieldnames(params.bridge);
    for i = 1:numel(names)
        cfg.(names{i}) = params.bridge.(names{i});
    end
end
% 验证参数合法性
validateattributes(cfg.turn_rate_rad_per_sec, {'numeric'}, {'scalar','finite','positive'});
validateattributes(cfg.accel_std_mps2, {'numeric'}, {'scalar','finite','nonnegative'});
validateattributes(cfg.mode_transition, {'numeric'}, {'size',[3 3],'finite','nonnegative'});
validateattributes(cfg.mode_probability_init, {'numeric'}, {'vector','numel',3,'finite','nonnegative'});
if any(abs(sum(cfg.mode_transition, 2) - 1) > 1e-10)  % 转移概率矩阵每行和应为1
    error('bridge_smoother:invalidTransition', 'Bridge mode transition rows must sum to one.');
end
cfg.mode_probability_init = reshape(cfg.mode_probability_init, 1, []);
if sum(cfg.mode_probability_init) <= 0  % 初始概率和必须为正
    error('bridge_smoother:invalidInitialProbability', ...
        'Bridge initial mode probabilities must have a positive sum.');
end
cfg.mode_probability_init = cfg.mode_probability_init / sum(cfg.mode_probability_init);  % 归一化
cfg.dt_sec = params.dt_sec;  % 采样周期
end

function result = empty_result(snapshots)
% empty_result 创建空结果结构体（当桥接被禁用时返回）
rts = package_method('RTS', snapshots, cell(size(snapshots)), empty_diag());
imm = package_method('IMM', snapshots, cell(size(snapshots)), empty_diag());
result = struct('rts', rts, 'imm', imm, 'default_method', 'IMM', ...
    'bridge_snapshots', {cell(size(snapshots))}, ...
    'reconstructed_snapshots', {snapshots}, 'diagnostics', empty_diag());
end

function packaged = package_method(name, snapshots, bridge, diagnostics)
% package_method 封装单个方法的桥接结果。
% 将 bridge（只含填洞帧的快照）与原始 snapshots（含所有帧）合并为 reconstructed。
reconstructed = snapshots;
for k = 1:numel(bridge)
    % 只在原始帧为空且桥接帧非空时才替换
    if ~isempty(bridge{k}) && (isempty(reconstructed{k}) || isempty(reconstructed{k}.trackList))
        reconstructed{k} = bridge{k};
    end
end
packaged = struct('method', name, 'bridge_snapshots', {bridge}, ...
    'reconstructed_snapshots', {reconstructed}, 'diagnostics', diagnostics, ...
    'bridge_frame_count', count_nonempty(bridge), ...
    'reconstructed_coverage_frames', count_nonempty(reconstructed), ...
    'low_confidence_bridge_count', count_low_confidence(bridge));
end

function frames = nonempty_frames(snapshots)
% nonempty_frames 找出所有非空帧号（锚点帧）
mask = false(size(snapshots));
for k = 1:numel(snapshots)
    mask(k) = ~isempty(snapshots{k}) && isfield(snapshots{k}, 'trackList') && ...
        ~isempty(snapshots{k}.trackList);
end
frames = find(mask);
end

function [x, P] = track_state(track)
% track_state 从航迹结构体中提取状态和协方差（兼容两种字段命名）
if isfield(track, 'ukf') && isfield(track.ukf, 'x')
    x = track.ukf.x;
    P = track.ukf.P;
else
    x = track.ukf_x;
    P = track.ukf_P;
end
x = x(:);  % 确保是列向量
P = regularize_cov(P);  % 正则化协方差
end

function [xe, Pe, J] = geodetic_to_enu_state(x, P, origin, cfg)
% geodetic_to_enu_state 将经纬度状态转换为 ENU（东北天）局部直角坐标。
% 在局部直角坐标系中做平滑计算更简单（不需要处理球面几何）。
lon0 = origin(1); lat0 = origin(2);  % 原点经纬度
meters_lon = cfg.earth_radius_m * cosd(lat0) * pi / 180;  % 经度方向的米/度
meters_lat = cfg.earth_radius_m * pi / 180;  % 纬度方向的米/度
if abs(meters_lon) < 1, meters_lon = sign_or_one(meters_lon); end  % 防除零
% 雅可比矩阵：经纬度 → 米
J = diag([meters_lon, meters_lon, meters_lat, meters_lat]);
% 状态转换
xe = [meters_lon * (x(1) - lon0); meters_lon * x(2); ...
    meters_lat * (x(3) - lat0); meters_lat * x(4)];
Pe = regularize_cov(J * P * J');  % 协方差变换
end

function [x, P] = enu_to_geodetic_state(xe, Pe, origin, J)
% enu_to_geodetic_state 将 ENU 局部直角坐标转回经纬度
x = [origin(1) + xe(1) / J(1,1); xe(2) / J(2,2); ...
    origin(2) + xe(3) / J(3,3); xe(4) / J(4,4)];
Jinv = diag(1 ./ diag(J));  % 雅可比逆矩阵
P = regularize_cov(Jinv * Pe * Jinv');  % 协方差逆变换
end

function value = sign_or_one(value)
% sign_or_one 0 时返回 1（防除零），否则返回符号
if value == 0, value = 1; else, value = sign(value); end
end

function out = smooth_gap(x0, P0, x_right, P_right, steps, cfg, kind)
% smooth_gap 用 RTS 平滑器填补 gap。
%
% 【算法流程】
%   前向传播（forward pass）：从 x0/P0 开始，用 IMM 或 CV 模型逐步外推
%   到右锚点，同时计算交叉协方差
%   右锚点更新（right endpoint update）：用 x_right/P_right 修正最后一步
%   后向平滑（backward pass）：从最后一步反向传播到第一步
%
% kind='RTS': 单模型常速（CV）平滑
% kind='IMM': 三模型 IMM（CV + 左转弯 + 右转弯）平滑

if strcmp(kind, 'RTS')
    model_rates = 0;       % 单模型 CV
    Pi = 1;                % 无条件转移概率
    mu0 = 1;               % 单模型初始概率
else
    model_rates = [0, cfg.turn_rate_rad_per_sec, -cfg.turn_rate_rad_per_sec];  % CV/左弯/右弯
    Pi = cfg.mode_transition;  % 3x3 转移矩阵
    mu0 = cfg.mode_probability_init;  % 初始模式概率
end
M = numel(model_rates);  % 模式数（RTS=1, IMM=3）

% 前向时间序列存储
mode_x = cell(steps+1, M);   % 各模式状态
mode_P = cell(steps+1, M);   % 各模式协方差
mu_forward = zeros(steps+1, M);  % 前向模式概率
x_filtered = zeros(4, steps+1);  % 滤波后状态
P_filtered = zeros(4, 4, steps+1);  % 滤波后协方差
x_predicted = zeros(4, steps+1);  % 预测状态
P_predicted = zeros(4, 4, steps+1);  % 预测协方差
cross_cov = zeros(4, 4, steps);  % 前后向交叉协方差（RTS 需要）

% 初始化：t=0 时刻所有模式从 x0/P0 开始
for m = 1:M
    mode_x{1,m} = x0;
    mode_P{1,m} = P0;
end
mu_forward(1,:) = mu0;
x_filtered(:,1) = x0;
P_filtered(:,:,1) = P0;
x_predicted(:,1) = x0;
P_predicted(:,:,1) = P0;

% ===== 前向传播 =====
for k = 1:steps
    % IMM 混洗：计算混合概率
    joint = mu_forward(k,:)' .* Pi;  % 外积：转移概率 × 当前概率
    mu_next = normalize_prob(sum(joint, 1));  % 归一化
    x_sources = mode_x(k,:);  % 上一时刻各模式状态
    P_sources = mode_P(k,:);  % 上一时刻各模式协方差
    % 预测：对所有 (i,j) 模式对
    x_pair = cell(M, M);
    P_pair = cell(M, M);
    for i = 1:M
        for j = 1:M
            F = transition_matrix(model_rates(j), cfg.dt_sec);  % 模式 j 的转移矩阵
            Q = process_noise(cfg.dt_sec, cfg.accel_std_mps2);  % 过程噪声
            x_pair{i,j} = F * x_sources{i};  % 状态预测
            P_pair{i,j} = regularize_cov(F * P_sources{i} * F' + Q);  % 协方差预测
        end
    end
    % IMM 合并：对各模式 j 的高斯混合做合并
    for j = 1:M
        weights = joint(:,j)' / max(mu_next(j), eps);
        [mode_x{k+1,j}, mode_P{k+1,j}] = combine_gaussians(x_pair(:,j)', P_pair(:,j)', weights);
    end
    % 合并所有模式得到滤波后估计
    [x_next, P_next] = combine_gaussians(mode_x(k+1,:), mode_P(k+1,:), mu_next);
    % 计算交叉协方差（RTS 平滑器需要）
    C = zeros(4);
    for i = 1:M
        for j = 1:M
            F = transition_matrix(model_rates(j), cfg.dt_sec);
            dx0 = x_sources{i} - x_filtered(:,k);  % 模式 i 与滤波估计的偏差
            dx1 = x_pair{i,j} - x_next;  % 预测与合并估计的偏差
            C = C + joint(i,j) * (P_sources{i} * F' + dx0 * dx1');
        end
    end
    cross_cov(:,:,k) = C;
    mu_forward(k+1,:) = mu_next;
    x_filtered(:,k+1) = x_next;
    P_filtered(:,:,k+1) = P_next;
    x_predicted(:,k+1) = x_next;
    P_predicted(:,:,k+1) = P_next;
end

% ===== 右锚点更新 =====
% 用右锚点的观测修正最后一步的估计
likelihood = zeros(1, M);
for m = 1:M
    S = regularize_cov(mode_P{end,m} + P_right);  % 新息协方差
    innovation = x_right - mode_x{end,m};  % 新息
    likelihood(m) = gaussian_likelihood(innovation, S);  % 似然值
    K = mode_P{end,m} / S;  % 卡尔曼增益（标量简化）
    mode_x{end,m} = mode_x{end,m} + K * innovation;  % 状态更新
    mode_P{end,m} = regularize_cov(mode_P{end,m} - K * S * K');  % 协方差更新
end
mu_endpoint = normalize_prob(mu_forward(end,:) .* likelihood);  % 后向模式概率
[x_endpoint, P_endpoint] = combine_gaussians(mode_x(end,:), mode_P(end,:), mu_endpoint);

% ===== 组装平滑结果 =====
states = x_filtered;       % 滤波状态
covariances = P_filtered;  % 滤波协方差
states(:,end) = x_endpoint;  % 用端点修正值替换最后一步
covariances(:,:,end) = P_endpoint;
% RTS 后向平滑
for k = steps:-1:1
    G = cross_cov(:,:,k) / regularize_cov(P_predicted(:,:,k+1));  % 平滑增益
    states(:,k) = x_filtered(:,k) + G * (states(:,k+1) - x_predicted(:,k+1));
    covariances(:,:,k) = regularize_cov(P_filtered(:,:,k) + ...
        G * (covariances(:,:,k+1) - P_predicted(:,:,k+1)) * G');
end

% 后向模式概率
mu_smooth = mu_forward;
mu_smooth(end,:) = mu_endpoint;
for k = steps:-1:1
    ratio = mu_smooth(k+1,:) ./ max(mu_forward(k+1,:), eps);
    mu_smooth(k,:) = normalize_prob(mu_forward(k,:) .* (Pi * ratio')');
end

% ===== 置信度评估 =====
innovation = x_right - x_predicted(:,end);  % 预测新息
S = regularize_cov(P_predicted(:,:,end) + P_right);
mahal = real(innovation' * (S \ innovation));  % 新息的马氏距离
confidence = 'high';
if ~isfinite(mahal) || mahal > cfg.confidence_mahal_gate  % 超过卡方门限 → 低置信度
    confidence = 'low';
end
out = struct('states', states, 'covariances', covariances, ...
    'mode_probabilities', mu_smooth, 'mahalanobis', mahal, ...
    'confidence', confidence);
end

function F = transition_matrix(rate, dt)
% transition_matrix 构建 2D 恒速（rate=0）或恒转弯（rate!=0）状态转移矩阵
if abs(rate) < 1e-12  % 常速模型
    F = [1 dt 0 0; 0 1 0 0; 0 0 1 dt; 0 0 0 1];
    return;
end
wT = rate * dt;  % 转弯角
s = sin(wT); c = cos(wT); omc = 1 - c;
% 恒转弯模型（Turn Rate 模型）
F = [1 s/rate 0 -omc/rate; 0 c 0 -s; ...
     0 omc/rate 1 s/rate; 0 s 0 c];
end

function Q = process_noise(dt, accel_std)
% process_noise 构建 2D 加速度白噪声的过程噪声协方差矩阵
% 连续白噪声离散化：对加速度白噪声在 dt 内积分得到位置和速度的协方差
q = accel_std^2;  % 加速度功率谱密度
block = q * [dt^3/3, dt^2/2; dt^2/2, dt];  % 连续白噪声离散化
Q = zeros(4);
Q(1:2,1:2) = block;  % 经度方向
Q(3:4,3:4) = block;  % 纬度方向
Q = regularize_cov(Q);
end

function [x, P] = combine_gaussians(xs, Ps, weights)
% combine_gaussians 合并多个高斯分布为单个高斯分布（IMM 合并用）
% 使用高斯混合的矩匹配：均值加权平均，协方差 = 加权内协方差 + 外协方差
weights = normalize_prob(weights);  % 归一化权重
x = zeros(4,1);
for i = 1:numel(weights), x = x + weights(i) * xs{i}; end  % 加权平均
P = zeros(4);
for i = 1:numel(weights)
    dx = xs{i} - x;  % 各模式与合并均值的偏差
    P = P + weights(i) * (Ps{i} + dx * dx');  % 合并协方差
end
P = regularize_cov(P);
end

function p = normalize_prob(p)
% normalize_prob 将概率向量归一化，处理 NaN/Inf/负值
p = reshape(real(p), 1, []);  % 取实部并展平
p(~isfinite(p) | p < 0) = 0;  % 无效值清零
total = sum(p);
if total <= eps, p = ones(size(p)) / numel(p); else, p = p / total; end  % 归一化
end

function value = gaussian_likelihood(innovation, S)
% gaussian_likelihood 计算新息在高斯分布下的对数似然值（防下溢）
S = regularize_cov(S);
[R, flag] = chol(S);  % Cholesky 分解
if flag ~= 0  % 分解失败（非正定）
    value = realmin;
    return;
end
y = R' \ innovation;  % 白化新息
log_value = -0.5 * (y' * y) - sum(log(diag(R))) - 0.5 * numel(innovation) * log(2*pi);
value = max(exp(max(log_value, log(realmin))), realmin);  % 防下溢
end

function snap = bridge_snapshot(frame, template, x, P, method, confidence, mahal, mode_prob)
% bridge_snapshot 构造桥接帧的航迹快照结构体
trk = template;  % 复用模板航迹的其他字段
trk.lat = x(3);  % 纬度
trk.lon = x(1);  % 经度
trk.ukf = struct('x', x, 'P', P);  % UKF 状态
trk.ukf_x = x;
trk.ukf_P = P;
trk.source = 'bridge_smoothed';  % 标记为桥接生成的虚拟航迹
trk.is_virtual = true;  % 虚拟航迹（无实际检测支持）
trk.has_measurement_support = false;
trk.bridge_method = method;  % 桥接方法（RTS/IMM）
trk.bridge_confidence = confidence;  % 置信度（high/low）
trk.bridge_endpoint_mahalanobis = mahal;  % 端点马氏距离
trk.bridge_mode_probabilities = mode_prob;  % 模式概率
snap = struct('frameID', frame, 'trackList', {{trk}});
end

function diag = gap_diag(left_frame, right_frame, out, cfg)
% gap_diag 构造空洞诊断信息
diag = struct('left_frame', left_frame, 'right_frame', right_frame, ...
    'gap_frames', right_frame-left_frame-1, ...
    'endpoint_mahalanobis', out.mahalanobis, ...
    'confidence_gate', cfg.confidence_mahal_gate, ...
    'confidence', out.confidence, ...
    'mode_probabilities', out.mode_probabilities);
end

function value = count_nonempty(snapshots)
% count_nonempty 统计非空快照数量
value = 0;
for k = 1:numel(snapshots)
    value = value + (~isempty(snapshots{k}) && isfield(snapshots{k}, 'trackList') && ...
        ~isempty(snapshots{k}.trackList));
end
end

function value = count_low_confidence(snapshots)
% count_low_confidence 统计低置信度桥接帧数量
value = 0;
for k = 1:numel(snapshots)
    if isempty(snapshots{k}) || isempty(snapshots{k}.trackList), continue; end
    trk = snapshots{k}.trackList{1};
    value = value + (isfield(trk, 'bridge_confidence') && strcmp(trk.bridge_confidence, 'low'));
end
end

function diag = empty_diag()
% empty_diag 创建空诊断信息结构体
diag = struct('left_frame', {}, 'right_frame', {}, 'gap_frames', {}, ...
    'endpoint_mahalanobis', {}, 'confidence_gate', {}, ...
    'confidence', {}, 'mode_probabilities', {});
end
