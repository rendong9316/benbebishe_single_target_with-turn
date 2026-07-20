function result = bridge_smoother(snapshots, params)
% BRIDGE_SMOOTHER Reconstruct bounded gaps in an offline fused track.
% Existing fused snapshots are immutable. Only empty frames bounded by two
% nonempty snapshots are reconstructed; leading/trailing gaps stay empty.

cfg = bridge_config(params);
result = empty_result(snapshots);
if ~cfg.enabled || isempty(snapshots)
    return;
end

anchor_frames = nonempty_frames(snapshots);
if numel(anchor_frames) < 2
    return;
end

rts_bridge = cell(size(snapshots));
imm_bridge = cell(size(snapshots));
rts_diag = empty_diag();
imm_diag = empty_diag();

for q = 1:numel(anchor_frames)-1
    left_frame = anchor_frames(q);
    right_frame = anchor_frames(q+1);
    if right_frame <= left_frame + 1
        continue;
    end

    left_track = snapshots{left_frame}.trackList{1};
    right_track = snapshots{right_frame}.trackList{1};
    [x_left, P_left] = track_state(left_track);
    [x_right, P_right] = track_state(right_track);
    origin = [mean([x_left(1), x_right(1)]), mean([x_left(3), x_right(3)])];
    [xe_left, Pe_left, J] = geodetic_to_enu_state(x_left, P_left, origin, cfg);
    [xe_right, Pe_right] = geodetic_to_enu_state(x_right, P_right, origin, cfg);

    gap_steps = right_frame - left_frame;
    rts = smooth_gap(xe_left, Pe_left, xe_right, Pe_right, gap_steps, cfg, 'RTS');
    imm = smooth_gap(xe_left, Pe_left, xe_right, Pe_right, gap_steps, cfg, 'IMM');

    rts_diag(end+1) = gap_diag(left_frame, right_frame, rts, cfg); %#ok<AGROW>
    imm_diag(end+1) = gap_diag(left_frame, right_frame, imm, cfg); %#ok<AGROW>

    for step = 1:gap_steps-1
        frame = left_frame + step;
        [x_rts, P_rts] = enu_to_geodetic_state(rts.states(:, step+1), ...
            rts.covariances(:, :, step+1), origin, J);
        [x_imm, P_imm] = enu_to_geodetic_state(imm.states(:, step+1), ...
            imm.covariances(:, :, step+1), origin, J);
        rts_bridge{frame} = bridge_snapshot(frame, left_track, x_rts, P_rts, ...
            'RTS', rts.confidence, rts.mahalanobis, [1 0 0]);
        imm_bridge{frame} = bridge_snapshot(frame, left_track, x_imm, P_imm, ...
            'IMM', imm.confidence, imm.mahalanobis, imm.mode_probabilities(step+1, :));
    end
end

result.rts = package_method('RTS', snapshots, rts_bridge, rts_diag);
result.imm = package_method('IMM', snapshots, imm_bridge, imm_diag);
result.default_method = 'IMM';
result.bridge_snapshots = result.imm.bridge_snapshots;
result.reconstructed_snapshots = result.imm.reconstructed_snapshots;
result.diagnostics = result.imm.diagnostics;
end

function cfg = bridge_config(params)
defaults = struct('enabled', true, 'earth_radius_m', 6371000, ...
    'turn_rate_rad_per_sec', pi/180, 'accel_std_mps2', 0.35, ...
    'mode_transition', [0.96 0.02 0.02; 0.08 0.90 0.02; 0.08 0.02 0.90], ...
    'mode_probability_init', [0.80 0.10 0.10], ...
    'confidence_mahal_gate', 13.2767);
cfg = defaults;
if isfield(params, 'bridge')
    names = fieldnames(params.bridge);
    for i = 1:numel(names)
        cfg.(names{i}) = params.bridge.(names{i});
    end
end
validateattributes(cfg.turn_rate_rad_per_sec, {'numeric'}, {'scalar','finite','positive'});
validateattributes(cfg.accel_std_mps2, {'numeric'}, {'scalar','finite','nonnegative'});
validateattributes(cfg.mode_transition, {'numeric'}, {'size',[3 3],'finite','nonnegative'});
validateattributes(cfg.mode_probability_init, {'numeric'}, {'vector','numel',3,'finite','nonnegative'});
if any(abs(sum(cfg.mode_transition, 2) - 1) > 1e-10)
    error('bridge_smoother:invalidTransition', 'Bridge mode transition rows must sum to one.');
end
cfg.mode_probability_init = reshape(cfg.mode_probability_init, 1, []);
if sum(cfg.mode_probability_init) <= 0
    error('bridge_smoother:invalidInitialProbability', ...
        'Bridge initial mode probabilities must have a positive sum.');
end
cfg.mode_probability_init = cfg.mode_probability_init / sum(cfg.mode_probability_init);
cfg.dt_sec = params.dt_sec;
end

function result = empty_result(snapshots)
rts = package_method('RTS', snapshots, cell(size(snapshots)), empty_diag());
imm = package_method('IMM', snapshots, cell(size(snapshots)), empty_diag());
result = struct('rts', rts, 'imm', imm, 'default_method', 'IMM', ...
    'bridge_snapshots', {cell(size(snapshots))}, ...
    'reconstructed_snapshots', {snapshots}, 'diagnostics', empty_diag());
end

function packaged = package_method(name, snapshots, bridge, diagnostics)
reconstructed = snapshots;
for k = 1:numel(bridge)
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
mask = false(size(snapshots));
for k = 1:numel(snapshots)
    mask(k) = ~isempty(snapshots{k}) && isfield(snapshots{k}, 'trackList') && ...
        ~isempty(snapshots{k}.trackList);
end
frames = find(mask);
end

function [x, P] = track_state(track)
if isfield(track, 'ukf') && isfield(track.ukf, 'x')
    x = track.ukf.x;
    P = track.ukf.P;
else
    x = track.ukf_x;
    P = track.ukf_P;
end
x = x(:);
P = regularize_cov(P);
end

function [xe, Pe, J] = geodetic_to_enu_state(x, P, origin, cfg)
lon0 = origin(1); lat0 = origin(2);
meters_lon = cfg.earth_radius_m * cosd(lat0) * pi / 180;
meters_lat = cfg.earth_radius_m * pi / 180;
if abs(meters_lon) < 1, meters_lon = sign_or_one(meters_lon); end
J = diag([meters_lon, meters_lon, meters_lat, meters_lat]);
xe = [meters_lon * (x(1) - lon0); meters_lon * x(2); ...
    meters_lat * (x(3) - lat0); meters_lat * x(4)];
Pe = regularize_cov(J * P * J');
end

function [x, P] = enu_to_geodetic_state(xe, Pe, origin, J)
x = [origin(1) + xe(1) / J(1,1); xe(2) / J(2,2); ...
    origin(2) + xe(3) / J(3,3); xe(4) / J(4,4)];
Jinv = diag(1 ./ diag(J));
P = regularize_cov(Jinv * Pe * Jinv');
end

function value = sign_or_one(value)
if value == 0, value = 1; else, value = sign(value); end
end

function out = smooth_gap(x0, P0, x_right, P_right, steps, cfg, kind)
if strcmp(kind, 'RTS')
    model_rates = 0;
    Pi = 1;
    mu0 = 1;
else
    model_rates = [0, cfg.turn_rate_rad_per_sec, -cfg.turn_rate_rad_per_sec];
    Pi = cfg.mode_transition;
    mu0 = cfg.mode_probability_init;
end
M = numel(model_rates);

mode_x = cell(steps+1, M);
mode_P = cell(steps+1, M);
mu_forward = zeros(steps+1, M);
x_filtered = zeros(4, steps+1);
P_filtered = zeros(4, 4, steps+1);
x_predicted = zeros(4, steps+1);
P_predicted = zeros(4, 4, steps+1);
cross_cov = zeros(4, 4, steps);

for m = 1:M
    mode_x{1,m} = x0;
    mode_P{1,m} = P0;
end
mu_forward(1,:) = mu0;
x_filtered(:,1) = x0;
P_filtered(:,:,1) = P0;
x_predicted(:,1) = x0;
P_predicted(:,:,1) = P0;

for k = 1:steps
    joint = mu_forward(k,:)' .* Pi;
    mu_next = normalize_prob(sum(joint, 1));
    x_sources = mode_x(k,:);
    P_sources = mode_P(k,:);
    x_pair = cell(M, M);
    P_pair = cell(M, M);
    for i = 1:M
        for j = 1:M
            F = transition_matrix(model_rates(j), cfg.dt_sec);
            Q = process_noise(cfg.dt_sec, cfg.accel_std_mps2);
            x_pair{i,j} = F * x_sources{i};
            P_pair{i,j} = regularize_cov(F * P_sources{i} * F' + Q);
        end
    end
    for j = 1:M
        weights = joint(:,j)' / max(mu_next(j), eps);
        [mode_x{k+1,j}, mode_P{k+1,j}] = combine_gaussians(x_pair(:,j)', P_pair(:,j)', weights);
    end
    [x_next, P_next] = combine_gaussians(mode_x(k+1,:), mode_P(k+1,:), mu_next);
    C = zeros(4);
    for i = 1:M
        for j = 1:M
            F = transition_matrix(model_rates(j), cfg.dt_sec);
            dx0 = x_sources{i} - x_filtered(:,k);
            dx1 = x_pair{i,j} - x_next;
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

likelihood = zeros(1, M);
for m = 1:M
    S = regularize_cov(mode_P{end,m} + P_right);
    innovation = x_right - mode_x{end,m};
    likelihood(m) = gaussian_likelihood(innovation, S);
    K = mode_P{end,m} / S;
    mode_x{end,m} = mode_x{end,m} + K * innovation;
    mode_P{end,m} = regularize_cov(mode_P{end,m} - K * S * K');
end
mu_endpoint = normalize_prob(mu_forward(end,:) .* likelihood);
[x_endpoint, P_endpoint] = combine_gaussians(mode_x(end,:), mode_P(end,:), mu_endpoint);

states = x_filtered;
covariances = P_filtered;
states(:,end) = x_endpoint;
covariances(:,:,end) = P_endpoint;
for k = steps:-1:1
    G = cross_cov(:,:,k) / regularize_cov(P_predicted(:,:,k+1));
    states(:,k) = x_filtered(:,k) + G * (states(:,k+1) - x_predicted(:,k+1));
    covariances(:,:,k) = regularize_cov(P_filtered(:,:,k) + ...
        G * (covariances(:,:,k+1) - P_predicted(:,:,k+1)) * G');
end

mu_smooth = mu_forward;
mu_smooth(end,:) = mu_endpoint;
for k = steps:-1:1
    ratio = mu_smooth(k+1,:) ./ max(mu_forward(k+1,:), eps);
    mu_smooth(k,:) = normalize_prob(mu_forward(k,:) .* (Pi * ratio')');
end

innovation = x_right - x_predicted(:,end);
S = regularize_cov(P_predicted(:,:,end) + P_right);
mahal = real(innovation' * (S \ innovation));
confidence = 'high';
if ~isfinite(mahal) || mahal > cfg.confidence_mahal_gate
    confidence = 'low';
end
out = struct('states', states, 'covariances', covariances, ...
    'mode_probabilities', mu_smooth, 'mahalanobis', mahal, ...
    'confidence', confidence);
end

function F = transition_matrix(rate, dt)
if abs(rate) < 1e-12
    F = [1 dt 0 0; 0 1 0 0; 0 0 1 dt; 0 0 0 1];
    return;
end
wT = rate * dt;
s = sin(wT); c = cos(wT); omc = 1 - c;
F = [1 s/rate 0 -omc/rate; 0 c 0 -s; ...
     0 omc/rate 1 s/rate; 0 s 0 c];
end

function Q = process_noise(dt, accel_std)
q = accel_std^2;
block = q * [dt^3/3, dt^2/2; dt^2/2, dt];
Q = zeros(4);
Q(1:2,1:2) = block;
Q(3:4,3:4) = block;
Q = regularize_cov(Q);
end

function [x, P] = combine_gaussians(xs, Ps, weights)
weights = normalize_prob(weights);
x = zeros(4,1);
for i = 1:numel(weights), x = x + weights(i) * xs{i}; end
P = zeros(4);
for i = 1:numel(weights)
    dx = xs{i} - x;
    P = P + weights(i) * (Ps{i} + dx * dx');
end
P = regularize_cov(P);
end

function p = normalize_prob(p)
p = reshape(real(p), 1, []);
p(~isfinite(p) | p < 0) = 0;
total = sum(p);
if total <= eps, p = ones(size(p)) / numel(p); else, p = p / total; end
end

function value = gaussian_likelihood(innovation, S)
S = regularize_cov(S);
[R, flag] = chol(S);
if flag ~= 0
    value = realmin;
    return;
end
y = R' \ innovation;
log_value = -0.5 * (y' * y) - sum(log(diag(R))) - 0.5 * numel(innovation) * log(2*pi);
value = max(exp(max(log_value, log(realmin))), realmin);
end

function snap = bridge_snapshot(frame, template, x, P, method, confidence, mahal, mode_prob)
trk = template;
trk.lat = x(3);
trk.lon = x(1);
trk.ukf = struct('x', x, 'P', P);
trk.ukf_x = x;
trk.ukf_P = P;
trk.source = 'bridge_smoothed';
trk.is_virtual = true;
trk.has_measurement_support = false;
trk.bridge_method = method;
trk.bridge_confidence = confidence;
trk.bridge_endpoint_mahalanobis = mahal;
trk.bridge_mode_probabilities = mode_prob;
snap = struct('frameID', frame, 'trackList', {{trk}});
end

function diag = gap_diag(left_frame, right_frame, out, cfg)
diag = struct('left_frame', left_frame, 'right_frame', right_frame, ...
    'gap_frames', right_frame-left_frame-1, ...
    'endpoint_mahalanobis', out.mahalanobis, ...
    'confidence_gate', cfg.confidence_mahal_gate, ...
    'confidence', out.confidence, ...
    'mode_probabilities', out.mode_probabilities);
end

function value = count_nonempty(snapshots)
value = 0;
for k = 1:numel(snapshots)
    value = value + (~isempty(snapshots{k}) && isfield(snapshots{k}, 'trackList') && ...
        ~isempty(snapshots{k}.trackList));
end
end

function value = count_low_confidence(snapshots)
value = 0;
for k = 1:numel(snapshots)
    if isempty(snapshots{k}) || isempty(snapshots{k}.trackList), continue; end
    trk = snapshots{k}.trackList{1};
    value = value + (isfield(trk, 'bridge_confidence') && strcmp(trk.bridge_confidence, 'low'));
end
end

function diag = empty_diag()
diag = struct('left_frame', {}, 'right_frame', {}, 'gap_frames', {}, ...
    'endpoint_mahalanobis', {}, 'confidence_gate', {}, ...
    'confidence', {}, 'mode_probabilities', {});
end
