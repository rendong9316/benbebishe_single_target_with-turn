addpath(genpath('.'));
params = simulation_params();
rng(params.random_seed + 2e7);
way_A = [127.0, 31.0, 0; 130.0, 34.0, 0];
way_B = [126.5, 33.0, 0; 130.0, 31.0, 0];
way_C = [126.0, 32.5, 0; 131.0, 32.5, 0];
traj_A = aircraft_trajectory_create(way_A, params.aircraft_speed_ms, params.dt_sec);
traj_B = aircraft_trajectory_create(way_B, params.aircraft_speed_ms, params.dt_sec);
traj_C = aircraft_trajectory_create(way_C, params.aircraft_speed_ms, params.dt_sec);
ttA = aircraft_trajectory_interpolate('generate', traj_A);
ttB = aircraft_trajectory_interpolate('generate', traj_B);
ttC = aircraft_trajectory_interpolate('generate', traj_C);
t2_grid = params.time_offset_radar2_sec : params.dt_sec : 2039;

t2 = t2_grid(1);
fprintf('t2(1) = %.0fs\n', t2);

% 检查目标A在t=13s时的位置
tlA = interp1(ttA(:,5), ttA(:,1), t2, 'linear', 'extrap');
tbA = interp1(ttA(:,5), ttA(:,2), t2, 'linear', 'extrap');
fprintf('Target A at t=13s: lon=%.4f lat=%.4f\n', tlA, tbA);

[inA,~,~] = radar_coverage_check(params.radar2_lon, params.radar2_lat, ...
    tlA, tbA, params.radar2_beam_center_deg, params);
fprintf('  in_R2_coverage = %d\n', inA);

% 手动调用generate_frame_detections_multi
truths = {ttA, ttB, ttC};
labels = {'A', 'B', 'C'};
tgt_states = zeros(3, 5);
for ac = 1:3
    tt = truths{ac};
    tl = interp1(tt(:,5), tt(:,1), t2, 'linear', 'extrap');
    tb = interp1(tt(:,5), tt(:,2), t2, 'linear', 'extrap');
    lr = interp1(tt(:,5), tt(:,3), t2, 'linear', 'extrap');
    latr = interp1(tt(:,5), tt(:,4), t2, 'linear', 'extrap');
    [in_cov,~,~] = radar_coverage_check(params.radar2_lon, params.radar2_lat, tl, tb, params.radar2_beam_center_deg, params);
    fprintf('  Target %s: (%.4f,%.4f) in_cov=%d\n', labels{ac}, tl, tb, in_cov);
    tgt_states(ac,:) = [tl, tb, lr, latr, ac];
end
tgt_states = tgt_states(~isnan(tgt_states(:,1)), :);

[detRaw, has_tgt] = generate_frame_detections_multi(params.radar2_lon, params.radar2_lat, ...
    params.radar2_tx_lon, params.radar2_tx_lat, tgt_states, 1, t2, ...
    params.radar2_range_bias_m, params.radar2_azimuth_bias_deg, ...
    params.radar2_beam_center_deg, params, ...
    params.radar2_range_noise_std_m, params.radar2_azimuth_noise_std_deg);

fprintf('\nR2 Frame 1 detections:\n');
fprintf('  has_target_dets: ');
for h = 1:length(has_tgt), fprintf('%d ', has_tgt(h)); end
fprintf('\n');
fprintf('  Total dets: %d\n', length(detRaw));
for i = 1:length(detRaw)
    dp = detRaw(i);
    fprintf('    det%d: lat=%.4f lon=%.4f clutter=%d ac_id=', dp.lat, dp.lon, dp.is_clutter);
    if isfield(dp, 'aircraft_id'), fprintf('%d', dp.aircraft_id); else fprintf('?'); end
    fprintf('\n');
end
