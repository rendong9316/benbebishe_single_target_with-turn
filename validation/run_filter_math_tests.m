function run_filter_math_tests()
    test_ukf_sigma_measurement_mean();
    test_imm_prediction_weights();
    test_tracking_error_time_grids();
    disp('filter math tests ok');
end

function test_ukf_sigma_measurement_mean()
    params = radar_params(simulation_params_oracle(), 1);
    ukf = ukf_jichu('create', params, params.radar1_lon, params.radar1_lat, ...
        params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
    det1 = fixture_detection(1, 0, 128.0, 31.0, ukf);
    det2 = fixture_detection(3, 60, 128.1, 31.05, ukf);
    ukf = ukf_jichu('init', ukf, det1, det2);

    [~, ~, ~, z_pred, Z_pred, P_zz, ukf] = ukf_jichu('prepare', ukf);
    expected_linear = Z_pred([1, 3], :) * ukf.Wm;
    assert(abs(z_pred(1) - expected_linear(1)) < 1e-6);
    assert(abs(z_pred(3) - expected_linear(2)) < 1e-9);
    assert(all(isfinite(P_zz(:))));
    assert(norm(P_zz - P_zz', 'fro') < 1e-8 * max(1, norm(P_zz, 'fro')));

    Wm = ukf.Wm;
    angles = repmat(359.9, size(Wm));
    angles(2:5) = 0.1;
    angles(6:9) = 359.7;
    mean_a = local_angle_mean(angles, Wm);
    assert(abs(local_wrap(mean_a)) < 0.2);

    shifted = angles;
    shifted(2:5) = shifted(2:5) + 360;
    mean_shifted = local_angle_mean(shifted, Wm);
    assert(abs(local_wrap(mean_a - mean_shifted)) < 1e-9);

    residuals = arrayfun(@(a) local_wrap(a - mean_a), angles);
    assert(all(abs(residuals) <= 180));
end

function test_imm_prediction_weights()
    params = radar_params(simulation_params_oracle(), 1);
    imm = ukf_imm('create', params, params.radar1_lon, params.radar1_lat, ...
        params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
    det1 = fixture_detection(1, 0, 128.0, 31.0, imm.ukf_cv);
    det2 = fixture_detection(3, 60, 128.1, 31.05, imm.ukf_cv);
    imm = ukf_imm('init', imm, det1, det2);

    imm.mu = [0.85; 0.15];
    imm.Pi = [0.65, 0.35; 0.10, 0.90];
    c_bar = imm.Pi' * imm.mu;
    assert(norm(c_bar - imm.mu) > 0.1);

    [x_pred, P_pred, ~, z_pred, ~, P_zz, imm] = ukf_imm('prepare', imm);
    cache = imm.cache;
    expected_x = c_bar(1) * cache.x_pred_cv + c_bar(2) * cache.x_pred_ct;
    old_x = imm.mu(1) * cache.x_pred_cv + imm.mu(2) * cache.x_pred_ct;
    expected_z = c_bar(1) * cache.z_pred_cv + c_bar(2) * cache.z_pred_ct;
    assert(norm(x_pred - expected_x) < 1e-12);
    assert(norm(x_pred - old_x) > 1e-10);
    assert(norm(z_pred - expected_z) < 1e-8);
    assert(abs(sum(cache.c_bar) - 1) < 1e-12);
    assert(all(isfinite(P_pred(:))) && all(isfinite(P_zz(:))));
    assert(norm(P_pred - P_pred', 'fro') < 1e-8 * max(1, norm(P_pred, 'fro')));
    assert(norm(P_zz - P_zz', 'fro') < 1e-8 * max(1, norm(P_zz, 'fro')));

    imm_identity = ukf_imm('create', params, params.radar1_lon, params.radar1_lat, ...
        params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
    imm_identity = ukf_imm('init', imm_identity, det1, det2);
    imm_identity.mu = [0.85; 0.15];
    imm_identity.Pi = eye(2);
    [x_identity, ~, ~, ~, ~, ~, imm_identity] = ukf_imm('prepare', imm_identity);
    expected_identity = imm_identity.mu(1) * imm_identity.cache.x_pred_cv + ...
        imm_identity.mu(2) * imm_identity.cache.x_pred_ct;
    assert(norm(x_identity - expected_identity) < 1e-12);
end

function test_tracking_error_time_grids()
    truth_times = [0, 13, 30, 43, 60, 73];
    truth_lon = 128 + 0.001 * truth_times;
    truth_lat = 31 + 0.0005 * truth_times;
    truth = {struct('time_sec', truth_times, 'lon', truth_lon, ...
        'lat', truth_lat, 'label', 'A')};

    snapshot_times = [13, 43, 73];
    detection_times = [13, 43, 73];
    snapshots = cell(3, 1);
    detList = cell(3, 1);
    for k = 1:3
        lon = 128 + 0.001 * snapshot_times(k);
        lat = 31 + 0.0005 * snapshot_times(k);
        trk = struct('id', 1, 'type', 1, 'truth_idx', 1, ...
            'lon', lon, 'lat', lat);
        snapshots{k} = struct('frameID', k, 'trackList', {{trk}});
        detList{k} = struct('aircraft_id', int32(1), 'is_clutter', false, ...
            'lon', lon, 'lat', lat, 'raw_lon', lon, 'raw_lat', lat);
    end

    stats = evaluate_all_multi('tracking_errors', snapshots, detList, truth, ...
        snapshot_times, detection_times, 'R2');
    assert(stats.overall.ukf.rms < 1e-9);
    assert(stats.overall.det.rms < 1e-9);
    assert(isequal(stats.snapshot_times, snapshot_times));
    assert(isequal(stats.detection_times, detection_times));

    aligned_times = [0, 30, 60];
    aligned_snapshots = cell(3, 1);
    for k = 1:3
        lon = 128 + 0.001 * aligned_times(k);
        lat = 31 + 0.0005 * aligned_times(k);
        trk = struct('id', 1, 'type', 1, 'truth_idx', 1, ...
            'lon', lon, 'lat', lat);
        aligned_snapshots{k} = struct('frameID', k, 'trackList', {{trk}});
    end
    mixed_stats = evaluate_all_multi('tracking_errors', aligned_snapshots, ...
        detList, truth, aligned_times, detection_times, 'R2_aligned');
    assert(mixed_stats.overall.ukf.rms < 1e-9);
    assert(mixed_stats.overall.det.rms < 1e-9);

    assert_throws_time_grid(@() evaluate_all_multi('tracking_errors', ...
        snapshots, detList, truth, [13, 43], detection_times, 'bad'));
    assert_throws_time_grid(@() evaluate_all_multi('tracking_errors', ...
        snapshots, detList, truth, [13, NaN, 73], detection_times, 'bad'));
    assert_throws_time_grid(@() evaluate_all_multi('tracking_errors', ...
        snapshots, detList, truth, [13, 13, 73], detection_times, 'bad'));
end

function assert_throws_time_grid(fn)
    failed = false;
    try
        fn();
    catch exception
        failed = strcmp(exception.identifier, 'evaluate_all_multi:timeGridLength') || ...
            strcmp(exception.identifier, 'evaluate_all_multi:invalidTimeGrid');
    end
    assert(failed);
end

function det = fixture_detection(frame_id, time_sec, lon, lat, ukf)
    x = [lon; 0.001; lat; 0.0005];
    z = ukf_jichu('measurement', ukf, x);
    det = struct('frameID', frame_id, 'time_sec', time_sec, ...
        'range_meas', z(1), 'azimuth_meas', z(2), ...
        'drange', z(1), 'daz', z(2), 'radial_vel_meas', z(3), ...
        'pvr', z(3), 'lat', lat, 'lon', lon, ...
        'aircraft_id', int32(1), 'is_clutter', false);
end

function mean_angle = local_angle_mean(angles, weights)
    ref = angles(1);
    delta = arrayfun(@(a) local_wrap(a - ref), angles);
    mean_angle = mod(ref + delta' * weights, 360);
end

function angle = local_wrap(angle)
    angle = mod(angle + 180, 360) - 180;
end
