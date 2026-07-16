addpath(genpath(pwd));
params = simulation_params();
params.trajectory_mode = 'uturn';
params.n_targets = 1;
params.random_seed = 1;
rng(1);

traj = aircraft_trajectory_create('uturn', params);
tt = aircraft_trajectory_interpolate('generate', traj);
t1 = params.time_offset_radar1_sec : params.dt_sec : traj.duration_sec;
nf = length(t1);
fprintf('Duration=%.0f, Frames=%d\n', traj.duration_sec, nf);
fprintf('Waypoints: [%.2f,%.2f]->[%.2f,%.2f]->[%.2f,%.2f]\n', ...
    traj.waypoints(1,1),traj.waypoints(1,2), ...
    traj.waypoints(2,1),traj.waypoints(2,2), ...
    traj.waypoints(3,1),traj.waypoints(3,2));

params.ukf_Q_scale = params.radar1_ukf_Q_scale;
params.ukf_P_pos_std = params.radar1_ukf_P_pos_std;
params.ukf_P_vel_std = params.radar1_ukf_P_vel_std;
params.ukf_range_std_m = params.radar1_range_noise_std_m;
params.ukf_azimuth_std_deg = params.radar1_azimuth_noise_std_deg;
params.gate_sigma = params.radar1_gate_sigma;
params.gate_vr_ms = params.radar1_gate_vr_ms;
params.tracker_K_loss = params.radar1_tracker_K_loss;
params.multi_truth_init_enable = true;
params.detection_probability = 0.6;

ukf1_tpl = ukf_imm('create', params, params.radar1_lon, params.radar1_lat, ...
    params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
truth_all_cell = {tt};

trackList = {}; tempPool = {}; next_id = 1;
for k = 1:nf
    tgt_states = [tt(k,1), tt(k,2), tt(k,3), tt(k,4), 1];
    dets = generate_frame_detections_multi(params.radar1_lon, params.radar1_lat, ...
        params.radar1_tx_lon, params.radar1_tx_lat, tgt_states, k, t1(k), ...
        params.radar1_range_bias_m, params.radar1_azimuth_bias_deg, ...
        params.radar1_beam_center_deg, params, ...
        params.radar1_range_noise_std_m, params.radar1_azimuth_noise_std_deg);
    dets = augment_dets(dets, 0, 0, params.radar1_tx_lon, params.radar1_tx_lat, ...
        params.radar1_lon, params.radar1_lat);
    [trackList, tempPool, snap, next_id] = multi_track_runner_kf(trackList, tempPool, dets, ...
        ukf1_tpl, params, k, next_id, truth_all_cell, t1);
    
    if k <= 3 || k == 10 || k == 20 || k == 30 || k == 40 || k == 50 || k == 60 || k == 70 || ...
       k == 80 || k == 90 || k == 100 || k == 110 || k == 120 || k == nf
        for t = 1:length(trackList)
            trk = trackList{t};
            mu = '';
            if isfield(trk.ukf, 'mu') && ~isempty(trk.ukf.mu)
                mu = sprintf(' mu=[%.2f %.2f]', trk.ukf.mu(1), trk.ukf.mu(2));
            end
            true_lon = tt(k,1); true_lat = tt(k,2);
            d = sphere_utils_haversine_distance(trk.lon, trk.lat, true_lon, true_lat) / 1000;
            fprintf('  Frame %3d: id=%d type=%d lat=%.4f lon=%.4f RMSE=%.1fkm%s\n', ...
                k, trk.id, trk.type, trk.lat, trk.lon, d, mu);
        end
        if length(trackList) == 0
            fprintf('  Frame %3d: 0 tracks!!!\n', k);
        end
    end
end
