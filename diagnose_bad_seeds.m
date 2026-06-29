% =========================================================================
% diagnose_bad_seeds.m — 逐帧分析坏种子的发散原因
% 直接调用真实 runner，不复制逻辑，完全对齐 MC
% =========================================================================
clear; close all; clc;
addpath(genpath('.'));
addpath(genpath('nanyang'));

% 选取代表性坏种子
% R2发散型: 7, 12 (R1好, R2坏)
% R1发散型: 20, 21 (R1坏, R2好)
% 双站发散: 39
% 极端R1发散: 116 (R1=182.6, type=H)
% 极端R2发散: 182 (R2=191.7, R1好)
bad_seeds_to_check = [7, 12, 20, 21, 39, 116, 182];

for si = 1:length(bad_seeds_to_check)
    seed = bad_seeds_to_check(si);
    diagnose_single_seed(seed);
end

fprintf('\n===== 诊断完成 =====\n');

% =========================================================================
function diagnose_single_seed(seed)
% =========================================================================
    fprintf('\n############################################################\n');
    fprintf('##### 诊断种子 %d\n', seed);
    fprintf('############################################################\n');

    params = simulation_params();
    params.random_seed = seed;
    rng(params.random_seed);

    [traj, ~] = aircraft_trajectory_create('turn', params);
    true_track_full = aircraft_trajectory_interpolate('generate', traj);

    t1_grid = params.time_offset_radar1_sec : params.dt_sec : traj.duration_sec;
    t2_grid = params.time_offset_radar2_sec : params.dt_sec : traj.duration_sec;
    n_frames = min(length(t1_grid), length(t2_grid));

    turn_start = traj.segments{2}.t_start;
    fprintf('转弯时刻: t=%.0fs (R1帧~%d, R2帧~%d)\n', ...
        turn_start, round(turn_start/params.dt_sec), ...
        round((turn_start - params.time_offset_radar2_sec)/params.dt_sec));

    % ---- ADS-B 标定 (完全对齐 MC run_one) ----
    rng(params.random_seed);
    T_adsb = readtable(params.adsb_csv_path, 'ReadVariableNames', false);
    adsb_lat = T_adsb.Var2; adsb_lon = T_adsb.Var3;
    dr1_list=[]; da1_list=[]; dr2_list=[]; da2_list=[];
    n_check = min(5000, height(T_adsb));
    cal_step = max(1, floor(height(T_adsb)/n_check));
    for idx = 1:cal_step:height(T_adsb)
        t_lon=adsb_lon(idx); t_lat=adsb_lat(idx);
        if isnan(t_lon)||isnan(t_lat), continue; end
        [in1,~,~]=radar_coverage_check(params.radar1_lon,params.radar1_lat,t_lon,t_lat,params.radar1_beam_center_deg,params);
        if in1
            Rg_true=skywave_geometry('group_range',params.radar1_tx_lon,params.radar1_tx_lat,params.radar1_lon,params.radar1_lat,t_lon,t_lat);
            az_true=sphere_utils_azimuth(params.radar1_lon,params.radar1_lat,t_lon,t_lat);
            Rg_meas=Rg_true+params.radar1_range_bias_m+randn()*params.radar1_range_noise_std_m;
            az_meas=az_true+params.radar1_azimuth_bias_deg+randn()*params.radar1_azimuth_noise_std_deg;
            dr1_list(end+1)=Rg_meas-Rg_true;
            daz=az_meas-az_true;
            if daz>180,daz=daz-360;elseif daz<-180,daz=daz+360;end
            da1_list(end+1)=daz;
        end
        [in2,~,~]=radar_coverage_check(params.radar2_lon,params.radar2_lat,t_lon,t_lat,params.radar2_beam_center_deg,params);
        if in2
            Rg_true=skywave_geometry('group_range',params.radar2_tx_lon,params.radar2_tx_lat,params.radar2_lon,params.radar2_lat,t_lon,t_lat);
            az_true=sphere_utils_azimuth(params.radar2_lon,params.radar2_lat,t_lon,t_lat);
            Rg_meas=Rg_true+params.radar2_range_bias_m+randn()*params.radar2_range_noise_std_m;
            az_meas=az_true+params.radar2_azimuth_bias_deg+randn()*params.radar2_azimuth_noise_std_deg;
            dr2_list(end+1)=Rg_meas-Rg_true;
            daz=az_meas-az_true;
            if daz>180,daz=daz-360;elseif daz<-180,daz=daz+360;end
            da2_list(end+1)=daz;
        end
    end
    dr1_est=mean(dr1_list); da1_est=mean(da1_list);
    dr2_est=mean(dr2_list); da2_est=mean(da2_list);

    % ---- 生成检测 (完全对齐 MC) ----
    detRaw_R1=cell(n_frames,1); detRaw_R2=cell(n_frames,1);
    for k=1:n_frames
        rng(params.random_seed+k);
        [pos,vel]=aircraft_trajectory_interpolate(traj,t1_grid(k));
        detRaw_R1{k}=generate_frame_detections(params.radar1_lon,params.radar1_lat,...
            params.radar1_tx_lon,params.radar1_tx_lat,pos(1),pos(2),vel(1),vel(2),k,t1_grid(k),...
            params.radar1_range_bias_m,params.radar1_azimuth_bias_deg,params.radar1_beam_center_deg,params,...
            params.radar1_range_noise_std_m,params.radar1_azimuth_noise_std_deg);
        for d=1:length(detRaw_R1{k}), detRaw_R1{k}(d).aircraft_id=1; end

        rng(params.random_seed+10000+k);
        [pos2,vel2]=aircraft_trajectory_interpolate(traj,t2_grid(k));
        detRaw_R2{k}=generate_frame_detections(params.radar2_lon,params.radar2_lat,...
            params.radar2_tx_lon,params.radar2_tx_lat,pos2(1),pos2(2),vel2(1),vel2(2),k,t2_grid(k),...
            params.radar2_range_bias_m,params.radar2_azimuth_bias_deg,params.radar2_beam_center_deg,params,...
            params.radar2_range_noise_std_m,params.radar2_azimuth_noise_std_deg);
        for d=1:length(detRaw_R2{k}), detRaw_R2{k}(d).aircraft_id=1; end
    end

    % ---- 标定+反解 ----
    detList_R1=cell(n_frames,1); detList_R2=cell(n_frames,1);
    for k=1:n_frames
        dets=detRaw_R1{k};
        for d=1:length(dets)
            dets(d).drange=dets(d).prange-dr1_est; dets(d).daz=dets(d).paz-da1_est;
            dets(d).range_meas=dets(d).drange; dets(d).azimuth_meas=dets(d).daz;
            if ~(isfield(dets(d),'lat')&&~isnan(dets(d).lat))
                [~,lat_e,lon_e]=bistatic_inverse_solver(dets(d).drange,dets(d).daz,...
                    params.radar1_tx_lon,params.radar1_tx_lat,params.radar1_lon,params.radar1_lat);
                dets(d).lat=lat_e; dets(d).lon=lon_e;
            end
        end
        detList_R1{k}=dets;

        dets=detRaw_R2{k};
        for d=1:length(dets)
            dets(d).drange=dets(d).prange-dr2_est; dets(d).daz=dets(d).paz-da2_est;
            dets(d).range_meas=dets(d).drange; dets(d).azimuth_meas=dets(d).daz;
            if ~(isfield(dets(d),'lat')&&~isnan(dets(d).lat))
                [~,lat_e,lon_e]=bistatic_inverse_solver(dets(d).drange,dets(d).daz,...
                    params.radar2_tx_lon,params.radar2_tx_lat,params.radar2_lon,params.radar2_lat);
                dets(d).lat=lat_e; dets(d).lon=lon_e;
            end
        end
        detList_R2{k}=dets;
    end

    % ---- 建 UKF 模板 ----
    params_r1 = params;
    params_r1.ukf_range_std_m=params.radar1_range_noise_std_m;
    params_r1.ukf_azimuth_std_deg=params.radar1_azimuth_noise_std_deg;
    params_r1.ukf_Q_scale=params.radar1_ukf_Q_scale;
    params_r1.ukf_P_pos_std=params.radar1_ukf_P_pos_std;
    params_r1.ukf_P_vel_std=params.radar1_ukf_P_vel_std;
    params_r1.gate_sigma=params.radar1_gate_sigma;
    params_r1.tracker_K_loss=params.radar1_tracker_K_loss;

    params_r2 = params;
    params_r2.ukf_range_std_m=params.radar2_range_noise_std_m;
    params_r2.ukf_azimuth_std_deg=params.radar2_azimuth_noise_std_deg;
    params_r2.gate_sigma=params.radar2_gate_sigma;
    params_r2.ukf_Q_scale=params.radar2_ukf_Q_scale;
    params_r2.ukf_P_pos_std=params.radar2_ukf_P_pos_std;
    params_r2.ukf_P_vel_std=params.radar2_ukf_P_vel_std;
    params_r2.tracker_M=4; params_r2.tracker_N=8;
    params_r2.tracker_K_loss=params.radar2_tracker_K_loss;

    ukf1_tpl = ukf_jichu('create',params_r1,params.radar1_lon,params.radar1_lat,...
        params.radar1_tx_lon,params.radar1_tx_lat,params.dt_sec);
    ukf2_tpl = ukf_jichu('create',params_r2,params.radar2_lon,params.radar2_lat,...
        params.radar2_tx_lon,params.radar2_tx_lat,params.dt_sec);

    % ---- 跑两个 runner ----
    [snaps_R1, finalTrk1] = single_track_runner_nanyang(detList_R1, ukf1_tpl, params_r1, n_frames);
    [snaps_R2, finalTrk2] = single_track_runner_nanyang(detList_R2, ukf2_tpl, params_r2, n_frames);

    % ---- 整体 RMSE ----
    rmse_R1 = compute_rmse(snaps_R1, true_track_full, t1_grid, n_frames);
    rmse_R2 = compute_rmse(snaps_R2, true_track_full, t2_grid, n_frames);
    fprintf('整体 RMSE: R1=%.1f km, R2=%.1f km\n', rmse_R1, rmse_R2);

    % ---- 分析哪个站发散 ----
    if rmse_R1 > 30, analyze_station('R1', snaps_R1, detList_R1, true_track_full, t1_grid, n_frames, traj, params);
    end
    if rmse_R2 > 30, analyze_station('R2', snaps_R2, detList_R2, true_track_full, t2_grid, n_frames, traj, params);
    end
end

% =========================================================================
function analyze_station(name, snaps, detList, true_track, t_grid, n_frames, traj, params)
% =========================================================================
    turn_start = traj.segments{2}.t_start;

    % 统计每帧状态
    fprintf('\n--- %s 逐帧分析 (转弯 t=%.0fs) ---\n', name, turn_start);
    fprintf('%-5s %-9s %-10s %-10s %-10s %-6s %-30s\n', ...
        '帧', '状态', 'Err(km)', 'vDir(°)', 'TrueDir', 'nDet', '备注');

    last_state = ''; state_start = 0; errs_in_phase = [];

    for k = 1:n_frames
        [pos_true, vel_true] = aircraft_trajectory_interpolate(traj, t_grid(k));
        true_lon = pos_true(1); true_lat = pos_true(2);
        true_vdir = atan2d(vel_true(2), vel_true(1));

        snap = snaps{k};
        n_det = length(detList{k});

        if isempty(snap.trackList)
            state = 'EMPTY';
            err = NaN; vdir = NaN; note = '';
        else
            trk = snap.trackList{1};
            if trk.type == 7
                state = 'WAITING';
                err = NaN; vdir = NaN; note = '';
            else
                state = 'TRACK';
                if ~isnan(trk.lat)
                    err = sphere_utils_haversine_distance(trk.lon, trk.lat, true_lon, true_lat) / 1000;
                else
                    err = NaN;
                end
                if ~isempty(trk.ukf) && isfield(trk.ukf,'x') && ~isempty(trk.ukf.x)
                    vdir = atan2d(trk.ukf.x(4), trk.ukf.x(2));
                else
                    vdir = NaN;
                end
                note = '';
                if err > 30 && ~isnan(err), note = '!!LARGE_ERR'; end
                if trk.missed > 0, note = [note sprintf(' miss=%d', trk.missed)]; end
                if ~isempty(trk.assoc_det)
                    if trk.assoc_det.is_clutter, note = [note ' assoc=CLUTTER']; end
                else
                    note = [note ' NO_DET'];
                end
            end
        end

        in_turn = (t_grid(k) >= turn_start-60 && t_grid(k) <= turn_start+180);

        % 打印转弯附近或大误差帧
        if in_turn || (err > 20 && ~isnan(err))
            fprintf('%-5d %-9s %-10.1f %-10.1f %-10.1f %-6d %-30s\n', ...
                k, state, err, vdir, true_vdir, n_det, note);
        end

        % 跟踪阶段切换
        if ~strcmp(state, last_state) && k > 1
            if strcmp(last_state, 'TRACK') && ~isempty(errs_in_phase)
                % 刚结束一段TRACK
            end
            last_state = state; state_start = k;
            errs_in_phase = [];
        end
        if strcmp(state, 'TRACK') && ~isnan(err)
            errs_in_phase(end+1) = err;
        end
    end
end

% =========================================================================
function v = compute_rmse(snaps, true_track, t_grid, n_frames)
    errs = [];
    for k = 1:n_frames
        tl = interp1(true_track(:,5), true_track(:,1), t_grid(k), 'linear', 'extrap');
        tb = interp1(true_track(:,5), true_track(:,2), t_grid(k), 'linear', 'extrap');
        snap = snaps{k};
        if ~isempty(snap.trackList)
            trk = snap.trackList{1};
            if trk.type ~= 7 && ~isnan(trk.lat)
                errs(end+1) = sphere_utils_haversine_distance(trk.lon, trk.lat, tl, tb) / 1000;
            end
        end
    end
    if isempty(errs), v = NaN; else, v = sqrt(mean(errs.^2)); end
end
