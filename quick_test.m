% quick_test.m — 单次运行快速测试
clear; close all; clc;
addpath(genpath('.'));
addpath(genpath('nanyang'));

params = simulation_params();
params.random_seed = 1;
rng(params.random_seed);

[traj, ~] = aircraft_trajectory_create('turn', params);
true_track = aircraft_trajectory_interpolate('generate', traj);

t1_grid = params.time_offset_radar1_sec : params.dt_sec : traj.duration_sec;
t2_grid = params.time_offset_radar2_sec : params.dt_sec : traj.duration_sec;
n_frames = min(length(t1_grid), length(t2_grid));

% ADS-B 标定 (简化)
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

% 点迹生成
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

% 偏差校正 + 反解
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

% UKF 模板
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

% 运行基础 UKF
fprintf('=== 运行基础 UKF ===\n');
[snaps_R1, finalTrk1] = single_track_runner_nanyang(detList_R1, ukf1_tpl, params_r1, n_frames);
[snaps_R2, finalTrk2] = single_track_runner_nanyang(detList_R2, ukf2_tpl, params_r2, n_frames);

% 计算 RMSE
errs = [];
for k=1:n_frames
    tl = interp1(true_track(:,5), true_track(:,1), t1_grid(k), 'linear', 'extrap');
    tb = interp1(true_track(:,5), true_track(:,2), t1_grid(k), 'linear', 'extrap');
    snap = snaps_R1{k};
    if ~isempty(snap.trackList)
        trk = snap.trackList{1};
        if trk.type ~= 7 && ~isnan(trk.lat)
            errs(end+1) = sphere_utils_haversine_distance(trk.lon, trk.lat, tl, tb) / 1000;
        end
    end
end
fprintf('R1 基础UKF RMSE: %.1f km\n', sqrt(mean(errs.^2)));

errs = [];
for k=1:n_frames
    tl = interp1(true_track(:,5), true_track(:,1), t2_grid(k), 'linear', 'extrap');
    tb = interp1(true_track(:,5), true_track(:,2), t2_grid(k), 'linear', 'extrap');
    snap = snaps_R2{k};
    if ~isempty(snap.trackList)
        trk = snap.trackList{1};
        if trk.type ~= 7 && ~isnan(trk.lat)
            errs(end+1) = sphere_utils_haversine_distance(trk.lon, trk.lat, tl, tb) / 1000;
        end
    end
end
fprintf('R2 基础UKF RMSE: %.1f km\n', sqrt(mean(errs.^2)));

% 段长统计
segs1 = extract_segments(snaps_R1, n_frames);
segs2 = extract_segments(snaps_R2, n_frames);
fprintf('R1 段数=%d MTL=%.1f\n', size(segs1,1), compute_mtl(segs1));
fprintf('R2 段数=%d MTL=%.1f\n', size(segs2,1), compute_mtl(segs2));
if ~isempty(segs1)
    for i=1:size(segs1,1)
        fprintf('  R1段%d: 帧%d-%d (长度%d)\n', i, segs1(i,1), segs1(i,2), segs1(i,3));
    end
end
if ~isempty(segs2)
    for i=1:size(segs2,1)
        fprintf('  R2段%d: 帧%d-%d (长度%d)\n', i, segs2(i,1), segs2(i,2), segs2(i,3));
    end
end

fprintf('type_R1=%d life_R1=%d  type_R2=%d life_R2=%d\n', ...
    finalTrk1.type, finalTrk1.life, finalTrk2.type, finalTrk2.life);
fprintf('\nDone.\n');

% ---- 局部函数 ----
function segs = extract_segments(snaps, n_frames)
    segs = [];
    in_seg = false;  seg_start = 0;
    for k = 1:n_frames
        is_tracking = false;
        if ~isempty(snaps{k}) && ~isempty(snaps{k}.trackList)
            trk = snaps{k}.trackList{1};
            if trk.type == 1 && ~isnan(trk.lat)
                is_tracking = true;
            end
        end
        if is_tracking && ~in_seg
            in_seg = true;  seg_start = k;
        elseif ~is_tracking && in_seg
            in_seg = false;
            segs(end+1, :) = [seg_start, k-1, k - seg_start];
        end
    end
    if in_seg
        segs(end+1, :) = [seg_start, n_frames, n_frames - seg_start + 1];
    end
end

function v = compute_mtl(segs)
    if isempty(segs), v = 0;
    else, v = mean(segs(:,3)); end
end
