% diag_r2_seg.m — 检查 seed=21 R2 snapshot 实际 lat 值
clear; close all; clc; addpath(genpath('.'));
params = simulation_params(); params.random_seed = 21; rng(21);
traj = aircraft_trajectory_create(params.aircraft_waypoints, params.aircraft_speed_ms, params.dt_sec);
true_track = aircraft_trajectory_interpolate('generate', traj);
t1_grid = params.time_offset_radar1_sec : params.dt_sec : traj.duration_sec;
t2_grid = params.time_offset_radar2_sec : params.dt_sec : traj.duration_sec;
n_frames = min(length(t1_grid), length(t2_grid));

T_adsb = readtable(params.adsb_csv_path, 'ReadVariableNames', false);
adsb_lat = T_adsb.Var2; adsb_lon = T_adsb.Var3;
dr1_list=[]; da1_list=[]; dr2_list=[]; da2_list=[];
step = max(1, floor(height(T_adsb)/min(5000,height(T_adsb))));
for idx=1:step:height(T_adsb)
    t_lon=adsb_lon(idx); t_lat=adsb_lat(idx);
    if isnan(t_lon)||isnan(t_lat), continue; end
    [in2,~,~]=radar_coverage_check(params.radar2_lon,params.radar2_lat,t_lon,t_lat,params.radar2_beam_center_deg,params);
    if in2
        Rgt=skywave_geometry('group_range',params.radar2_tx_lon,params.radar2_tx_lat,params.radar2_lon,params.radar2_lat,t_lon,t_lat);
        azt=sphere_utils_azimuth(params.radar2_lon,params.radar2_lat,t_lon,t_lat);
        Rgm=Rgt+params.radar2_range_bias_m+randn()*params.radar2_range_noise_std_m;
        azm=azt+params.radar2_azimuth_bias_deg+randn()*params.radar2_azimuth_noise_std_deg;
        dr2_list(end+1)=Rgm-Rgt; daz=azm-azt;
        if daz>180,daz=daz-360; elseif daz<-180,daz=daz+360; end
        da2_list(end+1)=daz;
    end
    [in1,~,~]=radar_coverage_check(params.radar1_lon,params.radar1_lat,t_lon,t_lat,params.radar1_beam_center_deg,params);
    if in1
        Rgt=skywave_geometry('group_range',params.radar1_tx_lon,params.radar1_tx_lat,params.radar1_lon,params.radar1_lat,t_lon,t_lat);
        azt=sphere_utils_azimuth(params.radar1_lon,params.radar1_lat,t_lon,t_lat);
        Rgm=Rgt+params.radar1_range_bias_m+randn()*params.radar1_range_noise_std_m;
        azm=azt+params.radar1_azimuth_bias_deg+randn()*params.radar1_azimuth_noise_std_deg;
        dr1_list(end+1)=Rgm-Rgt; daz=azm-azt;
        if daz>180,daz=daz-360; elseif daz<-180,daz=daz+360; end
        da1_list(end+1)=daz;
    end
end
dr2_est=mean(dr2_list); da2_est=mean(da2_list); dr1_est=mean(dr1_list); da1_est=mean(da1_list);
detList_R2=cell(n_frames,1);
for k=1:n_frames
    rng(21+10000+k);
    [pos2,vel2]=aircraft_trajectory_interpolate(traj,t2_grid(k));
    detRaw2=generate_frame_detections(params.radar2_lon,params.radar2_lat,params.radar2_tx_lon,params.radar2_tx_lat,pos2(1),pos2(2),vel2(1),vel2(2),k,t2_grid(k),params.radar2_range_bias_m,params.radar2_azimuth_bias_deg,params.radar2_beam_center_deg,params,params.radar2_range_noise_std_m,params.radar2_azimuth_noise_std_deg);
    for d=1:length(detRaw2)
        detRaw2(d).aircraft_id=1; Rgc=detRaw2(d).prange-dr2_est; azc=detRaw2(d).paz-da2_est;
        detRaw2(d).drange=Rgc; detRaw2(d).daz=azc; detRaw2(d).range_meas=Rgc; detRaw2(d).azimuth_meas=azc;
        if ~(isfield(detRaw2(d),'lat')&&~isnan(detRaw2(d).lat))
            [~,lat_e,lon_e]=bistatic_inverse_solver(Rgc,azc,params.radar2_tx_lon,params.radar2_tx_lat,params.radar2_lon,params.radar2_lat);
            detRaw2(d).lat=lat_e; detRaw2(d).lon=lon_e;
        end
    end
    detList_R2{k}=detRaw2;
end

params_r2=params;
params_r2.ukf_range_std_m=params.radar2_range_noise_std_m; params_r2.ukf_azimuth_std_deg=params.radar2_azimuth_noise_std_deg;
params_r2.gate_sigma=params.radar2_gate_sigma; params_r2.gate_vr_ms=params.radar2_gate_vr_ms;
params_r2.ukf_Q_scale=params.radar2_ukf_Q_scale; params_r2.ukf_P_pos_std=params.radar2_ukf_P_pos_std; params_r2.ukf_P_vel_std=params.radar2_ukf_P_vel_std;
params_r2.tracker_M=4; params_r2.tracker_N=8; params_r2.tracker_K_loss=params.radar2_tracker_K_loss;
ukf2_tpl=ukf_jichu('create',params_r2,params.radar2_lon,params.radar2_lat,params.radar2_tx_lon,params.radar2_tx_lat,params.dt_sec);
[snaps_R2,~]=single_track_runner(detList_R2,ukf2_tpl,params_r2,n_frames,true_track,t2_grid);

fprintf('===== R2 每帧 lat/lon (seed=21) =====\n');
fprintf('%-4s %-5s %-12s %-12s %-8s\n', '帧','type','lat','lon','isnan?');
for k=1:n_frames
    if ~isempty(snaps_R2{k}) && ~isempty(snaps_R2{k}.trackList)
        trk=snaps_R2{k}.trackList{1};
        fprintf('%-4d %-5d %-12.4f %-12.4f %-8d\n', k, trk.type, trk.lat, trk.lon, isnan(trk.lat));
    else
        fprintf('%-4d EMPTY\n', k);
    end
end

% 统计连续有效段
fprintf('\n===== 连续有效段 =====\n');
in_seg=false; seg_start=0; seg_count=0;
for k=1:n_frames
    valid=false;
    if ~isempty(snaps_R2{k})&&~isempty(snaps_R2{k}.trackList)
        trk=snaps_R2{k}.trackList{1};
        if trk.type==1 && ~isnan(trk.lat), valid=true; end
    end
    if valid && ~in_seg
        in_seg=true; seg_start=k;
    elseif ~valid && in_seg
        in_seg=false; seg_count=seg_count+1;
        fprintf('  段%d: [%d-%d] 共%d帧\n', seg_count, seg_start, k-1, k-seg_start);
    end
end
if in_seg
    seg_count=seg_count+1;
    fprintf('  段%d: [%d-%d] 共%d帧\n', seg_count, seg_start, n_frames, n_frames-seg_start+1);
end

fprintf('\nDone.\n');
