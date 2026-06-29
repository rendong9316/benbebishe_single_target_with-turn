% diag_r1_turn.m — 诊断 R1 转弯附近 UKF 表现
clear; close all; clc;
addpath(genpath('.'));
addpath(genpath('nanyang'));

params = simulation_params();
params.random_seed = 1;
rng(params.random_seed);

[traj, ~] = aircraft_trajectory_create('turn', params);
true_track = aircraft_trajectory_interpolate('generate', traj);
turn_time = traj.segments{2}.t_start;
turn_frame = round(turn_time / params.dt_sec);

t1_grid = params.time_offset_radar1_sec : params.dt_sec : traj.duration_sec;
n_frames = min(length(t1_grid), length(t1_grid));

fprintf('转弯: t=%.0fs, R1帧≈%d\n', turn_time, turn_frame);

% ADS-B标定 (简化快速版)
rng(params.random_seed);
T_adsb = readtable(params.adsb_csv_path, 'ReadVariableNames', false);
dr1_list=[]; da1_list=[];
for idx = 1:max(1,floor(height(T_adsb)/500)):min(5000,height(T_adsb))
    t_lon=T_adsb.Var2(idx); t_lat=T_adsb.Var3(idx);
    if isnan(t_lon)||isnan(t_lat), continue; end
    [in1,~,~]=radar_coverage_check(params.radar1_lon,params.radar1_lat,t_lon,t_lat,params.radar1_beam_center_deg,params);
    if in1
        Rg_true=skywave_geometry('group_range',params.radar1_tx_lon,params.radar1_tx_lat,params.radar1_lon,params.radar1_lat,t_lon,t_lat);
        az_true=sphere_utils_azimuth(params.radar1_lon,params.radar1_lat,t_lon,t_lat);
        Rg_meas=Rg_true+params.radar1_range_bias_m+randn()*params.radar1_range_noise_std_m;
        az_meas=az_true+params.radar1_azimuth_bias_deg+randn()*params.radar1_azimuth_noise_std_deg;
        dr1_list(end+1)=Rg_meas-Rg_true;
        daz=az_meas-az_true; if daz>180,daz=daz-360;elseif daz<-180,daz=daz+360;end
        da1_list(end+1)=daz;
    end
end
dr1_est=mean(dr1_list); da1_est=mean(da1_list);

% 生成点迹
detRaw_R1=cell(n_frames,1);
for k=1:n_frames
    rng(params.random_seed+k);
    [pos,vel]=aircraft_trajectory_interpolate(traj,t1_grid(k));
    detRaw_R1{k}=generate_frame_detections(params.radar1_lon,params.radar1_lat,...
        params.radar1_tx_lon,params.radar1_tx_lat,pos(1),pos(2),vel(1),vel(2),k,t1_grid(k),...
        params.radar1_range_bias_m,params.radar1_azimuth_bias_deg,params.radar1_beam_center_deg,params,...
        params.radar1_range_noise_std_m,params.radar1_azimuth_noise_std_deg);
    for d=1:length(detRaw_R1{k}), detRaw_R1{k}(d).aircraft_id=1; end
end

% 偏差校正
detList_R1=cell(n_frames,1);
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
end

% UKF 配置
params_r1 = params;
params_r1.ukf_range_std_m=params.radar1_range_noise_std_m;
params_r1.ukf_azimuth_std_deg=params.radar1_azimuth_noise_std_deg;
params_r1.ukf_Q_scale=params.radar1_ukf_Q_scale;
params_r1.ukf_P_pos_std=params.radar1_ukf_P_pos_std;
params_r1.ukf_P_vel_std=params.radar1_ukf_P_vel_std;
params_r1.gate_sigma=params.radar1_gate_sigma;
params_r1.tracker_K_loss=params.radar1_tracker_K_loss;
ukf1_tpl=ukf_jichu('create',params_r1,params.radar1_lon,params.radar1_lat,...
    params.radar1_tx_lon,params.radar1_tx_lat,params.dt_sec);

[snaps,~]=single_track_runner_nanyang(detList_R1,ukf1_tpl,params_r1,n_frames);

% 逐帧分析
fprintf('\n=== R1 转弯附近逐帧分析 ===\n');
fprintf('%-5s %-6s %-8s %-8s %-8s %-6s %s\n', '帧','状态','Err(km)','vDir(°)','tDir(°)','nDet','备注');
errs_cal = []; errs_ukf = [];

for k=1:n_frames
    [pos_t,vel_t]=aircraft_trajectory_interpolate(traj,t1_grid(k));
    tdir=atan2d(vel_t(2),vel_t(1));
    snap=snaps{k};

    % 校准RMSE
    for d=1:length(detList_R1{k})
        dp=detList_R1{k}(d);
        if ~dp.is_clutter
            errs_cal(end+1)=sphere_utils_haversine_distance(dp.lon,dp.lat,pos_t(1),pos_t(2))/1000;
        end
    end

    if isempty(snap.trackList)
        if k>=turn_frame-5 && k<=turn_frame+15
            fprintf('%-5d EMPTY\n', k);
        end
        continue;
    end
    trk=snap.trackList{1};
    if trk.type==7
        if k>=turn_frame-5 && k<=turn_frame+15
            fprintf('%-5d WAIT\n', k);
        end
        continue;
    end
    err=NaN; vdir=NaN;
    if ~isnan(trk.lat)
        err=sphere_utils_haversine_distance(trk.lon,trk.lat,pos_t(1),pos_t(2))/1000;
        errs_ukf(end+1)=err;
    end
    if ~isempty(trk.ukf)&&isfield(trk.ukf,'x')&&~isempty(trk.ukf.x)
        vdir=atan2d(trk.ukf.x(4),trk.ukf.x(2));
    end
    note='';
    nd=length(detList_R1{k});
    if trk.missed>0, note=[note sprintf('miss=%d',trk.missed)]; end
    if ~isempty(trk.assoc_det)
        if trk.assoc_det.is_clutter, note=[note ' CLUT']; else, note=[note ' REAL']; end
    else, note=[note ' NODET']; end
    if k>=turn_frame-5 && k<=turn_frame+15
        fprintf('%-5d TRACK  %-8.1f %-8.1f %-8.1f %-6d %s (life=%d)\n',...
            k,err,vdir,tdir,nd,note,trk.life);
    end
end

fprintf('\n=== RMSE对比 ===\n');
fprintf('校准 RMSE: %.1f km (n=%d)\n', sqrt(mean(errs_cal.^2)), length(errs_cal));
fprintf('UKF  RMSE: %.1f km (n=%d)\n', sqrt(mean(errs_ukf.^2)), length(errs_ukf));

% 分段统计
fprintf('\n=== 航迹分段 ===\n');
in_seg=false; seg_start=0; segs=[];
for k=1:n_frames
    snap=snaps{k};
    is_track=~isempty(snap.trackList) && snap.trackList{1}.type==1 && ~isnan(snap.trackList{1}.lat);
    if is_track && ~in_seg
        in_seg=true; seg_start=k;
    elseif ~is_track && in_seg
        in_seg=false;
        segs(end+1,:)=[seg_start, k-1, k-seg_start];
    end
end
if in_seg, segs(end+1,:)=[seg_start, n_frames, n_frames-seg_start+1]; end
for i=1:size(segs,1)
    fprintf('段%d: 帧%d-%d (长度%d)\n', i, segs(i,1), segs(i,2), segs(i,3));
end
fprintf('总段数: %d, MTL: %.1f\n', size(segs,1), mean(segs(:,3)));

fprintf('\nDone.\n');
