% fusion_dist.m — 跑 K_loss=6 配置 200 种子，统计融合 RMSE 分布
clear; close all; clc; addpath(genpath('.'));
fprintf('K_loss=6 fusion RMSE distribution, N=200...\n'); tic;

N=200; fus_rmse=nan(N,1); r1_rmse=nan(N,1); r2_rmse=nan(N,1);
for mc=1:N
    seed=mc;
    params=simulation_params(); params.random_seed=seed;
    params.radar1_tracker_K_loss=6; params.tracker_K_loss=6;
    rng(seed);
    traj=aircraft_trajectory_create(params.aircraft_waypoints,params.aircraft_speed_ms,params.dt_sec);
    tt=aircraft_trajectory_interpolate('generate',traj);
    t1=params.time_offset_radar1_sec:params.dt_sec:traj.duration_sec;
    t2=params.time_offset_radar2_sec:params.dt_sec:traj.duration_sec;
    nf=min(length(t1),length(t2));

    % Phase 1: ADS-B calibration
    rng(seed);
    T=readtable(params.adsb_csv_path,'ReadVariableNames',false);
    al=T.Var2; ao=T.Var3;
    d1=[]; da1=[]; d2=[]; da2=[];
    nc=min(5000,height(T)); cs=max(1,floor(height(T)/nc));
    for idx=1:cs:height(T)
        tl2=ao(idx); tb2=al(idx);
        if isnan(tl2)||isnan(tb2), continue; end
        [in1,~,~]=radar_coverage_check(params.radar1_lon,params.radar1_lat,tl2,tb2,params.radar1_beam_center_deg,params);
        if in1
            Rg=skywave_geometry('group_range',params.radar1_tx_lon,params.radar1_tx_lat,params.radar1_lon,params.radar1_lat,tl2,tb2);
            az=sphere_utils_azimuth(params.radar1_lon,params.radar1_lat,tl2,tb2);
            Rm=Rg+params.radar1_range_bias_m+randn()*params.radar1_range_noise_std_m;
            am=az+params.radar1_azimuth_bias_deg+randn()*params.radar1_azimuth_noise_std_deg;
            d1(end+1)=Rm-Rg; daz=am-az;
            if daz>180,daz=daz-360;elseif daz<-180,daz=daz+360;end;da1(end+1)=daz;
        end
        [in2,~,~]=radar_coverage_check(params.radar2_lon,params.radar2_lat,tl2,tb2,params.radar2_beam_center_deg,params);
        if in2
            Rg=skywave_geometry('group_range',params.radar2_tx_lon,params.radar2_tx_lat,params.radar2_lon,params.radar2_lat,tl2,tb2);
            az=sphere_utils_azimuth(params.radar2_lon,params.radar2_lat,tl2,tb2);
            Rm=Rg+params.radar2_range_bias_m+randn()*params.radar2_range_noise_std_m;
            am=az+params.radar2_azimuth_bias_deg+randn()*params.radar2_azimuth_noise_std_deg;
            d2(end+1)=Rm-Rg; daz=am-az;
            if daz>180,daz=daz-360;elseif daz<-180,daz=daz+360;end;da2(end+1)=daz;
        end
    end
    de1=mean(d1); dae1=mean(da1); de2=mean(d2); dae2=mean(da2);

    % Phase 2+4: detection + bias correction (continuous rng)
    dR1=cell(nf,1); dR2=cell(nf,1);
    rng(seed+1e7);
    for k=1:nf
        [p,v]=aircraft_trajectory_interpolate(traj,t1(k));
        dR1{k}=generate_frame_detections(params.radar1_lon,params.radar1_lat,params.radar1_tx_lon,params.radar1_tx_lat,p(1),p(2),v(1),v(2),k,t1(k),params.radar1_range_bias_m,params.radar1_azimuth_bias_deg,params.radar1_beam_center_deg,params,params.radar1_range_noise_std_m,params.radar1_azimuth_noise_std_deg);
        for d=1:length(dR1{k})
            dR1{k}(d).aircraft_id=1;
            Rgc=dR1{k}(d).prange-de1; azc=dR1{k}(d).paz-dae1;
            dR1{k}(d).drange=Rgc; dR1{k}(d).daz=azc;
            dR1{k}(d).range_meas=Rgc; dR1{k}(d).azimuth_meas=azc;
            if ~(isfield(dR1{k}(d),'lat')&&~isnan(dR1{k}(d).lat))
                [~,le,lo]=bistatic_inverse_solver(Rgc,azc,params.radar1_tx_lon,params.radar1_tx_lat,params.radar1_lon,params.radar1_lat);
                dR1{k}(d).lat=le; dR1{k}(d).lon=lo;
            end
        end
    end
    rng(seed+2e7);
    for k=1:nf
        [p,v]=aircraft_trajectory_interpolate(traj,t2(k));
        dR2{k}=generate_frame_detections(params.radar2_lon,params.radar2_lat,params.radar2_tx_lon,params.radar2_tx_lat,p(1),p(2),v(1),v(2),k,t2(k),params.radar2_range_bias_m,params.radar2_azimuth_bias_deg,params.radar2_beam_center_deg,params,params.radar2_range_noise_std_m,params.radar2_azimuth_noise_std_deg);
        for d=1:length(dR2{k})
            dR2{k}(d).aircraft_id=1;
            Rgc=dR2{k}(d).prange-de2; azc=dR2{k}(d).paz-dae2;
            dR2{k}(d).drange=Rgc; dR2{k}(d).daz=azc;
            dR2{k}(d).range_meas=Rgc; dR2{k}(d).azimuth_meas=azc;
            if ~(isfield(dR2{k}(d),'lat')&&~isnan(dR2{k}(d).lat))
                [~,le,lo]=bistatic_inverse_solver(Rgc,azc,params.radar2_tx_lon,params.radar2_tx_lat,params.radar2_lon,params.radar2_lat);
                dR2{k}(d).lat=le; dR2{k}(d).lon=lo;
            end
        end
    end

    % Phase 5: UKF
    params.ukf_range_std_m=params.radar1_range_noise_std_m;
    params.ukf_azimuth_std_deg=params.radar1_azimuth_noise_std_deg;
    params.ukf_Q_scale=params.radar1_ukf_Q_scale;
    params.ukf_P_pos_std=params.radar1_ukf_P_pos_std;
    params.ukf_P_vel_std=params.radar1_ukf_P_vel_std;
    params.gate_sigma=params.radar1_gate_sigma;
    params.gate_vr_ms=params.radar1_gate_vr_ms;
    params.tracker_K_loss=6;
    u1=ukf_jichu('create',params,params.radar1_lon,params.radar1_lat,params.radar1_tx_lon,params.radar1_tx_lat,params.dt_sec);
    [sR1,~]=single_track_runner(dR1,u1,params,nf,tt,t1);

    pr2=params; pr2.ukf_range_std_m=params.radar2_range_noise_std_m;
    pr2.ukf_azimuth_std_deg=params.radar2_azimuth_noise_std_deg;
    pr2.gate_sigma=params.radar2_gate_sigma; pr2.gate_vr_ms=params.radar2_gate_vr_ms;
    pr2.ukf_Q_scale=params.radar2_ukf_Q_scale;
    pr2.ukf_P_pos_std=params.radar2_ukf_P_pos_std;
    pr2.ukf_P_vel_std=params.radar2_ukf_P_vel_std;
    pr2.tracker_M=4; pr2.tracker_N=8; pr2.tracker_K_loss=6;
    u2=ukf_jichu('create',pr2,params.radar2_lon,params.radar2_lat,params.radar2_tx_lon,params.radar2_tx_lat,params.dt_sec);
    [sR2,~]=single_track_runner(dR2,u2,pr2,nf,tt,t2);

    % RMSE
    er1=[]; er2=[];
    for k=1:nf
        tl=interp1(tt(:,5),tt(:,1),t1(k),'linear','extrap');
        tb=interp1(tt(:,5),tt(:,2),t1(k),'linear','extrap');
        if ~isempty(sR1{k})&&~isempty(sR1{k}.trackList)
            tr=sR1{k}.trackList{1};
            if isfield(tr,'type')&&tr.type~=7&&~isnan(tr.lat)
                er1(end+1)=sphere_utils_haversine_distance(tr.lon,tr.lat,tl,tb)/1000;
            end
        end
    end
    for k=1:nf
        tl=interp1(tt(:,5),tt(:,1),t2(k),'linear','extrap');
        tb=interp1(tt(:,5),tt(:,2),t2(k),'linear','extrap');
        if ~isempty(sR2{k})&&~isempty(sR2{k}.trackList)
            tr=sR2{k}.trackList{1};
            if isfield(tr,'type')&&tr.type~=7&&~isnan(tr.lat)
                er2(end+1)=sphere_utils_haversine_distance(tr.lon,tr.lat,tl,tb)/1000;
            end
        end
    end
    r1_rmse(mc)=rms_v(er1); r2_rmse(mc)=rms_v(er2);

    % Phase 6+7: time align + fusion
    aR2=time_align_tracks(sR2,params);
    mp=struct('R1_track_id',1,'R2_track_id',1,'match_count',0,'coexist_count',0,'match_ratio',1,'mean_dist_km',0,'quality',100);
    mn={'SCC','BC','CI','FCI'}; frms=nan(1,4);
    for m=1:4
        af=run_track_fusion(mp,sR1,aR2,params,mn{m});
        e=[];
        for k=1:nf
            tl=interp1(tt(:,5),tt(:,1),t1(k),'linear','extrap');
            tb=interp1(tt(:,5),tt(:,2),t1(k),'linear','extrap');
            if ~isempty(af{k})&&~isempty(af{k}.trackList)
                tr=af{k}.trackList{1};
                if ~isnan(tr.lat), e(end+1)=sphere_utils_haversine_distance(tr.lon,tr.lat,tl,tb)/1000; end
            end
        end
        frms(m)=rms_v(e);
    end
    [fus_rmse(mc),~]=min(frms);

    if mod(mc,20)==0
        et=toc;
        fprintf('  [%d/%d] %.0fs | R1=%.1f R2=%.1f Fus=%.1f\n',mc,N,et,nanmedian(r1_rmse(1:mc)),nanmedian(r2_rmse(1:mc)),nanmedian(fus_rmse(1:mc)));
    end
end
et=toc; fprintf('Total: %.0fs\n',et);

% Statistics
v=fus_rmse(~isnan(fus_rmse));
v1=r1_rmse(~isnan(r1_rmse)); v2=r2_rmse(~isnan(r2_rmse));

fprintf('\n===== K_loss=6 Fusion RMSE Distribution (N=%d valid) =====\n',length(v));
fprintf('Mean=%.1f  Median=%.1f  Std=%.2f\n',mean(v),median(v),std(v));
pct=[50 75 90 95 99];
p=prctile(v,pct);
for i=1:length(pct), fprintf('P%d=%.1f km\n',pct(i),p(i)); end

fprintf('\nBins:\n');
edges=[0 4 5 6 8 10 15 20 50];
cnt=histcounts(v,edges);
for i=1:length(edges)-1
    fprintf('  [%2.0f,%2.0f) km: %3d (%5.1f%%)\n',edges(i),edges(i+1),cnt(i),cnt(i)/length(v)*100);
end

fprintf('\nProbability:\n');
fprintf('  Fus > 5km:  %d/%d = %.1f%%\n',sum(v>5),length(v),sum(v>5)/length(v)*100);
fprintf('  Fus > 8km:  %d/%d = %.1f%%\n',sum(v>8),length(v),sum(v>8)/length(v)*100);
fprintf('  Fus > 10km: %d/%d = %.1f%%\n',sum(v>10),length(v),sum(v>10)/length(v)*100);
fprintf('  Fus > 15km: %d/%d = %.1f%%\n',sum(v>15),length(v),sum(v>15)/length(v)*100);

% Bad seed breakdown
fprintf('\nBad seed breakdown (by single-radar criterion):\n');
n_div=sum(r1_rmse>30|r2_rmse>30);
n_deg=sum((r1_rmse>15|r2_rmse>15)&~(r1_rmse>30|r2_rmse>30));
fprintf('  DIVERGED (>30km): %d (%.1f%%)\n',n_div,n_div/N*100);
fprintf('  DEGRADED (15-30km): %d (%.1f%%)\n',n_deg,n_deg/N*100);
fprintf('  Total "bad" single-radar: %d (%.1f%%)\n',n_div+n_deg,(n_div+n_deg)/N*100);
fprintf('  Of these, fusion RMSE > 10km: %d seeds\n',sum(fus_rmse>10));

% Correlation: if R1 bad, what's fusion?
fprintf('\nConditional probabilities:\n');
r1bad=r1_rmse>15; r2bad=r2_rmse>15;
both_bad=r1bad&r2bad; one_good=r1bad~=r2bad;
fprintf('  P(fus>5km | R1 bad): %.0f%%\n',sum(fus_rmse(r1bad)>5)/sum(r1bad)*100);
fprintf('  P(fus>5km | R2 bad): %.0f%%\n',sum(fus_rmse(r2bad)>5)/sum(r2bad)*100);
fprintf('  P(fus>5km | both bad): %.0f%%\n',sum(fus_rmse(both_bad)>5)/max(1,sum(both_bad))*100);
fprintf('  P(fus>5km | one good): %.0f%%\n',sum(fus_rmse(one_good)>5)/max(1,sum(one_good))*100);
fprintf('  P(fus>5km) overall: %.0f%%\n',sum(v>5)/length(v)*100);
fprintf('\nDone.\n');

function v=rms_v(e)
    if isempty(e), v=NaN; else, v=sqrt(mean(e.^2)); end
end
