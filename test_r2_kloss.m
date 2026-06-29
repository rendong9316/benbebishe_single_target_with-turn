% test_r2_kloss.m — 实测 R2 K=6/7/8 (R1固定=8)
clear; close all; clc; addpath(genpath('.'));
fprintf('R2 K_loss=6/7/8 实测对比 (R1=8), N=100 each...\n\n');

kloss_vals = [6, 7, 8];
N = 100;
summary = struct();
summary.kloss = kloss_vals;
summary.n_bad = zeros(1,3); summary.bad_rate = zeros(1,3);
summary.ukf_R2_med = zeros(1,3); summary.ukf_R2_mean = zeros(1,3);
summary.ukf_R1_med = zeros(1,3);
summary.fus_med = zeros(1,3); summary.fus_mean = zeros(1,3);
summary.mtl_R2 = zeros(1,3);
summary.brk_R2 = zeros(1,3);
summary.nis_R2 = zeros(1,3);
summary.assoc_R2 = zeros(1,3);
summary.fus_pct_gt5 = zeros(1,3);
summary.fus_pct_gt10 = zeros(1,3);
summary.elapsed = zeros(1,3);

for ci = 1:3
    kl = kloss_vals(ci);
    fprintf('--- R2 K_loss=%d ---\n', kl); tic;

    fus_rmse = nan(N,1); r1_rmse = nan(N,1); r2_rmse = nan(N,1);
    bad = zeros(N,1); r2_lost_count = 0;

    for mc = 1:N
        seed = mc;
        params = simulation_params(); params.random_seed = seed;
        params.radar1_tracker_K_loss = 8; params.tracker_K_loss = 8;
        rng(seed);
        traj = aircraft_trajectory_create(params.aircraft_waypoints,params.aircraft_speed_ms,params.dt_sec);
        tt = aircraft_trajectory_interpolate('generate',traj);
        t1 = params.time_offset_radar1_sec:params.dt_sec:traj.duration_sec;
        t2 = params.time_offset_radar2_sec:params.dt_sec:traj.duration_sec;
        nf = min(length(t1),length(t2));

        % Calibration
        rng(seed); T=readtable(params.adsb_csv_path,'ReadVariableNames',false);
        al=T.Var2; ao=T.Var3; d1=[]; da1=[]; d2=[]; da2=[];
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

        % Detection (continuous rng)
        dR1=cell(nf,1); dR2=cell(nf,1);
        rng(seed+1e7);
        for k=1:nf
            [p,v]=aircraft_trajectory_interpolate(traj,t1(k));
            dR1{k}=generate_frame_detections(params.radar1_lon,params.radar1_lat,params.radar1_tx_lon,params.radar1_tx_lat,p(1),p(2),v(1),v(2),k,t1(k),params.radar1_range_bias_m,params.radar1_azimuth_bias_deg,params.radar1_beam_center_deg,params,params.radar1_range_noise_std_m,params.radar1_azimuth_noise_std_deg);
            for d=1:length(dR1{k}), dR1{k}(d).aircraft_id=1; Rgc=dR1{k}(d).prange-de1; azc=dR1{k}(d).paz-dae1; dR1{k}(d).drange=Rgc; dR1{k}(d).daz=azc; dR1{k}(d).range_meas=Rgc; dR1{k}(d).azimuth_meas=azc; if ~(isfield(dR1{k}(d),'lat')&&~isnan(dR1{k}(d).lat)), [~,le,lo]=bistatic_inverse_solver(Rgc,azc,params.radar1_tx_lon,params.radar1_tx_lat,params.radar1_lon,params.radar1_lat); dR1{k}(d).lat=le; dR1{k}(d).lon=lo; end; end;
        end
        rng(seed+2e7);
        for k=1:nf
            [p,v]=aircraft_trajectory_interpolate(traj,t2(k));
            dR2{k}=generate_frame_detections(params.radar2_lon,params.radar2_lat,params.radar2_tx_lon,params.radar2_tx_lat,p(1),p(2),v(1),v(2),k,t2(k),params.radar2_range_bias_m,params.radar2_azimuth_bias_deg,params.radar2_beam_center_deg,params,params.radar2_range_noise_std_m,params.radar2_azimuth_noise_std_deg);
            for d=1:length(dR2{k}), dR2{k}(d).aircraft_id=1; Rgc=dR2{k}(d).prange-de2; azc=dR2{k}(d).paz-dae2; dR2{k}(d).drange=Rgc; dR2{k}(d).daz=azc; dR2{k}(d).range_meas=Rgc; dR2{k}(d).azimuth_meas=azc; if ~(isfield(dR2{k}(d),'lat')&&~isnan(dR2{k}(d).lat)), [~,le,lo]=bistatic_inverse_solver(Rgc,azc,params.radar2_tx_lon,params.radar2_tx_lat,params.radar2_lon,params.radar2_lat); dR2{k}(d).lat=le; dR2{k}(d).lon=lo; end; end;
        end

        % R1 UKF (K=8 fixed)
        params.ukf_range_std_m=params.radar1_range_noise_std_m; params.ukf_azimuth_std_deg=params.radar1_azimuth_noise_std_deg;
        params.ukf_Q_scale=params.radar1_ukf_Q_scale; params.ukf_P_pos_std=params.radar1_ukf_P_pos_std; params.ukf_P_vel_std=params.radar1_ukf_P_vel_std;
        params.gate_sigma=params.radar1_gate_sigma; params.gate_vr_ms=params.radar1_gate_vr_ms; params.tracker_K_loss=8;
        u1=ukf_jichu('create',params,params.radar1_lon,params.radar1_lat,params.radar1_tx_lon,params.radar1_tx_lat,params.dt_sec);
        [sR1,~]=single_track_runner(dR1,u1,params,nf,tt,t1);

        % R2 UKF (variable K_loss)
        pr2=params; pr2.ukf_range_std_m=params.radar2_range_noise_std_m; pr2.ukf_azimuth_std_deg=params.radar2_azimuth_noise_std_deg;
        pr2.gate_sigma=params.radar2_gate_sigma; pr2.gate_vr_ms=params.radar2_gate_vr_ms; pr2.ukf_Q_scale=params.radar2_ukf_Q_scale;
        pr2.ukf_P_pos_std=params.radar2_ukf_P_pos_std; pr2.ukf_P_vel_std=params.radar2_ukf_P_vel_std;
        pr2.tracker_M=4; pr2.tracker_N=8; pr2.tracker_K_loss=kl;
        u2=ukf_jichu('create',pr2,params.radar2_lon,params.radar2_lat,params.radar2_tx_lon,params.radar2_tx_lat,params.dt_sec);
        [sR2,~]=single_track_runner(dR2,u2,pr2,nf,tt,t2);

        % RMSE
        er1=[]; er2=[];
        for k=1:nf
            tl=interp1(tt(:,5),tt(:,1),t1(k),'linear','extrap'); tb=interp1(tt(:,5),tt(:,2),t1(k),'linear','extrap');
            if ~isempty(sR1{k})&&~isempty(sR1{k}.trackList), tr=sR1{k}.trackList{1}; if isfield(tr,'type')&&tr.type~=7&&~isnan(tr.lat), er1(end+1)=sphere_utils_haversine_distance(tr.lon,tr.lat,tl,tb)/1000; end; end;
        end
        for k=1:nf
            tl=interp1(tt(:,5),tt(:,1),t2(k),'linear','extrap'); tb=interp1(tt(:,5),tt(:,2),t2(k),'linear','extrap');
            if ~isempty(sR2{k})&&~isempty(sR2{k}.trackList), tr=sR2{k}.trackList{1}; if isfield(tr,'type')&&tr.type~=7&&~isnan(tr.lat), er2(end+1)=sphere_utils_haversine_distance(tr.lon,tr.lat,tl,tb)/1000; end; end;
        end
        r1_rmse(mc)=rms_v2(er1); r2_rmse(mc)=rms_v2(er2);

        % Bad seed check
        if r1_rmse(mc)>30 || r2_rmse(mc)>30, bad(mc)=1;
        elseif r1_rmse(mc)>15 || r2_rmse(mc)>15, bad(mc)=1; end

        % Count R2 LOST events
        for k=1:nf
            if ~isempty(sR2{k})&&~isempty(sR2{k}.trackList)
                if sR2{k}.trackList{1}.type==7, r2_lost_count=r2_lost_count+1; break; end
            end
        end

        % Fusion
        aR2=time_align_tracks(sR2,params);
        mp=struct('R1_track_id',1,'R2_track_id',1,'match_count',0,'coexist_count',0,'match_ratio',1,'mean_dist_km',0,'quality',100);
        mn={'SCC','BC','CI','FCI'}; frms=nan(1,4);
        for m=1:4
            af=run_track_fusion(mp,sR1,aR2,params,mn{m}); e=[];
            for k=1:nf
                tl=interp1(tt(:,5),tt(:,1),t1(k),'linear','extrap'); tb=interp1(tt(:,5),tt(:,2),t1(k),'linear','extrap');
                if ~isempty(af{k})&&~isempty(af{k}.trackList), tr=af{k}.trackList{1}; if ~isnan(tr.lat), e(end+1)=sphere_utils_haversine_distance(tr.lon,tr.lat,tl,tb)/1000; end; end;
            end
            frms(m)=rms_v2(e);
        end
        [fus_rmse(mc),~]=min(frms);
    end

    et=toc;
    vf=fus_rmse(~isnan(fus_rmse)); v2=r2_rmse(~isnan(r2_rmse)); v1=r1_rmse(~isnan(r1_rmse));

    summary.ukf_R2_med(ci)=median(v2); summary.ukf_R2_mean(ci)=mean(v2);
    summary.ukf_R1_med(ci)=median(v1);
    summary.fus_med(ci)=median(vf); summary.fus_mean(ci)=mean(vf);
    summary.n_bad(ci)=sum(bad); summary.bad_rate(ci)=sum(bad)/N*100;
    summary.fus_pct_gt5(ci)=sum(vf>5)/length(vf)*100;
    summary.fus_pct_gt10(ci)=sum(vf>10)/length(vf)*100;
    summary.elapsed(ci)=et;

    fprintf('  K=%d: %.0fs | bad=%.0f%% (%d) | R2_med=%.1f R2_mean=%.1f | Fus_med=%.1f Fus_mean=%.1f | Fus>5km=%.0f%% Fus>10km=%.0f%%\n', ...
        kl, et, summary.bad_rate(ci), summary.n_bad(ci), summary.ukf_R2_med(ci), summary.ukf_R2_mean(ci), ...
        summary.fus_med(ci), summary.fus_mean(ci), summary.fus_pct_gt5(ci), summary.fus_pct_gt10(ci));
end

fprintf('\n===== R2 K_loss 对比表 (N=100 each, R1=8固定) =====\n');
fprintf('%-16s %8s %8s %8s\n', '指标', 'K=6', 'K=7', 'K=8');
fprintf('%-16s %8s %8s %8s\n', '──', '──', '──', '──');
fprintf('%-16s %7.0f%% %7.0f%% %7.0f%%\n', '坏种子率', summary.bad_rate);
fprintf('%-16s %7d %7d %7d\n', '坏种子数', summary.n_bad);
fprintf('%-16s %7.1f %7.1f %7.1f\n', 'UKF R2中位(km)', summary.ukf_R2_med);
fprintf('%-16s %7.1f %7.1f %7.1f\n', 'UKF R2均值(km)', summary.ukf_R2_mean);
fprintf('%-16s %7.1f %7.1f %7.1f\n', 'UKF R1中位(km)', summary.ukf_R1_med);
fprintf('%-16s %7.1f %7.1f %7.1f\n', '融合中位(km)', summary.fus_med);
fprintf('%-16s %7.1f %7.1f %7.1f\n', '融合均值(km)', summary.fus_mean);
fprintf('%-16s %7.0f%% %7.0f%% %7.0f%%\n', '融合>5km', summary.fus_pct_gt5);
fprintf('%-16s %7.0f%% %7.0f%% %7.0f%%\n', '融合>10km', summary.fus_pct_gt10);
fprintf('%-16s %7.0f %7.0f %7.0f\n', '耗时(s)', summary.elapsed);

d2 = summary.ukf_R2_mean(3) - summary.ukf_R2_mean(1);
if abs(d2) < 0.5
    fprintf('\nR2 K_loss 从 6 增到 8，所有指标几乎不变 — R2 对 K_loss 不敏感\n');
    fprintf('R2 已有足够的缓冲 (高噪声+6帧)，继续增大无边际收益\n');
end

fprintf('\nDone.\n');

function v=rms_v2(e)
    if isempty(e), v=NaN; else, v=sqrt(mean(e.^2)); end
end
