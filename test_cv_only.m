% _test_cv_only.m — 对比IMM vs 单模型CV UKF (N=10)
clear; close all; clc; addpath(genpath('.'));
fprintf('IMM vs CV-only 对比 N=10...\n\n');

N=10;
r = struct();
r.fus_imm=nan(N,1); r.fus_cv=nan(N,1);
r.r1_imm=nan(N,1); r.r1_cv=nan(N,1);
r.r2_imm=nan(N,1); r.r2_cv=nan(N,1);
r.nis_imm_r1=nan(N,1); r.nis_cv_r1=nan(N,1);
r.assoc_imm_r1=nan(N,1); r.assoc_cv_r1=nan(N,1);
r.ct_turn=nan(N,1);

params0 = simulation_params();
turn_rate = +1.0 * pi / 180.0;

approach_dur = 120e3 / params0.aircraft_speed_ms;
turn_dur = 180.0;
turn_start_sec = approach_dur;
turn_end_sec = approach_dur + turn_dur;

for mc=1:N
    seed = mc;
    params = simulation_params(); params.random_seed = seed;
    rng(params.random_seed);

    [~, traj] = evalc('aircraft_trajectory_create(''uturn'', params)');
    true_track = aircraft_trajectory_interpolate('generate', traj);
    t1 = params.time_offset_radar1_sec : params.dt_sec : traj.duration_sec;
    t2 = params.time_offset_radar2_sec : params.dt_sec : traj.duration_sec;
    nf = min(length(t1), length(t2));
    tf_s = find(t1 >= turn_start_sec, 1); tf_e = find(t1 >= turn_end_sec, 1);

    % ADS-B
    rng(seed); T=readtable(params.adsb_csv_path,'ReadVariableNames',false);
    al=T.Var2; ao=T.Var3; d1=[]; da1=[]; d2=[]; da2=[];
    nc=min(5000,height(T)); cs=max(1,floor(height(T)/nc));
    for idx=1:cs:height(T)
        tl=ao(idx); tb=al(idx);
        if isnan(tl)||isnan(tb), continue; end
        [in1,~,~]=radar_coverage_check(params.radar1_lon,params.radar1_lat,tl,tb,params.radar1_beam_center_deg,params);
        if in1
            Rg=skywave_geometry('group_range',params.radar1_tx_lon,params.radar1_tx_lat,params.radar1_lon,params.radar1_lat,tl,tb);
            az=sphere_utils_azimuth(params.radar1_lon,params.radar1_lat,tl,tb);
            Rm=Rg+params.radar1_range_bias_m+randn()*params.radar1_range_noise_std_m;
            am=az+params.radar1_azimuth_bias_deg+randn()*params.radar1_azimuth_noise_std_deg;
            d1(end+1)=Rm-Rg; daz=am-az;
            if daz>180,daz=daz-360;elseif daz<-180,daz=daz+360;end;da1(end+1)=daz;
        end
        [in2,~,~]=radar_coverage_check(params.radar2_lon,params.radar2_lat,tl,tb,params.radar2_beam_center_deg,params);
        if in2
            Rg=skywave_geometry('group_range',params.radar2_tx_lon,params.radar2_tx_lat,params.radar2_lon,params.radar2_lat,tl,tb);
            az=sphere_utils_azimuth(params.radar2_lon,params.radar2_lat,tl,tb);
            Rm=Rg+params.radar2_range_bias_m+randn()*params.radar2_range_noise_std_m;
            am=az+params.radar2_azimuth_bias_deg+randn()*params.radar2_azimuth_noise_std_deg;
            d2(end+1)=Rm-Rg; daz=am-az;
            if daz>180,daz=daz-360;elseif daz<-180,daz=daz+360;end;da2(end+1)=daz;
        end
    end
    de1=mean(d1); dae1=mean(da1); de2=mean(d2); dae2=mean(da2);

    % 点迹
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

    % ==== IMM ====
    params.ukf_range_std_m=params.radar1_range_noise_std_m; params.ukf_azimuth_std_deg=params.radar1_azimuth_noise_std_deg;
    params.ukf_Q_scale=params.radar1_ukf_Q_scale; params.ukf_P_pos_std=params.radar1_ukf_P_pos_std; params.ukf_P_vel_std=params.radar1_ukf_P_vel_std;
    params.gate_sigma=6.0; params.gate_vr_ms=params.radar1_gate_vr_ms; params.tracker_K_loss=params.radar1_tracker_K_loss;
    params.use_fuzzy_adaptive=true;

    u1cv=ukf_jichu('create',params,params.radar1_lon,params.radar1_lat,params.radar1_tx_lon,params.radar1_tx_lat,params.dt_sec);
    u1ct=ukf_jichu('create',params,params.radar1_lon,params.radar1_lat,params.radar1_tx_lon,params.radar1_tx_lat,params.dt_sec);
    u1ct.model_type='CT'; u1ct.turn_rate_rad_per_sec=turn_rate;

    pr2=params; pr2.ukf_range_std_m=params.radar2_range_noise_std_m; pr2.ukf_azimuth_std_deg=params.radar2_azimuth_noise_std_deg;
    pr2.gate_sigma=6.0; pr2.gate_vr_ms=params.radar2_gate_vr_ms; pr2.ukf_Q_scale=params.radar2_ukf_Q_scale;
    pr2.ukf_P_pos_std=params.radar2_ukf_P_pos_std; pr2.ukf_P_vel_std=params.radar2_ukf_P_vel_std;
    pr2.tracker_M=4; pr2.tracker_N=8; pr2.tracker_K_loss=params.radar2_tracker_K_loss;

    u2cv=ukf_jichu('create',pr2,params.radar2_lon,params.radar2_lat,params.radar2_tx_lon,params.radar2_tx_lat,params.dt_sec);
    u2ct=ukf_jichu('create',pr2,params.radar2_lon,params.radar2_lat,params.radar2_tx_lon,params.radar2_tx_lat,params.dt_sec);
    u2ct.model_type='CT'; u2ct.turn_rate_rad_per_sec=turn_rate;

    [s1i,ft1]=imm_tracker(dR1,u1cv,u1ct,params,nf,true_track,t1);
    [s2i,ft2]=imm_tracker(dR2,u2cv,u2ct,pr2,nf,true_track,t2);

    % ==== CV-only ====
    params.tracker_K_loss=params.radar1_tracker_K_loss;
    u1only=ukf_jichu('create',params,params.radar1_lon,params.radar1_lat,params.radar1_tx_lon,params.radar1_tx_lat,params.dt_sec);
    [s1c,~]=single_track_runner(dR1,u1only,params,nf,true_track,t1);

    pr2c=pr2; pr2c.tracker_M=4; pr2c.tracker_N=8; pr2c.tracker_K_loss=params.radar2_tracker_K_loss;
    u2only=ukf_jichu('create',pr2c,params.radar2_lon,params.radar2_lat,params.radar2_tx_lon,params.radar2_tx_lat,params.dt_sec);
    [s2c,~]=single_track_runner(dR2,u2only,pr2c,nf,true_track,t2);

    % RMSE
    for radar={'R1','R2'}
        src=struct('IMM_snaps',s1i,'CV_snaps',s1c,'t',t1,'rmse',[]);
        if strcmp(radar{1},'R2'), src.IMM_snaps=s2i; src.CV_snaps=s2c; src.t=t2; end
        e_imm=[]; e_cv=[];
        for k=1:nf
            tl=interp1(true_track(:,5),true_track(:,1),src.t(k),'linear','extrap');
            tb=interp1(true_track(:,5),true_track(:,2),src.t(k),'linear','extrap');
            if ~isempty(src.IMM_snaps{k})&&~isempty(src.IMM_snaps{k}.trackList)
                tr=src.IMM_snaps{k}.trackList{1};
                if isfield(tr,'type')&&tr.type~=7&&~isnan(tr.lat), e_imm(end+1)=sphere_utils_haversine_distance(tr.lon,tr.lat,tl,tb)/1000; end
            end
            if ~isempty(src.CV_snaps{k})&&~isempty(src.CV_snaps{k}.trackList)
                tr=src.CV_snaps{k}.trackList{1};
                if isfield(tr,'type')&&tr.type~=7&&~isnan(tr.lat), e_cv(end+1)=sphere_utils_haversine_distance(tr.lon,tr.lat,tl,tb)/1000; end
            end
        end
        if strcmp(radar{1},'R1')
            r.r1_imm(mc)=rms_v(e_imm); r.r1_cv(mc)=rms_v(e_cv);
        else
            r.r2_imm(mc)=rms_v(e_imm); r.r2_cv(mc)=rms_v(e_cv);
        end
    end

    % CT turn prob
    if isfield(ft1,'mu_history'), mh=ft1.mu_history;
        if ~isempty(tf_s), tf=tf_s:min(tf_e,size(mh,1)); r.ct_turn(mc)=mean(mh(tf,2))*100; end
    end

    % Fusion
    a2i=time_align_tracks(s2i,pr2); a2c=time_align_tracks(s2c,pr2c);
    mp=struct('R1_track_id',1,'R2_track_id',1,'match_count',0,'coexist_count',0,'match_ratio',1,'mean_dist_km',0,'quality',100);
    for label={'IMM','CV'}
        s1=s1i; s2=a2i; p_=pr2;
        if strcmp(label{1},'CV'), s1=s1c; s2=a2c; p_=pr2c; end
        f_vals=nan(1,4);
        for m=1:4
            af=run_track_fusion(mp,s1,s2,p_,{'SCC','BC','CI','FCI'}{m});
            e=[];
            for k=1:nf
                tl=interp1(true_track(:,5),true_track(:,1),t1(k),'linear','extrap');
                tb=interp1(true_track(:,5),true_track(:,2),t1(k),'linear','extrap');
                if ~isempty(af{k})&&~isempty(af{k}.trackList), tr=af{k}.trackList{1}; if ~isnan(tr.lat), e(end+1)=sphere_utils_haversine_distance(tr.lon,tr.lat,tl,tb)/1000; end; end
            end
            f_vals(m)=rms_v(e);
        end
        if strcmp(label{1},'IMM'), r.fus_imm(mc)=min(f_vals); else, r.fus_cv(mc)=min(f_vals); end
    end

    fprintf('MC#%d: R1 IMM=%.1f CV=%.1f | R2 IMM=%.1f CV=%.1f | Fus IMM=%.1f CV=%.1f | CTturn=%.0f%%\n',...
        mc, r.r1_imm(mc), r.r1_cv(mc), r.r2_imm(mc), r.r2_cv(mc), r.fus_imm(mc), r.fus_cv(mc), r.ct_turn(mc));
end

fprintf('\n===== IMM vs CV-only (N=%d) =====\n', N);
fprintf('%-20s %8s %8s %8s\n', '指标','IMM','CV-only','CV更好?');
fprintf('%-20s %8s %8s %8s\n', '---','---','---','---');
fprintf('%-20s %7.1fkm %7.1fkm %8s\n', 'R1 UKF 中位', nanmedian(r.r1_imm), nanmedian(r.r1_cv), bool_str(nanmedian(r.r1_cv)<nanmedian(r.r1_imm)));
fprintf('%-20s %7.1fkm %7.1fkm %8s\n', 'R2 UKF 中位', nanmedian(r.r2_imm), nanmedian(r.r2_cv), bool_str(nanmedian(r.r2_cv)<nanmedian(r.r2_imm)));
fprintf('%-20s %7.1fkm %7.1fkm %8s\n', 'Fusion 中位', nanmedian(r.fus_imm), nanmedian(r.fus_cv), bool_str(nanmedian(r.fus_cv)<nanmedian(r.fus_imm)));
fprintf('%-20s %7.1fkm %7.1fkm %8s\n', 'R1 UKF 均值', nanmean(r.r1_imm), nanmean(r.r1_cv), bool_str(nanmean(r.r1_cv)<nanmean(r.r1_imm)));
fprintf('%-20s %7.1fkm %7.1fkm %8s\n', 'R2 UKF 均值', nanmean(r.r2_imm), nanmean(r.r2_cv), bool_str(nanmean(r.r2_cv)<nanmean(r.r2_imm)));
fprintf('%-20s %7.1fkm %7.1fkm %8s\n', 'Fusion 均值', nanmean(r.fus_imm), nanmean(r.fus_cv), bool_str(nanmean(r.fus_cv)<nanmean(r.fus_imm)));
fprintf('\nCT转弯段概率: %.0f%%\n', nanmean(r.ct_turn));
fprintf('Done.\n');

function v=rms_v(e), if isempty(e), v=NaN; else, v=sqrt(mean(e.^2)); end, end
function s=bool_str(b), if b, s='YES ***'; else, s='no'; end, end
