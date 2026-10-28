function probe_online_d2()
%PROBE_ONLINE_D2 诊断在线链路里两站局部航迹之间的马氏距离分布
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'config'), fullfile(root,'simulation'), fullfile(root,'ukf'), ...
            fullfile(root,'tracker'), fullfile(root,'fusion'), fullfile(root,'evaluation'), ...
            fullfile(root,'utils'), fullfile(root,'io'));
    fid = fopen('results/_probe_d2.txt','w');
    p = params_coverage_online();
    p.imm_ct_omega_slow_deg = 0.05;  p.imm_ct_omega_fast_deg = 0.2;
    scenario = load_adsb_scenario('adsb_ns_fr24', p);
    truth_all = scenario.truth_all;
    t1 = scenario.t1_grid;  t2 = scenario.t2_grid;
    nf = 700;
    pr1 = radar_params(p,1);  pr2 = radar_params(p,2);
    u1 = ukf_imm('create', pr1, p.radar1_lon, p.radar1_lat, p.radar1_tx_lon, p.radar1_tx_lat, p.dt_sec);
    u2 = ukf_imm('create', pr2, p.radar2_lon, p.radar2_lat, p.radar2_tx_lon, p.radar2_tx_lat, p.dt_sec);
    tl1 = {}; tm1 = struct([]); id1 = 1;
    tl2 = {}; tm2 = struct([]); id2 = 1;
    d2same = []; d2diff = []; distsame = [];
    for k = 1:nf
        rng(p.random_seed + 1e7);
        s1 = build_target_states_at_time(truth_all, t1(k));
        r1 = generate_frame_detections_multi(p.radar1_lon,p.radar1_lat,p.radar1_tx_lon,p.radar1_tx_lat, s1,k,t1(k),0,0,p.radar1_beam_center_deg,p,p.radar1_range_noise_std_m,p.radar1_azimuth_noise_std_deg);
        rng(p.random_seed + 2e7);
        s2 = build_target_states_at_time(truth_all, t2(k));
        r2 = generate_frame_detections_multi(p.radar2_lon,p.radar2_lat,p.radar2_tx_lon,p.radar2_tx_lat, s2,k,t2(k),0,0,p.radar2_beam_center_deg,p,p.radar2_range_noise_std_m,p.radar2_azimuth_noise_std_deg);
        r1 = fix_geo(r1, 1, p);  r2 = fix_geo(r2, 2, p);
        [tl1,tm1,sn1,id1] = TRACK_MAIN_ORACLE(tl1,tm1,r1,u1,pr1,k,id1,truth_all,t1);
        [tl2,tm2,sn2,id2] = TRACK_MAIN_ORACLE(tl2,tm2,r2,u2,pr2,k,id2,truth_all,t2);
        % R2 需回退到 R1 时间基准，否则两站快照对不上
        sn2 = time_align_tracks({sn2}, p);  sn2 = sn2{1};
        L1 = locs(sn1);  L2 = locs(sn2);
        if k == 1 || mod(k,100)==0
            fprintf(fid,'k=%d nL1=%d nL2=%d\n', k, numel(L1), numel(L2));
        end
        for a = 1:numel(L1)
            for b = 1:numel(L2)
                [d2, dist] = maha2(L1(a), L2(b));
                if L1(a).truth == L2(b).truth
                    d2same(end+1) = d2;  distsame(end+1) = dist; %#ok<AGROW>
                else
                    d2diff(end+1) = d2; %#ok<AGROW>
                end
            end
        end
    end
    fprintf(fid,'同源对: n=%d  D2 中位 %.2f 均值 %.2f  P90 %.2f  <5.99 占比 %.1f%%\n', ...
        numel(d2same), median(d2same), mean(d2same), prctile(d2same,90), 100*mean(d2same<5.99));
    fprintf(fid,'异源对: n=%d  D2 中位 %.2f 均值 %.2f  P10 %.2f  <5.99 占比 %.1f%%\n', ...
        numel(d2diff), median(d2diff), mean(d2diff), prctile(d2diff,10), 100*mean(d2diff<5.99));
    fprintf(fid,'同源距离 km: 中位 %.2f P90 %.2f\n', median(distsame), prctile(distsame,90));
    fclose(fid);
end

function dets = fix_geo(dets, radar_id, p)
% 补做偏差校正与双基地反演（run.m 的 generate_one_frame_detections 做的事）
    if radar_id == 1
        rx_lon = p.radar1_lon; rx_lat = p.radar1_lat; tx_lon = p.radar1_tx_lon; tx_lat = p.radar1_tx_lat;
    else
        rx_lon = p.radar2_lon; rx_lat = p.radar2_lat; tx_lon = p.radar2_tx_lon; tx_lat = p.radar2_tx_lat;
    end
    for i = 1:numel(dets)
        d = dets(i);
        d.drange = d.prange;  d.daz = d.paz;
        d.range_meas = d.prange;  d.azimuth_meas = d.paz;
        if ~isfield(d,'radial_vel_meas') || isnan(d.radial_vel_meas), d.radial_vel_meas = d.pvr; end
        if ~isfield(d,'lat') || isnan(d.lat) || ~isfield(d,'lon') || isnan(d.lon)
            [~, d.lat, d.lon] = bistatic_inverse_solver(d.prange, d.paz, tx_lon, tx_lat, ...
                rx_lon, rx_lat, iono_h(p,'assumed',radar_id));
        end
        [~, d.raw_lat, d.raw_lon] = bistatic_inverse_solver(d.prange, d.paz, tx_lon, tx_lat, ...
            rx_lon, rx_lat, iono_h(p,'assumed',radar_id));
        dets(i) = d;
    end
end

function L = locs(snap)
    L = struct('id',{},'x',{},'P',{},'truth',{},'life',{});
    if isempty(snap) || ~isfield(snap,'trackList'), return; end
    for i = 1:numel(snap.trackList)
        t = snap.trackList{i};
        if t.type==6 || t.type==7, continue; end
        if isempty(t.ukf) || ~isfield(t.ukf,'x'), continue; end
        L(end+1) = struct('id',t.id,'x',t.ukf.x(:),'P',t.ukf.P, ...
            'truth', double(t.truth_idx), 'life', t.life); %#ok<AGROW>
    end
end

function [d2, dist] = maha2(a, b)
    latm = 0.5*(a.x(3)+b.x(3));
    S = diag([111.32*cosd(latm), 110.57]);
    dx = S*[a.x(1)-b.x(1); a.x(3)-b.x(3)];
    Pp = [a.P(1,1)+b.P(1,1), a.P(1,3)+b.P(1,3); a.P(3,1)+b.P(3,1), a.P(3,3)+b.P(3,3)];
    Pp = S*Pp*S'; Pp = 0.5*(Pp+Pp');
    if rcond(Pp) < 1e-12, Pp = Pp + 1e-6*eye(2); end
    d2 = dx'*(Pp\dx);
    R = 6371; p1 = deg2rad(a.x(3)); p2 = deg2rad(b.x(3));
    dp = deg2rad(b.x(3)-a.x(3)); dl = deg2rad(b.x(1)-a.x(1));
    q = sin(dp/2)^2 + cos(p1)*cos(p2)*sin(dl/2)^2;
    dist = 2*R*asin(min(1,sqrt(q)));
end
