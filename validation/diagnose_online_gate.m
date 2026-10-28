function diagnose_online_gate(n_frames, target_truth)
%DIAGNOSE_ONLINE_GATE 诊断在线配对为何不并入：看跨站同源航迹的马氏距离
%
%   diagnose_online_gate(n_frames, target_truth)
%
%   逐帧跑单站跟踪与时间对齐，对每一对"同一真值的 R1/R2 局部航迹"
%   计算马氏距离平方，报告其分布与门限的关系。不做融合。

    if nargin < 1 || isempty(n_frames), n_frames = 400; end
    if nargin < 2, target_truth = []; end

    params = params_coverage_online();
    rng(params.random_seed);
    scenario = load_adsb_scenario('adsb_ns', params);
    truth_all = scenario.truth_all;
    t1 = scenario.t1_grid;  t2 = scenario.t2_grid;
    n_frames = min(n_frames, scenario.n_frames);

    det1 = gen_det(1, params, truth_all, t1, n_frames);
    det2 = gen_det(2, params, truth_all, t2, n_frames);
    p1 = radar_params(params, 1);  p2 = radar_params(params, 2);
    u1 = ukf_imm('create', p1, params.radar1_lon, params.radar1_lat, ...
        params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
    u2 = ukf_imm('create', p2, params.radar2_lon, params.radar2_lat, ...
        params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);

    tl1 = {}; tp1 = struct([]); id1 = 1;
    tl2 = {}; tp2 = struct([]); id2 = 1;
    D = [];  info = [];

    for k = 1:n_frames
        [tl1, tp1, s1, id1, ~] = TRACK_MAIN_ORACLE(tl1, tp1, det1{k}, u1, p1, k, id1, truth_all, t1);
        [tl2, tp2, s2, id2, ~] = TRACK_MAIN_ORACLE(tl2, tp2, det2{k}, u2, p2, k, id2, truth_all, t2);
        s2a = time_align_tracks({s2}, params);  s2a = s2a{1};

        A = loc(s1);  B = loc(s2a);
        for i = 1:numel(A)
            for j = 1:numel(B)
                if isnan(A(i).truth) || A(i).truth ~= B(j).truth, continue; end
                if ~isempty(target_truth) && A(i).truth ~= target_truth, continue; end
                d2 = m2(A(i).x, A(i).P, B(j).x, B(j).P);
                D(end+1, 1) = d2; %#ok<AGROW>
                info(end+1, :) = [k, A(i).id, B(j).id, d2, A(i).truth]; %#ok<AGROW>
            end
        end
    end

    fprintf('门限 chi2 = %.2f，样本 %d 个（同一真值的跨站航迹对）%s', ...
        params.online_gate_chi2, numel(D), newline);
    if isempty(D), return; end
    fprintf('d2 分位：min %.2f, 25%% %.2f, 中位 %.2f, 75%% %.2f, 90%% %.2f, max %.2f%s', ...
        min(D), q(D, 25), median(D), q(D, 75), q(D, 90), max(D), newline);
    fprintf('落在门限内的比例 %.1f%%%s', 100 * mean(D < params.online_gate_chi2), newline);

    fprintf('%s前 15 个样本：帧  R1id  R2id   d2   真值%s', newline, newline);
    for i = 1:min(15, size(info, 1))
        fprintf('  %4d  %4d  %4d  %8.2f  %d%s', info(i,1), info(i,2), info(i,3), info(i,4), info(i,5), newline);
    end
end

function v = q(x, p)
    x = sort(x(:));
    v = x(max(1, min(numel(x), round(numel(x) * p / 100))));
end

function L = loc(snap)
    L = struct('id', {}, 'x', {}, 'P', {}, 'truth', {});
    if isempty(snap) || ~isfield(snap, 'trackList'), return; end
    for t = 1:numel(snap.trackList)
        trk = snap.trackList{t};
        if trk.type == 6 || trk.type == 7, continue; end
        if ~isfield(trk, 'ukf') || isempty(trk.ukf), continue; end
        L(end+1) = struct('id', trk.id, 'x', trk.ukf.x(:), 'P', trk.ukf.P, ...
            'truth', double(trk.truth_idx)); %#ok<AGROW>
    end
end

function d2 = m2(x1, P1, x2, P2)
    lat = 0.5 * (x1(3) + x2(3));
    S = diag([111.32 * cosd(lat), 110.57]);
    dx = S * [x1(1) - x2(1); x1(3) - x2(3)];
    Pp = [P1(1,1)+P2(1,1), P1(1,3)+P2(1,3); P1(3,1)+P2(3,1), P1(3,3)+P2(3,3)];
    Pp = S * Pp * S';
    d2 = dx' * (Pp \ dx);
end

function detList = gen_det(radar_id, params, truth_all, t_grid, n_frames)
    detList = cell(n_frames, 1);
    if radar_id == 1
        rng(params.random_seed + 1e7);
        rx = [params.radar1_lon, params.radar1_lat];
        tx = [params.radar1_tx_lon, params.radar1_tx_lat];
        bc = params.radar1_beam_center_deg;
        rn = params.radar1_range_noise_std_m; an = params.radar1_azimuth_noise_std_deg;
    else
        rng(params.random_seed + 2e7);
        rx = [params.radar2_lon, params.radar2_lat];
        tx = [params.radar2_tx_lon, params.radar2_tx_lat];
        bc = params.radar2_beam_center_deg;
        rn = params.radar2_range_noise_std_m; an = params.radar2_azimuth_noise_std_deg;
    end
    for k = 1:n_frames
        st = build_target_states_at_time(truth_all, t_grid(k));
        raw = generate_frame_detections_multi(rx(1), rx(2), tx(1), tx(2), st, ...
            k, t_grid(k), 0, 0, bc, params, rn, an);
        d = cell(1, numel(raw));
        for i = 1:numel(raw)
            dp = raw(i);
            dp.drange = dp.prange;  dp.daz = dp.paz;
            dp.range_meas = dp.prange;  dp.azimuth_meas = dp.paz;
            if isnan(dp.lat)
                [~, dp.lat, dp.lon] = bistatic_inverse_solver(dp.prange, dp.paz, ...
                    tx(1), tx(2), rx(1), rx(2), iono_h(params, 'assumed', radar_id));
            end
            d{i} = dp;
        end
        if isempty(d), detList{k} = []; else, detList{k} = [d{:}]; end
    end
end
