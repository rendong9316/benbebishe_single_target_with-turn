function result = run_online(scene_id, params_override, opts)
%RUN_ONLINE 双站在线配对与融合的逐帧主流程
%
%   result = run_online(scene_id, params_override, opts)
%
%   与 run.m 的区别：
%     run.m 是"按阶段扫全时间轴"的批处理（对齐整段 → 匹配整段 → 融合整段）；
%     本函数是"按帧推进"的在线流程，每一帧依次完成
%       单站跟踪 → 时间对齐 → 系统航迹分配与融合 → 输出
%     融合中心从第一帧就开机，系统航迹在第一条局部航迹上报时即建立。
%
%   对应文档：双站航迹配对与融合在线处理模型_20261028.md
%
%   输入：
%     scene_id        — load_adsb_scenario 的场景名：
%                       'adsb_ns'        派生场景（data/adsb/adsb_ns.csv）
%                       'adsb_ns_fr24'   FR24 原始文件（data/adsb_ns/scene_fr24.csv）
%     params_override — 覆盖参数，缺省用 params_coverage_online()
%     opts            — 选项结构体：
%                         .max_frames — 只跑前若干帧（冒烟测试用），缺省全部
%                         .verbose    — 是否逐帧打印，缺省 false
%                         .log_file   — 逐帧诊断日志路径，缺省不写
%
%   输出：
%     result — 结构体，含 lengthTable / metrics / ctSnap / log 等

    if nargin < 1 || isempty(scene_id), scene_id = 'adsb_ns'; end
    if nargin < 2 || isempty(params_override)
        params = params_coverage_online();
    else
        params = params_override;
    end
    if nargin < 3 || isempty(opts), opts = struct(); end
    max_frames = get_opt(opts, 'max_frames', Inf);
    verbose = get_opt(opts, 'verbose', false);
    log_file = get_opt(opts, 'log_file', '');

    rng(params.random_seed);

    % ====== 场景 ======
    scenario = load_adsb_scenario(scene_id, params);
    truth_all = scenario.truth_all;
    t1_grid = scenario.t1_grid;
    t2_grid = scenario.t2_grid;
    n_frames = min(scenario.n_frames, max_frames);

    fprintf('场景 %s：%d 条航迹，运行 %d 帧（dt=%.0fs）%s', ...
        scenario.name, scenario.n_targets, n_frames, params.dt_sec, newline);

    % ====== 点迹生成（每帧独立、可流式；此处一次性生成以便并行化）======
    % 系统偏差在本实验中置零（params_coverage_online 已设），不调用 ADS-B 标定。
    det1 = generate_detections(1, params, truth_all, t1_grid, n_frames, 0, 0);
    det2 = generate_detections(2, params, truth_all, t2_grid, n_frames, 0, 0);

    % ====== 滤波器模板 ======
    params_r1 = radar_params(params, 1);
    params_r2 = radar_params(params, 2);
    ukf1_tpl = ukf_imm('create', params_r1, params.radar1_lon, params.radar1_lat, ...
        params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
    ukf2_tpl = ukf_imm('create', params_r2, params.radar2_lon, params.radar2_lat, ...
        params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);

    % ====== 在线主循环 ======
    trackList1 = {};  temp1 = struct([]);  next_id1 = 1;
    trackList2 = {};  temp2 = struct([]);  next_id2 = 1;
    CT = central_track_manager('init', params);

    ctSnap = cell(n_frames, 1);
    snap1s = cell(n_frames, 1);
    snap2s = cell(n_frames, 1);
    logs = cell(n_frames, 1);

    % ---- 在线 NIS 协方差校准（CCF）状态 ----
    % 每站各维护一个 NIS 滑动窗口；κ_i = mean(NIS)/m（m=3 为量测维数）。
    % 滤波器过度自信（κ>1）时放大该站的 P，融合权重随之向更准的站倾斜。
    ccf_win = get_opt(opts, 'ccf_window', 60);
    nis1 = zeros(0, 1);  nis2 = zeros(0, 1);
    params.online_ccf_enable = get_opt(opts, 'ccf_enable', false);
    params.online_ccf_kappa1 = 1.0;
    params.online_ccf_kappa2 = 1.0;

    fid = [];
    if ~isempty(log_file)
        fid = fopen(log_file, 'w');
        if fid < 0
            error('run_online:logOpen', '无法写入日志文件：%s', log_file);
        end
        fprintf(fid, 'frame,n_local1,n_local2,n_ct,n_two,n_confirmed,n_new,n_merge\n');
    end

    tic;
    for k = 1:n_frames
        % ---- 单站跟踪（各站独立，逐帧因果）----
        [trackList1, temp1, snap1, next_id1, ~] = TRACK_MAIN_ORACLE( ...
            trackList1, temp1, det1{k}, ukf1_tpl, params_r1, k, next_id1, truth_all, t1_grid);
        [trackList2, temp2, snap2, next_id2, ~] = TRACK_MAIN_ORACLE( ...
            trackList2, temp2, det2{k}, ukf2_tpl, params_r2, k, next_id2, truth_all, t2_grid);

        % ---- 时间对齐（只用 R2 在 t 的状态回退，不读后续帧）----
        snap2a = time_align_tracks({snap2}, params);
        snap2a = snap2a{1};

        % ---- 在线 NIS 统计 → 协方差尺度因子 ----
        nis1 = push_nis(nis1, collect_nis(snap1), ccf_win);
        nis2 = push_nis(nis2, collect_nis(snap2a), ccf_win);
        params.online_ccf_kappa1 = nis_kappa(nis1, 3);
        params.online_ccf_kappa2 = nis_kappa(nis2, 3);

        % ---- 融合中心：分配 + 融合 + 系统航迹管理 ----
        [CT, logs{k}] = central_track_manager('step', CT, snap1, snap2a, k, params);
        logs{k}.ccf_kappa1 = params.online_ccf_kappa1;
        logs{k}.ccf_kappa2 = params.online_ccf_kappa2;

        snap1s{k} = snap1;
        snap2s{k} = snap2a;
        ctSnap{k} = CT;

        if ~isempty(fid)
            fprintf(fid, '%d,%d,%d,%d,%d,%d,%d,%d\n', k, logs{k}.n_local1, logs{k}.n_local2, ...
                logs{k}.n_ct, logs{k}.n_two, logs{k}.n_confirmed, logs{k}.n_new, logs{k}.n_merge);
        end

        if verbose && (k == 1 || mod(k, 50) == 0 || k == n_frames)
            fprintf('  frame %4d/%4d | R1航迹 %d, R2航迹 %d | 系统航迹 %d（双源 %d）%s', ...
                k, n_frames, numel(snap1.trackList), numel(snap2a.trackList), ...
                numel(CT), logs{k}.n_two, newline);
        end
    end
    elapsed = toc;
    if ~isempty(fid), fclose(fid); end
    fprintf('在线流程完成：%d 帧，用时 %.1f s（%.1f ms/帧）%s', ...
        n_frames, elapsed, 1000 * elapsed / n_frames, newline);

    % ====== 评估 ======
    lengthTable = evaluate_online_track_length(ctSnap, snap1s, snap2s, n_frames, ...
        scenario.labels);
    print_length_summary(lengthTable, params);

    metrics = evaluate_online_metrics(ctSnap, snap1s, snap2s, truth_all, ...
        scenario.labels, t1_grid, params);
    print_metric_summary(metrics);

    result = struct();
    result.scene = scenario.name;
    result.n_frames = n_frames;
    result.params = params;
    result.scenario = scenario;
    result.truth_all = truth_all;
    result.truthTrajs = scenario.truthTrajs;
    result.lengthTable = lengthTable;
    result.metrics = metrics;
    result.ctSnap = ctSnap;
    result.snap1s = snap1s;
    result.snap2s = snap2s;
    result.log = logs;
    result.elapsed_sec = elapsed;
end

% =====================================================================
function detList = generate_detections(radar_id, params, truth_all, t_grid, n_frames, dr_est, da_est)
    detList = cell(n_frames, 1);
    if radar_id == 1
        rng(params.random_seed + 1e7);
        rx_lon = params.radar1_lon; rx_lat = params.radar1_lat;
        tx_lon = params.radar1_tx_lon; tx_lat = params.radar1_tx_lat;
        range_bias = params.radar1_range_bias_m; az_bias = params.radar1_azimuth_bias_deg;
        beam_center = params.radar1_beam_center_deg;
        range_noise = params.radar1_range_noise_std_m; az_noise = params.radar1_azimuth_noise_std_deg;
    else
        rng(params.random_seed + 2e7);
        rx_lon = params.radar2_lon; rx_lat = params.radar2_lat;
        tx_lon = params.radar2_tx_lon; tx_lat = params.radar2_tx_lat;
        range_bias = params.radar2_range_bias_m; az_bias = params.radar2_azimuth_bias_deg;
        beam_center = params.radar2_beam_center_deg;
        range_noise = params.radar2_range_noise_std_m; az_noise = params.radar2_azimuth_noise_std_deg;
    end

    for k = 1:n_frames
        tgt_states = build_target_states_at_time(truth_all, t_grid(k));
        detRaw = generate_frame_detections_multi(rx_lon, rx_lat, tx_lon, tx_lat, tgt_states, ...
            k, t_grid(k), range_bias, az_bias, beam_center, params, range_noise, az_noise);
        dets = cell(1, length(detRaw));
        for d = 1:length(detRaw)
            dp = detRaw(d);
            Rgc = dp.prange - dr_est;
            azc = dp.paz - da_est;
            dp.drange = Rgc;  dp.daz = azc;
            dp.range_meas = Rgc;  dp.azimuth_meas = azc;
            if ~isfield(dp, 'radial_vel_meas') || isnan(dp.radial_vel_meas)
                dp.radial_vel_meas = dp.pvr;
            end
            if ~isfield(dp, 'lat') || isnan(dp.lat) || ~isfield(dp, 'lon') || isnan(dp.lon)
                [~, dp.lat, dp.lon] = bistatic_inverse_solver(Rgc, azc, tx_lon, tx_lat, ...
                    rx_lon, rx_lat, iono_h(params, 'assumed', radar_id));
            end
            [~, dp.raw_lat, dp.raw_lon] = bistatic_inverse_solver(dp.prange, dp.paz, ...
                tx_lon, tx_lat, rx_lon, rx_lat, iono_h(params, 'assumed', radar_id));
            dets{d} = dp;
        end
        if isempty(dets), detList{k} = []; else, detList{k} = [dets{:}]; end
    end
end

% =====================================================================
function print_length_summary(T, params)
    n = numel(T);
    if n == 0
        fprintf('没有可评估的航迹%s', newline);  return;
    end
    ct = [T.ct_len];  sg = [T.single];  rt = [T.ratio];
    ok = sg > 0 & ct > 0;

    fprintf('%s======== 航迹长度对比（%d 条真值）========%s', newline, n, newline);
    fprintf('单站基线 均值 %.1f 帧，系统航迹 均值 %.1f 帧%s', ...
        mean(sg(ok)), mean(ct(ok)), newline);
    fprintf('长度比值 均值 %.2f，中位 %.2f，最大 %.2f，最小 %.2f%s', ...
        mean(rt(ok)), median(rt(ok)), max(rt(ok)), min(rt(ok)), newline);
    fprintf('延长（比值>1）的目标 %d/%d，缩短（比值<1）%d 条%s', ...
        sum(rt(ok) > 1), sum(ok), sum(rt(ok) < 1), newline);

    d = [T.delay];
    d = d(~isnan(d));
    if ~isempty(d)
        fprintf('系统航迹建立 → 双源确认 的延迟：均值 %.1f 帧（%.0f s）%s', ...
            mean(d), mean(d) * params.dt_sec, newline);
    end

    fprintf('%s前 10 条明细（单站 / 系统 / 比值）：%s', newline, newline);
    [~, ord] = sort([T.single], 'descend');
    for i = ord(1:min(10, n))
        fprintf('  %-12s R1=%3d R2=%3d 单站=%3d 系统=%3d 比值=%.2f 确认帧=%s%s', ...
            T(i).label, T(i).r1_len, T(i).r2_len, T(i).single, T(i).ct_len, ...
            T(i).ratio, num2str(T(i).confirm_k), newline);
    end
end

% =====================================================================
function print_metric_summary(m)
    fprintf('%s======== 在线指标 ==========%s', newline, newline);
    if isfield(m, 'length_ratio_mean')
        fprintf('航迹长度比（系统/单站）均值 %.3f，中位 %.3f，范围 [%.2f, %.2f]%s', ...
            m.length_ratio_mean, m.length_ratio_median, m.length_ratio_min, m.length_ratio_max, newline);
        fprintf('延长目标 %d/%d，缩短 %d%s', m.n_longer, m.n_targets, m.n_shorter, newline);
    end
    fprintf('双源帧数 %d：R1 %.2f km，R2 %.2f km，系统 %.2f km（增益 %+.1f%%）%s', ...
        m.n_dual_frames, m.rmse_r1_dual, m.rmse_r2_dual, m.rmse_fused_dual, ...
        m.gain_dual_pct, newline);
    fprintf('两站一致性：马氏距离平方均值 %.2f（同源期望 2.0），位置分歧均值 %.2f km%s', ...
        m.d2_dual_mean, m.dis_dual_mean_km, newline);
    fprintf('两站误差互相关 rho：均值 %.3f，中位 %.3f（%d 条目标）%s', ...
        m.rho_dual_mean, m.rho_dual_median, m.n_rho_targets, newline);
    fprintf('确认配对 %d 对，正确 %d 对（查准率 %.1f%%，查全率 %.1f%%）%s', ...
        m.n_pair, m.n_pair_correct, m.pair_precision, m.pair_recall, newline);
end

function v = collect_nis(snap)
%COLLECT_NIS 取本帧有量测更新的航迹的归一化新息平方
    v = [];
    if isempty(snap) || ~isfield(snap, 'trackList'), return; end
    for i = 1:numel(snap.trackList)
        trk = snap.trackList{i};
        if isfield(trk, 'updated') && ~trk.updated, continue; end
        if ~isfield(trk, 'combined_nis'), continue; end
        n = double(trk.combined_nis);
        if isfinite(n) && n > 0, v(end+1) = n; end %#ok<AGROW>
    end
end

function buf = push_nis(buf, v, win)
%PUSH_NIS 把本帧 NIS 并入滑动窗口
    if isempty(v), return; end
    buf = [buf; v(:)];
    if numel(buf) > win, buf = buf(end-win+1:end); end
end

function k = nis_kappa(buf, m)
%NIS_KAPPA 由 NIS 均值估计协方差尺度因子
    if numel(buf) < 10
        k = 1.0;  return;
    end
    k = mean(buf) / m;
    k = min(max(k, 0.5), 5.0);
end

function v = get_opt(s, name, default)
    if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
        v = s.(name);
    else
        v = default;
    end
end
