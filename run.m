function result = run(scenario_name, params_override, opts)
%RUN 双照射源外辐射源雷达主链路（全在线，逐帧推进）
%
%   result = run(scenario_name)
%   result = run(scenario_name, params_override)
%   result = run(scenario_name, params_override, opts)
%
%   【主链路的九个阶段，逐帧推进】
%     Stage 0  场景构建         → 第2章 2.3  数据驱动场景
%     Stage 1  时空配准         → 第2章 2.4  三参数 MLE (dsv41) + h 反馈
%     Stage 2  点迹生成         → 第2章 2.2  双基地天波量测模型
%     Stage 3  单站跟踪         → 第3章      5 模型 IMM-UKF
%     Stage 4  时间对齐         → 第4章 4.1  CV 状态外推
%     Stage 5  跨站关联         → 第4章 4.2  在线分配与 M/N 序贯确认
%     Stage 6  航迹融合         → 第5章      SCC / BC / CI / FCI
%     Stage 7  指标与出图       → 第6章      6 项考核指标 + 航迹长度
%
%   【与旧批处理版的根本差别】
%     旧版按阶段扫全时间轴：整段对齐 → 整段匹配 → 整段融合，
%     匹配与融合内部都用到了未来帧（见 双站航迹配对与融合在线处理模型_20261028.md
%     第 12 节缺陷清单）。本版把 Stage 2~6 全部放进同一个逐帧循环，
%     第 k 帧的判决只允许使用第 k 帧及以前的信息。
%
%   【场景与部署】
%     scenario_name:
%       'multi_cross' / 'single_*' / 'left_*' 等 → 合成场景
%       'adsb_0002' / 'adsb_0608' / 'adsb_0911'   → 派生 ADS-B 场景
%       'adsb_ns'                              → 南北向派生场景
%       'adsb_ns_fr24'                         → FR24 原始文件南北向场景
%     部署由 params 决定：
%       simulation_params_oracle()   近共视（15° 视场两站几乎重合）
%       params_coverage_online()     覆盖互补（15° 视场两站错开 100 km）
%
%   【输出】
%     见 docs/主链路在线化重构契约_20261028.md 第 6 节。

    if nargin < 1 || isempty(scenario_name), scenario_name = 'multi_cross'; end
    if nargin < 2 || isempty(params_override), params_override = []; end
    if nargin < 3 || isempty(opts), opts = struct(); end

    % 代码目录显式加入搜索路径：不用 genpath（会扫描 results/ 下上万个 .mat）
    project_root = fileparts(mfilename('fullpath'));
    code_dirs = {'config','simulation','ukf','tracker','fusion', ...
                 'evaluation','utils','visualization','validation','experiments','io'};
    addpath(project_root);
    for cd_i = 1:numel(code_dirs)
        d_i = fullfile(project_root, code_dirs{cd_i});
        if exist(d_i, 'dir'), addpath(d_i); end
    end

    if isempty(params_override)
        params = simulation_params_oracle();
    else
        params = params_override;
    end
    rng(params.random_seed);
    % 点迹噪声用两站各自独立的随机流：与配准/跟踪的随机消耗解耦，两站量测相互独立
    detection_rng('init', params);

    stage_log = struct('stage', {}, 'elapsed_sec', {}, 'note', {});

    % =====================================================================
    % Stage 0：场景构建（第2章 2.3）
    % =====================================================================
    t_stage = tic;
    scenario = build_truth_scenario(scenario_name, params);
    truth_all = scenario.truth_all;
    truthTrajs = scenario.truthTrajs;
    t1_grid = scenario.t1_grid;
    t2_grid = scenario.t2_grid;
    n_frames = scenario.n_frames;
    max_frames = get_opt(opts, 'max_frames', Inf);
    n_frames = min(n_frames, max_frames);
    % 时间网格与实际运行帧数对齐（截断运行时要同步截断网格）
    t1_grid = t1_grid(1:n_frames);
    t2_grid = t2_grid(1:n_frames);
    fprintf('场景 %s：%d 条航迹，%d 帧（dt=%.0fs）%s', ...
        scenario.name, scenario.n_targets, n_frames, params.dt_sec, newline);
    stage_log(end+1) = struct('stage', 'S0_scenario', 'elapsed_sec', toc(t_stage), ...
        'note', sprintf('%d targets', scenario.n_targets)); %#ok<AGROW>

    % =====================================================================
    % Stage 1：时空配准（第2章 2.4）
    % =====================================================================
    t_stage = tic;
    [dr1_est, da1_est, dr2_est, da2_est, reg] = calibrate_radar_bias(params);
    % 三参数 MLE：把估计出的电离层虚高反馈给反演与滤波模型
    if strcmpi(reg.mode, 'mle3') && isfield(params, 'registration_apply_h') && params.registration_apply_h
        if isfinite(reg.h1_km), params.radar1_assumed_h_km = reg.h1_km; end
        if isfinite(reg.h2_km), params.radar2_assumed_h_km = reg.h2_km; end
        fprintf('  虚高已反馈到反演/滤波: R1=%.1fkm, R2=%.1fkm (真值 %.0fkm)%s', ...
            iono_h(params,'assumed',1)/1000, iono_h(params,'assumed',2)/1000, ...
            params.ionosphere_true_h_km, newline);
    end
    fprintf('R1 偏差估计: dr=%.0fm, da=%.2fdeg (真实: %.0fm, %.2fdeg)%s', ...
        dr1_est, da1_est, params.radar1_range_bias_m, params.radar1_azimuth_bias_deg, newline);
    fprintf('R2 偏差估计: dr=%.0fm, da=%.2fdeg (真实: %.0fm, %.2fdeg)%s', ...
        dr2_est, da2_est, params.radar2_range_bias_m, params.radar2_azimuth_bias_deg, newline);
    stage_log(end+1) = struct('stage', 'S1_registration', 'elapsed_sec', toc(t_stage), ...
        'note', reg.mode); %#ok<AGROW>

    % =====================================================================
    % Stage 2~6：逐帧主循环
    % =====================================================================
    % 滤波器模板（两站各一份）
    params_r1 = radar_params(params, 1);
    params_r2 = radar_params(params, 2);
    ukf1_tpl = ukf_imm('create', params_r1, params.radar1_lon, params.radar1_lat, ...
        params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
    ukf2_tpl = ukf_imm('create', params_r2, params.radar2_lon, params.radar2_lat, ...
        params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);

    % 单站跟踪状态（跨帧保持）
    trackList1 = {};  temp1 = struct([]);  next_id1 = 1;
    trackList2 = {};  temp2 = struct([]);  next_id2 = 1;

    % 融合中心状态（系统航迹 + 关联策略 + 融合算法）
    CT = central_track_manager('init', params);

    % 输出缓存
    trackSnapshots_R1 = cell(n_frames, 1);
    trackSnapshots_R2 = cell(n_frames, 1);
    aligned_R2       = cell(n_frames, 1);
    ctSnap           = cell(n_frames, 1);
    fused_snapshots  = cell(n_frames, 1);
    frameLog         = cell(n_frames, 1);
    detList_R1       = cell(n_frames, 1);
    detList_R2       = cell(n_frames, 1);
    pair_log         = struct('frame', {}, 'ct_id', {}, 'src1_id', {}, 'src2_id', {}, ...
                              'truth1', {}, 'truth2', {}, 'ok', {});

    verbose = get_opt(opts, 'verbose', false);
    log_file = get_opt(opts, 'log_file', '');
    fid = [];
    if ~isempty(log_file)
        fid = fopen(log_file, 'w');
        if fid < 0
            error('run:logOpen', '无法写入日志文件：%s', log_file);
        end
        fprintf(fid, 'frame,n_local1,n_local2,n_ct,n_two,n_confirmed,n_new,n_merge,n_cand,n_assigned\n');
    end

    t_stage = tic;
    for k = 1:n_frames
        % ---- Stage 2：点迹生成（双基地天波量测模型）----
        detRaw1 = generate_one_frame_detections(1, params, truth_all, t1_grid(k), k, dr1_est, da1_est);
        detRaw2 = generate_one_frame_detections(2, params, truth_all, t2_grid(k), k, dr2_est, da2_est);
        detList_R1{k} = detRaw1;
        detList_R2{k} = detRaw2;

        % ---- Stage 3：单站跟踪（5 模型 IMM-UKF，跨帧状态保持）----
        [trackList1, temp1, snap1, next_id1, ~] = TRACK_MAIN_ORACLE( ...
            trackList1, temp1, detRaw1, ukf1_tpl, params_r1, k, next_id1, truth_all, t1_grid);
        [trackList2, temp2, snap2, next_id2, ~] = TRACK_MAIN_ORACLE( ...
            trackList2, temp2, detRaw2, ukf2_tpl, params_r2, k, next_id2, truth_all, t2_grid);

        % ---- Stage 4：时间对齐（R2 状态回退到 R1 时间基准，只用本帧）----
        snap2a = time_align_tracks({snap2}, params);
        snap2a = snap2a{1};

        % ---- Stage 5+6：跨站关联 + 航迹融合（融合中心逐周期执行）----
        [CT, log] = central_track_manager('step', CT, snap1, snap2a, k, params);

        trackSnapshots_R1{k} = snap1;
        trackSnapshots_R2{k} = snap2;
        aligned_R2{k} = snap2a;
        ctSnap{k} = CT.CT;
        fused_snapshots{k} = CT.fused;
        frameLog{k} = log;

        % 配对事件汇总（跨帧）
        % 只取本帧新产生的事件：CT(i).pair_log 是累积列表，每帧全量搬一遍
        % 会变成 O(帧数^2) 的重复追加（1950 帧下内存膨胀到触发堆损坏）。
        for i = 1:numel(CT.CT)
            pl = CT.CT(i).pair_log;
            for p = 1:numel(pl)
                if pl{p}.frame == k
                    pair_log(end+1) = pl{p}; %#ok<AGROW>
                end
            end
        end

        if ~isempty(fid)
            fprintf(fid, '%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n', k, log.n_local1, log.n_local2, ...
                log.n_ct, log.n_two, log.n_confirmed, log.n_new, log.n_merge, ...
                log.n_cand, log.n_assigned);
        end
        if verbose && (k == 1 || mod(k, 50) == 0 || k == n_frames)
            fprintf('  frame %4d/%4d | R1 %d, R2 %d | 系统航迹 %d（双源 %d）%s', ...
                k, n_frames, log.n_local1, log.n_local2, log.n_ct, log.n_two, newline);
        end
    end
    if ~isempty(fid), fclose(fid); end
    stage_log(end+1) = struct('stage', 'S2_S6_online_loop', 'elapsed_sec', toc(t_stage), ...
        'note', sprintf('%d frames', n_frames)); %#ok<AGROW>

    % =====================================================================
    % Stage 7：指标与出图（第6章）
    % =====================================================================
    t_stage = tic;
    method_names = CT.method_names;

    % 单站跟踪误差（复用既有评估模块）
    errorStats_R1 = evaluate_all_multi('tracking_errors', trackSnapshots_R1, detList_R1, ...
        truthTrajs, t1_grid, t1_grid, 'R1');
    % aligned_R2 的状态已回退到 t1 时间基准，真值必须按 t1_grid 查；
    % 点迹仍生成于 t2_grid，故 detection_times 保持 t2_grid。
    errorStats_R2 = evaluate_all_multi('tracking_errors', aligned_R2, detList_R2, ...
        truthTrajs, t1_grid, t2_grid, 'R2');

    % 融合误差：把逐帧系统航迹转成"配对 × 算法"的传统结构，复用既有评估口径
    [fusion_eval, all_fused_snapshots] = evaluate_online_fusion(fused_snapshots, ...
        ctSnap, method_names, truthTrajs, n_frames, params);

    % 跨站配对结果（由配对事件汇总而来）
    matched_pairs = pairs_from_log(pair_log);

    % 航迹长度对比（系统航迹 vs 单站基线）
    % 合成场景（build_truth_scenario）没有 labels 字段，ADS-B 场景才有；
    % 原写法直接用 scenario.labels，导致 'multi_cross' 等合成场景在 Stage 7 报
    % "无法识别的字段名称 labels"（2026-10-28 复核发现）。
    if isfield(scenario, 'labels')
        labels = scenario.labels;
    else
        labels = {};
    end
    lengthTable = evaluate_online_track_length(ctSnap, trackSnapshots_R1, aligned_R2, n_frames, labels);

    % 六项论文指标（在线口径）
    % 先把已算出的部分装配成 result，再交给指标模块，保证口径与最终输出完全一致
    result = struct();
    result.params = params;
    result.scenario = scenario;
    result.truthTrajs = truthTrajs;
    result.trackSnapshots_R1 = trackSnapshots_R1;
    result.aligned_R2 = aligned_R2;
    result.ctSnap = ctSnap;
    result.fused_snapshots = fused_snapshots;
    result.method_names = method_names;
    result.matched_pairs = matched_pairs;
    result.errorStats_R1 = errorStats_R1;
    result.errorStats_R2 = errorStats_R2;
    result.fusion_eval = fusion_eval;
    result.lengthTable = lengthTable;
    result.t1_grid = t1_grid;
    result.t2_grid = t2_grid;
    result.n_frames = n_frames;

    metrics = evaluate_paper_metrics_online(result);
    stage_log(end+1) = struct('stage', 'S7_metrics', 'elapsed_sec', toc(t_stage), ...
        'note', 'paper metrics'); %#ok<AGROW>

    % =====================================================================
    % 组装输出（result 已在 Stage 7 前半段装配，这里只补齐剩余字段）
    % =====================================================================
    result.params = params;
    result.scenario = scenario;
    result.truth_all = truth_all;
    result.truthTrajs = truthTrajs;
    result.detList_R1 = detList_R1;
    result.detList_R2 = detList_R2;
    result.trackList_R1 = trackList1;
    result.trackList_R2 = trackList2;
    result.trackSnapshots_R1 = trackSnapshots_R1;
    result.trackSnapshots_R2 = trackSnapshots_R2;
    result.aligned_R2 = aligned_R2;
    result.ctSnap = ctSnap;
    result.fused_snapshots = fused_snapshots;
    result.method_names = method_names;
    result.all_fused_snapshots = all_fused_snapshots;
    result.matched_pairs = matched_pairs;
    result.pair_log = pair_log;
    result.fusion_eval = fusion_eval;
    result.errorStats_R1 = errorStats_R1;
    result.errorStats_R2 = errorStats_R2;
    result.lengthTable = lengthTable;
    result.metrics = metrics;
    result.registration = struct( ...
        'mode', reg.mode, 'dr1_m', dr1_est, 'da1_deg', da1_est, ...
        'dr2_m', dr2_est, 'da2_deg', da2_est, ...
        'h1_km', reg.h1_km, 'h2_km', reg.h2_km, ...
        'eta1', reg.eta1, 'eta2', reg.eta2, ...
        'crlb1', reg.crlb1, 'crlb2', reg.crlb2, ...
        'n1', reg.n1, 'n2', reg.n2, ...
        'h_true_km', params.ionosphere_true_h_km, ...
        'h_used1_km', iono_h(params,'assumed',1)/1000, ...
        'h_used2_km', iono_h(params,'assumed',2)/1000, ...
        'bias_true_R1', [params.radar1_range_bias_m, params.radar1_azimuth_bias_deg], ...
        'bias_true_R2', [params.radar2_range_bias_m, params.radar2_azimuth_bias_deg]);
    result.frameLog = frameLog;
    result.stage_log = stage_log;
    result.n_frames = n_frames;

    if get_opt(opts, 'save_results', false)
        save_results_if_needed(result, params, scenario.name);
    end
    if get_opt(opts, 'plot', false)
        try, plot_oracle_figures(result); catch ME, warning('run:plotFailed', '%s', ME.message); end
    end

    fprintf('%s主链路完成：%d 帧%s', newline, n_frames, newline);
end

% =====================================================================
% 逐帧点迹生成（Stage 2）
% =====================================================================
function detList = generate_one_frame_detections(radar_id, params, truth_all, t_now, frame_id, dr_est, da_est)
%GENERATE_ONE_FRAME_DETECTIONS 生成一帧的双基地量测点迹
%   只用当前帧的真值状态与当前帧的随机数，不涉及任何未来信息。
    if radar_id == 1
        rx_lon = params.radar1_lon; rx_lat = params.radar1_lat;
        tx_lon = params.radar1_tx_lon; tx_lat = params.radar1_tx_lat;
        range_bias = params.radar1_range_bias_m; az_bias = params.radar1_azimuth_bias_deg;
        beam_center = params.radar1_beam_center_deg;
        range_noise = params.radar1_range_noise_std_m; az_noise = params.radar1_azimuth_noise_std_deg;
    else
        rx_lon = params.radar2_lon; rx_lat = params.radar2_lat;
        tx_lon = params.radar2_tx_lon; tx_lat = params.radar2_tx_lat;
        range_bias = params.radar2_range_bias_m; az_bias = params.radar2_azimuth_bias_deg;
        beam_center = params.radar2_beam_center_deg;
        range_noise = params.radar2_range_noise_std_m; az_noise = params.radar2_azimuth_noise_std_deg;
    end

    tgt_states = build_target_states_at_time(truth_all, t_now);

    % 切到该站自己的随机流，出函数时自动切回全局流（见 simulation/detection_rng.m）
    rs = detection_rng('get', params, radar_id);
    prev_stream = RandStream.getGlobalStream();
    RandStream.setGlobalStream(rs);
    restore_stream = onCleanup(@() RandStream.setGlobalStream(prev_stream)); %#ok<NASGU>

    detRaw = generate_frame_detections_multi(rx_lon, rx_lat, tx_lon, tx_lat, tgt_states, ...
        frame_id, t_now, range_bias, az_bias, beam_center, params, range_noise, az_noise);

    dets = cell(1, length(detRaw));
    for d = 1:length(detRaw)
        dp = detRaw(d);
        Rgc = dp.prange - dr_est;               % 系统偏差校正：群距离
        azc = dp.paz - da_est;                  % 系统偏差校正：方位角
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
    if isempty(dets)
        detList = [];
    else
        detList = [dets{:}];
    end
end

% =====================================================================
% 配对事件 → 传统 matched_pairs 结构
% =====================================================================
function pairs = pairs_from_log(pair_log)
    pairs = struct('R1_track_id', {}, 'R2_track_id', {}, 'truth1', {}, 'truth2', {}, ...
                    'ok', {}, 'first_frame', {}, 'n_frames', {});
    if isempty(pair_log), return; end
    seen = containers.Map('KeyType', 'char', 'ValueType', 'double');
    for i = 1:numel(pair_log)
        ev = pair_log(i);
        key = sprintf('%.0f_%.0f', ev.src1_id, ev.src2_id);
        if isKey(seen, key)
            j = seen(key);
            pairs(j).n_frames = pairs(j).n_frames + 1;
        else
            seen(key) = numel(pairs) + 1;
            pairs(end+1) = struct('R1_track_id', ev.src1_id, 'R2_track_id', ev.src2_id, ...
                'truth1', ev.truth1, 'truth2', ev.truth2, 'ok', ev.ok, ...
                'first_frame', ev.frame, 'n_frames', 1); %#ok<AGROW>
        end
    end
end

% =====================================================================
% 指标模块用的临时结构
% =====================================================================
function v = get_opt(s, name, default)
    if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
        v = s.(name);
    else
        v = default;
    end
end
