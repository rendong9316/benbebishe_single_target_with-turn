function result = run(scenario_name)
    % 主入口：双基地 OTH-SWR 单/多目标跟踪仿真全流程
    %
    % 完整流水线（Phase 0-10）：
    %   Phase 0: Oracle 场景初始化（参数 + 真值航迹）
    %   Phase 1: ADS-B 数据标定系统偏差（dr, da）
    %   Phase 2: 多目标点迹生成 + 偏差校正
    %   Phase 3: UKF/IMM 滤波器模板初始化
    %   Phase 4: 南阳式 Oracle 航迹维护（每站独立）
    %   Phase 5: 跨雷达航迹时间对齐（R2 → R1 时间基准）
    %   Phase 6: 跨雷达航迹匹配（双门限法 / 传统法）
    %   Phase 7: 航迹级融合（SCC/BC/CI/FCI 四种算法）
    %   Phase 8: RMSE 定量误差评估（单站 + 融合）
    %   Phase 9: 可视化绘图
    %   Phase 10: 数据保存

    if nargin < 1 || isempty(scenario_name)
        scenario_name = 'single_turn';
    end

    addpath(genpath('.'));

    fprintf('========== Phase 0: Oracle 场景初始化 ==========%s', newline);
    % 加载全局参数（雷达几何、UKF 噪声、IMM 转移概率、双门限配置等）
    params = simulation_params_oracle();
    rng(params.random_seed);
    % 根据场景名生成真值航迹（支持 single_straight / single_turn / single_uturn / multi_cross）
    scenario = build_truth_scenario(scenario_name, params);
    truth_all = scenario.truth_all;
    truthTrajs = scenario.truthTrajs;
    t1_grid = scenario.t1_grid;
    t2_grid = scenario.t2_grid;
    n_frames = scenario.n_frames;
    fprintf('场景: %s | 目标数=%d | 帧数=%d | dt=%.0fs%s', scenario.name, scenario.n_targets, n_frames, params.dt_sec, newline);
    fprintf('雷达硬约束: Pd=%.2f, Pfa=%.4f%s', params.detection_probability, params.false_alarm_rate, newline);
    print_truth_summary(truthTrajs);

    fprintf('%s========== Phase 1: ADS-B 系统偏差标定 ==========%s', newline, newline);
    % 从 ADS-B CSV 数据中采样目标位置，计算 R1/R2 的距离和方位偏差估计
    % 原理：ADS-B 提供高精度位置 → 计算理论群距离/方位角 → 与实际雷达量测比较 → 取均值
    [dr1_est, da1_est, dr2_est, da2_est] = calibrate_bias(params);
    fprintf('R1 偏差估计: dr=%.0fm, da=%.2fdeg (真实: %.0fm, %.2fdeg)%s', ...
        dr1_est, da1_est, params.radar1_range_bias_m, params.radar1_azimuth_bias_deg, newline);
    fprintf('R2 偏差估计: dr=%.0fm, da=%.2fdeg (真实: %.0fm, %.2fdeg)%s', ...
        dr2_est, da2_est, params.radar2_range_bias_m, params.radar2_azimuth_bias_deg, newline);

    fprintf('%s========== Phase 2: 多目标点迹生成 + 偏差校正 ==========%s', newline, newline);
    % 对每个雷达站，逐帧生成检测点迹（目标+杂波），然后减去估计的系统偏差
    detList_R1 = generate_radar_detections(1, params, truth_all, t1_grid, n_frames, dr1_est, da1_est);
    detList_R2 = generate_radar_detections(2, params, truth_all, t2_grid, n_frames, dr2_est, da2_est);
    print_detection_summary(detList_R1, 'R1');
    print_detection_summary(detList_R2, 'R2');

    fprintf('%s========== Phase 3: UKF/IMM 模板初始化 ==========%s', newline, newline);
    % 从全局参数中提取 R1/R2 各自的雷达专属参数（噪声/Q/门限）
    % 然后通过 ukf_imm('create') 创建 IMM UKF 模板（内含 CV+CT 两个子 UKF）
    params_r1 = radar_params(params, 1);
    params_r2 = radar_params(params, 2);
    ukf1_tpl = ukf_imm('create', params_r1, params.radar1_lon, params.radar1_lat, ...
        params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
    ukf2_tpl = ukf_imm('create', params_r2, params.radar2_lon, params.radar2_lat, ...
        params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);
    fprintf('R1 IMM: Q=%.2g, range_std=%.0fm, az_std=%.2fdeg%s', ...
        params_r1.ukf_Q_scale, params_r1.ukf_range_std_m, params_r1.ukf_azimuth_std_deg, newline);
    fprintf('R2 IMM: Q=%.2g, range_std=%.0fm, az_std=%.2fdeg%s', ...
        params_r2.ukf_Q_scale, params_r2.ukf_range_std_m, params_r2.ukf_azimuth_std_deg, newline);

    fprintf('%s========== Phase 4: 南阳式 Oracle 航迹维护 ==========%s', newline, newline);
    % 对每个雷达的校准点迹序列，执行完整的 Oracle 航迹处理：
    %   航迹生命周期管理 → UKF预测 → Oracle关联 → UKF更新 → 航迹起始
    % 返回：航迹列表 + 临时起始缓冲区 + 逐帧快照 + 诊断信息
    [trackList_R1, tempTrackList_R1, trackSnapshots_R1, diag_R1] = run_oracle_tracker( ...
        detList_R1, ukf1_tpl, params_r1, truth_all, t1_grid);
    [trackList_R2, tempTrackList_R2, trackSnapshots_R2, diag_R2] = run_oracle_tracker( ...
        detList_R2, ukf2_tpl, params_r2, truth_all, t2_grid);
    print_track_summary(trackList_R1, 'R1', params);
    print_track_summary(trackList_R2, 'R2', params);
    % 验证 Oracle 不变量：点迹不重复消耗、快照字段完整、生命周期事件合法等
    validate_oracle_invariants(trackSnapshots_R1, detList_R1, diag_R1, params_r1, trackList_R1);
    validate_oracle_invariants(trackSnapshots_R2, detList_R2, diag_R2, params_r2, trackList_R2);
    fprintf('Oracle lifecycle invariants: R1/R2 通过%s', newline);

    fprintf('%s========== Phase 5: 航迹级时间对齐 ==========%s', newline, newline);
    % R2 比 R1 晚 dt_offset 秒采样，需要将 R2 航迹状态回退到 R1 的时间网格
    % 使用 CV 模型逆向传播：x(t-Δt) = F(-Δt) * x(t), P(t-Δt) = FPF' + Q_scaled
    aligned_R2 = time_align_tracks(trackSnapshots_R2, params);
    fprintf('R2 已回退对齐到 R1 时间基准: %.1fs%s', params.time_offset_radar2_sec, newline);

    fprintf('%s========== Phase 6: 跨雷达航迹匹配 ==========%s', newline, newline);
    % 根据配置的匹配方法（dualgate 或传统 matcher）对 R1/R2 航迹进行配对
    % 匹配结果包含：共现帧数、平均距离、质量评分等
    if isfield(params, 'track_matcher_method') && strcmp(params.track_matcher_method, 'dualgate')
        matched_pairs_struct = track_matcher_dualgate(trackSnapshots_R1, aligned_R2, params);
    else
        matched_pairs_struct = track_matcher(trackSnapshots_R1, aligned_R2, params);
    end
    print_match_summary(matched_pairs_struct);

    fprintf('%s========== Phase 7: 航迹融合 ==========%s', newline, newline);
    % 对每对匹配航迹，依次执行 SCC/BC/CI/FCI 四种融合算法
    % 输出：all_fused_snapshots{pair_idx, method_idx}[frame] → 融合航迹快照
    method_names = {'SCC', 'BC', 'CI', 'FCI'};
    all_fused_snapshots = cell(length(matched_pairs_struct), length(method_names));
    for p = 1:length(matched_pairs_struct)
        fprintf('融合匹配对 %d/%d: R1#%d <-> R2#%d%s', p, length(matched_pairs_struct), ...
            matched_pairs_struct(p).R1_track_id, matched_pairs_struct(p).R2_track_id, newline);
        for m = 1:length(method_names)
            all_fused_snapshots{p, m} = run_track_fusion(matched_pairs_struct(p), ...
                trackSnapshots_R1, aligned_R2, params, method_names{m});
        end
    end
    fprintf('融合完成: %d 对 x %d 算法%s', length(matched_pairs_struct), length(method_names), newline);

    fprintf('%s========== Phase 8: RMSE 定量误差评估 ==========%s', newline, newline);
    % 构建匹配上下文（ID→位置立方体映射），用于融合评估中的 pair→aircraft 映射
    matcher_multi = build_matcher_context(matched_pairs_struct, trackSnapshots_R1, aligned_R2, n_frames);
    % 融合 RMSE：每种算法在所有匹配对上的误差统计
    fusion_eval = evaluate_all_multi('fusion', all_fused_snapshots, method_names, ...
        matched_pairs_struct, trackSnapshots_R1, aligned_R2, truthTrajs, n_frames, params.dt_sec, matcher_multi);
    % 单站 RMSE：R1/R2 各自的 UKF 跟踪误差
    errorStats_R1 = evaluate_all_multi('tracking_errors', trackSnapshots_R1, detList_R1, ...
        truthTrajs, n_frames, params.dt_sec, 'R1');
    errorStats_R2 = evaluate_all_multi('tracking_errors', aligned_R2, detList_R2, ...
        truthTrajs, n_frames, params.dt_sec, 'R2');
    print_tracking_rmse(errorStats_R1);
    print_tracking_rmse(errorStats_R2);
    print_fusion_rmse(fusion_eval);

    % ---- 组装返回结果 ----
    result = struct();
    result.params = params;
    result.scenario = scenario;
    result.truth_all = truth_all;
    result.truthTrajs = truthTrajs;
    result.detList_R1 = detList_R1;
    result.detList_R2 = detList_R2;
    result.trackList_R1 = trackList_R1;
    result.trackList_R2 = trackList_R2;
    result.tempTrackList_R1 = tempTrackList_R1;
    result.tempTrackList_R2 = tempTrackList_R2;
    result.trackSnapshots_R1 = trackSnapshots_R1;
    result.trackSnapshots_R2 = trackSnapshots_R2;
    result.aligned_R2 = aligned_R2;
    result.diag_R1 = diag_R1;
    result.diag_R2 = diag_R2;
    result.matched_pairs = matched_pairs_struct;
    result.method_names = method_names;
    result.all_fused_snapshots = all_fused_snapshots;
    result.fusion_eval = fusion_eval;
    result.errorStats_R1 = errorStats_R1;
    result.errorStats_R2 = errorStats_R2;

    fprintf('%s========== Phase 9: Figure 图窗可视化 ==========%s', newline, newline);
    plot_oracle_figures(result);

    fprintf('%s========== Phase 10: 数据保存 ==========%s', newline, newline);
    save_results_if_needed(result, params, scenario.name);
    fprintf('%sDone.%s', newline, newline);
end

function print_truth_summary(truthTrajs)
    for a = 1:length(truthTrajs)
        tt = truthTrajs{a};
        fprintf('目标%s: 点数=%d, 时长=%.0fs, 起点=(%.2fE, %.2fN), 终点=(%.2fE, %.2fN)%s', ...
            tt.label, length(tt.time_sec), tt.time_sec(end)-tt.time_sec(1), ...
            tt.lon(1), tt.lat(1), tt.lon(end), tt.lat(end), newline);
    end
end

function print_detection_summary(detList, label)
    total = 0;
    target = 0;
    clutter = 0;
    for k = 1:length(detList)
        dets = detList{k};
        total = total + length(dets);
        for i = 1:length(dets)
            if isfield(dets(i), 'is_clutter') && dets(i).is_clutter
                clutter = clutter + 1;
            else
                target = target + 1;
            end
        end
    end
    fprintf('%s 点迹: 总数=%d, 真实=%d, 虚警=%d, 平均每帧=%.2f%s', ...
        label, total, target, clutter, total / max(length(detList), 1), newline);
end

function print_track_summary(trackList, label, params)
    n_reliable = 0;
    n_temporary = 0;
    n_history = 0;
    for i = 1:length(trackList)
        type = trackList{i}.type;
        if type == params.RELIABLE_TRACK
            n_reliable = n_reliable + 1;
        elseif type == params.TEMPORARY_TRACK
            n_temporary = n_temporary + 1;
        elseif type == params.HISTORY_TRACK
            n_history = n_history + 1;
        end
    end
    fprintf('%s 航迹: 总数=%d, 可靠=%d, 临时=%d, 历史=%d%s', ...
        label, length(trackList), n_reliable, n_temporary, n_history, newline);
end

function print_match_summary(matched_pairs)
    fprintf('匹配到 %d 对航迹%s', length(matched_pairs), newline);
    for p = 1:length(matched_pairs)
        mp = matched_pairs(p);
        if isfield(mp, 'mean_dist_km')
            fprintf('  Pair %d: R1#%d <-> R2#%d, 共现=%d帧, 平均距离=%.1fkm%s', ...
                p, mp.R1_track_id, mp.R2_track_id, mp.coexist_count, mp.mean_dist_km, newline);
        else
            fprintf('  Pair %d: R1#%d <-> R2#%d%s', p, mp.R1_track_id, mp.R2_track_id, newline);
        end
    end
end

function print_tracking_rmse(errorStats)
    fprintf('%s 单站跟踪 RMSE:%s', errorStats.radar, newline);
    for a = 1:length(errorStats.summary)
        s = errorStats.summary(a).ukf;
        fprintf('  目标%d: n=%d, median=%.1fkm, mean=%.1fkm, RMSE=%.1fkm, 95%%=%.1fkm%s', ...
            a, s.n, s.median, s.mean, s.rms, s.pct95, newline);
    end
    s = errorStats.overall.ukf;
    fprintf('  Overall: n=%d, median=%.1fkm, mean=%.1fkm, RMSE=%.1fkm, 95%%=%.1fkm%s', ...
        s.n, s.median, s.mean, s.rms, s.pct95, newline);
end

function print_fusion_rmse(fusion_eval)
    fprintf('融合 RMSE 对比:%s', newline);
    fprintf('  %-8s %8s %8s %8s %8s%s', '算法', 'n', 'median', 'mean', 'RMSE', newline);
    for m = 1:length(fusion_eval.overall)
        s = fusion_eval.overall(m).s;
        fprintf('  %-8s %8d %8.1f %8.1f %8.1f%s', ...
            fusion_eval.overall(m).method, s.n, s.median, s.mean, s.rms, newline);
    end
end

function [dr1_est, da1_est, dr2_est, da2_est] = calibrate_bias(params)
    rng(params.random_seed);
    T_adsb = readtable(params.adsb_csv_path, 'ReadVariableNames', false);
    adsb_lat = T_adsb.Var2;
    adsb_lon = T_adsb.Var3;
    dr1_list = [];
    da1_list = [];
    dr2_list = [];
    da2_list = [];
    n_check = min(5000, height(T_adsb));
    cal_step = max(1, floor(height(T_adsb) / n_check));
    for idx = 1:cal_step:height(T_adsb)
        t_lon = adsb_lon(idx);
        t_lat = adsb_lat(idx);
        if isnan(t_lon) || isnan(t_lat)
            continue;
        end
        [in1, ~, ~] = radar_coverage_check(params.radar1_lon, params.radar1_lat, ...
            t_lon, t_lat, params.radar1_beam_center_deg, params);
        if in1
            Rg_true = skywave_geometry('group_range', params.radar1_tx_lon, params.radar1_tx_lat, ...
                params.radar1_lon, params.radar1_lat, t_lon, t_lat);
            az_true = sphere_utils_azimuth(params.radar1_lon, params.radar1_lat, t_lon, t_lat);
            Rg_meas = Rg_true + params.radar1_range_bias_m + randn() * params.radar1_range_noise_std_m;
            az_meas = az_true + params.radar1_azimuth_bias_deg + randn() * params.radar1_azimuth_noise_std_deg;
            dr1_list(end+1) = Rg_meas - Rg_true;
            da1_list(end+1) = wrap_angle_run(az_meas - az_true);
        end
        [in2, ~, ~] = radar_coverage_check(params.radar2_lon, params.radar2_lat, ...
            t_lon, t_lat, params.radar2_beam_center_deg, params);
        if in2
            Rg_true = skywave_geometry('group_range', params.radar2_tx_lon, params.radar2_tx_lat, ...
                params.radar2_lon, params.radar2_lat, t_lon, t_lat);
            az_true = sphere_utils_azimuth(params.radar2_lon, params.radar2_lat, t_lon, t_lat);
            Rg_meas = Rg_true + params.radar2_range_bias_m + randn() * params.radar2_range_noise_std_m;
            az_meas = az_true + params.radar2_azimuth_bias_deg + randn() * params.radar2_azimuth_noise_std_deg;
            dr2_list(end+1) = Rg_meas - Rg_true;
            da2_list(end+1) = wrap_angle_run(az_meas - az_true);
        end
    end
    if isempty(dr1_list) || isempty(da1_list)
        error('run:calibrationNoSamples', 'R1 ADS-B 标定没有雷达覆盖内的有效样本');
    end
    if isempty(dr2_list) || isempty(da2_list)
        error('run:calibrationNoSamples', 'R2 ADS-B 标定没有雷达覆盖内的有效样本');
    end
    dr1_est = mean(dr1_list);
    da1_est = mean(da1_list);
    dr2_est = mean(dr2_list);
    da2_est = mean(da2_list);
    if any(~isfinite([dr1_est, da1_est, dr2_est, da2_est]))
        error('run:calibrationInvalid', 'ADS-B 标定结果包含非有限值');
    end
end

function detList = generate_radar_detections(radar_id, params, truth_all, t_grid, n_frames, dr_est, da_est)
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
            dp.drange = Rgc;
            dp.daz = azc;
            dp.range_meas = Rgc;
            dp.azimuth_meas = azc;
            if ~isfield(dp, 'radial_vel_meas') || isnan(dp.radial_vel_meas)
                dp.radial_vel_meas = dp.pvr;
            end
            if ~isfield(dp, 'lat') || isnan(dp.lat) || ~isfield(dp, 'lon') || isnan(dp.lon)
                [~, dp.lat, dp.lon] = bistatic_inverse_solver(Rgc, azc, tx_lon, tx_lat, rx_lon, rx_lat);
            end
            [~, dp.raw_lat, dp.raw_lon] = bistatic_inverse_solver(dp.prange, dp.paz, tx_lon, tx_lat, rx_lon, rx_lat);
            dets{d} = dp;
        end
        if isempty(dets)
            detList{k} = [];
        else
            detList{k} = [dets{:}];
        end
    end
end

function [trackList, tempTrackList, trackSnapshots, diagList] = run_oracle_tracker(detList, ukf_tpl, params, truth_all, t_grid)
    n_frames = length(detList);
    trackSnapshots = cell(n_frames, 1);
    diagList = cell(n_frames, 1);
    trackList = {};
    tempTrackList = struct([]);
    next_id = 1;
    for k = 1:n_frames
        [trackList, tempTrackList, trackSnapshots{k}, next_id, diagList{k}] = TRACK_MAIN_ORACLE( ...
            trackList, tempTrackList, detList{k}, ukf_tpl, params, k, next_id, truth_all, t_grid);
        if k == 1 || mod(k, 10) == 0 || k == n_frames
            fprintf('  frame %3d/%3d: active=%d, total=%d%s', k, n_frames, count_active_tracks(trackList), length(trackList), newline);
        end
    end
    trackList = sortTrackList_oracle(trackList);
end

function n = count_active_tracks(trackList)
    n = 0;
    for i = 1:length(trackList)
        if trackList{i}.type ~= 7
            n = n + 1;
        end
    end
end

function matcher = build_matcher_context(matched_pairs, trackSnapshots_R1, aligned_R2, n_frames)
    r1_ids = [];
    r2_ids = [];
    for k = 1:n_frames
        r1_ids = append_track_rows(r1_ids, trackSnapshots_R1{k}, k);
        r2_ids = append_track_rows(r2_ids, aligned_R2{k}, k);
    end
    unique_r1_ids = unique_or_empty(r1_ids);
    unique_r2_ids = unique_or_empty(r2_ids);
    r1_pos = build_pos_cube(r1_ids, unique_r1_ids, n_frames);
    r2_pos = build_pos_cube(r2_ids, unique_r2_ids, n_frames);
    matcher = struct('matched_pairs', matched_pairs, 'aligned_R2', {aligned_R2}, ...
        'r1_ids', r1_ids, 'r2_ids', r2_ids, 'unique_r1_ids', unique_r1_ids, ...
        'unique_r2_ids', unique_r2_ids, 'r1_pos', r1_pos, 'r2_pos', r2_pos);
end

function rows = append_track_rows(rows, snap, k)
    if isempty(snap) || ~isfield(snap, 'trackList')
        return;
    end
    for t = 1:length(snap.trackList)
        trk = snap.trackList{t};
        if trk.type ~= 7 && ~isnan(trk.lat)
            rows(end+1, :) = [trk.id, k, trk.lon, trk.lat];
        end
    end
end

function ids = unique_or_empty(rows)
    if isempty(rows)
        ids = [];
    else
        ids = unique(rows(:, 1));
    end
end

function pos = build_pos_cube(rows, ids, n_frames)
    pos = nan(length(ids), n_frames, 2);
    for i = 1:length(ids)
        rid = ids(i);
        rid_rows = rows(rows(:, 1) == rid, :);
        for r = 1:size(rid_rows, 1)
            fk = round(rid_rows(r, 2));
            if fk >= 1 && fk <= n_frames
                pos(i, fk, 1) = rid_rows(r, 3);
                pos(i, fk, 2) = rid_rows(r, 4);
            end
        end
    end
end

function plot_oracle_figures(result)
    if ~exist('results', 'dir')
        mkdir('results');
    end

    [true_track_A, true_track_B, true_track_C] = truth_tracks_for_legacy_plots(result.truth_all);
    params = result.params;
    warn_state = warning('off', 'all');

    plot_scene_overview_multi(true_track_A, true_track_B, true_track_C, params, 'results');

    plot_point_cloud_3d(result.detList_R1, 'R1', 'results/fig2a_R1_point_cloud.png');
    plot_point_cloud_3d(result.detList_R2, 'R2', 'results/fig2b_R2_point_cloud.png');

    plot_results_multi('single_track', true_track_A, true_track_B, true_track_C, ...
        result.detList_R1, result.detList_R2, result.trackSnapshots_R1, result.trackSnapshots_R2, ...
        params, 'results');

    matched_pairs_cell = cell(length(result.matched_pairs), 1);
    for p = 1:length(result.matched_pairs)
        matched_pairs_cell{p} = result.matched_pairs(p);
    end
    plot_results_multi('single_fusion', true_track_A, true_track_B, true_track_C, ...
        result.trackSnapshots_R1, result.aligned_R2, result.all_fused_snapshots, ...
        result.method_names, matched_pairs_cell, result.fusion_eval, result.truthTrajs, params, 'results');

    warning(warn_state);
end

function [true_track_A, true_track_B, true_track_C] = truth_tracks_for_legacy_plots(truth_all)
    empty_track = nan(1, 5);
    true_track_A = empty_track;
    true_track_B = empty_track;
    true_track_C = empty_track;
    if length(truth_all) >= 1 && ~isempty(truth_all{1})
        true_track_A = truth_all{1};
    end
    if length(truth_all) >= 2 && ~isempty(truth_all{2})
        true_track_B = truth_all{2};
    end
    if length(truth_all) >= 3 && ~isempty(truth_all{3})
        true_track_C = truth_all{3};
    end
end

function ax = make_geo_axes(fig)
    try
        ax = geoaxes('Parent', fig, 'Basemap', 'darkwater');
    catch
        ax = geoaxes('Parent', fig);
    end
end

function plot_detections_geo(ax, detList, color, label)
    lat = [];
    lon = [];
    for k = 1:length(detList)
        dets = detList{k};
        for i = 1:length(dets)
            if isfield(dets(i), 'lat') && isfield(dets(i), 'lon') && ~isnan(dets(i).lat) && ~isnan(dets(i).lon)
                lat(end+1) = dets(i).lat;
                lon(end+1) = dets(i).lon;
            end
        end
    end
    if ~isempty(lat)
        geoplot(ax, lat, lon, '.', 'Color', color, 'MarkerSize', 5, 'DisplayName', label);
    end
end

function plot_tracks_geo(ax, snapshots, color, label_prefix)
    ids = collect_track_ids(snapshots);
    for i = 1:length(ids)
        [lat, lon] = collect_track_line(snapshots, ids(i));
        if length(lat) > 1
            geoplot(ax, lat, lon, '-o', 'Color', color, 'LineWidth', 1.5, 'MarkerSize', 3, ...
                'DisplayName', sprintf('%s#%d', label_prefix, ids(i)));
        end
    end
end

function ids = collect_track_ids(snapshots)
    ids = [];
    for k = 1:length(snapshots)
        if isempty(snapshots{k}) || ~isfield(snapshots{k}, 'trackList')
            continue;
        end
        for i = 1:length(snapshots{k}.trackList)
            trk = snapshots{k}.trackList{i};
            if trk.type ~= 7 && ~isnan(trk.lat)
                ids(end+1) = trk.id;
            end
        end
    end
    ids = unique(ids);
end

function [lat, lon] = collect_track_line(snapshots, id)
    lat = [];
    lon = [];
    for k = 1:length(snapshots)
        if isempty(snapshots{k}) || ~isfield(snapshots{k}, 'trackList')
            continue;
        end
        for i = 1:length(snapshots{k}.trackList)
            trk = snapshots{k}.trackList{i};
            if trk.id == id && trk.type ~= 7 && ~isnan(trk.lat)
                lat(end+1) = trk.lat;
                lon(end+1) = trk.lon;
                break;
            end
        end
    end
end

function [best_idx, best_method, best_rmse] = best_fusion_method(fusion_eval, method_names)
    best_idx = 1;
    best_rmse = inf;
    for i = 1:length(method_names)
        if i <= length(fusion_eval.overall) && fusion_eval.overall(i).s.rms < best_rmse
            best_rmse = fusion_eval.overall(i).s.rms;
            best_idx = i;
        end
    end
    best_method = method_names{best_idx};
end

function plot_fused_geo(ax, all_fused_snapshots, method_idx, color, method_name)
    for p = 1:size(all_fused_snapshots, 1)
        snaps = all_fused_snapshots{p, method_idx};
        if isempty(snaps)
            continue;
        end
        lat = [];
        lon = [];
        for k = 1:length(snaps)
            trks = snaps{k}.trackList;
            for i = 1:length(trks)
                trk = trks{i};
                if isfield(trk, 'lat') && isfield(trk, 'lon') && ~isnan(trk.lat) && ~isnan(trk.lon)
                    lat(end+1) = trk.lat;
                    lon(end+1) = trk.lon;
                end
            end
        end
        if length(lat) > 1
            geoplot(ax, lat, lon, '-d', 'Color', color, 'LineWidth', 2.2, 'MarkerSize', 4, ...
                'DisplayName', sprintf('%s Pair#%d', method_name, p));
        end
    end
end

function save_results_if_needed(result, params, scenario_name)
    if ~exist('results', 'dir')
        mkdir('results');
    end
    outf = fullfile('results', sprintf('simulation_oracle_%s_%s.mat', scenario_name, datestr(now, 'yyyymmdd_HHMMSS')));
    save(outf, '-struct', 'result');
    fprintf('数据已保存: %s%s', outf, newline);
end

function a = wrap_angle_run(a)
    while a > 180
        a = a - 360;
    end
    while a < -180
        a = a + 360;
    end
end
