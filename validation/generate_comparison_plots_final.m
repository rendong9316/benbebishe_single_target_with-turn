% generate_comparison_plots_final — 全场景滤波前后对比可视化
% Usage: matlab -batch "addpath(genpath('D:/Desktop/single_target_with-turn')); generate_comparison_plots_final();"
% 数据策略: RMSE = 2雷达 × 10 seeds平均（与最终验证一致的量级）
%         轨迹图 = radar R1, seed=10001

function generate_comparison_plots_final()
    addpath(genpath('D:/Desktop/single_target_with-turn'));

    seed_start = 10001;
    n_seeds_rmse = 10;
    n_scenarios = 10;
    output_dir = 'D:\Desktop\single_target_with-turn\comparison_plots';
    if ~isdir(output_dir), mkdir(output_dir); end

    specs = { ...
        {'single_straight',       'straight',       '直线航迹',         1.0}, ...
        {'single_turn_left_short','left_short',     '左折短转',         1.0}, ...
        {'single_turn_right_short','right_short',   '右折短转',         1.0}, ...
        {'single_turn_left_sustained','left_sustained','左转持续',       1.0}, ...
        {'single_turn_right_sustained','right_sustained','右转持续',     1.0}, ...
        {'multi_cross',            'multi_cross',   '多目标交叉',       1.0}, ...
        {'single_turn_left_sustained','left_rate_0p7','左转缓速率(0.7)', 0.7}, ...
        {'single_turn_right_sustained','right_rate_0p7','右转缓速率(0.7)',0.7}, ...
        {'single_turn_left_sustained','left_rate_1p3','左转急速率(1.3)', 1.3}, ...
        {'single_turn_right_sustained','right_rate_1p3','右转急速率(1.3)',1.3}};

    weights = [0.15, 0.08, 0.08, 0.08, 0.08, 0.08, 0.06, 0.06, 0.07, 0.07];

    cfg51 = struct(...
        'imm_cv_dwell_time_sec', 2500, ...
        'imm_ct_dwell_time_sec', 660, ...
        'imm_ct_fixed_Q_scale', 5.3, ...
        'imm_transient_gain_max', 11.0, ...
        'imm_transient_nis_start', 3.0, ...
        'imm_transient_nis_full', 12.0, ...
        'imm_transient_ewma_alpha', 0.65, ...
        'imm_mu_init_CV', 0.5);

    fprintf('============================================================\n');
    fprintf('全场景对比 (2雷达×10 seeds平均)\n');
    fprintf('============================================================\n\n');

    rmse_default_all = zeros(n_scenarios, 1);
    rmse_cfg51_all = zeros(n_scenarios, 1);

    for si = 1:n_scenarios
        scenario_name = specs{si}{1};
        short_name = specs{si}{2};
        chinese_name = specs{si}{3};
        turn_rate = specs{si}{4};

        fprintf('[%d/%d] %s (%s)...\n', si, n_scenarios, chinese_name, short_name);

        prepared = prepare_oracle_tracking_inputs(scenario_name, ...
            struct('random_seed', seed_start, ...
                   'truth_turn_rate_deg_per_sec', turn_rate));
        p = prepared.params;

        % --- Default RMSE ---
        rmse_d = 0;
        for r = 1:2
            for sk = 1:n_seeds_rmse
                rv = seed_start + sk - 1;
                rmse_d = rmse_d + eval_r1(prepared, p, r, rv, []);
            end
        end
        rmse_default_all(si) = rmse_d / (2 * n_seeds_rmse);

        % --- Config#51 RMSE ---
        rmse_c = 0;
        for r = 1:2
            for sk = 1:n_seeds_rmse
                rv = seed_start + sk - 1;
                ov = overlay_cfg(struct(), cfg51);
                ov.imm_adapt_mode = '3in1';
                ov.random_seed = rv;
                rmse_c = rmse_c + eval_r1(prepared, p, r, seed_start + sk - 1, ov);
            end
        end
        rmse_cfg51_all(si) = rmse_c / (2 * n_seeds_rmse);

        fprintf('  Default: %.3f km, Config#51: %.3f km, Delta: %+.1f%%\n', ...
            rmse_default_all(si), rmse_cfg51_all(si), ...
            (rmse_default_all(si) - rmse_cfg51_all(si)) / rmse_default_all(si) * 100);

        plot_single_scene(si, n_scenarios, chinese_name, short_name, ...
            scenario_name, seed_start, turn_rate, cfg51, output_dir, ...
            rmse_default_all(si), rmse_cfg51_all(si));
    end

    data_file = fullfile(output_dir, 'comparison_data.csv');
    fid = fopen(data_file, 'w');
    fprintf(fid, '场景英文名,中文名称,Default RMSE(km),Config#51 RMSE(km),改善(%),权重\n');
    for si = 1:n_scenarios
        fprintf(fid, '%s,%s,%.4f,%.4f,%.2f,%.2f\n', ...
            specs{si}{2}, specs{si}{3}, ...
            rmse_default_all(si), rmse_cfg51_all(si), ...
            (rmse_default_all(si) - rmse_cfg51_all(si)) / rmse_default_all(si) * 100, ...
            weights(si));
    end
    fclose(fid);

    figure('Position', [100, 100, 700, 500], 'Color', 'white', ...
        'Name', 'RMSE汇总对比', 'NumberTitle', 'off');
    bar_data = [rmse_default_all, rmse_cfg51_all];
    b = bar(1:n_scenarios, bar_data, 0.6);
    b(1).FaceColor = [0.85 0.85 0.85];
    b(2).FaceColor = [0.2 0.6 0.9];
    snames = cell(1, n_scenarios);
    for k = 1:n_scenarios, snames{k} = specs{k}{2}; end
    set(gca, 'XTickLabel', snames, 'FontSize', 8);
    ylabel('RMSE (km)', 'FontSize', 11);
    xlabel('场景', 'FontSize', 11);
    legend('Default IMM (3in1)', 'Config#51 IMM (3in1)', 'Location', 'best');
    grid on;
    xlim(gca, [0.5, n_scenarios+0.5]);
    title('全场景 RMSE 对比汇总', 'FontSize', 13, 'FontWeight', 'bold');
    saveas(gcf, fullfile(output_dir, 'summary_rmse_bar.png'), 'png');
    close(gcf);

    save(fullfile(output_dir, 'comparison_results.mat'), ...
        'rmse_default_all', 'rmse_cfg51_all', 'specs', 'weights');

    fprintf('\n============================================================\n');
    fprintf('全部结果已保存到 %s\n', output_dir);
    fprintf('============================================================\n');

    weighted_default = sum(rmse_default_all .* weights);
    weighted_cfg51 = sum(rmse_cfg51_all .* weights);
    fprintf('加权 RMSE -- Default: %.3f km, Config#51: %.3f km, Delta: %.1f%%\n', ...
        weighted_default, weighted_cfg51, ...
        (weighted_default - weighted_cfg51) / weighted_default * 100);
end

% ===========================================================================
% Overlay config params onto overrides struct
% ===========================================================================
function ov = overlay_cfg(ov, cfg51)
    fn = fieldnames(cfg51);
    for fi = 1:numel(fn)
        ov.(fn{fi}) = cfg51.(fn{fi});
    end
end

% ===========================================================================
% eval_r1 — evaluate one radar, one seed, using final_validate_config51 pattern
% ===========================================================================
function rmse_km = eval_r1(inp, params, radar_id, seed_val, override_cfg51)
    is_cfg51 = ~isempty(override_cfg51);

    if radar_id == 1
        det = inp.detList_R1; tg = inp.t1_grid;
        rl = params.radar1_lon; rlat = params.radar1_lat;
        tl = params.radar1_tx_lon; tlat = params.radar1_tx_lat;
    else
        det = inp.detList_R2; tg = inp.t2_grid;
        rl = params.radar2_lon; rlat = params.radar2_lat;
        tl = params.radar2_tx_lon; tlat = params.radar2_tx_lat;
    end

    prm = radar_params(params, radar_id);
    ov = struct('imm_adapt_mode', '3in1', 'random_seed', seed_val);
    if is_cfg51
        ov = overlay_cfg(ov, override_cfg51);
    end

    fn = fieldnames(ov);
    for fi = 1:numel(fn)
        if ~strcmp(fn{fi}, 'random_seed')
            prm.(fn{fi}) = ov.(fn{fi});
        end
    end

    tpl = ukf_imm('create', prm, rl, rlat, tl, tlat, prm.dt_sec);
    [~, ~, snaps] = run_oracle_tracker_sequence(det, tpl, params, inp.truth_all, tg, false);

    pos_sq = [];
    for f = 1:numel(snaps)
        if isempty(snaps{f}) || ~isfield(snaps{f}, 'trackList')
            continue;
        end
        for ti = 1:numel(snaps{f}.trackList)
            trk = snaps{f}.trackList{ti};
            if ~trk.updated || ~isfinite(trk.combined_nis)
                continue;
            end
            % Handle both cell and struct truthTrajs
            tt = inp.truthTrajs;
            idx = trk.truth_idx;
            if iscell(tt)
                truth = tt{idx};
            else
                truth = tt(idx);
            end
            if isempty(truth) || ~isfield(truth, 'time_sec')
                continue;
            end
            t_now = tg(f);
            if t_now < truth.time_sec(1) || t_now > truth.time_sec(end)
                continue;
            end
            true_lon = interp1(truth.time_sec, truth.lon, t_now, 'linear');
            true_lat = interp1(truth.time_sec, truth.lat, t_now, 'linear');
            if ~all(isfinite([true_lon, true_lat]))
                continue;
            end
            dist_m = haversine_distance(trk.ukf.x(1), trk.ukf.x(3), true_lon, true_lat);
            pos_sq(end+1) = dist_m^2;
        end
    end
    clear tpl det snaps;
    if ~isempty(pos_sq)
        rmse_km = sqrt(mean(pos_sq)) / 1000;
    else
        rmse_km = NaN;
    end
end

% ===========================================================================
% plot_single_scene — trajectory comparison for one scenario
% ===========================================================================
function plot_single_scene(fig_num, n_total, ch_name, sh_name, ...
        scenario_name, seed_val, turn_rate, cfg51, outdir, rmse_def, rmse_cfg)
    inp = prepare_oracle_tracking_inputs(scenario_name, ...
        struct('random_seed', seed_val, ...
               'truth_turn_rate_deg_per_sec', turn_rate));

    detList_R1 = inp.detList_R1;
    truth_input = inp.truth_all;
    truthTrajs = inp.truthTrajs;
    p1 = inp.params;

    rp = radar_params(p1, 1);
    rp.imm_adapt_mode = '3in1';
    tp_def = ukf_imm('create', rp, p1.radar1_lon,p1.radar1_lat,p1.radar1_tx_lon,p1.radar1_tx_lat,rp.dt_sec);
    [~,~,sn_def] = run_oracle_tracker_sequence(detList_R1, tp_def, p1, truth_input, inp.t1_grid, false);

    rp2 = radar_params(p1, 1);
    rp2.imm_cv_dwell_time_sec = 2500; rp2.imm_ct_dwell_time_sec = 660;
    rp2.imm_ct_fixed_Q_scale = 5.3; rp2.imm_transient_gain_max = 11.0;
    rp2.imm_transient_nis_start = 3.0; rp2.imm_transient_nis_full = 12.0;
    rp2.imm_transient_ewma_alpha = 0.65; rp2.imm_mu_init_CV = 0.5;
    rp2.imm_adapt_mode = '3in1';
    tp_cfg = ukf_imm('create', rp2, p1.radar1_lon,p1.radar1_lat,p1.radar1_tx_lon,p1.radar1_tx_lat,rp2.dt_sec);
    [~,~,sn_cfg] = run_oracle_tracker_sequence(detList_R1, tp_cfg, p1, truth_input, inp.t1_grid, false);

    trk_def = collect_track_xy(sn_def, inp.t1_grid, truthTrajs);
    trk_cfg = collect_track_xy(sn_cfg, inp.t1_grid, truthTrajs);

    fig = figure('Position', [100, 100, 900, 750], 'Color', 'white', ...
        'Name', sprintf('%d/%d: %s', fig_num, n_total, ch_name), 'NumberTitle', 'off');
    subplot(2,1,1);
    plot_truth_and_tracks(truthTrajs, detList_R1, trk_def(:,1), trk_def(:,2), trk_cfg(:,1), trk_cfg(:,2));
    title(sprintf('%s 轨迹对比 (%s)', ch_name, sh_name), 'FontSize', 12, 'FontWeight', 'bold');

    subplot(2,1,2);
    ax2 = gca;
    ax2.XLabel.String = '时间 (s)';
    ax2.YLabel.String = '位置误差 (m)';
    ax2.FontSize = 10;
    valid = isfinite(trk_def(:,3)) & isfinite(trk_cfg(:,3)) & isfinite(trk_def(:,4));
    t = trk_def(valid, 4);
    if ~isempty(t)
        ed = movmean(trk_def(valid,3), 21);
        ec = movmean(trk_cfg(valid,3), 21);
        plot(ax2, t, ed, 'LineWidth', 1.5, 'Color', [0.6 0.6 0.6], 'DisplayName', 'Default (smoothed)');
        plot(ax2, t, ec, 'LineWidth', 1.5, 'Color', [0.2 0.6 0.9], 'DisplayName', 'Config#51 (smoothed)');
        plot(ax2, t, trk_def(valid,3), 'LineStyle','none', 'Marker','.','Color',[0.7 0.7 0.7],'MarkerSize',2);
        plot(ax2, t, trk_cfg(valid,3), 'LineStyle','none', 'Marker','.','Color',[0.3 0.7 1.0],'MarkerSize',2);
    end
    legend(ax2, 'Location', 'best', 'FontSize', 9);
    grid(ax2, 'on');
    txt = sprintf('Default RMSE = %.2f km\nConfig#51 RMSE = %.2f km\nDelta: %+.1f%%', rmse_def, rmse_cfg, (rmse_def-rmse_cfg)/rmse_def*100);
    title(txt, 'FontSize', 10);

    saveas(fig, fullfile(outdir, sprintf('scenario_%s.png', sh_name)), 'png');
    close(fig);
end

function pts = collect_track_xy(snaps, t_grid, truthTrajs)
    all_pts = [];
    for f = 1:numel(snaps)
        if isempty(snaps{f}) || ~isfield(snaps{f}, 'trackList')
            continue;
        end
        t_now = t_grid(f);
        for ti = 1:numel(snaps{f}.trackList)
            trk = snaps{f}.trackList{ti};
            if ~trk.updated || ~isfinite(trk.combined_nis)
                continue;
            end
            tt = truthTrajs;
            if iscell(tt)
                truth = tt{trk.truth_idx};
            else
                truth = tt(trk.truth_idx);
            end
            if ~isfield(truth, 'time_sec') || numel(truth.time_sec) == 0
                continue;
            end
            if t_now < truth.time_sec(1) || t_now > truth.time_sec(end)
                continue;
            end
            true_lon = interp1(truth.time_sec, truth.lon, t_now, 'linear');
            true_lat = interp1(truth.time_sec, truth.lat, t_now, 'linear');
            if ~all(isfinite([true_lon, true_lat]))
                continue;
            end
            dist_m = haversine_distance(trk.lon, trk.lat, true_lon, true_lat);
            all_pts(end+1,:) = [trk.lon, trk.lat, dist_m, t_now];
        end
    end
    pts = all_pts;
end

function plot_truth_and_tracks(truthTrajs, detList_R1, def_lons, def_lats, cfg_lons, cfg_lats)
    ax = gca;
    ax.XLabel.String = '经度 (deg E)';
    ax.YLabel.String = '纬度 (deg N)';
    ax.FontSize = 10;

    max_det = min(numel(detList_R1), 500);
    det_x = []; det_y = [];
    for k = 1:max_det
        if ~isempty(detList_R1{k})
            dets = detList_R1{k};
            if isstruct(dets) && isfield(dets, 'lon')
                det_x = [det_x, dets.lon];
                det_y = [det_y, dets.lat];
            end
        end
    end
    if ~isempty(det_x)
        plot(ax, det_x, det_y, '.', 'Color', [0.9 0.9 0.9], 'MarkerSize', 1);
    end

    colors = lines(max(numel(truthTrajs), 3));
    for ti = 1:numel(truthTrajs)
        if iscell(truthTrajs)
            truth = truthTrajs{ti};
        else
            truth = truthTrajs(ti);
        end
        if isfield(truth, 'time_sec') && ~isempty(truth.time_sec)
            plot(ax, truth.lon, truth.lat, '-', 'Color', colors(ti,:), 'LineWidth', 2, ...
                'DisplayName', sprintf('Truth %d', ti));
        end
    end

    if ~isempty(def_lons)
        plot(ax, def_lons, def_lats, '-s', 'Color', [0.6 0.6 0.6], 'MarkerSize', 3, ...
            'LineWidth', 1.2, 'DisplayName', 'Default Track');
    end
    if ~isempty(cfg_lons)
        plot(ax, cfg_lons, cfg_lats, '-^', 'Color', [0.2 0.6 0.9], 'MarkerSize', 3, ...
            'LineWidth', 1.2, 'DisplayName', 'Config#51 Track');
    end

    legend(ax, 'Location', 'best', 'FontSize', 9);
    axis equal;
    grid(ax, 'on');
end

function dist = haversine_distance(lon1, lat1, lon2, lat2)
    dlat = deg2rad(lat2 - lat1);
    dlon = deg2rad(lon2 - lon1);
    a = sin(dlon/2)^2 + cosd(lat1)*cosd(lat2)*sin(dlat/2)^2;
    a = max(0, min(1, a));
    dist = 6371000 * 2 * atan2(sqrt(a), sqrt(1-a));
end
