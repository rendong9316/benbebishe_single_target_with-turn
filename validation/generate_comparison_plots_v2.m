% generate_comparison_plots — 全场景滤波前后对比可视化
% Usage: matlab -batch "addpath(genpath('D:/Desktop/single_target_with-turn')); generate_comparison_plots();"
%
% 数据策略：RMSE = 2雷达 × 10 seeds 平均（与最终验证一致）；
%         轨迹图 = radar R1, seed=10001（便于直观对比两种参数）

function generate_comparison_plots()
    addpath(genpath('D:/Desktop/single_target_with-turn'));

    seed_start = 10001;
    n_seeds_rmse = 10;          % 用于RMSE统计的seeds
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
    fprintf('全场景滤波前后对比可视化（2雷达×%d seeds平均）\n', n_seeds_rmse);
    fprintf('============================================================\n\n');

    rmse_default_all = zeros(n_scenarios, 1);
    rmse_cfg51_all = zeros(n_scenarios, 1);

    for si = 1:n_scenarios
        scenario_name = specs{si}{1};
        short_name = specs{si}{2};
        chinese_name = specs{si}{3};
        turn_rate = specs{si}{4};

        fprintf('[%d/%d] %s (%s)...\n', si, n_scenarios, chinese_name, short_name);

        % --- Compute RMSE: 2 radars × n_seeds ---
        rmse_default_all(si) = compute_avg_rmse(scenario_name, seed_start, n_seeds_rmse, ...
            turn_rate, cfg51, false);   % false = use default params
        rmse_cfg51_all(si) = compute_avg_rmse(scenario_name, seed_start, n_seeds_rmse, ...
            turn_rate, cfg51, true);     % true = use config51 params

        %% Plot trajectories (using single frame from seed=10001)
        plot_single_scenario(scenario_name, seed_start, turn_rate, ...
            cfg51, output_dir, si, n_scenarios, chinese_name, short_name, ...
            rmse_default_all(si), rmse_cfg51_all(si));

        fprintf('  Default: %.3f km, Config#51: %.3f km, Delta: %+.1f%%\n', ...
            rmse_default_all(si), rmse_cfg51_all(si), ...
            (rmse_default_all(si) - rmse_cfg51_all(si)) / rmse_default_all(si) * 100);
    end

    %% Save CSV
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

    %% Summary bar chart
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
% compute_avg_rmse — run n_seeds × 2 radars and return mean RMSE (km)
% ===========================================================================
function avg_rmse = compute_avg_rmse(scenario_name, seed_base, n_seeds, turn_rate, cfg51, use_cfg)
    all_errs = [];
    p1 = simulation_params_oracle();
    truth_inputs = {};

    for sk = 1:n_seeds
        seed_val = seed_base + sk - 1;
        inp = prepare_oracle_tracking_inputs(scenario_name, ...
            struct('random_seed', seed_val, ...
                   'truth_turn_rate_deg_per_sec', turn_rate));
        truth_inputs{end+1} = inp;

        % Create template with desired params
        if use_cfg
            prm = radar_params(inp.params, 1);
            fn = fieldnames(cfg51);
            for fi = 1:numel(fn)
                prm.(fn{fi}) = cfg51.(fn{fi});
            end
        else
            prm = radar_params(inp.params, 1);
        end
        prm.imm_adapt_mode = '3in1';

        truthTrajs_for_track = get_trutharray(inp);
        for r = 1:2
            if r == 1
                det = inp.detList_R1; tp = ukf_imm('create', prm, ...
                    inp.params.radar1_lon,inp.params.radar1_lat,...
                    inp.params.radar1_tx_lon,inp.params.radar1_tx_lat,prm.dt_sec);
                ta = inp.truth_all; tg = inp.t1_grid;
            else
                det = inp.detList_R2; tp = ukf_imm('create', prm, ...
                    inp.params.radar2_lon,inp.params.radar2_lat,...
                    inp.params.radar2_tx_lon,inp.params.radar2_tx_lat,prm.dt_sec);
                ta = inp.truth_all; tg = inp.t2_grid;
            end
            [~,~,snaps] = run_oracle_tracker_sequence(det, tp, inp.params, ta, tg, false);
            clear tp;

            tr = collect_tracks_rms(snaps, tg, truthTrajs_for_track);
            all_errs(end+1) = tr;
        end
    end

    tr = collect_tracks_rms(snaps, tg, get_trutharray(inp));
        all_errs(end+1) = tr;
    end

    if ~isempty(all_errs)
        avg_rmse = sqrt(mean(all_errs.^2)) / 1000;  % all_errs in meters -> km
    else
        avg_rmse = NaN;
    end
end

% ===========================================================================
% get_trutharray — unwrap truthTrajs correctly for any scenario type
% ===========================================================================
function truth_out = get_trutharray(inp)
    tt = inp.truthTrajs;
    % MATLAB struct constructor spreads cells for multi-target
    if iscell(tt)
        % Check if elements are structs (multi-cross case) or scalars
        if isstruct(tt{1})
            % It's a struct — just pass it
            truth_out = tt{1};
        else
            % It's the original cell array
            truth_out = tt{1};
        end
    else
        truth_out = tt;
    end
end

% ===========================================================================
% collect_tracks_rms — one-pass RMSE extraction (returns meters, not km)
% ===========================================================================
function err_m = collect_tracks_rms(snaps, t_grid, truthTrajs)
    pos_sq = [];
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
            if iscell(truthTrajs)
                truth = truthTrajs{trk.truth_idx};
            else
                truth = truthTrajs(trk.truth_idx);
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
            pos_sq(end+1) = dist_m^2;
        end
    end
    err_m = sqrt(mean(pos_sq));
end

% ===========================================================================
% plot_single_scenario — draw trajectory comparison (R1, seed=10001 only)
% ===========================================================================
function plot_single_scenario(scenario_name, seed_base, turn_rate, ...
        cfg51, output_dir, fig_num, n_total, ch_name, sh_name, ...
        rmse_def, rmse_cfg)
    inp = prepare_oracle_tracking_inputs(scenario_name, ...
        struct('random_seed', seed_base, ...
               'truth_turn_rate_deg_per_sec', turn_rate));

    detList_R1 = inp.detList_R1;
    truth_input = inp.truth_all;
    truthTrajs = inp.truthTrajs;
    p1 = inp.params;

    % Default track
    prm_def = radar_params(p1, 1);
    prm_def.imm_adapt_mode = '3in1';
    tp_def = ukf_imm('create', prm_def, ...
        p1.radar1_lon,p1.radar1_lat,p1.radar1_tx_lon,p1.radar1_tx_lat,prm_def.dt_sec);
    [~,~,sn_def] = run_oracle_tracker_sequence(detList_R1, tp_def, p1, truth_input, inp.t1_grid, false);

    % Config#51 track
    prm_cfg = radar_params(p1, 1);
    fn = fieldnames(cfg51);
    for fi = 1:numel(fn), prm_cfg.(fn{fi}) = cfg51.(fn{fi}); end
    prm_cfg.imm_adapt_mode = '3in1';
    tp_cfg = ukf_imm('create', prm_cfg, ...
        p1.radar1_lon,p1.radar1_lat,p1.radar1_tx_lon,p1.radar1_tx_lat,prm_cfg.dt_sec);
    [~,~,sn_cfg] = run_oracle_tracker_sequence(detList_R1, tp_cfg, p1, truth_input, inp.t1_grid, false);

    trk_def_lons = extract_coords(sn_def);
    trk_cfg_lons = extract_coords(sn_cfg);

    fig = figure('Position', [100, 100, 900, 750], 'Color', 'white', ...
        'Name', sprintf('%d/%d: %s', fig_num, n_total, ch_name), 'NumberTitle', 'off');
    subplot(2,1,1);
    plot_truth_and_tracks(truthTrajs, detList_R1, ...
        trk_def_lons(:,1), trk_def_lons(:,2), trk_cfg_lons(:,1), trk_cfg_lons(:,2));
    title(sprintf('%s 轨迹对比 (%s)', ch_name, sh_name), 'FontSize', 12, 'FontWeight', 'bold');

    % Error curves
    subplot(2,1,2);
    ax2 = gca;
    ax2.XLabel.String = '时间 (s)';
    ax2.YLabel.String = '位置误差 (m)';
    ax2.FontSize = 10;
    valid = isfinite(trk_def_lons(:,3)) & isfinite(trk_cfg_lons(:,3)) & isfinite(trk_def_lons(:,4));
    t = trk_def_lons(valid, 4);
    if ~isempty(t)
        ed = movmean(trk_def_lons(valid,3), 21);
        ec = movmean(trk_cfg_lons(valid,3), 21);
        plot(ax2, t, ed, 'LineWidth', 1.5, 'Color', [0.6 0.6 0.6], 'DisplayName', 'Default (smoothed)');
        plot(ax2, t, ec, 'LineWidth', 1.5, 'Color', [0.2 0.6 0.9], 'DisplayName', 'Config#51 (smoothed)');
        plot(ax2, t, trk_def_lons(valid,3), 'LineStyle','none', 'Marker','.','Color',[0.7 0.7 0.7],'MarkerSize',2);
        plot(ax2, t, trk_cfg_lons(valid,3), 'LineStyle','none', 'Marker','.','Color',[0.3 0.7 1.0],'MarkerSize',2);
    end
    legend(ax2, 'Location', 'best', 'FontSize', 9);
    grid(ax2, 'on');
    txt = sprintf('Default RMSE = %.2f km\nConfig#51 RMSE = %.2f km\nDelta: %+.1f%%', ...
        rmse_def, rmse_cfg, (rmse_def - rmse_cfg) / rmse_def * 100);
    title(txt, 'FontSize', 10);

    png_name = fullfile(output_dir, sprintf('scenario_%s.png', sh_name));
    saveas(fig, png_name, 'png');
    close(fig);
end

% ===========================================================================
% Extract track positions (lon, lat, error_m, time_sec) from snapshots
% ===========================================================================
function pts = extract_coords(snaps)
    all_pts = [];
    for f = 1:numel(snaps)
        if isempty(snaps{f}) || ~isfield(snaps{f}, 'trackList')
            continue;
        end
        for ti = 1:numel(snaps{f}.trackList)
            trk = snaps{f}.trackList{ti};
            if ~trk.updated || ~isfinite(trk.combined_nis)
                continue;
            end
            all_pts(end+1,:) = [trk.lon, trk.lat, nan, nan];
        end
    end
    pts = all_pts;
end

% ===========================================================================
% plot_truth_and_tracks — spatial view with detections + truth + two tracks
% ===========================================================================
function plot_truth_and_tracks(truthTrajs, detList_R1, ...
        def_lons, def_lats, cfg_lons, cfg_lats)

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
