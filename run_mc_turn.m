% =========================================================================
% run_mc_turn.m — 拐弯场景蒙特卡洛仿真
% =========================================================================
% 不弹图窗、不保存中间.mat、控制台输出进度 + 逐run明细 + 最终统计。
% 最终统计含均值/标准差/中位数/最小/最大，并保存完整数据到 .mat。
% =========================================================================

clear; close all; clc;
addpath(genpath('.'));
addpath(genpath('nanyang'));

N_MC = 200;

% ---- 预分配结果数组 ----
rmse.raw_R1   = nan(N_MC, 1);
rmse.raw_R2   = nan(N_MC, 1);
rmse.cal_R1   = nan(N_MC, 1);
rmse.cal_R2   = nan(N_MC, 1);
rmse.ukf_R1   = nan(N_MC, 1);
rmse.ukf_R2   = nan(N_MC, 1);
rmse.ad_R1    = nan(N_MC, 1);
rmse.ad_R2    = nan(N_MC, 1);
rmse.fus_base = nan(N_MC, 1);
rmse.fus_ad   = nan(N_MC, 1);
rmse.sgl_base = nan(N_MC, 1);
rmse.sgl_ad   = nan(N_MC, 1);

% ---- 详细诊断信息（用于坏种子分析） ----
diag = struct();
diag.type_R1 = nan(N_MC,1);  diag.type_R2 = nan(N_MC,1);
diag.type_R1_ad = nan(N_MC,1);  diag.type_R2_ad = nan(N_MC,1);
diag.life_R1 = nan(N_MC,1);  diag.life_R2 = nan(N_MC,1);
diag.life_R1_ad = nan(N_MC,1);  diag.life_R2_ad = nan(N_MC,1);
diag.nTrk_R1 = nan(N_MC,1);  diag.nTrk_R2 = nan(N_MC,1);
diag.nTrk_R1_ad = nan(N_MC,1);  diag.nTrk_R2_ad = nan(N_MC,1);
diag.n_frames = nan(N_MC,1);

% ---- 坏种子收集 ----
bad_seeds = {};

fprintf('========== 拐弯场景蒙特卡洛仿真 N=%d ==========\n', N_MC);
fprintf('%-6s %6s %8s %8s %8s %8s %8s %8s %8s %8s  %8s %8s %8s %8s %5s %5s %5s %5s\n', ...
    '运次', '种子', '原R1', '校R1', 'UK1', '自R1', ...
    '原R2', '校R2', 'UK2', '自R2', ...
    '融基', '融自', '单基', '单自', ...
    't1', 't2', 'nT1', 'nT2');
fprintf('%-6s %6s %8s %8s %8s %8s %8s %8s %8s %8s  %8s %8s %8s %8s %5s %5s %5s %5s\n', ...
    '---', '----', '------', '------', '------', '------', ...
    '------', '------', '------', '------', ...
    '------', '------', '------', '------', ...
    '---', '---', '---', '---');

tic;

for mc = 1:N_MC
    close all;
    r = run_one(mc);

    rmse.raw_R1(mc)   = r.raw_R1;
    rmse.raw_R2(mc)   = r.raw_R2;
    rmse.cal_R1(mc)   = r.cal_R1;
    rmse.cal_R2(mc)   = r.cal_R2;
    rmse.ukf_R1(mc)   = r.ukf_R1;
    rmse.ukf_R2(mc)   = r.ukf_R2;
    rmse.ad_R1(mc)    = r.ad_R1;
    rmse.ad_R2(mc)    = r.ad_R2;
    rmse.fus_base(mc) = r.fus_base;
    rmse.fus_ad(mc)   = r.fus_ad;
    rmse.sgl_base(mc) = r.sgl_base;
    rmse.sgl_ad(mc)   = r.sgl_ad;

    % 存储诊断信息
    diag.type_R1(mc) = r.type_R1;  diag.type_R2(mc) = r.type_R2;
    diag.type_R1_ad(mc) = r.type_R1_ad;  diag.type_R2_ad(mc) = r.type_R2_ad;
    diag.life_R1(mc) = r.life_R1;  diag.life_R2(mc) = r.life_R2;
    diag.life_R1_ad(mc) = r.life_R1_ad;  diag.life_R2_ad(mc) = r.life_R2_ad;
    diag.nTrk_R1(mc) = r.nTrk_R1;  diag.nTrk_R2(mc) = r.nTrk_R2;
    diag.nTrk_R1_ad(mc) = r.nTrk_R1_ad;  diag.nTrk_R2_ad(mc) = r.nTrk_R2_ad;
    diag.n_frames(mc) = r.n_frames;

    % ---- 坏种子判定 ----
    is_bad = false;
    bad_cat = '';
    if r.ukf_R1 > 30 || r.ukf_R2 > 30 || r.ad_R1 > 30 || r.ad_R2 > 30
        is_bad = true;  bad_cat = 'DIVERGED';
    end
    if ~is_bad
        if (r.ukf_R1 > r.cal_R1*1.5 && r.ukf_R1 > 15) || ...
           (r.ukf_R2 > r.cal_R2*1.5 && r.ukf_R2 > 15) || ...
           (r.ad_R1 > r.cal_R1*1.5 && r.ad_R1 > 15) || ...
           (r.ad_R2 > r.cal_R2*1.5 && r.ad_R2 > 15)
            is_bad = true;  bad_cat = 'DEGRADED';
        end
    end
    if ~is_bad && r.n_frames > 0
        pct = max([r.nTrk_R1 r.nTrk_R2 r.nTrk_R1_ad r.nTrk_R2_ad]) / r.n_frames;
        if pct < 0.10
            is_bad = true;  bad_cat = 'LOW_OUTPUT';
        end
    end

    % 打印行
    t1c = track_type_char(r.type_R1);  t2c = track_type_char(r.type_R2);
    fprintf('%-6d %6d %8.1f %8.1f %8.1f %8.1f %8.1f %8.1f %8.1f %8.1f  %8.1f %8.1f %8.1f %8.1f %5c %5c %5d %5d', ...
        mc, mc, r.raw_R1, r.cal_R1, r.ukf_R1, r.ad_R1, ...
        r.raw_R2, r.cal_R2, r.ukf_R2, r.ad_R2, ...
        r.fus_base, r.fus_ad, r.sgl_base, r.sgl_ad, ...
        t1c, t2c, max(r.nTrk_R1,0), max(r.nTrk_R2,0));

    if is_bad
        fprintf('  - BAD[%s]', bad_cat);
        bs.seed = mc;
        bs.rmse_ukf_R1 = r.ukf_R1;  bs.rmse_ukf_R2 = r.ukf_R2;
        bs.rmse_ad_R1 = r.ad_R1;    bs.rmse_ad_R2 = r.ad_R2;
        bs.rmse_fus_base = r.fus_base;  bs.rmse_fus_ad = r.fus_ad;
        bs.type_R1 = r.type_R1;  bs.type_R2 = r.type_R2;
        bs.type_R1_ad = r.type_R1_ad;  bs.type_R2_ad = r.type_R2_ad;
        bs.life_R1 = r.life_R1;  bs.life_R2 = r.life_R2;
        bs.life_R1_ad = r.life_R1_ad;  bs.life_R2_ad = r.life_R2_ad;
        bs.nTrk_R1 = r.nTrk_R1;  bs.nTrk_R2 = r.nTrk_R2;
        bs.nTrk_R1_ad = r.nTrk_R1_ad;  bs.nTrk_R2_ad = r.nTrk_R2_ad;
        bs.n_frames = r.n_frames;
        bs.category = bad_cat;
        bad_seeds{end+1} = bs;
    end
    fprintf('\n');

    if mod(mc, 50) == 0
        elapsed = toc;
        fprintf('--- 已完成 %d/%d (%.0f%%), 耗时 %.0fs, 坏种子: %d ---\n', ...
            mc, N_MC, mc/N_MC*100, elapsed, length(bad_seeds));
    end
end
close all;

elapsed_total = toc;

% ---- 计算改善率（逐次算）----
imp_cal_R1   = (1 - rmse.cal_R1   ./ rmse.raw_R1)   * 100;
imp_cal_R2   = (1 - rmse.cal_R2   ./ rmse.raw_R2)   * 100;
imp_ukf_R1   = (1 - rmse.ukf_R1   ./ rmse.cal_R1)   * 100;
imp_ukf_R2   = (1 - rmse.ukf_R2   ./ rmse.cal_R2)   * 100;
imp_ad_R1    = (1 - rmse.ad_R1    ./ rmse.ukf_R1)    * 100;
imp_ad_R2    = (1 - rmse.ad_R2    ./ rmse.ukf_R2)    * 100;
imp_fus_base = (1 - rmse.fus_base ./ rmse.sgl_base)  * 100;
imp_fus_ad   = (1 - rmse.fus_ad   ./ rmse.sgl_ad)    * 100;

% ---- 输出统计表 ----
fprintf('\n========== 蒙特卡洛 %d 次统计结果 ==========\n', N_MC);

fprintf('\n--- RMSE 绝对值 (km) ---\n');
fprintf('%-28s %8s %8s %8s %8s %8s\n', ...
    '指标', '均值', '标准差', '中位数', '最小', '最大');
fprintf('%-28s %8s %8s %8s %8s %8s\n', ...
    '----------------------------', '------', '------', '------', '------', '------');

print_row('原始点迹 R1',          rmse.raw_R1);
print_row('原始点迹 R2',          rmse.raw_R2);
print_row('校准后 R1',            rmse.cal_R1);
print_row('校准后 R2',            rmse.cal_R2);
print_row('基础UKF R1',           rmse.ukf_R1);
print_row('基础UKF R2',           rmse.ukf_R2);
print_row('自适应UKF R1',         rmse.ad_R1);
print_row('自适应UKF R2',         rmse.ad_R2);
print_row('基础融合最优',         rmse.fus_base);
print_row('自适应融合最优',       rmse.fus_ad);
print_row('基础单站最优(对齐)',   rmse.sgl_base);
print_row('自适应单站最优(对齐)', rmse.sgl_ad);

fprintf('\n--- 阶段改善率 (%%) ---\n');
fprintf('%-28s %8s %8s %8s %8s %8s\n', ...
    '指标', '均值', '标准差', '中位数', '最小', '最大');
fprintf('%-28s %8s %8s %8s %8s %8s\n', ...
    '----------------------------', '------', '------', '------', '------', '------');

print_row('校准改善 R1',        imp_cal_R1);
print_row('校准改善 R2',        imp_cal_R2);
print_row('UKF改善 R1',         imp_ukf_R1);
print_row('UKF改善 R2',         imp_ukf_R2);
print_row('自适应改善 R1',      imp_ad_R1);
print_row('自适应改善 R2',      imp_ad_R2);
print_row('融合改善(基础)',     imp_fus_base);
print_row('融合改善(自适应)',   imp_fus_ad);

fprintf('\n总耗时: %.0f 秒\n', elapsed_total);

% =========================================================================
% 坏种子分析
% =========================================================================
n_bad = length(bad_seeds);
fprintf('\n========== 坏种子分析 ==========\n');
fprintf('总坏种子数: %d / %d (%.1f%%)\n', n_bad, N_MC, n_bad/N_MC*100);

if n_bad > 0
    % 按类别统计
    all_cats = cell(1, n_bad);
    for ib = 1:n_bad; all_cats{ib} = bad_seeds{ib}.category; end
    [uniq_cats, ~, cat_idx] = unique(all_cats);
    fprintf('\n--- 按失败类别分布 ---\n');
    for c = 1:length(uniq_cats)
        cnt = sum(cat_idx == c);
        fprintf('  %-20s: %2d 个 (%.1f%%)\n', uniq_cats{c}, cnt, cnt/n_bad*100);
    end

    % 坏种子明细
    fprintf('\n--- 坏种子明细 ---\n');
    fprintf('%-6s %8s %8s %8s %8s %8s %8s %5s %5s %5s %5s %5s %5s %5s %5s %-16s\n', ...
        '种子', 'UKF_R1', 'UKF_R2', 'ad_R1', 'ad_R2', 'fus基', 'fus自', ...
        't1', 't2', 't1a', 't2a', 'lif1', 'lif2', 'nT1', 'nT2', '类别');
    fprintf('%-6s %8s %8s %8s %8s %8s %8s %5s %5s %5s %5s %5s %5s %5s %5s %-16s\n', ...
        '----', '------', '------', '------', '------', '------', '------', ...
        '---', '---', '---', '---', '---', '---', '---', '---', '----');
    for i = 1:n_bad
        bs = bad_seeds{i};
        t1c = track_type_char(bs.type_R1);  t2c = track_type_char(bs.type_R2);
        t1ac = track_type_char(bs.type_R1_ad);  t2ac = track_type_char(bs.type_R2_ad);
        fprintf('%-6d %8.1f %8.1f %8.1f %8.1f %8.1f %8.1f %5c %5c %5c %5c %5d %5d %5d %5d %-16s\n', ...
            bs.seed, bs.rmse_ukf_R1, bs.rmse_ukf_R2, bs.rmse_ad_R1, bs.rmse_ad_R2, ...
            bs.rmse_fus_base, bs.rmse_fus_ad, ...
            t1c, t2c, t1ac, t2ac, ...
            max(bs.life_R1,0), max(bs.life_R2,0), ...
            max(bs.nTrk_R1,0), max(bs.nTrk_R2,0), ...
            bs.category);
    end

    % RMSE分布
    fprintf('\n--- 坏种子 RMSE 分布 (km) ---\n');
    fprintf('%-16s %8s %8s %8s %8s %8s\n', '指标', '均值', '中位', '最小', '最大', 'std');
    all_ukf_r1 = cellfun(@(b) b.rmse_ukf_R1, bad_seeds);
    all_ukf_r2 = cellfun(@(b) b.rmse_ukf_R2, bad_seeds);
    all_ad_r1  = cellfun(@(b) b.rmse_ad_R1, bad_seeds);
    all_ad_r2  = cellfun(@(b) b.rmse_ad_R2, bad_seeds);
    all_fus_b  = cellfun(@(b) b.rmse_fus_base, bad_seeds);
    all_fus_a  = cellfun(@(b) b.rmse_fus_ad, bad_seeds);
    print_bs_row('基础UKF R1', all_ukf_r1);
    print_bs_row('基础UKF R2', all_ukf_r2);
    print_bs_row('自适应UKF R1', all_ad_r1);
    print_bs_row('自适应UKF R2', all_ad_r2);
    print_bs_row('基础融合', all_fus_b);
    print_bs_row('自适应融合', all_fus_a);

    % 单站/双站发散
    r1_div = sum(all_ukf_r1 > 30 | all_ad_r1 > 30);
    r2_div = sum(all_ukf_r2 > 30 | all_ad_r2 > 30);
    both_div = sum((all_ukf_r1 > 30 | all_ad_r1 > 30) & (all_ukf_r2 > 30 | all_ad_r2 > 30));
    fprintf('\n仅R1发散: %d, 仅R2发散: %d, 双站均发散: %d\n', ...
        r1_div - both_div, r2_div - both_div, both_div);
else
    fprintf('无坏种子！所有%d次实验均通过。\n', N_MC);
end

% ---- 保存完整数据 ----
if ~exist('results', 'dir'), mkdir('results'); end
outf = fullfile('results', sprintf('mc_turn_%s.mat', datestr(now, 'yyyymmdd_HHMMSS')));
save(outf, 'rmse', 'imp_cal_R1', 'imp_cal_R2', 'imp_ukf_R1', 'imp_ukf_R2', ...
    'imp_ad_R1', 'imp_ad_R2', 'imp_fus_base', 'imp_fus_ad', ...
    'diag', 'bad_seeds', 'N_MC');
fprintf('\n完整数据已保存: %s\n', outf);


% =========================================================================
% run_one — 单次仿真运行, 返回各阶段RMSE
% =========================================================================
function r = run_one(seed)
    % ---- Phase 0: 场景初始化 ----
    params = simulation_params();
    params.random_seed = seed;
    rng(params.random_seed);

    [traj, ~] = aircraft_trajectory_create('turn', params);
    true_track = aircraft_trajectory_interpolate('generate', traj);

    t1_grid = params.time_offset_radar1_sec : params.dt_sec : traj.duration_sec;
    t2_grid = params.time_offset_radar2_sec : params.dt_sec : traj.duration_sec;
    n_frames = min(length(t1_grid), length(t2_grid));

    % ---- Phase 1: ADS-B偏差标定 ----
    rng(params.random_seed);
    T_adsb = readtable(params.adsb_csv_path, 'ReadVariableNames', false);
    adsb_lat = T_adsb.Var2;
    adsb_lon = T_adsb.Var3;

    dr1_list = []; da1_list = []; dr2_list = []; da2_list = [];
    n_check = min(5000, height(T_adsb));
    cal_step = max(1, floor(height(T_adsb) / n_check));

    for idx = 1:cal_step:height(T_adsb)
        t_lon = adsb_lon(idx);  t_lat = adsb_lat(idx);
        if isnan(t_lon) || isnan(t_lat), continue; end

        [in1, ~, ~] = radar_coverage_check(params.radar1_lon, params.radar1_lat, ...
            t_lon, t_lat, params.radar1_beam_center_deg, params);
        if in1
            Rg_true = skywave_geometry('group_range', params.radar1_tx_lon, params.radar1_tx_lat, ...
                params.radar1_lon, params.radar1_lat, t_lon, t_lat);
            az_true = sphere_utils_azimuth(params.radar1_lon, params.radar1_lat, t_lon, t_lat);
            Rg_meas = Rg_true + params.radar1_range_bias_m + randn() * params.radar1_range_noise_std_m;
            az_meas = az_true + params.radar1_azimuth_bias_deg + randn() * params.radar1_azimuth_noise_std_deg;
            dr1_list(end+1) = Rg_meas - Rg_true;
            daz = az_meas - az_true;
            if daz > 180, daz = daz - 360; elseif daz < -180, daz = daz + 360; end
            da1_list(end+1) = daz;
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
            daz = az_meas - az_true;
            if daz > 180, daz = daz - 360; elseif daz < -180, daz = daz + 360; end
            da2_list(end+1) = daz;
        end
    end
    dr1_est = mean(dr1_list);  da1_est = mean(da1_list);
    dr2_est = mean(dr2_list);  da2_est = mean(da2_list);

    % ---- Phase 2: 原始点迹生成 ----
    detRaw_R1 = cell(n_frames, 1);
    detRaw_R2 = cell(n_frames, 1);

    for k = 1:n_frames
        rng(params.random_seed + k);
        [pos, vel] = aircraft_trajectory_interpolate(traj, t1_grid(k));
        detRaw_R1{k} = generate_frame_detections(params.radar1_lon, params.radar1_lat, ...
            params.radar1_tx_lon, params.radar1_tx_lat, ...
            pos(1), pos(2), vel(1), vel(2), k, t1_grid(k), ...
            params.radar1_range_bias_m, params.radar1_azimuth_bias_deg, ...
            params.radar1_beam_center_deg, params, ...
            params.radar1_range_noise_std_m, params.radar1_azimuth_noise_std_deg);
        for d = 1:length(detRaw_R1{k})
            detRaw_R1{k}(d).aircraft_id = 1;
        end

        rng(params.random_seed + 10000 + k);
        [pos2, vel2] = aircraft_trajectory_interpolate(traj, t2_grid(k));
        detRaw_R2{k} = generate_frame_detections(params.radar2_lon, params.radar2_lat, ...
            params.radar2_tx_lon, params.radar2_tx_lat, ...
            pos2(1), pos2(2), vel2(1), vel2(2), k, t2_grid(k), ...
            params.radar2_range_bias_m, params.radar2_azimuth_bias_deg, ...
            params.radar2_beam_center_deg, params, ...
            params.radar2_range_noise_std_m, params.radar2_azimuth_noise_std_deg);
        for d = 1:length(detRaw_R2{k})
            detRaw_R2{k}(d).aircraft_id = 1;
        end
    end

    % ---- Phase 4: 偏差校正 + 几何反解 ----
    detList_R1 = cell(n_frames, 1);
    detList_R2 = cell(n_frames, 1);

    for k = 1:n_frames
        dets_r1 = detRaw_R1{k};
        for d = 1:length(dets_r1)
            Rgc = dets_r1(d).prange - dr1_est;
            azc = dets_r1(d).paz - da1_est;
            dets_r1(d).drange = Rgc;
            dets_r1(d).daz = azc;
            dets_r1(d).range_meas = Rgc;
            dets_r1(d).azimuth_meas = azc;
            if ~(isfield(dets_r1(d), 'lat') && ~isnan(dets_r1(d).lat))
                [~, lat_e, lon_e] = bistatic_inverse_solver(Rgc, azc, ...
                    params.radar1_tx_lon, params.radar1_tx_lat, ...
                    params.radar1_lon, params.radar1_lat);
                dets_r1(d).lat = lat_e;
                dets_r1(d).lon = lon_e;
            end
            [~, raw_lat, raw_lon] = bistatic_inverse_solver(dets_r1(d).prange, dets_r1(d).paz, ...
                params.radar1_tx_lon, params.radar1_tx_lat, ...
                params.radar1_lon, params.radar1_lat);
            dets_r1(d).raw_lat = raw_lat;
            dets_r1(d).raw_lon = raw_lon;
        end
        detList_R1{k} = dets_r1;

        dets_r2 = detRaw_R2{k};
        for d = 1:length(dets_r2)
            Rgc = dets_r2(d).prange - dr2_est;
            azc = dets_r2(d).paz - da2_est;
            dets_r2(d).drange = Rgc;
            dets_r2(d).daz = azc;
            dets_r2(d).range_meas = Rgc;
            dets_r2(d).azimuth_meas = azc;
            if ~(isfield(dets_r2(d), 'lat') && ~isnan(dets_r2(d).lat))
                [~, lat_e, lon_e] = bistatic_inverse_solver(Rgc, azc, ...
                    params.radar2_tx_lon, params.radar2_tx_lat, ...
                    params.radar2_lon, params.radar2_lat);
                dets_r2(d).lat = lat_e;
                dets_r2(d).lon = lon_e;
            end
            [~, raw_lat, raw_lon] = bistatic_inverse_solver(dets_r2(d).prange, dets_r2(d).paz, ...
                params.radar2_tx_lon, params.radar2_tx_lat, ...
                params.radar2_lon, params.radar2_lat);
            dets_r2(d).raw_lat = raw_lat;
            dets_r2(d).raw_lon = raw_lon;
        end
        detList_R2{k} = dets_r2;
    end

    % ---- 点迹RMSE ----
    r.raw_R1 = rmse_detlist(detList_R1, true_track, t1_grid, n_frames, 'raw');
    r.raw_R2 = rmse_detlist(detList_R2, true_track, t2_grid, n_frames, 'raw');
    r.cal_R1 = rmse_detlist(detList_R1, true_track, t1_grid, n_frames, 'cal');
    r.cal_R2 = rmse_detlist(detList_R2, true_track, t2_grid, n_frames, 'cal');

    % ---- Phase 5.1: 基础UKF ----
    params.ukf_range_std_m    = params.radar1_range_noise_std_m;
    params.ukf_azimuth_std_deg = params.radar1_azimuth_noise_std_deg;
    params.ukf_Q_scale     = params.radar1_ukf_Q_scale;
    params.ukf_P_pos_std   = params.radar1_ukf_P_pos_std;
    params.ukf_P_vel_std   = params.radar1_ukf_P_vel_std;
    params.gate_sigma      = params.radar1_gate_sigma;
    params.tracker_K_loss  = params.radar1_tracker_K_loss;

    ukf1_tpl = ukf_jichu('create', params, params.radar1_lon, params.radar1_lat, ...
        params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);

    params_r2 = params;
    params_r2.ukf_range_std_m    = params.radar2_range_noise_std_m;
    params_r2.ukf_azimuth_std_deg = params.radar2_azimuth_noise_std_deg;
    params_r2.gate_sigma      = params.radar2_gate_sigma;
    params_r2.ukf_Q_scale     = params.radar2_ukf_Q_scale;
    params_r2.ukf_P_pos_std   = params.radar2_ukf_P_pos_std;
    params_r2.ukf_P_vel_std   = params.radar2_ukf_P_vel_std;
    params_r2.tracker_M       = 4;
    params_r2.tracker_N       = 8;
    params_r2.tracker_K_loss  = params.radar2_tracker_K_loss;

    ukf2_tpl = ukf_jichu('create', params_r2, params.radar2_lon, params.radar2_lat, ...
        params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);

    [trackSnapshots_R1, finalTrk1] = single_track_runner_nanyang(detList_R1, ukf1_tpl, params, n_frames);
    [trackSnapshots_R2, finalTrk2] = single_track_runner_nanyang(detList_R2, ukf2_tpl, params_r2, n_frames);

    r.ukf_R1 = rmse_tracks(trackSnapshots_R1, true_track, t1_grid, n_frames);
    r.ukf_R2 = rmse_tracks(trackSnapshots_R2, true_track, t2_grid, n_frames);

    r.type_R1 = finalTrk1.type;  r.life_R1 = finalTrk1.life;
    r.type_R2 = finalTrk2.type;  r.life_R2 = finalTrk2.life;
    r.nTrk_R1 = count_tracking_frames(trackSnapshots_R1, n_frames);
    r.nTrk_R2 = count_tracking_frames(trackSnapshots_R2, n_frames);

    % ---- Phase 5.2: 机动自适应UKF ----
    rng(params.random_seed);
    ukf1_tpl_ad = ukf_jichu('create', params, params.radar1_lon, params.radar1_lat, ...
        params.radar1_tx_lon, params.radar1_tx_lat, params.dt_sec);
    ukf2_tpl_ad = ukf_jichu('create', params_r2, params.radar2_lon, params.radar2_lat, ...
        params.radar2_tx_lon, params.radar2_tx_lat, params.dt_sec);

    [trackSnapshots_R1_ad, finalTrk1_ad] = single_track_runner_nanyang_adaptive(detList_R1, ukf1_tpl_ad, params, n_frames);
    [trackSnapshots_R2_ad, finalTrk2_ad] = single_track_runner_nanyang_adaptive(detList_R2, ukf2_tpl_ad, params_r2, n_frames);

    r.ad_R1 = rmse_tracks(trackSnapshots_R1_ad, true_track, t1_grid, n_frames);
    r.ad_R2 = rmse_tracks(trackSnapshots_R2_ad, true_track, t2_grid, n_frames);

    r.type_R1_ad = finalTrk1_ad.type;  r.life_R1_ad = finalTrk1_ad.life;
    r.type_R2_ad = finalTrk2_ad.type;  r.life_R2_ad = finalTrk2_ad.life;
    r.nTrk_R1_ad = count_tracking_frames(trackSnapshots_R1_ad, n_frames);
    r.nTrk_R2_ad = count_tracking_frames(trackSnapshots_R2_ad, n_frames);
    r.n_frames = n_frames;

    % ---- Phase 6: 时间对齐 ----
    aligned_R2    = time_align_tracks(trackSnapshots_R2,    params);
    aligned_R2_ad = time_align_tracks(trackSnapshots_R2_ad, params);

    % ---- Phase 7: 融合 ----
    matched_pair = struct('R1_track_id', 1, 'R2_track_id', 1, ...
        'match_count', 0, 'coexist_count', 0, 'match_ratio', 1.0, ...
        'mean_dist_km', 0, 'quality', 100);
    method_names = {'SCC', 'BC', 'CI', 'FCI'};

    fus_base_rmses = nan(1, 4);
    fus_ad_rmses   = nan(1, 4);

    for m = 1:4
        snaps_base = run_track_fusion(matched_pair, trackSnapshots_R1, aligned_R2, params, method_names{m});
        snaps_ad   = run_track_fusion(matched_pair, trackSnapshots_R1_ad, aligned_R2_ad, params, method_names{m});
        fus_base_rmses(m) = rmse_fusion_snaps(snaps_base, true_track, t1_grid, n_frames);
        fus_ad_rmses(m)   = rmse_fusion_snaps(snaps_ad,   true_track, t1_grid, n_frames);
    end

    r.fus_base = min(fus_base_rmses);
    r.fus_ad   = min(fus_ad_rmses);

    sgl_R1_base = rmse_tracks(trackSnapshots_R1, true_track, t1_grid, n_frames);
    sgl_R2_base = rmse_tracks(aligned_R2,       true_track, t1_grid, n_frames);
    sgl_R1_ad   = rmse_tracks(trackSnapshots_R1_ad, true_track, t1_grid, n_frames);
    sgl_R2_ad   = rmse_tracks(aligned_R2_ad,       true_track, t1_grid, n_frames);

    r.sgl_base = min(sgl_R1_base, sgl_R2_base);
    r.sgl_ad   = min(sgl_R1_ad,   sgl_R2_ad);
end

% =========================================================================
% 工具函数
% =========================================================================
function v = rmse_detlist(detList, true_track, t_grid, n_frames, mode)
    errs = [];
    for k = 1:n_frames
        tl = interp1(true_track(:,5), true_track(:,1), t_grid(k), 'linear', 'extrap');
        tb = interp1(true_track(:,5), true_track(:,2), t_grid(k), 'linear', 'extrap');
        for d = 1:length(detList{k})
            dp = detList{k}(d);
            if dp.is_clutter, continue; end
            if strcmp(mode, 'raw')
                if isfield(dp, 'raw_lat') && ~isnan(dp.raw_lat)
                    errs(end+1) = sphere_utils_haversine_distance(dp.raw_lon, dp.raw_lat, tl, tb) / 1000;
                end
            else
                if isfield(dp, 'lat') && ~isnan(dp.lat)
                    errs(end+1) = sphere_utils_haversine_distance(dp.lon, dp.lat, tl, tb) / 1000;
                end
            end
        end
    end
    v = rms_km_val(errs);
end

function v = rmse_tracks(snaps, true_track, t_grid, n_frames)
    errs = [];
    for k = 1:n_frames
        tl = interp1(true_track(:,5), true_track(:,1), t_grid(k), 'linear', 'extrap');
        tb = interp1(true_track(:,5), true_track(:,2), t_grid(k), 'linear', 'extrap');
        snap = snaps{k};
        if ~isempty(snap.trackList)
            trk = snap.trackList{1};
            if trk.type ~= 7 && ~isnan(trk.lat)
                errs(end+1) = sphere_utils_haversine_distance(trk.lon, trk.lat, tl, tb) / 1000;
            end
        end
    end
    v = rms_km_val(errs);
end

function v = rmse_fusion_snaps(snaps, true_track, t_grid, n_frames)
    errs = [];
    for k = 1:n_frames
        tl = interp1(true_track(:,5), true_track(:,1), t_grid(k), 'linear', 'extrap');
        tb = interp1(true_track(:,5), true_track(:,2), t_grid(k), 'linear', 'extrap');
        if ~isempty(snaps{k}) && ~isempty(snaps{k}.trackList)
            trk = snaps{k}.trackList{1};
            if ~isnan(trk.lat)
                errs(end+1) = sphere_utils_haversine_distance(trk.lon, trk.lat, tl, tb) / 1000;
            end
        end
    end
    v = rms_km_val(errs);
end

function v = rms_km_val(e)
    if isempty(e), v = NaN; else, v = sqrt(mean(e.^2)); end
end

function print_row(label, vals)
    v = vals(~isnan(vals) & ~isinf(vals));
    if isempty(v)
        fprintf('%-28s %8s %8s %8s %8s %8s\n', label, 'NaN', 'NaN', 'NaN', 'NaN', 'NaN');
    else
        fprintf('%-28s %8.1f %8.1f %8.1f %8.1f %8.1f\n', ...
            label, mean(v), std(v), median(v), min(v), max(v));
    end
end

function n = count_tracking_frames(snaps, n_frames)
    n = 0;
    for k = 1:n_frames
        if ~isempty(snaps{k}) && ~isempty(snaps{k}.trackList)
            trk = snaps{k}.trackList{1};
            if trk.type == 1 && ~isnan(trk.lat)
                n = n + 1;
            end
        end
    end
end

function c = track_type_char(t)
    if t == 1;  c = 'T';   % TRACKING
    else;       c = 'H';   % HISTORY / not tracking
    end
end

function print_bs_row(label, vals)
    vv = vals(~isnan(vals) & ~isinf(vals));
    if isempty(vv)
        fprintf('%-16s %8s %8s %8s %8s %8s\n', label, 'NaN', 'NaN', 'NaN', 'NaN', 'NaN');
    else
        fprintf('%-16s %8.1f %8.1f %8.1f %8.1f %8.1f\n', ...
            label, mean(vv), median(vv), min(vv), max(vv), std(vv));
    end
end
