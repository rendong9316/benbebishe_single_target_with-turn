function run_experiments(exp_id, seed_list, scene_filter)
%RUN_EXPERIMENTS 批量实验总控（断点续跑）
%
%   run_experiments(exp_id, seed_list)
%
%   断点续跑：每个 (场景, 配置, 种子) 的结果独立保存为
%     results/paper/<exp_id>/<scene>__<cfg>__s<seed>.mat
%   若文件已存在则跳过，因此中断后重新运行会从断点继续。
%
%   【2026-10-28】主链路已改为全在线（run.m 九阶段逐帧推进），
%   指标改用 evaluate_paper_metrics_online（在线口径）。
%   注意：在线关联的判决与旧的整段批处理匹配不同，
%   因此历史 results/paper/*.mat 的数值不能与重跑结果直接对比，
%   重跑时建议换一个 exp_id 前缀（如 E9_baseline_online）。
%
%   实验清单：
%     E1_density : 4 场景 × 默认配置（密度梯度）
%     E2_assoc   : 2 场景 × 双门限/三判据
%     E3_t1      : 2 场景 × T1 ∈ {20,35,50}
%     E4_m      : 2 场景 × M ∈ {4,8,12}
%
%   2026-10-25

    root = 'D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn';
    cd(root);
    % 【2026-10-26 优化】不用 genpath(root)：会扫描 results/ 下 9000+ 文件，
    % 批量实验时每次调用耗时数十秒。显式加代码目录。
    code_dirs = {'config','simulation','ukf','tracker','fusion', ...
                 'evaluation','utils','visualization','validation','experiments','io'};
    addpath(root);
    for cd_i = 1:numel(code_dirs)
        d_i = fullfile(root, code_dirs{cd_i});
        if exist(d_i, 'dir'), addpath(d_i); end
    end

    outdir = fullfile(root, 'results', 'paper', exp_id);
    if ~exist(outdir, 'dir'), mkdir(outdir); end
    logf = fullfile(outdir, 'progress.log');

    base = simulation_params_oracle();
    [scenes, cfgs, names] = define_exp(exp_id, base);
    % 可选场景过滤（并行跑单个场景时用，避免重复跑同场景的另一半）
    if nargin >= 3 && ~isempty(scene_filter)
        keep = ismember(scenes, scene_filter);
        scenes = scenes(keep);
    end
    n_total = numel(scenes) * numel(cfgs) * numel(seed_list);
    n_done = 0; n_skip = 0; n_fail = 0; n_run = 0;
    t_start = tic;

    logmsg(logf, sprintf('\n===== START %s  (%d units) =====', exp_id, n_total));

    for si = 1:numel(scenes)
        for ci = 1:numel(cfgs)
            for sd = seed_list
                f = fullfile(outdir, sprintf('%s__%s__s%02d.mat', scenes{si}, names{ci}, sd));
                if isfile(f)
                    n_skip = n_skip + 1; n_done = n_done + 1;
                    continue;
                end
                p = cfgs{ci};
                p.random_seed = sd;
                t0 = tic;
                try
                    % 抑制 run 与 metrics 的大量打印
                    evalc('r = run(scenes{si}, p);');
                    evalc('m = evaluate_paper_metrics_online(r);');
                    el = toc(t0);
                    meta = struct('exp', exp_id, 'scene', scenes{si}, 'cfg', names{ci}, ...
                        'seed', sd, 'elapsed_sec', el, 'created_at', datestr(now,31), ...
                        'matlab_version', version, 'n_targets', r.scenario.n_targets, ...
                        'n_frames', r.n_frames);
                    save(f, 'meta', 'm', '-v7.3');
                    n_run = n_run + 1;
                    msg = sprintf('[%d/%d] %s %s s%02d  OK  %.0fs  R1=%.2f SCC=%.2f cov=%.1f%% match=%.1f%%', ...
                        n_done+1, n_total, scenes{si}, names{ci}, sd, el, ...
                        m.rmse.R1_rms, method_rms(m,'SCC'), 100*m.coverage.coverage, ...
                        100*m.matching.match_rate);
                catch ME
                    n_fail = n_fail + 1;
                    msg = sprintf('[%d/%d] %s %s s%02d  FAIL  %s', ...
                        n_done+1, n_total, scenes{si}, names{ci}, sd, ME.message);
                end
                n_done = n_done + 1;
                logmsg(logf, msg);
            end
        end
    end

    el = toc(t_start);
    logmsg(logf, sprintf('===== END %s : run=%d skip=%d fail=%d elapsed=%.1f min =====', ...
        exp_id, n_run, n_skip, n_fail, el/60));
end

% ---------------------------------------------------------------
function [scenes, cfgs, names] = define_exp(exp_id, base)
    switch exp_id
        case 'E1_density'
            % 全四场景（大场景很慢，建议只配 1-2 个种子）
            scenes = {'multi_cross', 'adsb_0002', 'adsb_0608', 'adsb_0911'};
            cfgs = {base};
            names = {'default'};

        case 'E1_small'
            % 小场景高种子数：multi_cross(3目标) + adsb_0002(6架活跃)
            % 单次约 17 s + 55 s，10 种子约 12 分钟
            scenes = {'multi_cross', 'adsb_0002'};
            cfgs = {base};
            names = {'default'};

        case 'E1_mid'
            % 中密度：adsb_0608（28 架活跃，单次约 15-20 分钟）
            scenes = {'adsb_0608'};
            cfgs = {base};
            names = {'default'};

        case 'E1_large'
            % 高密度压力测试：adsb_0911（63 架活跃，单次可能 40+ 分钟）
            scenes = {'adsb_0911'};
            cfgs = {base};
            names = {'default'};

        case 'E2_assoc'
            scenes = {'adsb_0002', 'adsb_0608'};
            a = base; a.dualgate_use_maha = false; a.dualgate_use_vel = false;
            b = base; b.dualgate_use_maha = true;  b.dualgate_use_vel = true;
            cfgs = {a, b};
            names = {'dual', 'triple'};

        case 'E2_assoc_small'
            scenes = {'adsb_0002'};
            a = base; a.dualgate_use_maha = false; a.dualgate_use_vel = false;
            b = base; b.dualgate_use_maha = true;  b.dualgate_use_vel = true;
            cfgs = {a, b};
            names = {'dual', 'triple'};

        case 'E3_t1'
            scenes = {'adsb_0002'};
            cfgs = {}; names = {};
            for t1 = [20 35 50]
                q = base; q.dualgate_T1_km = t1;
                cfgs{end+1} = q; names{end+1} = sprintf('T1_%02d', t1); %#ok<AGROW>
            end

        case 'E4_m'
            scenes = {'adsb_0002'};
            cfgs = {}; names = {};
            for M = [4 8 12]
                q = base; q.dualgate_M = M;
                cfgs{end+1} = q; names{end+1} = sprintf('M_%02d', M); %#ok<AGROW>
            end

        case 'E5_fusion'
            % 融合配置对比：基线 / CCF 校准 / 距离门控 / 两者组合
            scenes = {'multi_cross', 'adsb_0002'};
            a = base;
            b = base; b.ccf_enable = true;
            c = base; c.fusion_gate_enable = true;
            d = base; d.ccf_enable = true; d.fusion_gate_enable = true;
            cfgs = {a, b, c, d};
            names = {'base', 'ccf', 'gate', 'ccf_gate'};

        case 'E6_kappa'
            % CCF 校准因子敏感性
            % 注意：单 seed(s42) 得 kappa=2.4，但多 seed 下 R1 反而更准，
            % 故范围覆盖 kappa<1（减小 R1 的 P = 增大其权重）与 kappa>1
            scenes = {'adsb_0002'};
            cfgs = {}; names = {};
            for k = [0.4 0.7 1.0 1.5 2.4]
                q = base; q.ccf_enable = true; q.ccf_kappa_1 = k;
                cfgs{end+1} = q; names{end+1} = sprintf('k_%.1f', k); %#ok<AGROW>
            end

        case 'E7_model'
            % CT 模型集对比：默认 0.7/1.5 vs 中速 0.2/0.7 vs 慢速 0.05/0.2
            % 依据：真实航迹转弯率中位 0.012、p90 0.23、p99 0.66
            scenes = {'adsb_0002', 'multi_cross'};
            sets = {[0.7 1.5], [0.2 0.7], [0.05 0.2]};
            lbl  = {'w07_15', 'w02_07', 'w005_02'};
            cfgs = {}; names = {};
            for i = 1:numel(sets)
                q = base;
                q.imm_ct_omega_slow_deg = sets{i}(1);
                q.imm_ct_omega_fast_deg = sets{i}(2);
                cfgs{end+1} = q; names{end+1} = lbl{i}; %#ok<AGROW>
            end

        case 'E8_optimal'
            % 蒙特卡洛：最优模型集 0.05/0.2 × 多场景 × 多种子
            scenes = {'adsb_0002', 'multi_cross'};
            q = base;
            q.imm_ct_omega_slow_deg = 0.05;
            q.imm_ct_omega_fast_deg = 0.20;
            cfgs = {q}; names = {'opt'};

        case 'E9_baseline'
            % 大规模 MC：基线模型集 0.7/1.5（配置名与 E7 一致，便于合并分析）
            scenes = {'adsb_0002', 'multi_cross'};
            cfgs = {base}; names = {'w07_15'};

        case 'E10_optimal'
            % 大规模 MC：最优模型集 0.05/0.2（配置名与 E7 一致）
            scenes = {'adsb_0002', 'multi_cross'};
            q = base;
            q.imm_ct_omega_slow_deg = 0.05;
            q.imm_ct_omega_fast_deg = 0.20;
            cfgs = {q}; names = {'w005_02'};

        case 'E11_mid'
            % 大规模 MC：中间档 0.2/0.7（配置名与 E7 一致）
            scenes = {'adsb_0002', 'multi_cross'};
            q = base;
            q.imm_ct_omega_slow_deg = 0.20;
            q.imm_ct_omega_fast_deg = 0.70;
            cfgs = {q}; names = {'w02_07'};

        case 'E12_sweep'
            % 模型集细扫：找最优档位（验证"匹配"而非"越小越好"）
            % 真实转弯率中位 0.012、p90 0.23，预期最优在慢速区间
            scenes = {'adsb_0002'};
            sets = {[0.7 1.5], [0.3 0.7], [0.2 0.4], [0.1 0.3], [0.05 0.2], [0.02 0.05]};
            lbl  = {'s07_15','s03_07','s02_04','s01_03','s005_02','s002_005'};
            cfgs = {}; names = {};
            for i = 1:numel(sets)
                q = base;
                q.imm_ct_omega_slow_deg = sets{i}(1);
                q.imm_ct_omega_fast_deg = sets{i}(2);
                cfgs{end+1} = q; names{end+1} = lbl{i}; %#ok<AGROW>
            end

        case 'E13_dense'
            % 高密度多目标：验证模型集杠杆在 28 架 / 63 架场景下是否仍然成立
            % 同时验证三判据配对在高密度下的价值（候选层误配会显著上升）
            scenes = {'adsb_0608', 'adsb_0911'};
            a = base;
            b = base;
            b.imm_ct_omega_slow_deg = 0.05;
            b.imm_ct_omega_fast_deg = 0.20;
            cfgs  = {a, b};
            names = {'w07_15', 'w005_02'};

        case 'E14_reg'
            % 配准三模式对比：none（不校正）/ mean（两参数均值）/ mle3（三参数 MLE）
            % 用于量化"电离层虚高失配"这一误差源，以及三参数配准能收回多少
            scenes = {'adsb_0002', 'adsb_0608'};
            cfgs = {}; names = {};
            for md = {'none','mean','mle3'}
                q = base; q.registration_mode = md{1};
                cfgs{end+1} = q; names{end+1} = ['reg_' md{1}]; %#ok<AGROW>
            end

        case 'E15_hsens'
            % 虚高失配敏感性：真值 h 从 310 到 380 km，看三参数配准的相对收益如何随失配增大
            % 预期：失配越大，mean 的残余距离相关偏差越大，mle3 的优势越明显
            scenes = {'adsb_0002'};
            cfgs = {}; names = {};
            for hh = [310 320 340 380]
                for md = {'mean','mle3'}
                    q = base;
                    q.ionosphere_true_h_km = hh;
                    q.registration_mode   = md{1};
                    cfgs{end+1} = q;
                    names{end+1} = sprintf('h%d_%s', hh, md{1}); %#ok<AGROW>
                end
            end

        otherwise
            error('run_experiments:unknownExp', '未知实验: %s', exp_id);
    end
end

% ---------------------------------------------------------------
function v = method_rms(m, name)
    v = NaN;
    for i = 1:numel(m.rmse.fusion)
        if strcmp(m.rmse.fusion(i).method, name), v = m.rmse.fusion(i).rms; return; end
    end
end

% ---------------------------------------------------------------
function logmsg(f, msg)
    fid = fopen(f, 'a');
    if fid > 0
        fprintf(fid, '%s  %s\n', datestr(now,'HH:MM:SS'), msg);
        fclose(fid);
    end
    fprintf('%s\n', msg);
end
