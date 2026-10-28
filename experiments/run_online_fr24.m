function out = run_online_fr24(opts)
%RUN_ONLINE_FR24 跨区双站在线配对融合的完整验证脚本
%
%   out = run_online_fr24()
%   out = run_online_fr24(struct('max_frames', 600, 'outdir', 'results/online_fr24'))
%
%   流程（全部走唯一主链路 run.m 的九阶段）：
%     1) 读取 FR24 原始数据（data/adsb_ns/scene_fr24.csv）构建南北向真实航迹场景
%     2) 逐帧在线运行：配准 → 点迹 → 单站跟踪 → 时间对齐 → 系统航迹分配与融合
%     3) 评估：航迹长度、融合精度、配对正确率
%     4) 出图 + 落盘（CSV / PNG / MAT）
%
%   选项 opts：
%     .max_frames — 只跑前若干帧（缺省全部 1950 帧）
%     .outdir    — 输出目录，缺省 results/online_fr24
%     .seed      — 随机种子，缺省用 params_coverage_online 的设定
%     .scene     — 场景名，缺省 'adsb_ns_fr24'（FR24 原始文件）
%     .omega_slow / .omega_fast — IMM 转弯率模型集，缺省用参数集设定

    if nargin < 1 || isempty(opts), opts = struct(); end
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'config'), fullfile(root,'simulation'), fullfile(root,'ukf'), ...
            fullfile(root,'tracker'), fullfile(root,'fusion'), fullfile(root,'evaluation'), ...
            fullfile(root,'utils'), fullfile(root,'visualization'), fullfile(root,'io'));

    max_frames = get_opt(opts, 'max_frames', Inf);
    outdir = get_opt(opts, 'outdir', fullfile(root, 'results', 'online_fr24'));
    scene = get_opt(opts, 'scene', 'adsb_ns_fr24');
    if ~exist(outdir, 'dir'), mkdir(outdir); end

    p = params_coverage_online();
    if isfield(opts, 'seed') && ~isempty(opts.seed), p.random_seed = opts.seed; end
    % IMM 模型集可配置：本场景南北向航迹机动很小，可切换到窄模型集
    if isfield(opts, 'omega_slow') && ~isempty(opts.omega_slow)
        p.imm_ct_omega_slow_deg = opts.omega_slow;
    end
    if isfield(opts, 'omega_fast') && ~isempty(opts.omega_fast)
        p.imm_ct_omega_fast_deg = opts.omega_fast;
    end

    log_file = fullfile(outdir, 'frame_log.csv');
    r = run(scene, p, struct('max_frames', max_frames, 'verbose', true, ...
        'log_file', log_file));

    % ---- 逐目标长度表 ----
    T = r.lengthTable;
    csv_path = fullfile(outdir, 'track_length.csv');
    fid = fopen(csv_path, 'w');
    fprintf(fid, 'truth,label,r1_len,r2_len,single_len,system_len,ratio\n');
    for i = 1:numel(T)
        fprintf(fid, '%d,%s,%d,%d,%d,%d,%.4f\n', T(i).truth, T(i).label, ...
            T(i).r1_len, T(i).r2_len, T(i).single, T(i).ct_len, T(i).ratio);
    end
    fclose(fid);

    % ---- 汇总 ----
    M = r.metrics;
    sum_path = fullfile(outdir, 'summary.txt');
    fid = fopen(sum_path, 'w');
    fprintf(fid, '场景: %s（FR24 原始文件 data/adsb_ns/scene_fr24.csv）%s', r.scenario.name, newline);
    fprintf(fid, '帧数: %d%s', r.n_frames, newline);
    fprintf(fid, '布设: R1(%.2fE %.2fN) 波束 %.2f deg / R2(%.2fE %.2fN) 波束 %.2f deg，视场 %.1f deg%s', ...
        p.radar1_lon, p.radar1_lat, p.radar1_beam_center_deg, ...
        p.radar2_lon, p.radar2_lat, p.radar2_beam_center_deg, p.beam_width_deg, newline);
    fprintf(fid, '关联策略: %s | 门限 chi2=%.2f, 确认 M/N = %d/%d, 最小成熟度 %d, K_loss=%d%s', ...
        p.online_assoc_method, p.online_gate_chi2, p.online_M, p.online_N, ...
        p.online_min_life, p.online_K_loss, newline);
    fprintf(fid, '\n[航迹长度]%s', newline);
    fprintf(fid, '  单站基线均值 %.1f 帧，系统航迹均值 %.1f 帧%s', ...
        M.length.single_mean, M.length.system_mean, newline);
    fprintf(fid, '  长度比 均值 %.3f，中位 %.3f，范围 [%.2f, %.2f]%s', ...
        M.length.ratio_mean, M.length.ratio_median, M.length.ratio_min, M.length.ratio_max, newline);
    fprintf(fid, '  延长 %d/%d 条，缩短 %d 条%s', ...
        M.length.n_longer, M.length.n_targets, M.length.n_shorter, newline);
    fprintf(fid, '\n[融合精度（共同帧）]%s', newline);
    fprintf(fid, '  共同帧 %d：R1 %.3f km，R2 %.3f km，系统 %.3f km%s', ...
        M.common_frame.n, M.common_frame.R1, M.common_frame.R2, M.common_frame.fused, newline);
    fprintf(fid, '  相对最优单站增益 %+.2f%%%s', ...
        100*(min(M.common_frame.R1, M.common_frame.R2)-M.common_frame.fused)/ ...
            min(M.common_frame.R1, M.common_frame.R2), newline);
    fprintf(fid, '\n[配对正确率]%s', newline);
    fprintf(fid, '  确认配对 %d 对，正确 %d 对，查准率 %.1f%%，查全率 %.1f%%%s', ...
        M.matching.n_pairs, M.matching.n_correct, ...
        100*M.matching.precision, 100*M.matching.recall, newline);
    fprintf(fid, '\n[融合覆盖率]%s', newline);
    fprintf(fid, '  双源 %d 帧 / 全部 %d 帧，覆盖率 %.1f%%%s', ...
        M.coverage.both, M.coverage.both+M.coverage.r1_only+M.coverage.r2_only, ...
        100*M.coverage.coverage, newline);
    fclose(fid);

    % ---- 图 ----
    figs = plot_online_fr24(r, outdir);

    % ---- 结果 MAT（v7，避免 v7.3 在本机读取崩溃）----
    mat_path = fullfile(outdir, 'online_fr24_result.mat');
    save('-v7', mat_path, 'r');

    out = struct('result', r, 'metrics', M, 'csv', csv_path, 'summary', sum_path, ...
        'figs', {figs}, 'mat', mat_path);
    fprintf('\n输出目录: %s%s', outdir, newline);
end

function v = get_opt(s, name, default)
    if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
        v = s.(name);
    else
        v = default;
    end
end
