function out = run_online_assoc_compare(opts)
%RUN_ONLINE_ASSOC_COMPARE 四种跨站配对器在同一在线链路上的对比
%
%   out = run_online_assoc_compare()
%   out = run_online_assoc_compare(struct('scene','adsb_ns_fr24','max_frames',900, ...
%                                       'outdir','results/assoc_compare'))
%
%   【为什么必须在同一条链路上对比】
%     旧批处理线（已归档到 attic/legacy_batch/）里，四种配对器都是"整段"算法：
%     判决时能看到整段序列的未来帧。在线版本里四种配对器都只看当前帧及以前，
%     因此对比测的是"证据本身好不好"，而不是"谁用了更狠的全局信息"。
%     这是第 4 章 4.4 节的核心内容。
%
%   对比的四种方法（params.online_assoc_method）：
%     dualgate   — 三判据（位置 + 马氏 + 速度一致性）
%     nn         — 朴素最近邻（无门限）
%     sequential — 序贯最近邻（累计马氏通过率）
%     mewf       — 多证据加权融合（四项证据滚动累计）
%
%   选项 opts：
%     .scene      — 场景名，默认 'adsb_ns_fr24'
%     .max_frames — 帧数上限，默认 900
%     .outdir     — 输出目录，默认 results/assoc_compare
%     .seed       — 随机种子，默认参数集设定
%     .methods    — 要对比的方法列表，默认全部四种
%     .omega_slow / .omega_fast — IMM 模型集

    if nargin < 1 || isempty(opts), opts = struct(); end
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'config'), fullfile(root,'simulation'), fullfile(root,'ukf'), ...
            fullfile(root,'tracker'), fullfile(root,'fusion'), fullfile(root,'evaluation'), ...
            fullfile(root,'utils'), fullfile(root,'visualization'), fullfile(root,'io'));

    scene     = get_opt(opts, 'scene', 'adsb_ns_fr24');
    max_frames= get_opt(opts, 'max_frames', 900);
    outdir    = get_opt(opts, 'outdir', fullfile(root, 'results', 'assoc_compare'));
    methods   = get_opt(opts, 'methods', {'dualgate', 'nn', 'sequential', 'mewf'});
    if ~exist(outdir, 'dir'), mkdir(outdir); end

    % 公共参数：除关联策略外，其余完全一致，保证对比公平
    p = params_coverage_online();
    if isfield(opts, 'seed') && ~isempty(opts.seed), p.random_seed = opts.seed; end
    if isfield(opts, 'omega_slow') && ~isempty(opts.omega_slow)
        p.imm_ct_omega_slow_deg = opts.omega_slow;
    end
    if isfield(opts, 'omega_fast') && ~isempty(opts.omega_fast)
        p.imm_ct_omega_fast_deg = opts.omega_fast;
    end

    rows = struct('method', {}, 'n_pairs', {}, 'n_correct', {}, 'n_wrong', {}, ...
                  'precision', {}, 'recall', {}, 'n_dual_frames', {}, ...
                  'coverage', {}, 'ct_len_mean', {}, 'ratio_mean', {}, ...
                  'rmse_r1', {}, 'rmse_r2', {}, 'rmse_fused', {}, 'gain_pct', {}, ...
                  'elapsed_sec', {});
    details = struct('method', {}, 'per_truth', {}, 'lengthTable', {});

    for mi = 1:numel(methods)
        q = p;
        q.online_assoc_method = methods{mi};
        t0 = tic;
        r = run(scene, q, struct('max_frames', max_frames, 'verbose', false));
        el = toc(t0);
        M = r.metrics;

        rows(end+1) = struct('method', methods{mi}, ...
            'n_pairs', M.matching.n_pairs, ...
            'n_correct', M.matching.n_correct, ...
            'n_wrong', M.matching.n_wrong, ...
            'precision', M.matching.precision, ...
            'recall', M.matching.recall, ...
            'n_dual_frames', M.common_frame.n, ...
            'coverage', M.coverage.coverage, ...
            'ct_len_mean', M.length.system_mean, ...
            'ratio_mean', M.length.ratio_mean, ...
            'rmse_r1', M.common_frame.R1, ...
            'rmse_r2', M.common_frame.R2, ...
            'rmse_fused', M.common_frame.fused, ...
            'gain_pct', 100*(min(M.common_frame.R1, M.common_frame.R2) - M.common_frame.fused) / ...
                        min(M.common_frame.R1, M.common_frame.R2), ...
            'elapsed_sec', el); %#ok<AGROW>

        details(end+1) = struct('method', methods{mi}, ...
            'per_truth', M.matching, 'lengthTable', r.lengthTable); %#ok<AGROW>

        fprintf('[%s] 配对 %d（正确 %d / 误配 %d）查准 %.1f%% | 双源帧 %d | 覆盖 %.1f%% | 长度比 %.3f | 融合 %.2f km（%+.1f%%）| %.0fs%s', ...
            methods{mi}, M.matching.n_pairs, M.matching.n_correct, M.matching.n_wrong, ...
            100*M.matching.precision, M.common_frame.n, 100*M.coverage.coverage, ...
            M.length.ratio_mean, M.common_frame.fused, rows(end).gain_pct, el, newline);
    end

    % ---- 落盘 ----
    csv_path = fullfile(outdir, 'assoc_compare.csv');
    fid = fopen(csv_path, 'w');
    fprintf(fid, 'method,n_pairs,n_correct,n_wrong,precision_pct,recall_pct,n_dual_frames,coverage_pct,ct_len_mean,ratio_mean,rmse_r1,rmse_r2,rmse_fused,gain_pct,elapsed_sec\n');
    for i = 1:numel(rows)
        fprintf(fid, '%s,%d,%d,%d,%.2f,%.2f,%d,%.2f,%.2f,%.4f,%.3f,%.3f,%.3f,%.2f,%.1f\n', ...
            rows(i).method, rows(i).n_pairs, rows(i).n_correct, rows(i).n_wrong, ...
            100*rows(i).precision, 100*rows(i).recall, rows(i).n_dual_frames, ...
            100*rows(i).coverage, rows(i).ct_len_mean, rows(i).ratio_mean, ...
            rows(i).rmse_r1, rows(i).rmse_r2, rows(i).rmse_fused, rows(i).gain_pct, ...
            rows(i).elapsed_sec);
    end
    fclose(fid);

    out = struct('rows', rows, 'details', details, 'csv', csv_path, ...
                 'scene', scene, 'n_frames', max_frames);
    fprintf('\n对比表已写入 %s%s', csv_path, newline);
end

function v = get_opt(s, name, default)
    if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
        v = s.(name);
    else
        v = default;
    end
end
