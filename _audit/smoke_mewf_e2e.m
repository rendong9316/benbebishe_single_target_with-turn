function smoke_mewf_e2e(scene_name, seed)
% smoke_mewf_e2e — MEWF 在主链路的端到端验证
%
% 目的：确认 run.m 的 Phase 6 分派能正确调用 track_matcher_mewf，
%       且其输出字段与 Phase 7（融合）/ Phase 8（评估）兼容。
%
% 背景：MEWF 此前只在离线脚本（run_mewf_compare.m）里跑过，主链路
%       switch 里没有 'mewf' 分支。本轮已补上，需端到端验证一次，
%       否则论文声称"提出 MEWF 方法"但代码主链路调不到，是硬伤。
%
% 用法：smoke_mewf_e2e('adsb_0002', 101)

    if nargin < 1 || isempty(scene_name), scene_name = 'adsb_0002'; end
    if nargin < 2 || isempty(seed),       seed = 101;              end

    here = fileparts(mfilename('fullpath'));
    repo = fileparts(here);
    addpath(repo);
    ds = {'config','simulation','ukf','tracker','fusion','evaluation', ...
          'utils','visualization','validation','experiments','io'};
    for i = 1:numel(ds)
        if exist(fullfile(repo, ds{i}), 'dir'), addpath(fullfile(repo, ds{i})); end
    end

    fprintf('\n===== MEWF 主链路端到端验证：%s seed %d =====\n', scene_name, seed);

    params = simulation_params_oracle();
    params.random_seed = seed;
    params.imm_ct_omega_slow_deg = 0.05;
    params.imm_ct_omega_fast_deg = 0.2;
    params.track_matcher_method = 'mewf';
    params.verbose = false;

    tic;
    r = run(scene_name, params);
    t_run = toc;

    fprintf('\n[结果] run() 耗时 %.1f s\n', t_run);
    fprintf('  配对数: %d\n', numel(r.matched_pairs));
    if ~isempty(r.matched_pairs)
        p1 = r.matched_pairs(1);
        fprintf('  首对字段: ');
        fn = fieldnames(p1);
        fprintf('%s ', fn{:});
        fprintf('\n');
        fprintf('  首对: R1#%d <-> R2#%d | mean_dist=%.2f km | coexist=%d | score=%.4f\n', ...
            p1.R1_track_id, p1.R2_track_id, p1.mean_dist_km, p1.coexist_count, p1.score);
    end

    % 评估指标：run() 返回 fusion_eval（evaluate_all_multi 结果）+ 单站误差
    if isfield(r, 'fusion_eval') && ~isempty(r.fusion_eval)
        fe = r.fusion_eval;
        fprintf('\n[融合评估] %-6s %8s %6s\n', 'method', 'RMSE', 'n');
        for i = 1:numel(fe.overall)
            fprintf('  %-8s %8.3f %6d\n', fe.overall(i).method, ...
                fe.overall(i).s.rms, fe.overall(i).s.n);
        end
    end
    if isfield(r, 'errorStats_R1') && isfield(r.errorStats_R1, 'rms')
        fprintf('[单站] R1 RMSE %.3f km | R2 RMSE %.3f km\n', ...
            r.errorStats_R1.rms, r.errorStats_R2.rms);
    end

    fprintf('\n[判定] %s\n', ternary_pass(~isempty(r.matched_pairs), ...
        '主链路 MEWF 调用成功，字段兼容', 'MEWF 未产生配对，需检查'));
end

function s = ternary_pass(cond, a, b)
    if cond, s = a; else, s = b; end
end
