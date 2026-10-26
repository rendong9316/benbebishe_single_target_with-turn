function cmp = compare_pairing_methods(S, do_fusion)
% compare_pairing_methods — 在同一份对齐航迹快照上对比跨站配对方法
%
% 对比四种方法（前三种为主，第四种为三判据的消融对照）：
%   'NN'               — 朴素最近邻（仅位置距离，无门限）
%   'Sequential'       — 序贯最近邻（马氏距离门限 + 贪心 1v1）
%   'DualGate(三判据)' — 位置 + 马氏 + 速度，含 1v1 互斥（默认 params 即此配置）
%   'DualGate(双门限)' — 关掉马氏 + 速度，仅位置 + 方差（原始双门限，消融用）
%
% 输出：
%   cmp.method{m}        — 方法名
%   cmp.metrics(m)       — 配对质量（查准/查全/误配/漏配），见 pairing.eval_pairing
%   cmp.fusion(m)        — 各融合算法 RMSE（仅在 do_fusion=true 时计算）
%   cmp.n_true_both      — 两站都出现的真实目标数（配对基准）
%
% 输入 S 为 load_aligned_snapshots 的返回值（含 trackSnapshots_R1 / aligned_R2 /
%   truthTrajs / params / n_frames）。也可直接传入 run() 的 result 结构体。
    if nargin < 2, do_fusion = false; end
    R1 = S.trackSnapshots_R1;
    R2 = S.aligned_R2;
    truthTrajs = S.truthTrajs;
    nf = S.n_frames;
    params = S.params;

    [n_true_both, ~, ~, ~, ~] = pairing.build_truth_pairs(R1, R2);

    % ---------- 四种配对方法 ----------
    p_nn  = track_matcher_nn(R1, R2, params);
    p_seq = track_matcher_sequential(R1, R2, params);
    p_dg  = track_matcher_dualgate(R1, R2, params);                 % 默认即三判据

    p_dg2 = [];
    if do_fusion
        p_dg2_params = params;
        p_dg2_params.dualgate_use_maha = false;                     % 消融：关马氏判据
        p_dg2_params.dualgate_use_vel  = false;                     % 消融：关速度判据
        p_dg2 = track_matcher_dualgate(R1, R2, p_dg2_params);
    end

    methods = {'NN', 'Sequential', 'DualGate(三判据)'};
    pairs_all = {p_nn, p_seq, p_dg};
    if do_fusion
        methods{end+1} = 'DualGate(双门限)';
        pairs_all{end+1} = p_dg2;
    end

    % ---------- 配对质量 ----------
    fprintf('\n--- 配对质量 ---\n');
    fprintf('  %-18s %5s %5s %5s %5s | %6s %6s %7s\n', ...
        'method', 'pairs', 'corr', 'wrong', 'miss', 'P', 'R', 'wrong%');
    for m = 1:numel(methods)
        M = pairing.eval_pairing(pairs_all{m}, R1, R2, n_true_both);
        cmp.method{m} = methods{m};
        cmp.metrics(m) = M;
        fprintf('  %-18s %5d %5d %5d %5d | %6.3f %6.3f %7.2f\n', ...
            methods{m}, M.n_pairs, M.n_correct, M.n_wrong, M.n_missed, ...
            M.precision, M.recall, 100*M.wrong_rate);
    end
    cmp.n_true_both = n_true_both;

    % ---------- 融合下游影响（可选） ----------
    if do_fusion
        method_names = {'SCC', 'BC', 'CI', 'FCI'};
        n_ac = length(truthTrajs);
        fprintf('\n--- 融合 RMSE（仅正确配对进入融合，km）---\n');
        fprintf('  %-18s %8s %8s %8s %8s | %6s %6s %6s %6s\n', ...
            'method', 'SCC', 'BC', 'CI', 'FCI', 'nSCC', 'R1', 'R2', 'cov%');
        for m = 1:numel(methods)
            pairs = pairs_all{m};
            % 构造极简 matcher：aligned_R2 + pair_to_aircraft（由 truth_idx 直接映射）
            matcher_min.aligned_R2 = R2;
            matcher_min.pair_to_aircraft = zeros(numel(pairs), 1);
            for p = 1:numel(pairs)
                t1 = pairing.truth_idx_of(R1, pairs(p).R1_track_id);
                t2 = pairing.truth_idx_of(R2, pairs(p).R2_track_id);
                ac = 0;
                if ~isnan(t1) && ~isnan(t2) && t1 == t2 && t1 > 0 && t1 <= n_ac
                    ac = t1;                                      % 正确配对才进入融合
                end
                matcher_min.pair_to_aircraft(p) = ac;
            end

            fs = cell(numel(pairs), numel(method_names));
            for p = 1:numel(pairs)
                for mi = 1:numel(method_names)
                    fs{p, mi} = run_track_fusion(pairs(p), R1, R2, params, method_names{mi});
                end
            end
            fe = evaluate_all_multi('fusion', fs, method_names, pairs, R1, R2, ...
                truthTrajs, nf, params.dt_sec, matcher_min);

            fout = struct('method', {}, 'rms', {}, 'n', {});
            for i = 1:numel(fe.overall)
                fout(i).method = fe.overall(i).method;
                fout(i).rms = fe.overall(i).s.rms;
                fout(i).n = fe.overall(i).s.n;
            end
            cmp.fusion{m} = fout;   % cell：fout 为 1×4 struct 数组（SCC/BC/CI/FCI）

            % 覆盖率 = SCC 的 both 帧 / (both+R1_only+R2_only)
            % fe.overall 顺序：[SCC, BC, CI, FCI, R1_only, R2_only]
            scc = fe.overall(1).s;
            r1o = fe.overall(5).s;  % R1_only
            r2o = fe.overall(6).s;  % R2_only
            cov = scc.n / max(scc.n + r1o.n + r2o.n, 1);
            fprintf('  %-18s %8.3f %8.3f %8.3f %8.3f | %6d %6.2f %6.2f %6.1f\n', ...
                methods{m}, fout(1).rms, fout(2).rms, fout(3).rms, fout(4).rms, ...
                scc.n, r1o.rms, r2o.rms, 100*cov);
        end
    end
end
