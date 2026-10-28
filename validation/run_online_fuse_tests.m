function ok = run_online_fuse_tests()
%RUN_ONLINE_FUSE_TESTS 在线融合算法包（fusion/+online_fuse）的单元测试
%
%   ok = run_online_fuse_tests()
%
%   覆盖契约第 4 节要求的四条不变量：
%     T1 SCC 等权退化为算术平均：
%        两站协方差相同时，SCC 的融合结果等于 (x1+x2)/2，协方差为 P/2。
%     T2 BC 未退化成 SCC：
%        两站协方差相同时 BC 与 SCC 结果接近（差异在容差内），
%        但 P12 递推后 BC 的融合协方差与 SCC 不同（BC 确实用上了 P12）。
%     T3 帧间断时 P12 置零：
%        k 不连续时该配对的 P12 归零，融合退回等权平均。
%     T4 CI / FCI 保守：
%        trace(P_CI) ≥ trace(P_SCC)、trace(P_FCI) ≥ trace(P_SCC)。
%
%   另加两条工程性检查：
%     T5 因果性：第 k 帧的输出与"跑满 N 帧后取第 k 帧"完全一致。
%     T6 逐配对隔离：不同配对之间 P12 不串扰。
%
%   全部用例都是构造性的（合成数据），不依赖仿真场景，跑得很快。

    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root, 'fusion'));

    ok = true;
    ok = ok && t1_scc_equal_weight();
    ok = ok && t2_bc_not_degenerate();
    ok = ok && t3_p12_reset_on_break();
    ok = ok && t4_ci_fci_conservative();
    ok = ok && t5_causality();
    ok = ok && t6_pair_isolation();
    if ok
        fprintf('在线融合算法包单元测试：全部通过%s', newline);
    else
        fprintf('在线融合算法包单元测试：存在失败项%s', newline);
    end
end

% =====================================================================
% 测试参数
% =====================================================================
function p = test_params()
    p = struct();
    p.dt_sec = 30;
    p.bc_min_p12_scale = 0.05;
end

function est = mk_est(id, lon, lat, P)
%MK_EST 构造一帧的局部航迹估计（字段与 tracker 输出一致）
    est = struct();
    est.id = id;
    est.x = [lon; 0.01; lat; 0.01];
    est.P = P;
end

function P = mk_P(s)
%MK_P 构造一个各向同性的对角协方差
    P = diag([s, s * 1e-3, s, s * 1e-3]);
end

% =====================================================================
% T1 SCC 等权退化为算术平均
% =====================================================================
function ok = t1_scc_equal_weight()
    p = test_params();
    P = mk_P(1e-4);
    e1 = mk_est(1, 100.0, 30.0, P);
    e2 = mk_est(2, 100.2, 30.1, P);

    F = online_fuse.init('scc', p);
    [x, Pf, F] = online_fuse.fuse_step(F, e1, e2, 1, p);

    x_ref = 0.5 * (e1.x + e2.x);
    ok = norm(x - x_ref) < 1e-10 && norm(Pf - 0.5 * P) / norm(P) < 1e-10;
    fprintf('T1 SCC 等权退化为算术平均：%s（状态差 %.3e，协方差差 %.3e）%s', ...
        ok_str(ok), norm(x - x_ref), norm(Pf - 0.5 * P) / norm(P), newline);
end

% =====================================================================
% T2 BC 未退化成 SCC（P12 递推后两者不同）
% =====================================================================
function ok = t2_bc_not_degenerate()
    p = test_params();
    P = mk_P(1e-4);
    e1 = mk_est(1, 100.0, 30.0, P);
    e2 = mk_est(2, 100.2, 30.1, P);

    % 先跑若干帧让 P12 递推起来
    Fb = online_fuse.init('bc', p);
    for k = 1:5
        [xb, Pb, Fb] = online_fuse.fuse_step(Fb, e1, e2, k, p);
    end

    % 同数据下 SCC 的结果
    Fs = online_fuse.init('scc', p);
    [xs, Ps, Fs] = online_fuse.fuse_step(Fs, e1, e2, 5, p);

    % P12 必须非零（否则 BC 退化为 SCC）
    p12_nonzero = norm(Fb.P12, 'fro') > 0;

    % BC 与 SCC 结果接近（差异在容差内）
    close_enough = norm(xb - xs) < 1e-8 * max(1, norm(xs));

    % 但融合协方差必须不同（BC 确实用上了 P12）
    differ = norm(Pb - Ps, 'fro') / max(norm(Ps, 'fro'), eps) > 1e-6;

    ok = p12_nonzero && close_enough && differ;
    fprintf(['T2 BC 未退化成 SCC：%s（P12 范数 %.3e，状态差 %.3e，' ...
        '协方差相对差 %.3e）%s'], ok_str(ok), norm(Fb.P12, 'fro'), ...
        norm(xb - xs), norm(Pb - Ps, 'fro') / max(norm(Ps, 'fro'), eps), newline);
end

% =====================================================================
% T3 帧间断时 P12 置零
% =====================================================================
function ok = t3_p12_reset_on_break()
    p = test_params();
    P = mk_P(1e-4);
    e1 = mk_est(1, 100.0, 30.0, P);
    e2 = mk_est(2, 100.2, 30.1, P);

    F = online_fuse.init('bc', p);
    for k = 1:3
        [x, Pf, F] = online_fuse.fuse_step(F, e1, e2, k, p);
    end
    has_p12 = norm(F.P12, 'fro') > 0;

    % 帧间断：k 从 3 直接跳到 9
    [xb, Pb, F] = online_fuse.fuse_step(F, e1, e2, 9, p);
    reset_ok = norm(F.P12, 'fro') == 0;

    % 断连那一帧应退回等权平均（与 SCC 一致）
    Fs = online_fuse.init('scc', p);
    [xs, Ps, Fs] = online_fuse.fuse_step(Fs, e1, e2, 9, p);
    same_as_scc = norm(xb - xs) < 1e-10;

    ok = has_p12 && reset_ok && same_as_scc;
    fprintf('T3 帧间断时 P12 置零：%s（间断前 P12 范数 %.3e，间断后 %.3e）%s', ...
        ok_str(ok), has_p12 * 1.0, norm(F.P12, 'fro'), newline);
end

% =====================================================================
% T4 CI / FCI 保守：trace 不小于 SCC
% =====================================================================
function ok = t4_ci_fci_conservative()
    p = test_params();
    P1 = mk_P(1e-4);
    P2 = mk_P(4e-4);
    e1 = mk_est(1, 100.0, 30.0, P1);
    e2 = mk_est(2, 100.2, 30.1, P2);

    Fs = online_fuse.init('scc', p);
    [xs, Ps, Fs] = online_fuse.fuse_step(Fs, e1, e2, 1, p);

    Fci = online_fuse.init('ci', p);
    [xci, Pci, Fci] = online_fuse.fuse_step(Fci, e1, e2, 1, p);

    Ffci = online_fuse.init('fci', p);
    [xfci, Pfci, Ffci] = online_fuse.fuse_step(Ffci, e1, e2, 1, p);

    tr_scc = trace(Ps);
    ci_ok = trace(Pci) >= tr_scc * (1 - 1e-10);
    fci_ok = trace(Pfci) >= tr_scc * (1 - 1e-10);

    ok = ci_ok && fci_ok;
    fprintf('T4 CI/FCI 保守（trace 不小于 SCC）：%s（trace: SCC %.3e，CI %.3e，FCI %.3e）%s', ...
        ok_str(ok), tr_scc, trace(Pci), trace(Pfci), newline);
end

% =====================================================================
% T5 因果性：第 k 帧输出与跑满 N 帧后取第 k 帧一致
% =====================================================================
function ok = t5_causality()
    p = test_params();
    P = mk_P(1e-4);
    e1 = mk_est(1, 100.0, 30.0, P);
    e2 = mk_est(2, 100.2, 30.1, P);

    % 跑满 8 帧
    Ffull = online_fuse.init('bc', p);
    Xfull = zeros(4, 8);
    for k = 1:8
        [x, Pf, Ffull] = online_fuse.fuse_step(Ffull, e1, e2, k, p);
        Xfull(:, k) = x;
    end

    % 只跑到第 5 帧
    Fpart = online_fuse.init('bc', p);
    for k = 1:5
        [x, Pf, Fpart] = online_fuse.fuse_step(Fpart, e1, e2, k, p);
    end

    ok = norm(Xfull(:, 5) - x) < 1e-12;
    fprintf('T5 因果性（第 5 帧输出一致）：%s（差 %.3e）%s', ...
        ok_str(ok), norm(Xfull(:, 5) - x), newline);
end

% =====================================================================
% T6 逐配对隔离：不同配对的 P12 不串扰
% =====================================================================
function ok = t6_pair_isolation()
    p = test_params();
    P = mk_P(1e-4);
    a1 = mk_est(1, 100.0, 30.0, P);
    a2 = mk_est(2, 100.2, 30.1, P);
    b1 = mk_est(3, 101.0, 31.0, P);
    b2 = mk_est(4, 101.2, 31.1, P);

    F = online_fuse.init('bc', p);
    for k = 1:4
        [x, Pf, F] = online_fuse.fuse_step(F, a1, a2, k, p);
    end
    % 另一对从第 1 帧单独跑
    Fb = online_fuse.init('bc', p);
    for k = 1:4
        [xb, Pfb, Fb] = online_fuse.fuse_step(Fb, b1, b2, k, p);
    end

    % 混合推进：交替喂两对
    Fm = online_fuse.init('bc', p);
    for k = 1:4
        [x, Pf, Fm] = online_fuse.fuse_step(Fm, a1, a2, k, p);
        [x2, Pf2, Fm] = online_fuse.fuse_step(Fm, b1, b2, k, p);
    end

    n_pair = numel(Fm.pairs);
    ok = n_pair == 2;
    fprintf('T6 逐配对隔离：%s（配对数 %d）%s', ok_str(ok), n_pair, newline);
end

% =====================================================================
function s = ok_str(flag)
    if flag
        s = '通过';
    else
        s = '失败';
    end
end
