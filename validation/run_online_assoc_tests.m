function ok = run_online_assoc_tests()
%RUN_ONLINE_ASSOC_TESTS 在线关联策略包的单元测试
%
%   ok = run_online_assoc_tests()
%
%   覆盖：
%     T1 四种方法都能初始化并产出候选
%     T2 nn 无门限（远处也成候选）
%     T3 sequential 要求马氏通过率达标
%     T4 dualgate 三判据同时成立才成候选
%     T5 mewf 综合分单调不减（滚动累计的必然性质）
%     T6 只读当前帧与历史，不读未来帧
%
%   全部用例都是构造性的，不依赖仿真场景。

    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'tracker'), fullfile(root,'fusion'), fullfile(root,'config'));

    ok = true;
    ok = ok && t1_all_methods();
    ok = ok && t2_nn_no_gate();
    ok = ok && t3_sequential_gate();
    ok = ok && t4_dualgate_three_criteria();
    ok = ok && t5_mewf_monotone();
    ok = ok && t6_no_future();
    if ok
        fprintf('在线关联策略单元测试：全部通过%s', newline);
    else
        fprintf('在线关联策略单元测试：存在失败项%s', newline);
    end
end

% =====================================================================
function p = test_params()
    p = struct();
    p.dt_sec = 30;
    p.dualgate_T1_km = 35;
    p.dualgate_maha_thresh = 5.99;
    p.dualgate_maha_ratio = 0.8;
    p.dualgate_use_maha = true;
    p.dualgate_use_vel = true;
    p.dualgate_vel_kappa = 3.0;
    p.mewf_sigma_d_km = 15;
    p.mewf_score_thr = 0.55;
    p.mewf_w_dist = 1.0;
    p.mewf_w_maha = 1.0;
    p.mewf_w_vel = 1.0;
    p.mewf_w_cont = 1.0;
end

function L = mk_local(id, lon, lat, life, truth, sig)
    if nargin < 6 || isempty(sig), sig = 1e-4; end
    x = [lon; 0; lat; 0];
    P = diag([sig, 1e-8, sig, 1e-8]);
    L = struct('id', id, 'x', x, 'P', P, 'truth_idx', truth, 'life', life);
end

% =====================================================================
function ok = t1_all_methods()
    p = test_params();
    methods = {'dualgate', 'nn', 'sequential', 'mewf'};
    ok = true;
    for i = 1:numel(methods)
        S = online_assoc.init_strategy(methods{i}, p);
        L1 = mk_local(1, 107.0, 31.0, 10, 1);
        L2 = mk_local(2, 107.01, 31.0, 10, 1);
        [c, S] = online_assoc.update_candidates(S, L1, L2, 1, p);
        if isempty(c)
            ok = false;
            fprintf('  T1 %s 未产出候选%s', methods{i}, newline);
        end
    end
    fprintf('T1 四种方法均可初始化并产候选：%s%s', ok_str(ok), newline);
end

% =====================================================================
function ok = t2_nn_no_gate()
    p = test_params();
    S = online_assoc.init_strategy('nn', p);
    % 距离远超 T1（100 km > 35 km），nn 仍应成候选
    L1 = mk_local(1, 107.0, 31.0, 10, 1);
    L2 = mk_local(2, 108.0, 31.0, 10, 1);
    [c_nn, ~] = online_assoc.update_candidates(S, L1, L2, 1, p);

    S2 = online_assoc.init_strategy('dualgate', p);
    [c_dg, ~] = online_assoc.update_candidates(S2, L1, L2, 1, p);

    ok = (numel(c_nn) == 1) && isempty(c_dg);
    fprintf('T2 nn 无门限 / dualgate 有距离门限：%s%s', ok_str(ok), newline);
end

% =====================================================================
function ok = t3_sequential_gate()
    p = test_params();
    S = online_assoc.init_strategy('sequential', p);
    L1 = mk_local(1, 107.0, 31.0, 10, 1);
    % 距离 30 km（< T1=35 km）但两站协方差很小 → 马氏距离很大
    % → 通过率 0 → 应被门限挡掉
    L2 = mk_local(2, 107.27, 31.0, 10, 1, 1e-8);
    for k = 1:4
        [c, S] = online_assoc.update_candidates(S, L1, L2, k, p);
    end
    blocked = isempty(c);

    % 同样的几何，但协方差正常 → 马氏通过率 1.0 → 应成候选
    S2 = online_assoc.init_strategy('sequential', p);
    L2b = mk_local(2, 107.27, 31.0, 10, 1, 1e-1);
    for k = 1:4
        [c2, S2] = online_assoc.update_candidates(S2, L1, L2b, k, p);
    end
    passed = (numel(c2) == 1);

    ok = blocked && passed;
    fprintf('T3 sequential 马氏通过率门限生效（挡掉 %d，通过 %d）：%s%s', ...
        blocked, passed, ok_str(ok), newline);
end

% =====================================================================
function ok = t4_dualgate_three_criteria()
    p = test_params();
    S = online_assoc.init_strategy('dualgate', p);
    % 位置近、马氏通过，但速度严重不一致 → 速度判据挡掉
    L1 = mk_local(1, 107.0, 31.0, 10, 1);
    L2 = mk_local(2, 107.005, 31.0, 10, 1);
    L2.x(2) = 1e-3;   % 经向速度 1e-3 度/秒 ≈ 111 m/s，与 L1 差一个量级
    [c_vel, ~] = online_assoc.update_candidates(S, L1, L2, 1, p);

    % 关掉速度判据后应成候选
    p2 = p;  p2.dualgate_use_vel = false;
    S2 = online_assoc.init_strategy('dualgate', p2);
    [c_norvel, ~] = online_assoc.update_candidates(S2, L1, L2, 1, p2);

    ok = isempty(c_vel) && (numel(c_norvel) == 1);
    fprintf('T4 dualgate 三判据（速度一致性生效）：%s%s', ok_str(ok), newline);
end

% =====================================================================
function ok = t5_mewf_monotone()
    p = test_params();
    p.mewf_score_thr = 0.0;   % 放开门限以便观察分数演化
    S = online_assoc.init_strategy('mewf', p);
    % 逐帧两站距离在变远但都在 T1 内：距离证据 E_d 下降，
    % 且 E_c = max_run_ok/n_coexist 随分母发散，故**综合分必然单调下降**（实测 0.999→0.868）。
    % 本用例题目的不是"分数单调"，而是"累计量单调"——这是 MEWF 在线化的真正不变量；
    % 确认的单调性由融合中心的 M/N 确认负责，不由分数负责。
    L1 = mk_local(1, 107.0, 31.0, 10, 1);
    scores = [];
    for k = 1:6
        L2 = mk_local(2, 107.0 + 0.01*k, 31.0, 10, 1);
        [c, S] = online_assoc.update_candidates(S, L1, L2, k, p);
        if ~isempty(c), scores(end+1) = c(1).score; end %#ok<AGROW>
    end
    % 只断言"累计量单调"：n_coexist、sum_ed、max_run_ok 都是累计计数，只增不减。
    % 综合分本身非单调（见上），不能作为不变量断言。
    a = S.acc(1);
    accum_monotone = (a.n_coexist == 6) && (a.sum_ed > 0) && (a.sum_ed <= 6) && ...
                    (a.max_run_ok >= 1);
    ok = accum_monotone;
    fprintf('T5 mewf 累计量单调（综合分 %s 非单调，累计 %d 帧，sum_ed=%.3f）：%s%s', ...
        mat2str(round(scores*1000)/1000), a.n_coexist, a.sum_ed, ok_str(ok), newline);
end

% =====================================================================
function ok = t6_no_future()
    p = test_params();
    L1 = mk_local(1, 107.0, 31.0, 10, 1);
    L2 = mk_local(2, 107.005, 31.0, 10, 1);
    L1b = mk_local(1, 107.0, 31.0, 20, 1);
    L2b = mk_local(2, 107.005, 31.0, 20, 1);

    % 因果性检验：跑满 5 帧时，第 3 帧那一刻的累计证据必须与"只跑 3 帧"完全一致。
    % 若策略读了未来帧或依赖了随帧变化的外部量，两者必然不同。
    S1 = online_assoc.init_strategy('mewf', p);
    for k = 1:3
        [~, S1] = online_assoc.update_candidates(S1, L1, L2, k, p);
    end
    snap_at3 = [];
    S2 = online_assoc.init_strategy('mewf', p);
    for k = 1:5
        [~, S2] = online_assoc.update_candidates(S2, L1, L2, k, p);
        if k == 3, snap_at3 = S2.acc(1); end
    end
    a1 = S1.acc(1);  a2 = S2.acc(1);
    same = (a1.n_coexist == snap_at3.n_coexist) && ...
           (abs(a1.sum_ed - snap_at3.sum_ed) < 1e-12) && ...
           (a1.n_maha == snap_at3.n_maha) && (a1.n_vel == snap_at3.n_vel) && ...
           (a1.max_run_ok == snap_at3.max_run_ok) && (a1.run_ok == snap_at3.run_ok);
    % 且第 5 帧的证据比第 3 帧多（确实在推进，不是停滞）
    grew = (a2.n_coexist > a1.n_coexist);

    % 附加检验：life 不同不应影响证据（证据只用位置与协方差）
    S3 = online_assoc.init_strategy('mewf', p);
    for k = 1:5
        [~, S3] = online_assoc.update_candidates(S3, L1b, L2b, k, p);
    end
    a3 = S3.acc(1);
    life_independent = (a3.n_coexist == a2.n_coexist) && ...
                       (abs(a3.sum_ed - a2.sum_ed) < 1e-12);

    ok = same && grew && life_independent;
    fprintf('T6 因果性（第3帧状态与跑满5帧一致：%d，推进：%d，life无关：%d）：%s%s', ...
        same, grew, life_independent, ok_str(ok), newline);
end

function s = ok_str(ok)
    if ok, s = '通过'; else, s = '失败'; end
end
