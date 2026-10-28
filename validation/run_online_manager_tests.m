function ok = run_online_manager_tests()
%RUN_ONLINE_MANAGER_TESTS 系统航迹管理器的单元测试
%
%   ok = run_online_manager_tests()
%
%   覆盖六个不变量：
%     T1 系统航迹预建：第一条局部航迹上报当帧即建立系统航迹
%     T2 M/N 序贯确认：第二个源必须累积到 M 帧才并入
%     T3 源独占：任一帧内一个局部航迹最多被一条系统航迹占用
%     T4 因果性：第 k 帧的判决只依赖第 k 帧及以前的信息
%     T5 合并历史拼接：重复系统航迹合并后覆盖帧是两者的并集
%     T6 长度不短于单站：系统航迹覆盖帧数 >= 单站覆盖帧数
%     T7 合并方向翻转：R2 先建 CT、R1 后建（覆盖互补布设的常态）也要能合并
%
%   全部用例都是构造性的（合成快照），不依赖仿真场景，跑得很快。

    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'tracker'), fullfile(root,'fusion'), fullfile(root,'config'), ...
            fullfile(root,'utils'));

    ok = true;
    ok = ok && t1_prebuild();
    ok = ok && t2_sequential_confirm();
    ok = ok && t3_source_exclusive();
    ok = ok && t4_causality();
    ok = ok && t5_merge_history();
    ok = ok && t6_length_not_shorter();
    ok = ok && t7_merge_reversed_orientation();
    if ok
        fprintf('在线融合管理器单元测试：全部通过%s', newline);
    else
        fprintf('在线融合管理器单元测试：存在失败项%s', newline);
    end
end

% =====================================================================
function p = test_params()
    p = struct();
    p.dt_sec = 30;
    p.online_gate_chi2 = 5.99;
    p.online_M = 3;
    p.online_N = 4;
    p.online_K_loss = 8;
    p.online_min_life = 1;
    p.online_merge_confirm = 2;
    p.online_fusion_method = 'SCC';
end

function snap = mk_snap(k, locals)
%MK_SNAP 由局部航迹数组构造一帧快照
    trk = cell(1, numel(locals));
    for i = 1:numel(locals)
        L = locals(i);
        trk{i} = struct('id', L.id, 'type', 1, 'lat', L.x(3), 'lon', L.x(1), ...
            'life', L.life, 'truth_idx', L.truth, ...
            'ukf', struct('x', L.x, 'P', L.P));
    end
    snap = struct('frameID', k, 'trackList', {trk});
end

function L = mk_local(id, lon, lat, life, truth, P)
    if nargin < 6 || isempty(P)
        P = diag([1e-4, 1e-8, 1e-4, 1e-8]);
    end
    L = struct('id', id, 'x', [lon; 0; lat; 0], 'P', P, 'life', life, 'truth', truth);
end

function ok = t1_prebuild()
    p = test_params();
    state = central_track_manager('init', p);
    s1 = mk_snap(1, mk_local(7, 107.0, 31.0, 5, 1));
    s2 = mk_snap(1, []);
    [state, log] = central_track_manager('step', state, s1, s2, 1, p);
    ok = (log.n_ct == 1) && (state.CT(1).n_src == 1) && (state.CT(1).src1_id == 7) && ...
         (state.CT(1).cov_frames(1) == 1);
    fprintf('T1 系统航迹预建：%s（n_ct=%d）%s', ok_str(ok), log.n_ct, newline);
end

function ok = t2_sequential_confirm()
    p = test_params();
    state = central_track_manager('init', p);
    confirmed_k = NaN;
    for k = 1:10
        s1 = mk_snap(k, mk_local(1, 107.0, 31.0, 10, 1));
        s2 = mk_snap(k, mk_local(2, 107.0 + 0.001, 31.0, 10, 1));
        [state, ~] = central_track_manager('step', state, s1, s2, k, p);
        if state.CT(1).n_src == 2 && isnan(confirmed_k), confirmed_k = k; end
    end
    % 语义：系统航迹在第 1 帧末才建立，证据从第 2 帧起算，
    % M=3 帧证据 → 第 4 帧确认（即确认帧 = M + 1）。
    ok = ~isnan(confirmed_k) && confirmed_k == 4;
    fprintf('T2 M/N 序贯确认：%s（确认帧=%s，应为 4 = M+1）%s', ok_str(ok), ...
        num2str(confirmed_k), newline);
end

function ok = t3_source_exclusive()
    p = test_params();
    state = central_track_manager('init', p);
    bad = false;
    for k = 1:12
        s1 = mk_snap(k, [mk_local(1, 107.0, 31.0, 10, 1), mk_local(2, 107.5, 31.5, 10, 2)]);
        s2 = mk_snap(k, [mk_local(1, 107.0, 31.0, 10, 1), mk_local(2, 107.5, 31.5, 10, 2)]);
        [state, ~] = central_track_manager('step', state, s1, s2, k, p);
        for radar = 1:2
            ids = [];
            for i = 1:numel(state.CT)
                if radar == 1, ids(end+1) = state.CT(i).src1_id; else, ids(end+1) = state.CT(i).src2_id; end %#ok<AGROW>
            end
            ids = ids(ids > 0);
            if numel(unique(ids)) ~= numel(ids), bad = true; end
        end
    end
    ok = ~bad;
    fprintf('T3 源独占（无共享局部航迹）：%s%s', ok_str(ok), newline);
end

function ok = t4_causality()
    p = test_params();
    % 跑 12 帧
    state1 = central_track_manager('init', p);
    for k = 1:12
        s1 = mk_snap(k, mk_local(1, 107.0 + 0.001*k, 31.0, 10, 1));
        s2 = mk_snap(k, mk_local(2, 107.0 + 0.001*k + 0.002, 31.0, 10, 1));
        [state1, ~] = central_track_manager('step', state1, s1, s2, k, p);
        if k == 5, CT_at5 = state1.CT; end   % 记录第 5 帧的状态快照
    end
    % 只跑 5 帧
    state2 = central_track_manager('init', p);
    for k = 1:5
        s1 = mk_snap(k, mk_local(1, 107.0 + 0.001*k, 31.0, 10, 1));
        s2 = mk_snap(k, mk_local(2, 107.0 + 0.001*k + 0.002, 31.0, 10, 1));
        [state2, ~] = central_track_manager('step', state2, s1, s2, k, p);
    end
    % 只比较"第 5 帧时已存在"的那部分系统航迹：
    % 后续帧新建的航迹本来就不该出现在第 5 帧的状态里。
    ids2 = [state2.CT.id];
    ok = true;
    for i = 1:numel(state2.CT)
        j = find([CT_at5.id] == state2.CT(i).id, 1);
        if isempty(j)
            ok = false;  continue;
        end
        if CT_at5(j).n_src ~= state2.CT(i).n_src || norm(CT_at5(j).x - state2.CT(i).x) > 1e-12 || ...
                CT_at5(j).src1_id ~= state2.CT(i).src1_id || CT_at5(j).src2_id ~= state2.CT(i).src2_id
            ok = false;
        end
    end
    fprintf('T4 因果性（第 5 帧状态与跑满 12 帧一致）：%s（第 5 帧 %d 条系统航迹）%s', ...
        ok_str(ok), numel(ids2), newline);
end

function ok = t5_merge_history()
    p = test_params();
    state = central_track_manager('init', p);
    % 1-3 帧：只有 R1 航迹（建立 CT-A）
    for k = 1:3
        s1 = mk_snap(k, mk_local(11, 107.0, 31.0, 10, 1));
        s2 = mk_snap(k, []);
        [state, ~] = central_track_manager('step', state, s1, s2, k, p);
    end
    n_before = numel(state.CT);
    % 4-5 帧：R2 航迹在远处（门限外）→ 建立第二条系统航迹 CT-B
    for k = 4:5
        s1 = mk_snap(k, mk_local(11, 107.0, 31.0, 10, 1));
        s2 = mk_snap(k, mk_local(22, 108.0, 31.0, 10, 1));
        [state, ~] = central_track_manager('step', state, s1, s2, k, p);
    end
    n_two_ct = numel(state.CT);
    % 6-8 帧：R2 航迹回到同一位置 → 触发重复系统航迹合并
    for k = 6:8
        s1 = mk_snap(k, mk_local(11, 107.0, 31.0, 10, 1));
        s2 = mk_snap(k, mk_local(22, 107.0, 31.0, 10, 1));
        [state, log] = central_track_manager('step', state, s1, s2, k, p);
    end
    % 合并后应只剩一条系统航迹，覆盖帧是两者并集（1-8 共 8 帧）
    has_alias = numel(state.CT) == 1 && ~isempty(state.CT(1).merge_alias);
    n_cov = numel(state.CT(1).cov_frames);
    ok = (n_before == 1) && (n_two_ct == 2) && has_alias && (n_cov == 8) && (state.CT(1).n_src == 2);
    fprintf('T5 合并历史拼接：%s（合并前 CT 数=%d，合并后=%d，覆盖帧=%d，应为 8）%s', ...
        ok_str(ok), n_two_ct, numel(state.CT), n_cov, newline);
end

function ok = t6_length_not_shorter()
    p = test_params();
    state = central_track_manager('init', p);
    % R1 覆盖 1-4 帧，R2 覆盖 5-10 帧：系统航迹应覆盖 1-10 帧
    for k = 1:10
        if k <= 4
            s1 = mk_snap(k, mk_local(11, 107.0, 31.0, 10, 1));
            s2 = mk_snap(k, []);
        else
            s1 = mk_snap(k, mk_local(11, 107.0, 31.0, 10, 1));
            s2 = mk_snap(k, mk_local(22, 107.0, 31.0, 10, 1));
        end
        [state, ~] = central_track_manager('step', state, s1, s2, k, p);
    end
    n_sys = 0;
    for i = 1:numel(state.CT), n_sys = n_sys + numel(state.CT(i).cov_frames); end
    ok = n_sys >= max(4, 6);
    fprintf('T6 系统航迹覆盖不短于单站：%s（系统 %d 帧，单站 6 帧）%s', ...
        ok_str(ok), n_sys, newline);
end

function s = ok_str(ok)
    if ok, s = '通过'; else, s = '失败'; end
end

% =====================================================================
% T7 合并方向翻转：保留者只持 R2 源时也必须合并
% =====================================================================
function ok = t7_merge_reversed_orientation()
%T7_MERGE_REVERSED_ORIENTATION 覆盖互补布设下的"R2 先建航迹"情形
%
%   目标从南侧进入走廊时 R2 先建系统航迹；等 R1 的局部航迹出现时，
%   若门限/成熟度还没放行，会另建一条 R1 单源系统航迹。此时"出生早的保留者"
%   是只持 R2 源的那条——原实现把查源方向写死成"保留者持 R1"，
%   find_local(L1, 0) 恒为空，merge_cnt 永久清零，两条航迹永不合并。
%   本用例是 #34 CSZ9278 的最小复现（单站 47/55 帧、并集 79 帧，系统只拿到 53 帧）。
    p = test_params();
    state = central_track_manager('init', p);
    % 1-3 帧：只有 R2 航迹 → 建立 CT-B（它将成为"出生早"的保留者）
    for k = 1:3
        s1 = mk_snap(k, []);
        s2 = mk_snap(k, mk_local(22, 107.0, 31.0, 10, 1));
        [state, ~] = central_track_manager('step', state, s1, s2, k, p);
    end
    n_before = numel(state.CT);
    % 4-5 帧：R1 航迹在门限外（约 96 km）→ 另建 CT-A，且晚于 CT-B
    for k = 4:5
        s1 = mk_snap(k, mk_local(11, 108.0, 31.0, 10, 1));
        s2 = mk_snap(k, mk_local(22, 107.0, 31.0, 10, 1));
        [state, ~] = central_track_manager('step', state, s1, s2, k, p);
    end
    n_two_ct = numel(state.CT);
    % 6-8 帧：R1 航迹回到同一位置 → 必须合并（保留者只持 R2 源）
    for k = 6:8
        s1 = mk_snap(k, mk_local(11, 107.0, 31.0, 10, 1));
        s2 = mk_snap(k, mk_local(22, 107.0, 31.0, 10, 1));
        [state, ~] = central_track_manager('step', state, s1, s2, k, p);
    end
    n_cov = 0;
    if numel(state.CT) == 1, n_cov = numel(state.CT(1).cov_frames); end
    ok = (n_before == 1) && (n_two_ct == 2) && (numel(state.CT) == 1) && ...
         (state.CT(1).n_src == 2) && (n_cov == 8);
    fprintf('T7 合并方向翻转（R2 先建 CT）：%s（合并前 CT 数=%d，合并后=%d，覆盖帧=%d，应为 8）%s', ...
        ok_str(ok), n_two_ct, numel(state.CT), n_cov, newline);
end
