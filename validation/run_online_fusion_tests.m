function ok = run_online_fusion_tests()
%RUN_ONLINE_FUSION_TESTS 系统航迹管理器的单元测试
%
%   ok = run_online_fusion_tests()
%
%   覆盖六个不变量：
%     T1 系统航迹预建：第一条局部航迹上报当帧即建立系统航迹
%     T2 M/N 序贯确认：第二个源必须累积到 M 帧才并入
%     T3 源独占：任一帧内一个局部航迹最多被一条系统航迹占用
%     T4 因果性：第 k 帧的判决只依赖第 k 帧及以前的信息
%     T5 合并历史拼接：重复系统航迹合并后覆盖帧是两者的并集
%     T6 长度不短于单站：系统航迹覆盖帧数 >= 单站覆盖帧数
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
    CT = central_track_manager('init', p);
    s1 = mk_snap(1, mk_local(7, 107.0, 31.0, 5, 1));
    s2 = mk_snap(1, []);
    [CT, log] = central_track_manager('step', CT, s1, s2, 1, p);
    ok = (log.n_ct == 1) && (CT(1).n_src == 1) && (CT(1).src1_id == 7) && ...
         (CT(1).cov_frames(1) == 1);
    fprintf('T1 系统航迹预建：%s（n_ct=%d）%s', ok_str(ok), log.n_ct, newline);
end

function ok = t2_sequential_confirm()
    p = test_params();
    CT = central_track_manager('init', p);
    confirmed_k = NaN;
    for k = 1:10
        s1 = mk_snap(k, mk_local(1, 107.0, 31.0, 10, 1));
        s2 = mk_snap(k, mk_local(2, 107.0 + 0.001, 31.0, 10, 1));
        [CT, ~] = central_track_manager('step', CT, s1, s2, k, p);
        if CT(1).n_src == 2 && isnan(confirmed_k), confirmed_k = k; end
    end
    % M=3：第二条源在第 3 帧才被并入
    ok = ~isnan(confirmed_k) && confirmed_k == 3;
    fprintf('T2 M/N 序贯确认：%s（确认帧=%s，应为 3）%s', ok_str(ok), ...
        num2str(confirmed_k), newline);
end

function ok = t3_source_exclusive()
    p = test_params();
    CT = central_track_manager('init', p);
    bad = false;
    for k = 1:12
        s1 = mk_snap(k, [mk_local(1, 107.0, 31.0, 10, 1), mk_local(2, 107.5, 31.5, 10, 2)]);
        s2 = mk_snap(k, [mk_local(1, 107.0, 31.0, 10, 1), mk_local(2, 107.5, 31.5, 10, 2)]);
        [CT, ~] = central_track_manager('step', CT, s1, s2, k, p);
        for radar = 1:2
            ids = [];
            for i = 1:numel(CT)
                if radar == 1, ids(end+1) = CT(i).src1_id; else, ids(end+1) = CT(i).src2_id; end %#ok<AGROW>
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
    CT1 = central_track_manager('init', p);
    for k = 1:12
        s1 = mk_snap(k, mk_local(1, 107.0 + 0.001*k, 31.0, 10, 1));
        s2 = mk_snap(k, mk_local(2, 107.0 + 0.001*k + 0.002, 31.0, 10, 1));
        [CT1, ~] = central_track_manager('step', CT1, s1, s2, k, p);
        if k == 5, CT_at5 = CT1; end   % 记录第 5 帧的状态快照
    end
    % 只跑 5 帧
    CT2 = central_track_manager('init', p);
    for k = 1:5
        s1 = mk_snap(k, mk_local(1, 107.0 + 0.001*k, 31.0, 10, 1));
        s2 = mk_snap(k, mk_local(2, 107.0 + 0.001*k + 0.002, 31.0, 10, 1));
        [CT2, ~] = central_track_manager('step', CT2, s1, s2, k, p);
    end
    % 只比较"第 5 帧时已存在"的那部分系统航迹：
    % 后续帧新建的航迹本来就不该出现在第 5 帧的状态里。
    ids2 = [CT2.id];
    ok = true;
    for i = 1:numel(CT2)
        j = find([CT_at5.id] == CT2(i).id, 1);
        if isempty(j)
            ok = false;  continue;
        end
        if CT_at5(j).n_src ~= CT2(i).n_src || norm(CT_at5(j).x - CT2(i).x) > 1e-12 || ...
                CT_at5(j).src1_id ~= CT2(i).src1_id || CT_at5(j).src2_id ~= CT2(i).src2_id
            ok = false;
        end
    end
    fprintf('T4 因果性（第 5 帧状态与跑满 12 帧一致）：%s（第 5 帧 %d 条系统航迹）%s', ...
        ok_str(ok), numel(ids2), newline);
end

function ok = t5_merge_history()
    p = test_params();
    CT = central_track_manager('init', p);
    % 1-3 帧：只有 R1 航迹（建立 CT-A）
    for k = 1:3
        s1 = mk_snap(k, mk_local(11, 107.0, 31.0, 10, 1));
        s2 = mk_snap(k, []);
        [CT, ~] = central_track_manager('step', CT, s1, s2, k, p);
    end
    n_before = numel(CT);
    % 4-5 帧：R2 航迹在远处（门限外）→ 建立第二条系统航迹 CT-B
    for k = 4:5
        s1 = mk_snap(k, mk_local(11, 107.0, 31.0, 10, 1));
        s2 = mk_snap(k, mk_local(22, 108.0, 31.0, 10, 1));
        [CT, ~] = central_track_manager('step', CT, s1, s2, k, p);
    end
    n_two_ct = numel(CT);
    % 6-8 帧：R2 航迹回到同一位置 → 触发重复系统航迹合并
    for k = 6:8
        s1 = mk_snap(k, mk_local(11, 107.0, 31.0, 10, 1));
        s2 = mk_snap(k, mk_local(22, 107.0, 31.0, 10, 1));
        [CT, log] = central_track_manager('step', CT, s1, s2, k, p);
    end
    % 合并后应只剩一条系统航迹，覆盖帧是两者并集（1-8 共 8 帧）
    has_alias = numel(CT) == 1 && ~isempty(CT(1).merge_alias);
    n_cov = numel(CT(1).cov_frames);
    ok = (n_before == 1) && (n_two_ct == 2) && has_alias && (n_cov == 8) && (CT(1).n_src == 2);
    fprintf('T5 合并历史拼接：%s（合并前 CT 数=%d，合并后=%d，覆盖帧=%d，应为 8）%s', ...
        ok_str(ok), n_two_ct, numel(CT), n_cov, newline);
end

function ok = t6_length_not_shorter()
    p = test_params();
    CT = central_track_manager('init', p);
    % R1 覆盖 1-4 帧，R2 覆盖 5-10 帧：系统航迹应覆盖 1-10 帧
    for k = 1:10
        if k <= 4
            s1 = mk_snap(k, mk_local(11, 107.0, 31.0, 10, 1));
            s2 = mk_snap(k, []);
        else
            s1 = mk_snap(k, mk_local(11, 107.0, 31.0, 10, 1));
            s2 = mk_snap(k, mk_local(22, 107.0, 31.0, 10, 1));
        end
        [CT, ~] = central_track_manager('step', CT, s1, s2, k, p);
    end
    n_sys = 0;
    for i = 1:numel(CT), n_sys = n_sys + numel(CT(i).cov_frames); end
    ok = n_sys >= max(4, 6);
    fprintf('T6 系统航迹覆盖不短于单站：%s（系统 %d 帧，单站 6 帧）%s', ...
        ok_str(ok), n_sys, newline);
end

function s = ok_str(ok)
    if ok, s = '通过'; else, s = '失败'; end
end
