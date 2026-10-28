function [CT, frameLog] = central_track_manager(action, CT, varargin)
%CENTRAL_TRACK_MANAGER 融合中心的系统航迹管理（逐周期在线）
%
%   CT = central_track_manager('init', params)
%   [CT, log] = central_track_manager('step', CT, snap1, snap2, k, params)
%
%   对应文档：双站航迹配对与融合在线处理模型_20261028.md 第 3、8、9 节。
%
%   核心模型：
%     系统航迹在第一条局部航迹上报时就建立，此时只有一个源。
%     配对不是"新建一条融合航迹"，而是"给已有的系统航迹增加一个源"。
%     因此系统航迹的覆盖范围与配对确认时刻无关。
%
%   单帧处理顺序（全部只用到当前帧及以前的信息）：
%     1. 把全部系统航迹预测到当前时刻
%     2. 为缺少某个源的系统航迹寻找另一站的候选局部航迹（马氏距离门限）
%     3. 当前帧内的贪心分配：同一局部航迹只归属一条系统航迹
%     4. M/N 确认：最近 N 帧中累计 M 帧落在门限内才真正并入
%        （不是"连续 M 帧"，否则个别帧越门限就会把计数清零）
%     5. 双源融合 / 单源透传 / 无源外推
%     6. 连续 online_K_loss 帧无源则删除
%     7. 未被任何系统航迹占用、也不在任何候选里的局部航迹，新建系统航迹
%
%   输入：
%     snap1 — R1 当前帧快照（未对齐）
%     snap2 — R2 当前帧快照（已时间对齐到 R1 网格）
%     k     — 当前帧号
%     params— 含 online_gate_chi2 / online_M / online_N / online_K_loss
%             / online_fusion_method
%
%   输出：
%     CT       — 系统航迹结构体数组
%     frameLog — 本帧诊断

    switch lower(action)
        case 'init'
            CT = empty_ct();
            frameLog = [];
        case 'step'
            [CT, frameLog] = step_impl(CT, varargin{:});
        otherwise
            error('central_track_manager:badAction', '未知动作：%s', action);
    end
end

% =====================================================================
% 单帧处理
% =====================================================================
function [CT, log] = step_impl(CT, snap1, snap2, k, params)
    dt = params.dt_sec;
    gate = getp(params, 'online_gate_chi2', 5.99);
    M = getp(params, 'online_M', 8);
    N = getp(params, 'online_N', M + 2);
    K_loss = getp(params, 'online_K_loss', 8);
    method = getp(params, 'online_fusion_method', 'SCC');

    % ---- 1. 预测到当前帧 ----
    F = [1, dt, 0, 0; 0, 1, 0, 0; 0, 0, 1, dt; 0, 0, 0, 1];
    q = diag([1e-10, 1e-14, 1e-10, 1e-14]);
    for i = 1:numel(CT)
        CT(i).x = F * CT(i).x;
        CT(i).P = F * CT(i).P * F' + q;
    end

    % ---- 2. 当前帧的局部航迹 ----
    L1 = local_tracks(snap1);
    L2 = local_tracks(snap2);

    % 已被双源系统航迹占用的局部航迹受保护，不允许被别的系统航迹抢走；
    % 只被单源系统航迹占用的可以抢（抢走后旧航迹无源，按 K_loss 自然删除，
    % 效果等价于两条重复系统航迹合并）。
    prot1 = protected_ids(CT, 1);
    prot2 = protected_ids(CT, 2);

    % ---- 3. 候选搜索 + 当前帧贪心分配 ----
    cand = struct('ct', {}, 'radar', {}, 'idx', {}, 'id', {}, 'd2', {});
    for i = 1:numel(CT)
        for r = 1:2
            if r == 1
                if CT(i).src1_id > 0, continue; end
                pool = L1; prot = prot1;
            else
                if CT(i).src2_id > 0, continue; end
                pool = L2; prot = prot2;
            end
            [jid, d2] = best_candidate(CT(i), pool, prot, gate);
            if jid > 0
                cand(end+1) = struct('ct', i, 'radar', r, 'idx', jid, ...
                    'id', pool(jid).id, 'd2', d2); %#ok<AGROW>
            end
        end
    end

    pick1 = zeros(numel(CT), 1);
    pick2 = zeros(numel(CT), 1);
    if ~isempty(cand)
        [~, ord] = sort([cand.d2]);
        cand = cand(ord);
        taken1 = false(1, numel(L1));
        taken2 = false(1, numel(L2));
        for c = 1:numel(cand)
            if cand(c).radar == 1
                if taken1(cand(c).idx), continue; end
                taken1(cand(c).idx) = true;
                pick1(cand(c).ct) = cand(c).id;
            else
                if taken2(cand(c).idx), continue; end
                taken2(cand(c).idx) = true;
                pick2(cand(c).ct) = cand(c).id;
            end
        end
    end

    % ---- 4. M/N 确认 ----
    for i = 1:numel(CT)
        if CT(i).src1_id == 0
            [CT(i), hit] = accumulate(CT(i), 1, pick1(i), M, N, k);
            if hit
                CT(i).src1_id = CT(i).pend1_id;
                CT(i).pend1_id = 0;  CT(i).hist1 = [];
                if CT(i).src2_id > 0 && isnan(CT(i).confirm_k)
                    CT(i).confirm_k = k;
                end
            end
        end
        if CT(i).src2_id == 0
            [CT(i), hit] = accumulate(CT(i), 2, pick2(i), M, N, k);
            if hit
                CT(i).src2_id = CT(i).pend2_id;
                CT(i).pend2_id = 0;  CT(i).hist2 = [];
                if CT(i).src1_id > 0 && isnan(CT(i).confirm_k)
                    CT(i).confirm_k = k;
                end
            end
        end
    end

    % ---- 5. 状态更新：双源融合 / 单源透传 / 无源外推 ----
    for i = 1:numel(CT)
        t1 = find_local(L1, CT(i).src1_id);
        t2 = find_local(L2, CT(i).src2_id);
        if isempty(t1), CT(i).src1_id = 0; end
        if isempty(t2), CT(i).src2_id = 0; end

        if ~isempty(t1) && ~isempty(t2)
            [x, P] = track_fusion_algorithms(method, t1.x, t1.P, t2.x, t2.P);
            CT(i).x = x;  CT(i).P = P;  CT(i).n_src = 2;
        elseif ~isempty(t1)
            CT(i).x = t1.x;  CT(i).P = t1.P;  CT(i).n_src = 1;
        elseif ~isempty(t2)
            CT(i).x = t2.x;  CT(i).P = t2.P;  CT(i).n_src = 1;
        else
            CT(i).n_src = 0;
        end

        if CT(i).n_src > 0
            CT(i).last_src_k = k;
            CT(i).n_covered = CT(i).n_covered + 1;
            if CT(i).n_src == 2 && isnan(CT(i).confirm_k)
                CT(i).confirm_k = k;
            end
        end
    end

    % ---- 6. 删除长期无源的系统航迹 ----
    keep = true(1, numel(CT));
    for i = 1:numel(CT)
        if k - CT(i).last_src_k > K_loss, keep(i) = false; end
    end
    CT = CT(keep);

    % ---- 7. 未被占用的局部航迹新建系统航迹 ----
    % 落在某条已有系统航迹门限内、正在累积候选的航迹不新建，
    % 否则同一目标会先被建成两条系统航迹。
    CT = CT(:)';
    for r = 1:2
        if r == 1, pool = L1; else, pool = L2; end
        for j = 1:numel(pool)
            lid = pool(j).id;
            if is_source(CT, r, lid), continue; end
            if is_pending(CT, r, lid), continue; end
            if in_any_gate(CT, pool(j), gate), continue; end
            CT(end+1) = new_ct(pool(j), r, k); %#ok<AGROW>
        end
    end

    % ---- 诊断 ----
    log = struct('frame', k, 'n_ct', numel(CT), ...
        'n_two', sum([CT.n_src] == 2), 'n_confirmed', sum(~isnan([CT.confirm_k])));
end

% =====================================================================
% M/N 计数器
% =====================================================================
function [ct, hit] = accumulate(ct, r, lid, M, N, k)
    % 同一候选才继续累计；候选变了就重新开始。历史窗口长度 N。
    if r == 1
        hist = ct.hist1;  pid = ct.pend1_id;
    else
        hist = ct.hist2;  pid = ct.pend2_id;
    end
    if lid <= 0
        hist = [hist, false];
    elseif lid == pid
        hist = [hist, true];
    else
        hist = false(1, numel(hist));
        hist = [hist, true];
        pid = lid;
    end
    if numel(hist) > N, hist = hist(end-N+1:end); end
    hit = sum(hist) >= M;
    if any(hist) && isnan(ct.first_cand_k)
        ct.first_cand_k = k;
    end

    if r == 1
        ct.hist1 = hist;  ct.pend1_id = pid;
    else
        ct.hist2 = hist;  ct.pend2_id = pid;
    end
end

% =====================================================================
% 局部航迹提取
% =====================================================================
function L = local_tracks(snap)
    L = struct('id', {}, 'x', {}, 'P', {}, 'truth_idx', {});
    if isempty(snap) || ~isfield(snap, 'trackList'), return; end
    trks = snap.trackList;
    for t = 1:numel(trks)
        trk = trks{t};
        if ~isfield(trk, 'type') || trk.type == 6 || trk.type == 7, continue; end
        if ~isfield(trk, 'ukf') || isempty(trk.ukf), continue; end
        if ~isfield(trk.ukf, 'x') || numel(trk.ukf.x) < 4, continue; end
        if ~isfield(trk.ukf, 'P') || any(size(trk.ukf.P) < 4), continue; end
        x = trk.ukf.x(:);
        if any(~isfinite(x(1:4))), continue; end
        L(end+1) = struct('id', trk.id, 'x', x, 'P', trk.ukf.P, ...
            'truth_idx', truth_of(trk)); %#ok<AGROW>
    end
end

function v = truth_of(trk)
    v = NaN;
    if isfield(trk, 'truth_idx') && isscalar(trk.truth_idx) && isfinite(double(trk.truth_idx))
        v = double(trk.truth_idx);
    end
end

function t = find_local(L, id)
    t = [];
    if id <= 0, return; end
    for i = 1:numel(L)
        if L(i).id == id, t = L(i); return; end
    end
end

% =====================================================================
% 候选与门限
% =====================================================================
function [jid, d2best] = best_candidate(ct, pool, prot, gate)
    jid = 0; d2best = inf;
    for j = 1:numel(pool)
        if ismember(pool(j).id, prot), continue; end
        d2 = maha2(ct.x, ct.P, pool(j).x, pool(j).P);
        if d2 < gate && d2 < d2best
            d2best = d2;  jid = j;
        end
    end
end

function tf = in_any_gate(CT, loc, gate)
    tf = false;
    for i = 1:numel(CT)
        if maha2(CT(i).x, CT(i).P, loc.x, loc.P) < gate
            tf = true;  return;
        end
    end
end

function d2 = maha2(x1, P1, x2, P2)
    % 位置维（索引 1 经度、3 纬度）的马氏距离平方，度换算到 km
    lat_mid = 0.5 * (x1(3) + x2(3));
    S = diag([111.32 * cosd(lat_mid), 110.57]);
    dx = S * [x1(1) - x2(1); x1(3) - x2(3)];
    Pp = [P1(1,1) + P2(1,1), P1(1,3) + P2(1,3);
          P1(3,1) + P2(3,1), P1(3,3) + P2(3,3)];
    Pp = S * Pp * S';
    Pp = 0.5 * (Pp + Pp');
    if rcond(Pp) < 1e-12
        Pp = Pp + 1e-6 * eye(2);
    end
    d2 = dx' * (Pp \ dx);
end

% =====================================================================
% 系统航迹构造与查询
% =====================================================================
function CT = empty_ct()
    CT = struct('id', {}, 'truth_idx', {}, 'src1_id', {}, 'src2_id', {}, ...
        'pend1_id', {}, 'hist1', {}, 'pend2_id', {}, 'hist2', {}, ...
        'x', {}, 'P', {}, 'n_src', {}, 'birth_k', {}, 'confirm_k', {}, ...
        'first_cand_k', {}, 'last_src_k', {}, 'n_covered', {});
end

function ct = new_ct(loc, radar, k)
    % 系统航迹编号沿用首个源航迹的编号，加雷达前缀避免两站编号冲突。
    % 编号一经建立不再改变，与"配对确认"无关。
    ct = struct('id', radar * 100000 + loc.id, 'truth_idx', loc.truth_idx, ...
        'src1_id', 0, 'src2_id', 0, ...
        'pend1_id', 0, 'hist1', false(1, 0), 'pend2_id', 0, 'hist2', false(1, 0), ...
        'x', loc.x, 'P', loc.P, 'n_src', 1, ...
        'birth_k', k, 'confirm_k', NaN, 'first_cand_k', NaN, ...
        'last_src_k', k, 'n_covered', 1);
    if radar == 1, ct.src1_id = loc.id; else, ct.src2_id = loc.id; end
end

function ids = protected_ids(CT, r)
    % 只保护"双源系统航迹"的源，单源系统航迹的源可被抢（等价于合并）
    ids = [];
    for i = 1:numel(CT)
        if CT(i).n_src < 2, continue; end
        if r == 1 && CT(i).src1_id > 0, ids(end+1) = CT(i).src1_id; end %#ok<AGROW>
        if r == 2 && CT(i).src2_id > 0, ids(end+1) = CT(i).src2_id; end %#ok<AGROW>
    end
end

function tf = is_source(CT, r, id)
    if r == 1
        tf = any([CT.src1_id] == id);
    else
        tf = any([CT.src2_id] == id);
    end
end

function tf = is_pending(CT, r, id)
    if r == 1
        tf = any([CT.pend1_id] == id & cellfun(@any, {CT.hist1}));
    else
        tf = any([CT.pend2_id] == id & cellfun(@any, {CT.hist2}));
    end
end

function v = getp(s, name, default)
    if isfield(s, name) && ~isempty(s.(name))
        v = s.(name);
    else
        v = default;
    end
end
