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
%     2. 取出两站当前帧的局部航迹（含 life，用于成熟度判断）
%     3. 释放失联过久的源绑定（连续 online_K_loss 帧无观测）
%     4. 合并重复的系统航迹：两条各持一站源、位置在门限内的单源系统航迹
%        合并为一条双源系统航迹（连续 online_merge_confirm 帧满足才合并）
%     5. 为缺少某个源的系统航迹寻找另一站的候选局部航迹（马氏距离门限）
%     6. 当前帧内的贪心分配：同一局部航迹只归属一条系统航迹
%     7. M/N 确认：最近 N 帧中累计 M 帧落在门限内才真正并入
%        （不是"连续 M 帧"，否则个别帧越门限就会把计数清零）
%     8. 双源融合 / 单源透传 / 无源外推
%     9. 连续 online_K_loss 帧无源则删除
%    10. 未被任何系统航迹占用、也不在任何候选里的局部航迹，新建系统航迹
%
%   输入：
%     snap1 — R1 当前帧快照（未对齐）
%     snap2 — R2 当前帧快照（已时间对齐到 R1 网格）
%     k     — 当前帧号
%     params— 含 online_gate_chi2 / online_M / online_N / online_K_loss
%             / online_fusion_method / online_min_life / online_merge_confirm
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
    M = getp(params, 'online_M', 6);
    N = getp(params, 'online_N', M + 2);
    K_loss = getp(params, 'online_K_loss', 8);
    method = getp(params, 'online_fusion_method', 'SCC');
    % 证据起算所需的最小成熟度：两条航迹都要 life >= min_life 才允许累积证据。
    % 这是"延迟起算"而不是"放宽门限"：用帧数余量换判别力。
    min_life = getp(params, 'online_min_life', 5);

    % ---- 1. 预测到当前帧 ----
    F = [1, dt, 0, 0; 0, 1, 0, 0; 0, 0, 1, dt; 0, 0, 0, 1];
    q = diag([1e-10, 1e-14, 1e-10, 1e-14]);
    for i = 1:numel(CT)
        if isempty(CT(i).x), continue; end
        CT(i).x = F * CT(i).x;
        CT(i).P = F * CT(i).P * F' + q;
    end

    % ---- 2. 当前帧的局部航迹 ----
    L1 = local_tracks(snap1);
    L2 = local_tracks(snap2);

    % ---- 3. 释放失联过久的源绑定 ----
    % 源绑定不因为某一帧缺观测而丢失，只在连续 K_loss 帧无观测后才释放，
    % 这样目标短暂被遮挡时系统航迹不会"忘掉"它，恢复后仍是同一个源。
    for i = 1:numel(CT)
        if CT(i).src1_id > 0
            if isempty(find_local(L1, CT(i).src1_id))
                CT(i).src1_miss = CT(i).src1_miss + 1;
                if CT(i).src1_miss > K_loss
                    CT(i).src1_id = 0;  CT(i).src1_miss = 0;
                    CT(i).src1_truth = NaN;
                    CT(i).pend1_id = 0;  CT(i).hist1 = false(1, 0);
                end
            else
                CT(i).src1_miss = 0;
            end
        end
        if CT(i).src2_id > 0
            if isempty(find_local(L2, CT(i).src2_id))
                CT(i).src2_miss = CT(i).src2_miss + 1;
                if CT(i).src2_miss > K_loss
                    CT(i).src2_id = 0;  CT(i).src2_miss = 0;
                    CT(i).src2_truth = NaN;
                    CT(i).pend2_id = 0;  CT(i).hist2 = false(1, 0);
                end
            else
                CT(i).src2_miss = 0;
            end
        end
    end

    % ---- 4. 合并重复的系统航迹 ----
    % 覆盖互补布设下，同一目标会先被一站建成一条单源系统航迹，进入重叠区后
    % 又被另一站建成第二条。两条各持一站的源、位置落在同一门限内，应合并为
    % 一条双源系统航迹；不合并的话，同一局部航迹会被两条系统航迹共享，
    % 显控上就出现"一个目标两条系统航迹"。
    % 合并保留出生早的那条编号，并把被合并者的历史拼接到保留者身上，
    % 因此合并后航迹长度是二者并集，而不是只算合并之后的部分。
    merge_M = getp(params, 'online_merge_confirm', 3);
    [CT, n_merge] = merge_duplicate_ct(CT, L1, L2, gate, merge_M, k);

    % 已被系统航迹占用的局部航迹受保护，不允许被别的系统航迹抢走。
    prot1 = protected_ids(CT, 1);
    prot2 = protected_ids(CT, 2);

    % ---- 5. 候选搜索 + 当前帧贪心分配 ----
    cand = struct('ct', {}, 'radar', {}, 'idx', {}, 'id', {}, 'd2', {});
    for i = 1:numel(CT)
        for r = 1:2
            if r == 1
                if CT(i).src1_id > 0, continue; end
                pool = L1; prot = prot1; other = find_local(L2, CT(i).src2_id);
            else
                if CT(i).src2_id > 0, continue; end
                pool = L2; prot = prot2; other = find_local(L1, CT(i).src1_id);
            end
            % 成熟度约束：已持有的源必须仍然成熟，新源也要 mature 才起算证据
            if isempty(other) || other.life < min_life, continue; end
            [jid, d2] = best_candidate(CT(i), pool, prot, gate, min_life);
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

    % ---- 6. M/N 确认 ----
    for i = 1:numel(CT)
        if CT(i).src1_id == 0
            [CT(i), hit] = accumulate(CT(i), 1, pick1(i), M, N, k);
            if hit
                CT(i).src1_id = CT(i).pend1_id;
                CT(i).src1_miss = 0;
                loc = find_local(L1, CT(i).src1_id);
                if ~isempty(loc), CT(i).src1_truth = loc.truth_idx; end
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
                CT(i).src2_miss = 0;
                loc = find_local(L2, CT(i).src2_id);
                if ~isempty(loc), CT(i).src2_truth = loc.truth_idx; end
                CT(i).pend2_id = 0;  CT(i).hist2 = [];
                if CT(i).src1_id > 0 && isnan(CT(i).confirm_k)
                    CT(i).confirm_k = k;
                end
            end
        end
    end

    % ---- 7. 状态更新：双源融合 / 单源透传 / 无源外推 ----
    for i = 1:numel(CT)
        t1 = find_local(L1, CT(i).src1_id);
        t2 = find_local(L2, CT(i).src2_id);

        if ~isempty(t1) && ~isempty(t2)
            % NIS 协方差校准（CCF）：把两站各自报告的 P 乘以在线估计的尺度因子
            % κ = mean(NIS)/m（SCC 的权重只看 P 的相对关系，因此必须分站标定，
            % 否则权重错配会把更差的站拉进来）。
            [P1f, P2f] = apply_ccf(t1, t2, params);
            [x, P] = track_fusion_algorithms(method, t1.x, P1f, t2.x, P2f);
            CT(i).x = x;  CT(i).P = P;  CT(i).n_src = 2;
            if isnan(CT(i).confirm_k), CT(i).confirm_k = k; end
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
            CT(i).cov_frames(end+1) = k; %#ok<AGROW>
            CT(i).hist{end+1} = struct('frame', k, 'x', CT(i).x, 'P', CT(i).P, ...
                'n_src', CT(i).n_src, 'src1_id', CT(i).src1_id, ...
                'src2_id', CT(i).src2_id); %#ok<AGROW>
        end
    end

    % ---- 8. 配对事件记录（供关联正确率评估）----
    for i = 1:numel(CT)
        if CT(i).n_src == 2
            t1 = find_local(L1, CT(i).src1_id);
            t2 = find_local(L2, CT(i).src2_id);
            if ~isempty(t1) && ~isempty(t2)
                CT(i).pair_log{end+1} = struct('frame', k, 'ct_id', CT(i).id, ...
                    'src1_id', t1.id, 'src2_id', t2.id, ...
                    'truth1', t1.truth_idx, 'truth2', t2.truth_idx, ...
                    'ok', isfinite(t1.truth_idx) && isfinite(t2.truth_idx) && ...
                           t1.truth_idx == t2.truth_idx); %#ok<AGROW>
            end
        end
    end

    % ---- 9. 删除长期无源的系统航迹 ----
    keep = true(1, numel(CT));
    for i = 1:numel(CT)
        if k - CT(i).last_src_k > K_loss, keep(i) = false; end
    end
    CT = CT(keep);

    % ---- 10. 未被占用的局部航迹新建系统航迹 ----
    % 落在某条已有系统航迹门限内、正在累积候选的航迹不新建，
    % 否则同一目标会先被建成两条系统航迹。
    CT = CT(:)';
    n_new = 0;
    for r = 1:2
        if r == 1, pool = L1; else, pool = L2; end
        for j = 1:numel(pool)
            lid = pool(j).id;
            if is_source(CT, r, lid), continue; end
            if is_pending(CT, r, lid), continue; end
            % 若该局部航迹落在某条"缺这一站源"的系统航迹门限内，
            % 则直接作为它的候选累积证据（而不是另建一条系统航迹），
            % 这样证据从它出现的第一帧就开始累积，不浪费一帧。
            host = host_ct(CT, pool(j), r, gate);
            if host > 0
                [CT(host), ~] = accumulate(CT(host), r, lid, M, N, k);
                continue;
            end
            CT(end+1) = new_ct(pool(j), r, k); %#ok<AGROW>
            n_new = n_new + 1;
        end
    end

    % ---- 诊断 ----
    log = struct('frame', k, 'n_ct', numel(CT), ...
        'n_two', sum([CT.n_src] == 2), 'n_confirmed', sum(~isnan([CT.confirm_k])), ...
        'n_new', n_new, 'n_merge', n_merge, ...
        'n_local1', numel(L1), 'n_local2', numel(L2));
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
    L = struct('id', {}, 'x', {}, 'P', {}, 'truth_idx', {}, 'life', {});
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
        life = 0;
        if isfield(trk, 'life') && isfinite(trk.life), life = trk.life; end
        L(end+1) = struct('id', trk.id, 'x', x, 'P', trk.ukf.P, ...
            'truth_idx', truth_of(trk), 'life', life); %#ok<AGROW>
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
function [jid, d2best] = best_candidate(ct, pool, prot, gate, min_life)
    jid = 0; d2best = inf;
    for j = 1:numel(pool)
        if ismember(pool(j).id, prot), continue; end
        if pool(j).life < min_life, continue; end
        d2 = maha2(ct.x, ct.P, pool(j).x, pool(j).P);
        if d2 < gate && d2 < d2best
            d2best = d2;  jid = j;
        end
    end
end

function [P1, P2] = apply_ccf(t1, t2, params)
%APPLY_CCF 按在线估计的 NIS 尺度因子放大两站的协方差
    P1 = t1.P;  P2 = t2.P;
    if ~getp(params, 'online_ccf_enable', false)
        return;
    end
    k1 = getp(params, 'online_ccf_kappa1', 1.0);
    k2 = getp(params, 'online_ccf_kappa2', 1.0);
    if isfinite(k1) && k1 > 0, P1 = k1 * P1; end
    if isfinite(k2) && k2 > 0, P2 = k2 * P2; end
end

function idx = host_ct(CT, loc, radar, gate)
%HOST_CT 找出可以接纳该局部航迹的系统航迹编号
%   要求：该系统航迹缺这一站的源，且位置落在门限内；取马氏距离最小者。
    idx = 0;
    best = inf;
    for i = 1:numel(CT)
        if radar == 1
            if CT(i).src1_id > 0, continue; end
        else
            if CT(i).src2_id > 0, continue; end
        end
        if isempty(CT(i).x), continue; end
        d2 = maha2(CT(i).x, CT(i).P, loc.x, loc.P);
        if d2 < gate && d2 < best
            best = d2;  idx = i;
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
    % 空系统航迹集合：0x0 结构体数组，才能支持 CT(end+1) = new_ct(...) 的追加
    CT = repmat(blank_ct(), 0, 0);
end

function CT = blank_ct()
    % 单条系统航迹的字段模板（1x1 结构体）
    % 用逐字段赋值而不是 struct(...)，避免 struct() 在遇到空 cell / 空数组时
    % 把整个结构体数组变成 0x0。
    CT = struct();
    CT.id = 0;             CT.truth_idx = NaN;
    CT.src1_id = 0;        CT.src2_id = 0;
    CT.src1_miss = 0;      CT.src2_miss = 0;
    CT.src1_truth = NaN;   CT.src2_truth = NaN;
    CT.pend1_id = 0;       CT.hist1 = false(1, 0);
    CT.pend2_id = 0;       CT.hist2 = false(1, 0);
    CT.x = [];             CT.P = [];
    CT.n_src = 0;          CT.birth_k = NaN;
    CT.confirm_k = NaN;    CT.first_cand_k = NaN;
    CT.last_src_k = NaN;   CT.n_covered = 0;
    CT.merge_cnt = 0;      CT.merge_partner = 0;
    CT.merge_alias = [];   CT.cov_frames = [];
    CT.hist = {};          CT.pair_log = {};
end

function e = hist_entry(x, P, frame, n_src)
%HIST_ENTRY 系统航迹单帧历史记录
    e = struct('frame', frame, 'x', x, 'P', P, 'n_src', n_src, ...
        'src1_id', 0, 'src2_id', 0);
end

function ct = new_ct(loc, radar, k)
    % 系统航迹编号沿用首个源航迹的编号，加雷达前缀避免两站编号冲突。
    % 编号一经建立不再改变，与"配对确认"无关。
    % 逐字段赋值（不用 struct(...)）：保证字段顺序与 empty_ct 一致，
    % 否则 struct 数组赋值会因字段顺序/空值而失败。
    ct = blank_ct();
    ct.id = radar * 100000 + loc.id;
    ct.truth_idx = loc.truth_idx;
    ct.x = loc.x;  ct.P = loc.P;
    ct.n_src = 1;
    ct.birth_k = k;  ct.last_src_k = k;
    ct.hist1 = false(1, 0);  ct.hist2 = false(1, 0);
    ct.cov_frames = k;
    ct.hist = {hist_entry(loc.x, loc.P, k, 1)};
    if radar == 1
        ct.src1_id = loc.id;  ct.src1_truth = loc.truth_idx;
    else
        ct.src2_id = loc.id;  ct.src2_truth = loc.truth_idx;
    end
end

% =====================================================================
% 重复系统航迹合并
% =====================================================================
function [CT, n_merge] = merge_duplicate_ct(CT, L1, L2, gate, merge_M, k)
    % 只合并"互补单源"对：一条只持 R1 源、另一条只持 R2 源。
    % 两条各自持有的源必须本帧都存在（否则本帧拿不到观测）。
    n_merge = 0;
    if numel(CT) < 2, return; end

    changed = true;
    while changed
        changed = false;
        for i = 1:numel(CT)
            for j = i + 1:numel(CT)
                [keep, drop, ok] = complementary_pair(CT, i, j);
                if ~ok, continue; end

                % 互补对的源：本帧必须都还在
                if isempty(find_local(L1, CT(keep).src1_id)) || ...
                        isempty(find_local(L2, CT(drop).src2_id))
                    CT(keep).merge_cnt = 0;
                    continue;
                end

                d2 = maha2(CT(keep).x, CT(keep).P, CT(drop).x, CT(drop).P);
                if d2 >= gate
                    CT(keep).merge_cnt = 0;
                    CT(keep).merge_partner = 0;
                    continue;
                end

                % 必须是同一对搭档才继续累计
                if CT(keep).merge_partner ~= CT(drop).id
                    CT(keep).merge_cnt = 0;
                    CT(keep).merge_partner = CT(drop).id;
                end
                CT(keep).merge_cnt = CT(keep).merge_cnt + 1;
                if CT(keep).merge_cnt < merge_M, continue; end

                CT(keep) = absorb_ct(CT(keep), CT(drop), k);
                CT(drop) = [];
                n_merge = n_merge + 1;
                changed = true;
                break;
            end
            if changed, break; end
        end
    end
end

function [keep, drop, ok] = complementary_pair(CT, i, j)
    % 判定 CT(i)、CT(j) 是否构成"一条只持 R1 源 + 一条只持 R2 源"
    ok = false;  keep = i;  drop = j;
    a = CT(i);  b = CT(j);
    a_r1 = a.src1_id > 0 && a.src2_id == 0;
    a_r2 = a.src2_id > 0 && a.src1_id == 0;
    b_r1 = b.src1_id > 0 && b.src2_id == 0;
    b_r2 = b.src2_id > 0 && b.src1_id == 0;
    if a_r1 && b_r2
        pair = [i, j];
    elseif a_r2 && b_r1
        pair = [j, i];
    else
        return;
    end
    ok = true;
    % 保留出生早的，编号与显控上的航迹号沿用先建立的那条
    if CT(pair(2)).birth_k < CT(pair(1)).birth_k
        pair = fliplr(pair);
    end
    keep = pair(1);  drop = pair(2);
end

function a = absorb_ct(a, b, k)
    % 把 b 的源并入 a。状态不在这里融合，交给后面的"状态更新"用本帧两个
    % 局部航迹重新融合，避免重复使用同一次观测。
    if a.src1_id == 0
        a.src1_id = b.src1_id;  a.src1_truth = b.src1_truth;
    end
    if a.src2_id == 0
        a.src2_id = b.src2_id;  a.src2_truth = b.src2_truth;
    end
    a.pend1_id = 0;  a.hist1 = false(1, 0);
    a.pend2_id = 0;  a.hist2 = false(1, 0);
    a.merge_cnt = 0;  a.merge_partner = 0;
    a.n_src = (a.src1_id > 0) + (a.src2_id > 0);
    if a.n_src == 2 && isnan(a.confirm_k), a.confirm_k = k; end
    cs = [a.first_cand_k, b.first_cand_k];
    cs = cs(~isnan(cs));
    if isempty(cs), a.first_cand_k = NaN; else, a.first_cand_k = min(cs); end
    a.last_src_k = max(a.last_src_k, b.last_src_k);
    % 历史拼接：合并后系统航迹的长度是两条并集，而不是只算合并之后
    a.hist = merge_hist(a.hist, b.hist);
    a.cov_frames = unique([a.cov_frames(:); b.cov_frames(:)])';
    a.n_covered = numel(a.cov_frames);
    a.pair_log = [a.pair_log, b.pair_log];
    % 记录被合并掉的编号，评估时才能把两条编号的历史并成一条系统航迹
    a.merge_alias = [a.merge_alias, b.id];
end

function h = merge_hist(ha, hb)
    % 合并两段历史（按帧号排序，同帧保留先到的）
    if isempty(hb), h = ha; return; end
    if isempty(ha), h = hb; return; end
    f1 = cellfun(@(s) s.frame, ha);
    f2 = cellfun(@(s) s.frame, hb);
    h = [ha(:); hb(:)];
    [~, ord] = sort(cellfun(@(s) s.frame, h));
    h = h(ord);
    keep = true(1, numel(h));
    seen = [];
    for i = 1:numel(h)
        fr = h{i}.frame;
        if any(seen == fr)
            keep(i) = false;
        else
            seen(end+1) = fr;
        end
    end
    h = h(keep);
end

function ids = protected_ids(CT, r)
    % 保护所有"已确认占用"的源，重复航迹改由 merge_duplicate_ct 显式合并。
    ids = [];
    for i = 1:numel(CT)
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
