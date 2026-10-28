function [state, frameLog] = central_track_manager(action, varargin)
%CENTRAL_TRACK_MANAGER 融合中心的系统航迹管理（逐周期在线，策略驱动）
%
%   state = central_track_manager('init', params)
%   [state, log] = central_track_manager('step', state, snap1, snap2, k, params)
%
%   对应文档：
%     双站航迹配对与融合在线处理模型_20261028.md 第 3、8、9 节（基准模型）
%     docs/主链路在线化重构契约_20261028.md 第 5 节（本实现的工程契约）
%
%   核心模型：
%     系统航迹在第一条局部航迹上报时就建立，此时只有一个源。
%     配对不是"新建一条融合航迹"，而是"给已有的系统航迹增加一个源"。
%     因此系统航迹的覆盖范围与配对确认时刻无关。
%
%   职责边界（重构后）：
%     跨站证据与候选生成  → tracker/+online_assoc/（可插拔策略包）
%     单帧融合算法        → fusion/+online_fuse/（可插拔算法包）
%     本文件只负责：系统航迹生命周期、1v1 指派、M/N 确认、
%                    重复航迹合并与历史拼接、多算法融合调度、诊断输出。
%
%   单帧处理顺序（全部只用到当前帧及以前的信息）：
%     1. 把全部系统航迹预测到当前时刻
%     2. 取出两站当前帧的局部航迹（含 life，用于成熟度判断）
%     3. 释放失联过久的源绑定（连续 online_K_loss 帧无观测）
%     4. 关联策略产出候选对（距离/马氏/速度证据 + 方法自身门限）
%     5. 当前帧内的贪心 1v1 指派（同一局部航迹只归属一条系统航迹）
%     6. M/N 确认：最近 N 帧中累计 M 帧确认过才真正并入第二个源
%     7. 并入时若两个源分属两条系统航迹 → 合并（保留早建编号 + 拼接历史）
%     8. 双源融合 / 单源透传 / 无源外推（多算法并行）
%     9. 连续 online_K_loss 帧无源则删除
%    10. 未被任何系统航迹占用的局部航迹，新建系统航迹
%
%   输入：
%     snap1 — R1 当前帧快照（未对齐）
%     snap2 — R2 当前帧快照（已时间对齐到 R1 网格）
%     k     — 当前帧号
%     params— 含 online_gate_chi2 / online_M / online_N / online_K_loss
%             / online_min_life / online_merge_confirm
%             / online_assoc_method（配对策略名）
%             / online_fusion_methods（融合算法名数组）
%
%   输出：
%     state     — 融合中心状态（CT 系统航迹数组 + 策略状态 + 融合状态）
%     frameLog  — 本帧诊断

    switch lower(action)
        case 'init'
            state = init_state(varargin{1});
            frameLog = [];
        case 'step'
            [state, frameLog] = step_impl(varargin{:});
        otherwise
            error('central_track_manager:badAction', '未知动作：%s', action);
    end
end

% =====================================================================
% 初始化
% =====================================================================
function state = init_state(params)
    state = struct();
    state.CT = empty_ct();
    state.method_names = fusion_methods(params);
    % 关联策略状态（策略包负责维护其自身的跨帧累计证据）
    state.S = online_assoc.init_strategy(assoc_method(params), params);
    % 每个融合算法一份状态（BC 需要跨帧互协方差 P12）
    state.F = cell(1, numel(state.method_names));
    for m = 1:numel(state.method_names)
        state.F{m} = online_fuse.init(state.method_names{m}, params);
    end
end

function name = assoc_method(params)
    name = 'dualgate';
    if isfield(params, 'online_assoc_method') && ~isempty(params.online_assoc_method)
        name = lower(char(params.online_assoc_method));
    end
end

function names = fusion_methods(params)
    names = {'SCC', 'BC', 'CI', 'FCI'};
    if isfield(params, 'online_fusion_methods') && ~isempty(params.online_fusion_methods)
        names = upper(cellstr(params.online_fusion_methods));
    end
end

% =====================================================================
% 单帧处理
% =====================================================================
function [state, log] = step_impl(state, snap1, snap2, k, params)
    CT = state.CT;
    dt = params.dt_sec;
    gate = getp(params, 'online_gate_chi2', 5.99);
    M = getp(params, 'online_M', 6);
    N = getp(params, 'online_N', M + 2);
    K_loss = getp(params, 'online_K_loss', 8);
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

    % ---- 4. 关联策略产出候选对 ----
    % 策略包只负责"证据 + 方法自身门限"，不做 1v1 指派与 M/N 确认。
    [cands, state.S] = online_assoc.update_candidates(state.S, L1, L2, k, params);

    % ---- 5. 贪心 1v1 指派（按 score 降序）----
    % 候选对是"两站局部航迹对"(r1_id, r2_id)。把它落到系统航迹上时：
    %   持有 r1 的系统航迹缺 R2 源 → 把 r2 作为它的第二源候选；
    %   持有 r2 的系统航迹缺 R1 源 → 把 r1 作为它的第一源候选。
    % 注意：r1 本身已属于某条系统航迹是正常情况（它就是那条航迹的第一源），
    % 不能因此丢弃候选——否则任何配对都永远不会发生。
    assign1 = zeros(numel(CT), 1);   % CT -> 待并入的 R1 航迹 ID
    assign2 = zeros(numel(CT), 1);   % CT -> 待并入的 R2 航迹 ID
    n_assigned = 0;
    if ~isempty(cands)
        [~, ord] = sort([cands.score], 'descend');
        used_r1 = [];
        used_r2 = [];
        for ci = ord
            c = cands(ci);
            i1 = find_local_idx(L1, c.r1_id);
            i2 = find_local_idx(L2, c.r2_id);
            if isempty(i1) || isempty(i2), continue; end
            % 1v1：同一条局部航迹只能参与一对
            if ismember(c.r1_id, used_r1) || ismember(c.r2_id, used_r2), continue; end
            used_r1(end+1) = c.r1_id;  %#ok<AGROW>
            used_r2(end+1) = c.r2_id;  %#ok<AGROW>
            n_assigned = n_assigned + 1;

            ct1 = ct_holding(CT, 1, c.r1_id);   % 持有 R1 源的系统航迹
            ct2 = ct_holding(CT, 2, c.r2_id);   % 持有 R2 源的系统航迹
            % 只有"对方源位空缺"且"该源没被另一条航迹同时占用"才分配，
            % 否则会出现两条系统航迹共享同一个局部航迹（显控上一个目标两条航迹）。
            if ct1 > 0 && CT(ct1).src2_id == 0 && ~source_taken_by_other(CT, ct1, 2, c.r2_id)
                assign2(ct1) = c.r2_id;
            end
            if ct2 > 0 && CT(ct2).src1_id == 0 && ~source_taken_by_other(CT, ct2, 1, c.r1_id)
                assign1(ct2) = c.r1_id;
            end
        end
    end

    % ---- 6. M/N 确认 ----
    for i = 1:numel(CT)
        if CT(i).src1_id == 0
            [CT(i), hit] = accumulate(CT(i), 1, assign1(i), M, N, k);
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
            [CT(i), hit] = accumulate(CT(i), 2, assign2(i), M, N, k);
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

    % ---- 7. 合并重复的系统航迹 ----
    % 覆盖互补布设下，同一目标会先被一站建成一条单源系统航迹，进入重叠区后
    % 又被另一站建成第二条。两条各持一站的源、位置落在同一门限内，应合并为
    % 一条双源系统航迹；不合并的话，同一局部航迹会被两条系统航迹共享，
    % 显控上就出现"一个目标两条系统航迹"。
    % 合并保留出生早的那条编号，并把被合并者的历史拼接到保留者身上，
    % 因此合并后航迹长度是二者并集，而不是只算合并之后的部分。
    merge_M = getp(params, 'online_merge_confirm', 3);
    [CT, n_merge] = merge_duplicate_ct(CT, L1, L2, gate, merge_M, k);

    % ---- 8. 状态更新：双源融合 / 单源透传 / 无源外推（多算法）----
    fused = cell(1, numel(state.method_names));
    for m = 1:numel(state.method_names)
        fused{m} = struct('trackList', {{}});
    end
    for i = 1:numel(CT)
        t1 = find_local(L1, CT(i).src1_id);
        t2 = find_local(L2, CT(i).src2_id);
        for m = 1:numel(state.method_names)
            [x, P, state.F{m}] = fuse_one(state.F{m}, state.method_names{m}, t1, t2, k, params);
            CT(i).x_m{m} = x;
            CT(i).P_m{m} = P;
        end
        % 系统航迹的主状态用第一（默认）算法，其余算法仅记录
        CT(i).x = CT(i).x_m{1};
        CT(i).P = CT(i).P_m{1};

        if ~isempty(t1) && ~isempty(t2)
            CT(i).n_src = 2;
            if isnan(CT(i).confirm_k), CT(i).confirm_k = k; end
        elseif ~isempty(t1) || ~isempty(t2)
            CT(i).n_src = 1;
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

        % 多算法融合快照
        for m = 1:numel(state.method_names)
            if CT(i).n_src == 0, continue; end
            trk = struct('id', CT(i).id, 'group_id', CT(i).id, ...
                'lat', CT(i).x_m{m}(3), 'lon', CT(i).x_m{m}(1), ...
                'ukf', struct('x', CT(i).x_m{m}, 'P', CT(i).P_m{m}), ...
                'ukf_x', CT(i).x_m{m}, 'ukf_P', CT(i).P_m{m}, ...
                'source', source_tag(CT(i).n_src), ...
                'r1_id', CT(i).src1_id, 'r2_id', CT(i).src2_id, ...
                'truth_idx', CT(i).truth_idx);
            fused{m}.trackList{end+1} = trk;
        end
    end

    % ---- 9. 配对事件记录（供关联正确率评估）----
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

    % ---- 10. 删除长期无源的系统航迹 ----
    keep = true(1, numel(CT));
    for i = 1:numel(CT)
        if k - CT(i).last_src_k > K_loss, keep(i) = false; end
    end
    CT = CT(keep);

    % ---- 11. 未被占用的局部航迹新建系统航迹 ----
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
            if in_any_gate(CT, pool(j), gate), continue; end
            CT(end+1) = new_ct(pool(j), r, k); %#ok<AGROW>
            n_new = n_new + 1;
        end
    end

    state.CT = CT;
    state.fused = fused;

    % ---- 诊断 ----
    log = struct('frame', k, 'n_ct', numel(CT), ...
        'n_two', sum([CT.n_src] == 2), 'n_confirmed', sum(~isnan([CT.confirm_k])), ...
        'n_new', n_new, 'n_merge', n_merge, 'n_cand', numel(cands), ...
        'n_assigned', n_assigned, ...
        'n_local1', numel(L1), 'n_local2', numel(L2));
end

% =====================================================================
% 单帧融合（委托给融合包；单源时原样透传）
% =====================================================================
function [x, P, F] = fuse_one(F, method, t1, t2, k, params)
    if ~isempty(t1) && ~isempty(t2)
        [x, P, F] = online_fuse.fuse_step(F, t1, t2, k, params);
    elseif ~isempty(t1)
        x = t1.x;  P = t1.P;
    elseif ~isempty(t2)
        x = t2.x;  P = t2.P;
    else
        x = [];  P = [];
    end
end

function s = source_tag(n_src)
    if n_src == 2
        s = 'both';
    elseif n_src == 1
        s = 'single';
    else
        s = 'none';
    end
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

function idx = find_local_idx(L, id)
    idx = [];
    for i = 1:numel(L)
        if L(i).id == id, idx = i; return; end
    end
end

% =====================================================================
% 候选与门限
% =====================================================================
function tf = source_taken_by_other(CT, self_idx, radar, id)
%SOURCE_TAKEN_BY_OTHER 该源是否已被另一条系统航迹占用
    tf = false;
    for i = 1:numel(CT)
        if i == self_idx, continue; end
        if radar == 1
            if CT(i).src1_id == id, tf = true; return; end
        else
            if CT(i).src2_id == id, tf = true; return; end
        end
    end
end

function idx = ct_holding(CT, radar, id)
%CT_HOLDING 找出持有指定源的系统航迹编号
    idx = 0;
    if id <= 0, return; end
    for i = 1:numel(CT)
        if radar == 1
            if CT(i).src1_id == id, idx = i; return; end
        else
            if CT(i).src2_id == id, idx = i; return; end
        end
    end
end

function tf = in_any_gate(CT, loc, gate)
    tf = false;
    for i = 1:numel(CT)
        if isempty(CT(i).x), continue; end
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
    % 用逐字段赋值而不是 struct(...)，避免 struct() 在遇到空 cell / 空数组时
    % 把整个结构体数组变成 0x0，随后 CT(end+1)=... 会报"结构体为空"错误。
    CT = struct();
    CT.id = 0;             CT.truth_idx = NaN;
    CT.src1_id = 0;        CT.src2_id = 0;
    CT.src1_miss = 0;      CT.src2_miss = 0;
    CT.src1_truth = NaN;   CT.src2_truth = NaN;
    CT.pend1_id = 0;       CT.hist1 = false(1, 0);
    CT.pend2_id = 0;       CT.hist2 = false(1, 0);
    CT.x = [];             CT.P = [];
    CT.x_m = {[]};         CT.P_m = {[]};
    CT.n_src = 0;          CT.birth_k = NaN;
    CT.confirm_k = NaN;    CT.first_cand_k = NaN;
    CT.last_src_k = NaN;   CT.n_covered = 0;
    CT.merge_cnt = 0;      CT.merge_partner = 0;
    CT.merge_alias = [];   CT.cov_frames = [];
    CT.hist = {};          CT.pair_log = {};
    % 返回 0x0 结构体数组：空集合才能支持 CT(end+1) = new_ct(...) 的追加
    CT = repmat(CT, 0, 0);
end

function CT = blank_ct()
    % 单条系统航迹的字段模板（1x1 结构体）
    CT = struct();
    CT.id = 0;             CT.truth_idx = NaN;
    CT.src1_id = 0;        CT.src2_id = 0;
    CT.src1_miss = 0;      CT.src2_miss = 0;
    CT.src1_truth = NaN;   CT.src2_truth = NaN;
    CT.pend1_id = 0;       CT.hist1 = false(1, 0);
    CT.pend2_id = 0;       CT.hist2 = false(1, 0);
    CT.x = [];             CT.P = [];
    CT.x_m = {[]};         CT.P_m = {[]};
    CT.n_src = 0;          CT.birth_k = NaN;
    CT.confirm_k = NaN;    CT.first_cand_k = NaN;
    CT.last_src_k = NaN;   CT.n_covered = 0;
    CT.merge_cnt = 0;      CT.merge_partner = 0;
    CT.merge_alias = [];   CT.cov_frames = [];
    CT.hist = {};          CT.pair_log = {};
end

function ct = new_ct(loc, radar, k)
    % 系统航迹编号沿用首个源航迹的编号，加雷达前缀避免两站编号冲突。
    % 编号一经建立不再改变，与"配对确认"无关。
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

function e = hist_entry(x, P, frame, n_src)
%HIST_ENTRY 系统航迹单帧历史记录
    e = struct('frame', frame, 'x', x, 'P', P, 'n_src', n_src, ...
        'src1_id', 0, 'src2_id', 0);
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

                % 互补对的源：本帧必须都还在。
                % 注意 keep 是"出生早的那条"，它可能是只持 R1 的，也可能是只持 R2 的。
                % 覆盖互补布设下目标多从南侧进入走廊，R2 往往先建航迹，于是 keep 常是 R2 侧。
                % 原写法固定按"keep 持 R1、drop 持 R2"去查，keep 一旦是 R2 侧，
                % find_local(L1, 0) 恒为空 → merge_cnt 永久清零 → 重复航迹永不合并。
                % 2026-10-28 复核由 #34 CSZ9278 暴露：单站 47/55 帧、并集 79 帧，
                % 系统航迹只拿到 53 帧（长度比 0.964）；当时马氏 d² 有 20 帧远低于门限。
                if CT(keep).src1_id > 0
                    keep_ok = ~isempty(find_local(L1, CT(keep).src1_id)) && ...
                              ~isempty(find_local(L2, CT(drop).src2_id));
                else
                    keep_ok = ~isempty(find_local(L2, CT(keep).src2_id)) && ...
                              ~isempty(find_local(L1, CT(drop).src1_id));
                end
                if ~keep_ok
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
