function matched_pairs = track_matcher_mewf(trackSnapshots_R1, trackSnapshots_R2, params)
% track_matcher_mewf — 多证据加权融合配对（Multi-Evidence Weighted Fusion）
%
% 【动机：为什么不是"把两个算法叠加"】
%   实测（密集 5 组，adsb_0608/0911）发现：
%     - 三判据（dualgate）用「连续 M 帧滑窗 + 速度/马氏窗口通过率」做**否决**，
%       M 存在结构性跷跷板：M=4 误配 10.4/漏配 25.4，M=10 误配 7.6/漏配 56.4，
%       任何 M 都无法同时压住两端。
%     - Sequential 用「全共现区间」的马氏通过率做**筛选**，短共现目标不丢，
%       但完全没有连续性约束、没有速度判据 → 对「偶然穿越」和「运动状态不同」
%       的误配零防护。
%
%   关键认识：两者不是"各有长处"，而是**都把有用信息用错了方式**——
%   三判据把连续性/速度当"一票否决"（太硬），Sequential 干脆不要（太松）。
%
% 【本算法：把"门槛"降级为"特征"】
%   在全部共现帧上计算四项归一化证据，加权融合成一个 0~1 的综合得分，
%   只用得分阈值判决，不再有"连续 M 帧"这类硬约束：
%
%     ① 距离证据 E_d = mean_k exp(-d_k^2 / (2*sigma_d^2))
%        距离越近证据越强，权重连续衰减（而非 T1 的 0/1 硬截断）
%     ② 马氏证据 E_m = mean_k 1[D2_k < chi2_thr]
%        用滤波器协方差归一化的位置一致性，自适应各站精度
%     ③ 速度证据 E_v = mean_k 1[|dv_k| < kappa * tol_k]
%        运动状态一致性 —— Sequential 完全没有这一项
%     ④ 连续证据 E_c = 最长距离合格段长度 / 共现帧数
%        把三判据的"必须连续 M 帧"软化为"连续段占多大比例" —— 只加分不否决
%
%   综合得分 S = (w_d*E_d + w_m*E_m + w_v*E_v + w_c*E_c) / sum(w)
%   判决 S >= S_thr；指派按 S 降序贪心 1v1。
%
% 【为什么这样能同时改善两端】
%   短共现目标：帧数少但每一帧都参与累积，不像 M 帧硬门限那样被整体否决 → 不漏
%   偶然穿越：  距离合格段占比 E_c 低 + 速度不一致 E_v 低 → 得分被压下去 → 不误
%   强证据单帧：E_d 连续加权，一帧强证据有实质贡献，不浪费
%
% 【天然可消融】置 w_v=0 即退化为「Sequential + 连续性」；置 w_c=0 即退化为
%   「Sequential + 速度」；四项全开为完整 MEWF。论文 4.4 节可直接给出每个
%   特征的边际贡献。
%
% 【输入/输出签名】与 track_matcher_dualgate / track_matcher_sequential 一致，
%   输出额外携带四项证据分量，便于诊断分析。
%
% 作者：任东  2026-10-26

    n_frames = length(trackSnapshots_R1);

    % ---- 共享门限：与三判据/Sequential 保持完全相同的物理口径 ----
    T1          = pairing.get_param_local(params, 'dualgate_T1_km', 35);
    maha_thresh = pairing.get_param_local(params, 'dualgate_maha_thresh', 5.99);
    vel_kappa   = pairing.get_param_local(params, 'dualgate_vel_kappa', 3.0);
    n_min       = pairing.get_param_local(params, 'dualgate_coexist_thresh', 3);

    % ---- MEWF 专有参数 ----
    sigma_d = pairing.get_param_local(params, 'mewf_sigma_d_km', 15);      % 距离证据尺度
    S_thr   = pairing.get_param_local(params, 'mewf_score_thr', 0.60);      % 综合得分门限
    e_d_min = pairing.get_param_local(params, 'mewf_min_dist_evidence', 0.30); % 距离证据下限
    w_d = pairing.get_param_local(params, 'mewf_w_dist', 1.0);
    w_m = pairing.get_param_local(params, 'mewf_w_maha', 1.0);
    w_v = pairing.get_param_local(params, 'mewf_w_vel',  1.0);
    w_c = pairing.get_param_local(params, 'mewf_w_cont', 1.0);
    assign_mode = pairing.get_param_local(params, 'mewf_assign', 'hungarian');
    w_sum = w_d + w_m + w_v + w_c;
    if w_sum <= 0
        w_d = 1; w_m = 1; w_v = 1; w_c = 1; w_sum = 4;
    end

    % 预建索引（与其它 fast 版 matcher 同一套基础设施）
    I1 = pairing.build_track_index(trackSnapshots_R1, n_frames);
    I2 = pairing.build_track_index(trackSnapshots_R2, n_frames);
    r1_ids = I1.ids;
    r2_ids = I2.ids;

    matched_pairs = struct('R1_track_id', {}, 'R2_track_id', {}, 'score', {}, ...
        'E_dist', {}, 'E_maha', {}, 'E_vel', {}, 'E_cont', {}, ...
        'mean_dist_km', {}, 'coexist_count', {});
    if isempty(r1_ids) || isempty(r2_ids), return; end

    % ================= 候选评估：四项证据累积 =================
    cand = {};
    for i = 1:numel(r1_ids)
        for j = 1:numel(r2_ids)
            [ds, D2, vdiff, vtol, u] = pairing.pair_evidence_fast(...
                I1, I2, r1_ids(i), r2_ids(j));
            nc = sum(u);
            if nc < n_min, continue; end          % 最少共现帧（防单帧巧合，唯一硬约束）

            du  = ds(u);
            d2u = D2(u);
            vu  = vdiff(u);
            tu  = vtol(u);

            % ---- 特征①：距离证据强度（连续加权，非 0/1 截断）----
            E_d = mean(exp(-(du.^2) / (2 * sigma_d^2)));
            if E_d < e_d_min, continue; end       % 平均距离明显过远，直接排除

            % ---- 特征②：马氏通过率（协方差归一化的位置一致性）----
            maha_ok = d2u < maha_thresh;
            E_m = mean(maha_ok);

            % ---- 特征③：速度一致率（Sequential 缺失的判据）----
            okv = isfinite(vu) & isfinite(tu);
            if any(okv)
                E_v = mean(vu(okv) < vel_kappa * tu(okv));
            else
                E_v = 0;
            end

            % ---- 特征④：距离合格段连续性（三判据 M 的软化版）----
            dist_ok = du < T1;
            E_c = max_run_len(dist_ok) / nc;

            % ---- 综合得分 ----
            S = (w_d * E_d + w_m * E_m + w_v * E_v + w_c * E_c) / w_sum;
            if S < S_thr, continue; end

            cand{end+1} = struct('r1', r1_ids(i), 'r2', r2_ids(j), ...
                'i1', i, 'i2', j, ...
                'S', S, 'Ed', E_d, 'Em', E_m, 'Ev', E_v, 'Ec', E_c, ...
                'md', mean(du), 'mh', mean(d2u), 'nc', nc); %#ok<AGROW>
        end
    end
    if isempty(cand), return; end

    % ================= 指派（与判定分层解耦）=================
    % 【为什么判定和指派必须分开】
    %   实测诊断（adsb_0608/301）发现，密集场景的残余误配几乎全是**成对交换**
    %   （swap）：两架同航路平行飞行的目标互相配反。此时距离 / 马氏 / 速度 /
    %   连续性四项特征**同时都很高**——它们在运动学上就是同一个样子，综合得分
    %   抹平了差异（σ_d 大时 4.3km 与 7.7km 得分只差 0.02）。
    %   唯一可用的信息是「谁和谁距离更近」，这属于**指派层**的职责。
    %   因此：
    %     判定层（上面）用四特征融合门限 → 决定「要不要配」
    %     指派层（这里）用距离/马氏最优    → 决定「配给谁」
    %
    % rank_key:
    %   'dist'  — 平均 Haversine 距离升序（默认，直击 swap 问题）
    %   'maha'  — 平均马氏 D² 升序（与 Sequential 排序键一致，便于对照）
    %   'score' — 综合得分降序（保留作消融，验证「混在一个分里」确实更差）
    rank_key = pairing.get_param_local(params, 'mewf_rank_key', 'dist');
    n1 = numel(r1_ids);
    n2 = numel(r2_ids);
    keep_idx = [];

    if strcmpi(assign_mode, 'hungarian') && exist('matchpairs', 'file') == 2
        % matchpairs 最小化总代价，且仅当 cost(i,j) < 2*costUnmatched 才匹配。
        % 各 rank_key 下把「不匹配代价」设为门限的一半，使匹配条件恰为
        % 「代价 < 门限」，与判定层口径一致。
        BIG = 1e6;
        cost = BIG * ones(n1, n2);
        switch lower(rank_key)
            case 'score'
                for ci = 1:numel(cand), cost(cand{ci}.i1, cand{ci}.i2) = 1 - cand{ci}.S; end
                cost_unmatched = (1 - S_thr) / 2;
            case 'maha'
                for ci = 1:numel(cand), cost(cand{ci}.i1, cand{ci}.i2) = cand{ci}.mh; end
                cost_unmatched = maha_thresh / 2;
            otherwise   % 'dist'
                for ci = 1:numel(cand), cost(cand{ci}.i1, cand{ci}.i2) = cand{ci}.md; end
                cost_unmatched = T1 / 2;
        end
        [Mp, ~, ~] = matchpairs(cost, cost_unmatched);
        if ~isempty(Mp)
            key = zeros(n1, n2);
            for ci = 1:numel(cand), key(cand{ci}.i1, cand{ci}.i2) = ci; end
            for r = 1:size(Mp, 1)
                ci = key(Mp(r, 1), Mp(r, 2));
                if ci > 0, keep_idx(end+1) = ci; end %#ok<AGROW>
            end
        end
    else
        % 贪心 1v1：按 rank_key 排序后依次占用
        v = zeros(1, numel(cand));
        switch lower(rank_key)
            case 'score'
                for ci = 1:numel(cand), v(ci) = cand{ci}.S;  end
                [~, ord] = sort(v, 'descend');
            case 'maha'
                for ci = 1:numel(cand), v(ci) = cand{ci}.mh; end
                [~, ord] = sort(v, 'ascend');
            otherwise
                for ci = 1:numel(cand), v(ci) = cand{ci}.md; end
                [~, ord] = sort(v, 'ascend');
        end
        used_r1 = [];
        used_r2 = [];
        for idx = ord
            c = cand{idx};
            if ismember(c.r1, used_r1) || ismember(c.r2, used_r2), continue; end
            used_r1(end+1) = c.r1; %#ok<AGROW>
            used_r2(end+1) = c.r2; %#ok<AGROW>
            keep_idx(end+1) = idx; %#ok<AGROW>
        end
    end

    for idx = keep_idx
        c = cand{idx};
        matched_pairs(end+1) = struct('R1_track_id', c.r1, 'R2_track_id', c.r2, ...
            'score', c.S, 'E_dist', c.Ed, 'E_maha', c.Em, 'E_vel', c.Ev, ...
            'E_cont', c.Ec, 'mean_dist_km', c.md, 'coexist_count', c.nc); %#ok<AGROW>
    end

    fprintf(['[MEWF] T1=%.0f maha=%.2f kappa=%.1f n_min=%d sigma_d=%.0f ' ...
        'S_thr=%.2f w=[%.1f %.1f %.1f %.1f] %s/%s → 候选 %d / 匹配 %d 对\n'], ...
        T1, maha_thresh, vel_kappa, n_min, sigma_d, S_thr, ...
        w_d, w_m, w_v, w_c, assign_mode, rank_key, numel(cand), numel(matched_pairs));
end

% =========================================================================
function L = max_run_len(mask)
% max_run_len — 逻辑序列中最长连续 true 段长度
    L = 0;
    cur = 0;
    for k = 1:numel(mask)
        if mask(k)
            cur = cur + 1;
            if cur > L, L = cur; end
        else
            cur = 0;
        end
    end
end
