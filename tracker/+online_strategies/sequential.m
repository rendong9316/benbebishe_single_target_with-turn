function [cands, S] = strategy_sequential(S, local1, local2, k, params)
%STRATEGY_SEQUENTIAL 序贯最近邻基线（马氏通过率）
%
%   在 NN 基础上引入马氏距离校验：共现帧中 D² < 门限 的占比达标，
%   再按马氏距离均值升序指派（指派由融合中心统一做）。
%   缺少连续性约束与速度判据，是介于 NN 与三判据之间的中间基线。

    T1 = online_assoc.getp(params, 'dualgate_T1_km', 35);
    maha_thresh = online_assoc.getp(params, 'dualgate_maha_thresh', 5.99);
    maha_ratio = online_assoc.getp(params, 'dualgate_maha_ratio', 0.8);

    % 先累计每对的共现帧证据
    S = online_assoc.accumulate_evidence(S, local1, local2, k, params);

    cands = online_assoc.empty_cands();
    for i = 1:numel(S.acc)
        a = S.acc(i);
        if a.n_coexist < 1, continue; end
        ev = a.last_ev;
        if ~isfinite(ev.d2), continue; end
        if ev.dist_km >= T1, continue; end
        if a.n_maha / a.n_coexist < maha_ratio, continue; end
        sc = 1 / (1 + ev.dist_km);
        cands(end+1) = online_assoc.make_cand(a.r1, a.r2, sc, ev, a.n_coexist); %#ok<AGROW>
    end
end
