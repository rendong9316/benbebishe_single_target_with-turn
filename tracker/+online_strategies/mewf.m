function [cands, S] = strategy_mewf(S, local1, local2, k, params)
%STRATEGY_MEWF 多证据加权融合（在线滚动累计版）
%
%   把"门槛"降级为"特征"：在共现帧上累积四项归一化证据，加权成一个 0~1
%   的综合得分，只用得分阈值判决，不再有"连续 M 帧"这类硬约束。
%
%     ① 距离证据 E_d = mean_k exp(-d_k^2 / (2*sigma_d^2))
%     ② 马氏证据 E_m = 累计马氏通过帧占比
%     ③ 速度证据 E_v = 累计速度通过帧占比
%     ④ 连续证据 E_c = 最长距离合格段长度 / 共现帧数
%
%   在线化要点（2026-10-28 更正）：**累计量**单调（n_coexist / sum_ed / n_maha /
%   n_vel / max_run_ok 只增不减），但**综合分不是单调的**——四项都是比值，
%   分母 n_coexist 一直涨，其中 E_c = max_run_ok/n_coexist 的分子封顶、分母发散，
%   必然单调衰减；实测一条完美共现对的分数走 0.999→0.868。
%   因此"确认单调"不能由综合分保证，而是由融合中心的 M/N 序贯确认（hist）保证；
%   本方法只负责"本帧是否给出候选"，分数跌破门限的帧会记为未命中，由 M/N 吸收。
%   这一条已写进 docs/收尾报告复核_20261028.md §5.2，原先"综合分单调上升"的说法作废。

    T1 = online_assoc.getp(params, 'dualgate_T1_km', 35);
    maha_thresh = online_assoc.getp(params, 'dualgate_maha_thresh', 5.99);
    vel_kappa = online_assoc.getp(params, 'dualgate_vel_kappa', 3.0);
    sigma_d = online_assoc.getp(params, 'mewf_sigma_d_km', 15);
    S_thr = online_assoc.getp(params, 'mewf_score_thr', 0.55);
    w_d = online_assoc.getp(params, 'mewf_w_dist', 1.0);
    w_m = online_assoc.getp(params, 'mewf_w_maha', 1.0);
    w_v = online_assoc.getp(params, 'mewf_w_vel', 1.0);
    w_c = online_assoc.getp(params, 'mewf_w_cont', 1.0);
    w_sum = w_d + w_m + w_v + w_c;
    if w_sum <= 0
        w_d = 1; w_m = 1; w_v = 1; w_c = 1; w_sum = 4;
    end

    % 先累计每对的共现帧证据
    S = online_assoc.accumulate_evidence(S, local1, local2, k, params);

    cands = online_assoc.empty_cands();
    for i = 1:numel(S.acc)
        a = S.acc(i);
        if a.n_coexist < 1, continue; end
        ev = a.last_ev;
        if ~isfinite(ev.d2), continue; end
        E_d = a.sum_ed / a.n_coexist;
        E_m = a.n_maha / a.n_coexist;
        E_v = a.n_vel / a.n_coexist;
        E_c = a.max_run_ok / a.n_coexist;   % 注意：分子封顶、分母发散，随时间单调衰减
        score = (w_d*E_d + w_m*E_m + w_v*E_v + w_c*E_c) / w_sum;
        if score < S_thr, continue; end
        cands(end+1) = online_assoc.make_cand(a.r1, a.r2, score, ev, a.n_coexist); %#ok<AGROW>
    end
end
