function [cands, S] = strategy_dualgate(S, local1, local2, k, params)
%STRATEGY_DUALGATE 三判据关联（位置 + 马氏 + 速度一致性）
%
%   逐帧硬门限：三个判据同时成立才成候选。
%   这是"双门限 + 速度一致性"的在线版本，只用当前帧与历史累计。

    T1 = online_assoc.getp(params, 'dualgate_T1_km', 35);
    maha_thresh = online_assoc.getp(params, 'dualgate_maha_thresh', 5.99);
    vel_kappa = online_assoc.getp(params, 'dualgate_vel_kappa', 3.0);
    use_maha = online_assoc.getp(params, 'dualgate_use_maha', true);
    use_vel = online_assoc.getp(params, 'dualgate_use_vel', true);

    cands = online_assoc.empty_cands();
    for i = 1:numel(local1)
        for j = 1:numel(local2)
            ev = online_assoc.evidence(local1(i), local2(j));
            if ~isfinite(ev.d2), continue; end
            ev.ok_dist = ev.dist_km < T1;
            ev.ok_maha = ev.d2 < maha_thresh;
            ev.ok_vel = ev.vdiff < vel_kappa * ev.vtol;
            if ~ev.ok_dist, continue; end
            if use_maha && ~ev.ok_maha, continue; end
            if use_vel && ~ev.ok_vel, continue; end
            % score：距离越近越可信（越大越好）
            sc = 1 / (1 + ev.dist_km);
            cands(end+1) = online_assoc.make_cand(local1(i).id, local2(j).id, sc, ev, 1); %#ok<AGROW>
        end
    end
end
