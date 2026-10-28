function S = accumulate_evidence(S, local1, local2, k, params)
%ACCUMULATE_EVIDENCE 逐帧累计每对 (r1,r2) 的证据量
%
%   维护 S.acc：每个共现过的航迹对一条记录，字段：
%     r1, r2          航迹 ID
%     n_coexist       共现帧计数
%     sum_ed          Σ exp(-d²/2σ_d²)
%     n_maha          马氏通过帧计数
%     n_vel           速度通过帧计数
%     run_ok          当前连续距离合格段长度
%     max_run_ok      历史最长连续合格段
%     last_ev         最近一帧的证据量
%     last_k          最近一帧号
%
%   只用当前帧与历史，不读未来帧。

    T1 = online_assoc.getp(params, 'dualgate_T1_km', 35);
    maha_thresh = online_assoc.getp(params, 'dualgate_maha_thresh', 5.99);
    vel_kappa = online_assoc.getp(params, 'dualgate_vel_kappa', 3.0);
    sigma_d = online_assoc.getp(params, 'mewf_sigma_d_km', 15);

    for i = 1:numel(local1)
        for j = 1:numel(local2)
            e = online_assoc.evidence(local1(i), local2(j));
            if ~isfinite(e.d2), continue; end
            e.ok_dist = e.dist_km < T1;
            e.ok_maha = e.d2 < maha_thresh;
            e.ok_vel = e.vdiff < vel_kappa * e.vtol;

            idx = find_pair(S.acc, local1(i).id, local2(j).id);
            if isempty(idx)
                a = online_assoc.blank_acc();
                a.r1 = local1(i).id;
                a.r2 = local2(j).id;
                a.last_ev = e;
                a.last_k = k;
                S.acc(end+1) = a; %#ok<AGROW>
                idx = numel(S.acc);
            end
            a = S.acc(idx);
            a.n_coexist = a.n_coexist + 1;
            a.sum_ed = a.sum_ed + exp(-(e.dist_km^2) / (2*sigma_d^2));
            if e.ok_maha, a.n_maha = a.n_maha + 1; end
            if e.ok_vel,  a.n_vel  = a.n_vel + 1; end
            if e.ok_dist
                a.run_ok = a.run_ok + 1;
                if a.run_ok > a.max_run_ok, a.max_run_ok = a.run_ok; end
            else
                a.run_ok = 0;
            end
            a.last_ev = e;
            a.last_k = k;
            S.acc(idx) = a;
        end
    end
end

function idx = find_pair(acc, r1, r2)
    idx = [];
    for i = 1:numel(acc)
        if acc(i).r1 == r1 && acc(i).r2 == r2
            idx = i;  return;
        end
    end
end
