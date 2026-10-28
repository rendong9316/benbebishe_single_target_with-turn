function ev = empty_ev()
%EMPTY_EV 空的逐帧证据量
    ev = struct('dist_km', NaN, 'd2', NaN, 'vdiff', NaN, 'vtol', NaN, ...
                'ok_maha', false, 'ok_vel', false, 'ok_dist', false);
end
