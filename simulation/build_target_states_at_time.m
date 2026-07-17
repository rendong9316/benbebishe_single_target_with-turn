function tgt_states = build_target_states_at_time(truth_all, time_sec)
    tgt_states = zeros(0, 5);
    for ac = 1:length(truth_all)
        tt = truth_all{ac};
        if isempty(tt) || time_sec < tt(1, 5) || time_sec > tt(end, 5)
            continue;
        end
        lon = interp1(tt(:, 5), tt(:, 1), time_sec, 'linear', 'extrap');
        lat = interp1(tt(:, 5), tt(:, 2), time_sec, 'linear', 'extrap');
        lon_rate = interp1(tt(:, 5), tt(:, 3), time_sec, 'linear', 'extrap');
        lat_rate = interp1(tt(:, 5), tt(:, 4), time_sec, 'linear', 'extrap');
        if any(isnan([lon, lat, lon_rate, lat_rate]))
            continue;
        end
        tgt_states(end+1, :) = [lon, lat, lon_rate, lat_rate, ac];
    end
end
