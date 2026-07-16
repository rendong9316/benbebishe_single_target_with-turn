function detList = generate_all_radar_detections(params, truth_all_cell, t_grid, radar_cfg, dr_est, da_est, seed_offset)
% generate_all_radar_detections — 按统一 N 目标真值生成单部雷达点迹

    n_frames = length(t_grid);
    n_targets = length(truth_all_cell);
    detList = cell(n_frames, 1);

    rng(params.random_seed + seed_offset);
    for k = 1:n_frames
        t = t_grid(k);
        tgt_states = nan(n_targets, 5);

        for ac = 1:n_targets
            tt_ac = truth_all_cell{ac};
            if t < tt_ac(1,5) || t > tt_ac(end,5)
                continue;
            end
            t_vals = tt_ac(:,5);
            pos = interp1(t_vals, [tt_ac(:,1), tt_ac(:,2)], t, 'linear', 'extrap');
            lon_rate = interp1(t_vals, tt_ac(:,3), t, 'linear', 'extrap');
            lat_rate = interp1(t_vals, tt_ac(:,4), t, 'linear', 'extrap');
            tgt_states(ac,:) = [pos(1), pos(2), lon_rate, lat_rate, ac];
        end

        tgt_states = tgt_states(~isnan(tgt_states(:,1)), :);
        if isempty(tgt_states)
            detList{k} = [];
            continue;
        end

        detRaw = generate_frame_detections_multi(radar_cfg.radar_lon, radar_cfg.radar_lat, ...
            radar_cfg.tx_lon, radar_cfg.tx_lat, tgt_states, k, t, ...
            radar_cfg.range_bias_m, radar_cfg.azimuth_bias_deg, radar_cfg.beam_center_deg, ...
            params, radar_cfg.range_noise_std_m, radar_cfg.azimuth_noise_std_deg);
        detList{k} = augment_dets(detRaw, dr_est, da_est, ...
            radar_cfg.tx_lon, radar_cfg.tx_lat, radar_cfg.radar_lon, radar_cfg.radar_lat);
    end
end
