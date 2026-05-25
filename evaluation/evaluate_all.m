% =========================================================================
% evaluate_all.m
% Class-based evaluation module containing compute_tracking_errors
% and evaluate_fusion as static methods.
% =========================================================================

classdef evaluate_all
    methods (Static)
        % ===== compute_tracking_errors =====
        function errorStats = compute_tracking_errors(trackSnapshots, detList, truthTrajs, ...
                n_frames, dt_sec, radar_label)
            n_ac = length(truthTrajs);
            frame_times = (0:n_frames-1) * dt_sec;

            ukf_errs  = cell(n_ac, 1);
            det_errs  = cell(n_ac, 1);
            raw_errs  = cell(n_ac, 1);
            ukf_lats  = cell(n_ac, 1);
            det_lats  = cell(n_ac, 1);
            raw_lats  = cell(n_ac, 1);

            for a = 1:n_ac
                tt = truthTrajs{a};
                ukf_errs{a} = [];  det_errs{a} = [];  raw_errs{a} = [];
                ukf_lats{a} = [];  det_lats{a} = [];  raw_lats{a} = [];

                for k = 1:n_frames
                    t_true_lat = interp1(tt.time_sec, tt.lat, frame_times(k), 'linear', 'extrap');
                    t_true_lon = interp1(tt.time_sec, tt.lon, frame_times(k), 'linear', 'extrap');

                    snap = trackSnapshots{k};
                    if ~isempty(snap.trackList)
                        best_ukf_dist = inf;
                        best_ukf_lat = NaN;
                        for t = 1:length(snap.trackList)
                            trk = snap.trackList{t};
                            if trk.type == 7 || isnan(trk.lat), continue; end
                            d = evaluate_all.haversine_km(trk.lon, trk.lat, t_true_lon, t_true_lat);
                            if d < best_ukf_dist && d < 200
                                best_ukf_dist = d;
                                best_ukf_lat = trk.lat;
                            end
                        end
                        if ~isinf(best_ukf_dist)
                            ukf_errs{a}(end+1) = best_ukf_dist;
                            ukf_lats{a}(end+1) = best_ukf_lat;
                        end
                    end

                    dets = detList{k};
                    for d = 1:length(dets)
                        dp = dets(d);
                        if dp.is_clutter, continue; end
                        if ~isfield(dp, 'aircraft_id') || dp.aircraft_id ~= a, continue; end
                        if isfield(dp, 'lat') && ~isnan(dp.lat)
                            d_cal = evaluate_all.haversine_km(dp.lon, dp.lat, t_true_lon, t_true_lat);
                            det_errs{a}(end+1) = d_cal;
                            det_lats{a}(end+1) = dp.lat;
                        end
                        if isfield(dp, 'raw_lat') && ~isnan(dp.raw_lat)
                            d_raw = evaluate_all.haversine_km(dp.raw_lon, dp.raw_lat, t_true_lon, t_true_lat);
                            raw_errs{a}(end+1) = d_raw;
                            raw_lats{a}(end+1) = dp.raw_lat;
                        end
                    end
                end
            end

            summary = struct();
            for a = 1:n_ac
                summary(a).aircraft = a;
                s_ukf = evaluate_all.compute_summary(ukf_errs{a});
                s_det = evaluate_all.compute_summary(det_errs{a});
                s_raw = evaluate_all.compute_summary(raw_errs{a});
                summary(a).ukf = s_ukf;
                summary(a).det_calibrated = s_det;
                summary(a).det_raw = s_raw;
                if s_ukf.n > 0 && s_det.n > 0
                    summary(a).ukf_vs_det_pct = (1 - s_ukf.median / max(s_det.median, 0.01)) * 100;
                else
                    summary(a).ukf_vs_det_pct = 0;
                end
            end

            all_ukf = []; all_det = []; all_raw = [];
            for a = 1:n_ac
                all_ukf = [all_ukf, ukf_errs{a}];
                all_det = [all_det, det_errs{a}];
                all_raw = [all_raw, raw_errs{a}];
            end
            overall.ukf = evaluate_all.compute_summary(all_ukf);
            overall.det = evaluate_all.compute_summary(all_det);
            overall.raw = evaluate_all.compute_summary(all_raw);

            errorStats = struct(...
                'radar', radar_label, ...
                'n_frames', n_frames, ...
                'ukf_errors_km', {ukf_errs}, ...
                'det_errors_km', {det_errs}, ...
                'raw_errors_km', {raw_errs}, ...
                'summary', summary, ...
                'overall', overall);
        end


        % ===== evaluate_fusion =====
        function fusion_eval = evaluate_fusion(all_fused_snapshots, method_names, ...
                matched_pairs, trackSnapshots_R1, trackSnapshots_R2, ...
                truthTrajs, n_frames, dt_sec, matcher)
            n_methods = length(method_names);
            n_ac = length(truthTrajs);
            frame_times = (0:n_frames-1) * dt_sec;

            % Step 1: Map R1-R2 matched pairs to true aircraft
            pair_to_aircraft = zeros(length(matched_pairs), 1);
            for p = 1:length(matched_pairs)
                mp = matched_pairs(p);
                r1_idx = find(matcher.r1_ids == mp.R1_track_id, 1);
                if isempty(r1_idx), continue; end
                r1_lons = squeeze(matcher.r1_pos(r1_idx, :, 1))';
                r1_lats = squeeze(matcher.r1_pos(r1_idx, :, 2))';
                best_ac = 0;
                best_dist = inf;
                for a = 1:n_ac
                    tt = truthTrajs{a};
                    t_lat = interp1(tt.time_sec, tt.lat, frame_times, 'linear', 'extrap');
                    t_lon = interp1(tt.time_sec, tt.lon, frame_times, 'linear', 'extrap');
                    mean_dist = nanmean(evaluate_all.haversine_km_vec(r1_lons, r1_lats, t_lon, t_lat));
                    if mean_dist < best_dist
                        best_dist = mean_dist;
                        best_ac = a;
                    end
                end
                pair_to_aircraft(p) = best_ac;
            end

            fprintf('\n匹配对 -> 真值飞机映射:\n');
            for p = 1:length(matched_pairs)
                fprintf('  Pair %d (R1#%d <-> R2#%d) -> 飞机%s\n', ...
                    p, matched_pairs(p).R1_track_id, matched_pairs(p).R2_track_id, ...
                    truthTrajs{pair_to_aircraft(p)}.label);
            end

            % Step 2: Compute fusion track errors frame by frame
            fusion_errs = cell(n_methods, n_ac);
            for m = 1:n_methods
                fused_snaps = all_fused_snapshots{m};
                for a = 1:n_ac
                    fusion_errs{m, a} = [];
                end
                for k = 1:n_frames
                    t_true_lat_all = zeros(n_ac, 1);
                    t_true_lon_all = zeros(n_ac, 1);
                    for a = 1:n_ac
                        t_true_lat_all(a) = interp1(truthTrajs{a}.time_sec, ...
                            truthTrajs{a}.lat, frame_times(k), 'linear', 'extrap');
                        t_true_lon_all(a) = interp1(truthTrajs{a}.time_sec, ...
                            truthTrajs{a}.lon, frame_times(k), 'linear', 'extrap');
                    end
                    snap = fused_snaps{k};
                    if isempty(snap.trackList), continue; end
                    for t = 1:length(snap.trackList)
                        ftrk = snap.trackList{t};
                        p_idx = ftrk.id;
                        if p_idx < 1 || p_idx > length(pair_to_aircraft), continue; end
                        ac = pair_to_aircraft(p_idx);
                        if ac == 0, continue; end
                        if isnan(ftrk.lon), continue; end
                        d = evaluate_all.haversine_km(ftrk.lon, ftrk.lat, t_true_lon_all(ac), t_true_lat_all(ac));
                        fusion_errs{m, ac}(end+1) = d;
                    end
                end
            end

            % Step 3: Single-station track errors (baseline)
            r1_errs = cell(1, n_ac);
            r2_errs = cell(1, n_ac);
            r1_snaps = trackSnapshots_R1;
            r2_snaps_aligned = matcher.aligned_R2;
            for a = 1:n_ac
                r1_errs{a} = [];
                r2_errs{a} = [];
            end
            for k = 1:n_frames
                for a = 1:n_ac
                    t_true_lat = interp1(truthTrajs{a}.time_sec, truthTrajs{a}.lat, ...
                        frame_times(k), 'linear', 'extrap');
                    t_true_lon = interp1(truthTrajs{a}.time_sec, truthTrajs{a}.lon, ...
                        frame_times(k), 'linear', 'extrap');
                    snap_r1 = r1_snaps{k};
                    if ~isempty(snap_r1.trackList)
                        for p = 1:length(matched_pairs)
                            if pair_to_aircraft(p) == a
                                r1_id = matched_pairs(p).R1_track_id;
                                trk1 = evaluate_all.find_track_by_id(snap_r1, r1_id);
                                if ~isempty(trk1) && ~isnan(trk1.lon)
                                    d = evaluate_all.haversine_km(trk1.lon, trk1.lat, t_true_lon, t_true_lat);
                                    r1_errs{a}(end+1) = d;
                                end
                                break;
                            end
                        end
                    end
                    snap_r2 = r2_snaps_aligned{k};
                    if ~isempty(snap_r2.trackList)
                        for p = 1:length(matched_pairs)
                            if pair_to_aircraft(p) == a
                                r2_id = matched_pairs(p).R2_track_id;
                                trk2 = evaluate_all.find_track_by_id(snap_r2, r2_id);
                                if ~isempty(trk2) && ~isnan(trk2.lon)
                                    d = evaluate_all.haversine_km(trk2.lon, trk2.lat, t_true_lon, t_true_lat);
                                    r2_errs{a}(end+1) = d;
                                end
                                break;
                            end
                        end
                    end
                end
            end

            % Step 4: Summary statistics
            summary = struct();
            for m = 1:n_methods
                for a = 1:n_ac
                    summary(m,a).method = method_names{m};
                    summary(m,a).aircraft = a;
                    summary(m,a).s = evaluate_all.compute_err_stats(fusion_errs{m,a});
                end
            end
            for a = 1:n_ac
                summary(n_methods+1, a).method = 'R1_only';
                summary(n_methods+1, a).aircraft = a;
                summary(n_methods+1, a).s = evaluate_all.compute_err_stats(r1_errs{a});
            end
            for a = 1:n_ac
                summary(n_methods+2, a).method = 'R2_only';
                summary(n_methods+2, a).aircraft = a;
                summary(n_methods+2, a).s = evaluate_all.compute_err_stats(r2_errs{a});
            end

            overall = struct();
            all_methods = [method_names, {'R1_only', 'R2_only'}];
            all_errs = [fusion_errs; r1_errs; r2_errs];
            for m = 1:length(all_methods)
                combined = [];
                for a = 1:n_ac
                    combined = [combined, all_errs{m,a}];
                end
                overall(m).method = all_methods{m};
                overall(m).s = evaluate_all.compute_err_stats(combined);
            end

            fusion_eval = struct(...
                'method_names', {method_names}, ...
                'pair_to_aircraft', pair_to_aircraft, ...
                'fusion_errors', {fusion_errs}, ...
                'r1_errors', {r1_errs}, ...
                'r2_errors', {r2_errs}, ...
                'summary', summary, ...
                'overall', overall);
        end
    end

    methods (Static, Access = private)
        % ===== compute_summary =====
        function s = compute_summary(errs)
            s.n = length(errs);
            if s.n > 0
                s.median = median(errs);
                s.mean = mean(errs);
                s.std = std(errs);
                s.rms = sqrt(mean(errs.^2));
                s.min = min(errs);
                s.max = max(errs);
                s.pct95 = prctile(errs, 95);
            else
                s.median = NaN; s.mean = NaN; s.std = NaN; s.rms = NaN;
                s.min = NaN; s.max = NaN; s.pct95 = NaN;
            end
        end

        % ===== haversine_km =====
        function d = haversine_km(lon1, lat1, lon2, lat2)
            R = 6371;
            dlat = deg2rad(lat2 - lat1);
            dlon = deg2rad(lon2 - lon1);
            a = sin(dlat/2)^2 + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * sin(dlon/2)^2;
            a = max(0, min(1, a));
            d = R * 2 * atan2(sqrt(a), sqrt(1 - a));
        end

        % ===== compute_err_stats =====
        function s = compute_err_stats(errs)
            s.n = length(errs);
            if s.n > 0
                s.median = median(errs);
                s.mean = mean(errs);
                s.std = std(errs);
                s.rms = sqrt(mean(errs.^2));
                s.pct95 = prctile(errs, 95);
                s.min = min(errs);
                s.max = max(errs);
            else
                s.median = NaN; s.mean = NaN; s.std = NaN; s.rms = NaN;
                s.pct95 = NaN; s.min = NaN; s.max = NaN;
            end
        end

        % ===== find_track_by_id =====
        function trk = find_track_by_id(snap, tid)
            trk = [];
            if isempty(snap) || ~isfield(snap, 'trackList'), return; end
            for t = 1:length(snap.trackList)
                if snap.trackList{t}.id == tid
                    trk = snap.trackList{t};
                    return;
                end
            end
        end

        % ===== haversine_km_vec =====
        function d_vec = haversine_km_vec(lon1, lat1, lon2, lat2)
            d_vec = zeros(size(lon1));
            for i = 1:length(lon1)
                if isnan(lon1(i)) || isnan(lat1(i)) || isnan(lon2(i)) || isnan(lat2(i))
                    d_vec(i) = NaN;
                else
                    d_vec(i) = evaluate_all.haversine_km(lon1(i), lat1(i), lon2(i), lat2(i));
                end
            end
        end
    end
end
