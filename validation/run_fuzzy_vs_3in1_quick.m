function run_fuzzy_vs_3in1_quick()
%RUN_FUZZY_VS_3IN1_QUICK Ultra-fast fuzzy_only vs 3in1 comparison.
% Only 3 representative scenarios x 5 seeds = 30 runs per mode.

    root = fileparts(mfilename('fullpath'));
    params = simulation_params_oracle();

    % 3 representative scenarios: straight + turn + sustained turn
    specs = { ...
        {'single_straight', 'straight', 1.0}, ...
        {'single_turn_right_short', 'right_short', 1.0}, ...
        {'single_turn_right_sustained', 'right_sustained', 1.0}};

    n_scenarios = numel(specs);
    n_seeds = 5;
    seed_start = 10001;

    fprintf('Quick fuzzy_only vs 3in1 comparison\n');
    fprintf('Scenarios: %d, Seeds: %d per mode\n\n', n_scenarios, n_seeds);

    % Prepare detection data
    prepared = cell(n_scenarios, 1);
    for s = 1:n_scenarios
        prepared{s} = prepare_oracle_tracking_inputs(specs{s}{1}, ...
            struct('random_seed', seed_start));
    end

    % Test both modes
    modes = {'3in1', 'fuzzy_only'};
    all_rmse = cell(2, 1);

    for mi = 1:2
        mode = modes{mi};
        fprintf('=== Mode: %s ===\n', mode);

        rmse_list = zeros(n_scenarios, 1);
        for si = 1:n_scenarios
            inp = prepared{si};
            total_pos_sq = [];

            for radar_id = 1:2
                for seed_idx = 1:n_seeds
                    seed_val = seed_start + seed_idx - 1;
                    overrides = struct('imm_adapt_mode', mode, 'random_seed', seed_val);

                    if radar_id == 1
                        det = inp.detList_R1; tg = inp.t1_grid;
                        rl = params.radar1_lon; rlat = params.radar1_lat;
                        tl = params.radar1_tx_lon; tlat = params.radar1_tx_lat;
                    else
                        det = inp.detList_R2; tg = inp.t2_grid;
                        rl = params.radar2_lon; rlat = params.radar2_lat;
                        tl = params.radar2_tx_lon; tlat = params.radar2_tx_lat;
                    end

                    prm = radar_params(params, radar_id);
                    fnames = fieldnames(overrides);
                    for fi = 1:numel(fnames)
                        prm.(fnames{fi}) = overrides.(fnames{fi});
                    end

                    tpl = ukf_imm('create', prm, rl, rlat, tl, tlat, prm.dt_sec);
                    [~, ~, snaps] = run_oracle_tracker_sequence(det, tpl, prm, inp.truth_all, tg, false);

                    for f = 1:numel(snaps)
                        if isempty(snaps{f}) || ~isfield(snaps{f}, 'trackList')
                            continue;
                        end
                        for ti = 1:numel(snaps{f}.trackList)
                            trk = snaps{f}.trackList{ti};
                            if ~trk.updated || ~isfinite(trk.combined_nis)
                                continue;
                            end
                            truth = inp.truthTrajs{trk.truth_idx};
                            if isempty(truth) || ~isfield(truth, 'time_sec')
                                continue;
                            end
                            t_now = tg(f);
                            if t_now < truth.time_sec(1) || t_now > truth.time_sec(end)
                                continue;
                            end
                            true_lon = interp1(truth.time_sec, truth.lon, t_now, 'linear');
                            true_lat = interp1(truth.time_sec, truth.lat, t_now, 'linear');
                            if ~all(isfinite([true_lon, true_lat]))
                                continue;
                            end
                            pos_err = haversine_distance(trk.ukf.x(1), trk.ukf.x(3), true_lon, true_lat);
                            total_pos_sq(end+1) = pos_err^2;
                        end
                    end
                    clear tracks tpl det snaps;
                end
            end

            rmse_km = sqrt(mean(total_pos_sq)) / 1000;
            rmse_list(si) = rmse_km;
            fprintf('  %s: %.3f km (R1+R2 avg)\n', specs{si}{2}, rmse_km);
            clear total_pos_sq;
        end
        all_rmse{mi} = rmse_list;
        fprintf('\n');
    end

    % Compare
    fprintf('=== QUICK COMPARISON (3 scenes, 5 seeds) ===\n');
    fprintf('%-15s  %-12s  %-12s  %-8s\n', 'Scenario', '3in1', 'fuzzy_only', 'Delta%');
    fprintf('%s\n', repmat('-', 1, 52));
    for si = 1:n_scenarios
        m3 = all_rmse{1}(si);
        mf = all_rmse{2}(si);
        delta = (mf - m3) / m3 * 100;
        fprintf('%-15s  %-12.3f  %-12.3f  %+7.1f%%\n', specs{si}{2}, m3, mf, delta);
    end
    avg_3in1 = mean(all_rmse{1});
    avg_fuzz = mean(all_rmse{2});
    overall_delta = (avg_fuzz - avg_3in1) / avg_3in1 * 100;
    fprintf('%-15s  %-12.3f  %-12.3f  %+7.1f%%\n', 'AVG', avg_3in1, avg_fuzz, overall_delta);

    if overall_delta > 20
        fprintf('\n*** fuzzy_only is >20%% worse than 3in1 on quick test.\n');
        fprintf('*** Recommendation: focus search on 3in1 mode only.\n\n');
    elseif overall_delta < -10
        fprintf('\n*** fuzzy_only is >10%% better than 3in1 on quick test!\n');
        fprintf('*** Recommendation: dedicate search to fuzzy_only mode.\n\n');
    else
        fprintf('\n*** fuzzy_only is within +/-20%% of 3in1. Worth searching both.\n\n');
    end
end

function dist = haversine_distance(lon1, lat1, lon2, lat2)
    dlat = deg2rad(lat2 - lat1);
    dlon = deg2rad(lon2 - lon1);
    a = sin(dlon/2)^2 + cosd(lat1)*cosd(lat2)*sin(dlat/2)^2;
    a = max(0, min(1, a));
    dist = 6371000 * 2 * atan2(sqrt(a), sqrt(1-a));
end
