function test_ukf_modes(mode)
%TEST_UKF_MODES Compare different UKF configurations on the same scenarios.
%
% Evaluates a grid of UKF "modes" (filter architecture + Q level + IMM settings)
% to find which combination achieves the lowest position RMSE.
%
% This does NOT modify the existing architecture. It only tests different
% parameter combinations and mode switches via the existing ukf_dispatch.
%
% Usage:
%   test_ukf_modes          % run full grid
%   test_ukf_modes('smoke') % quick test on one scenario
%
% Output:
%   Prints a table of mode x scenario RMSE to the console.
%   Saves mode_comparison.csv and mode_comparison.mat.

    quick = false;
    if nargin >= 1 && ischar(mode) && strcmpi(mode, 'smoke')
        quick = true;
    end

    root = fileparts(mfilename('fullpath'));
    params = simulation_params_oracle();

    % --- Scenarios --------------------------------------------------------
    if quick
        scenarios = {'single_turn'};
    else
        scenarios = {'single_turn', 'single_uturn'};
    end

    % --- Mode definitions -------------------------------------------------
    modes = cell(8, 1);
    modes{1}  = struct('name', 'imm_default',    'filter', 'imm',     'desc', 'IMM + default Q=0.02', 'overrides', struct());
    modes{2}  = struct('name', 'imm_highQ',      'filter', 'imm',     'desc', 'IMM + Q_psd=0.5 (25x higher)', 'overrides', struct());
    modes{3}  = struct('name', 'imm_veryHighQ',  'filter', 'imm',     'desc', 'IMM + Q_psd=2.0 (100x higher)', 'overrides', struct());
    modes{4}  = struct('name', 'imm_lowQ',       'filter', 'imm',     'desc', 'IMM + Q_psd=0.005 (4x lower)', 'overrides', struct());
    modes{5}  = struct('name', 'adaptive_default','filter','adaptive', 'desc', 'Adaptive UKF + default Q', 'overrides', struct());
    modes{6}  = struct('name', 'adaptive_highQ', 'filter','adaptive', 'desc', 'Adaptive UKF + Q_psd=0.5', 'overrides', struct());
    modes{7}  = struct('name', 'baseline_default','filter','baseline', 'desc', 'Baseline UKF + default Q', 'overrides', struct());
    modes{8}  = struct('name', 'baseline_highQ', 'filter','baseline', 'desc', 'Baseline UKF + Q_psd=0.5', 'overrides', struct());

    % Override Q_psd for modes that need it
    modes{2}.overrides.ukf_process_accel_psd_m2_s3 = 0.5;
    modes{3}.overrides.ukf_process_accel_psd_m2_s3 = 2.0;
    modes{4}.overrides.ukf_process_accel_psd_m2_s3 = 0.005;
    modes{6}.overrides.ukf_process_accel_psd_m2_s3 = 0.5;
    modes{8}.overrides.ukf_process_accel_psd_m2_s3 = 0.5;

    % Additional IMM-specific overrides
    modes{1}.overrides.imm_adapt_mode = '3in1';
    modes{2}.overrides.imm_adapt_mode = '3in1';
    modes{3}.overrides.imm_adapt_mode = '3in1';
    modes{4}.overrides.imm_adapt_mode = '3in1';

    % --- Prepare detection data -------------------------------------------
    prepared = cellfun(@prepare_oracle_tracking_inputs, scenarios, 'UniformOutput', false);

    n_modes = numel(modes);
    n_scenarios = numel(scenarios);
    n_radars = 2;

    % Storage: results{mode_idx, scenario_idx, radar_idx} = struct with metrics
    results = cell(n_modes, n_scenarios, n_radars);

    fprintf('============================================================\n');
    fprintf('UKF Mode Comparison Test\n');
    fprintf('============================================================\n\n');

    for m = 1:n_modes
        fprintf('[%d/%d] Testing mode: %s (%s)\n', m, n_modes, modes{m}.name, modes{m}.desc);
        for s = 1:n_scenarios
            inp = prepared{s};
            if ischar(inp) || isstring(inp)
                ld = load(char(inp), 'inputs');
                inp = ld.inputs;
            end

            for r = 1:n_radars
                if r == 1
                    det = inp.detList_R1;
                    tg = inp.t1_grid;
                    rl = params.radar1_lon; rlat = params.radar1_lat;
                    tl = params.radar1_tx_lon; tlat = params.radar1_tx_lat;
                else
                    det = inp.detList_R2;
                    tg = inp.t2_grid;
                    rl = params.radar2_lon; rlat = params.radar2_lat;
                    tl = params.radar2_tx_lon; tlat = params.radar2_tx_lat;
                end

                % Apply overrides
                prm = radar_params(params, r);
                fnames = fieldnames(modes{m}.overrides);
                for fi = 1:numel(fnames)
                    prm.(fnames{fi}) = modes{m}.overrides.(fnames{fi});
                end

                % Create UKF template based on mode
                switch modes{m}.filter
                    case 'imm'
                        tpl = ukf_imm('create', prm, rl, rlat, tl, tlat, prm.dt_sec);
                    case 'adaptive'
                        tpl = ukf_jichu('create', prm, rl, rlat, tl, tlat, prm.dt_sec);
                        tpl.filter_type = 'zishiying';
                    case 'baseline'
                        tpl = ukf_jichu('create', prm, rl, rlat, tl, tlat, prm.dt_sec);
                end

                % Run tracker
                [tracks, ~, snaps] = run_oracle_tracker_sequence( ...
                    det, tpl, prm, inp.truth_all, tg, false);

                % Compute metrics
                pos_sq = [];
                speed_sq = [];
                nis_arr = [];
                nees_arr = [];

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
                        truth_t = truth.time_sec;
                        if f > length(tg)
                            continue;
                        end
                        t_now = tg(f);
                        if t_now < truth_t(1) || t_now > truth_t(end)
                            continue;
                        end
                        truth_lon = interp1(truth_t, truth.lon, t_now, 'linear');
                        truth_lon_rate = interp1(truth_t, truth.lon_rate, t_now, 'linear');
                        truth_lat = interp1(truth_t, truth.lat, t_now, 'linear');
                        truth_lat_rate = interp1(truth_t, truth.lat_rate, t_now, 'linear');
                        if ~all(isfinite([truth_lon, truth_lon_rate, truth_lat, truth_lat_rate]))
                            continue;
                        end
                        ts_state = [truth_lon; truth_lon_rate; truth_lat; truth_lat_rate];

                        pos_err = geographic_distance(trk.ukf.x(1), trk.ukf.x(3), ...
                            ts_state(1), ts_state(3));
                        pos_sq(end+1) = pos_err^2;

                        est_spd = hypot(trk.ukf.x(2)*6371000*pi/180*cosd(trk.ukf.x(3)), ...
                            trk.ukf.x(4)*6371000*pi/180);
                        tru_spd = hypot(ts_state(2)*6371000*pi/180*cosd(ts_state(3)), ...
                            ts_state(4)*6371000*pi/180);
                        speed_sq(end+1) = (est_spd - tru_spd)^2;

                        nis_arr(end+1) = trk.combined_nis;
                    end
                end

                results{m,s,r} = struct( ...
                    'pos_rmse_km', sqrt(mean(pos_sq))/1000, ...
                    'speed_rmse_ms', sqrt(mean(speed_sq)), ...
                    'nis_mean', mean(nis_arr), ...
                    'samples', numel(pos_sq));

                fprintf('  R%d: pos=%.3f km vel=%.1f m/s samples=%d nis=%.2f\n', ...
                    r, results{m,s,r}.pos_rmse_km, results{m,s,r}.speed_rmse_ms, ...
                    results{m,s,r}.samples, results{m,s,r}.nis_mean);

                clear tracks tpl det snaps;
            end
        end
    end

    % --- Print results table ----------------------------------------------
    fprintf('\n');
    if n_scenarios == 1
        fprintf('%-18s  %-10s  %-10s  %-12s  %-12s\n', ...
            'Mode', 'Straight-R1', 'Straight-R2', 'R1-samples', 'R2-samples');
        fprintf('%s\n', repmat('-', 1, 60));
        for m = 1:n_modes
            s1 = results{m,1,1};
            s2 = results{m,1,2};
            fprintf('%-18s  %-10.3f  %-10.3f  %-12d  %-12d\n', ...
                modes{m}.name, s1.pos_rmse_km, s2.pos_rmse_km, ...
                s1.samples, s2.samples);
        end
        fprintf('\nBest mode (lowest avg RMSE):\n');
        for r = 1:n_radars
            best_m = 1;
            best_r = inf;
            for m = 1:n_modes
                r_val = results{m,1,r}.pos_rmse_km;
                if r_val < best_r && isfinite(r_val)
                    best_r = r_val;
                    best_m = m;
                end
            end
            fprintf('  R%d: %s (%.3f km)\n', r, modes{best_m}.name, best_r);
        end
    else
        fprintf('%-18s  %-10s  %-10s  %-10s  %-10s  %-10s  %-10s\n', ...
            'Mode', 'S-R1-pos', 'S-R1-vel', 'S-R2-pos', 'S-R2-vel', 'U-R1-pos', 'U-R2-pos');
        fprintf('%s\n', repmat('-', 1, 96));

        for m = 1:n_modes
            s1 = results{m,1,1}; s2 = results{m,1,2};
            s3 = results{m,2,1}; s4 = results{m,2,2};
            fprintf('%-18s  %-10.3f  %-10.1f  %-10.3f  %-10.1f  %-10.3f  %-10.3f\n', ...
                modes{m}.name, ...
                s1.pos_rmse_km, s1.speed_rmse_ms, ...
                s2.pos_rmse_km, s2.speed_rmse_ms, ...
                s3.pos_rmse_km, s4.pos_rmse_km);
        end

        fprintf('\nBest mode per scenario-radar (lowest RMSE):\n');
        for s = 1:n_scenarios
            for r = 1:n_radars
                best_m = 1;
                best_r = inf;
                for m = 1:n_modes
                    r_val = results{m,s,r}.pos_rmse_km;
                    if r_val < best_r && isfinite(r_val)
                        best_r = r_val;
                        best_m = m;
                    end
                end
                if s == 1 && r == 1
                    fprintf('  Straight R1: %s (%.3f km)\n', modes{best_m}.name, best_r);
                elseif s == 1 && r == 2
                    fprintf('  Straight R2: %s (%.3f km)\n', modes{best_m}.name, best_r);
                else
                    fprintf('  U-Turn  R%d: %s (%.3f km)\n', r, modes{best_m}.name, best_r);
                end
            end
        end
    end

    % --- Save results -----------------------------------------------------
    mat_path = fullfile(root, 'mode_comparison.mat');
    save(mat_path, 'results', 'modes', 'scenarios', 'prepared');
    fprintf('\nResults saved to %s\n', mat_path);
end


function dist = geographic_distance(lon1, lat1, lon2, lat2)
    dlat = deg2rad(lon2 - lon1);
    dlon = deg2rad(lat2 - lat1);
    a = sin(dlon/2)^2 + cosd(lat1)*cosd(lat2)*sin(dlat/2)^2;
    a = max(0, min(1, a));
    dist = 6371000 * 2 * atan2(sqrt(a), sqrt(1-a));
end
