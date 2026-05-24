% =========================================================================
% load_adsb.m
% Load ADS-B CSV data, extract aircraft trajectories, resample to sim grid
% =========================================================================

function [true_tracks, labels, speeds] = load_adsb(csv_path, icao_list, label_list, ...
        dt_sec, start_time, duration_sec, time_offset_sec)

    if nargin < 7, time_offset_sec = 0; end

    opts = detectImportOptions(csv_path, 'NumVariables', 19);
    opts.VariableNames = {'icao','lat','lon','heading','alt_ft','speed_kt',...
        'x7','rx','type','reg','ts','origin','dest','flight','flag1',...
        'vr_ft','icao_flt','flag2','airline'};
    T = readtable(csv_path, opts);

    n_ac = length(icao_list);
    true_tracks = cell(n_ac, 1);
    labels = cell(n_ac, 1);
    speeds = zeros(n_ac, 1);

    for a = 1:n_ac
        icao = icao_list{a};
        idx = strcmp(T.icao, icao);
        if sum(idx) == 0
            error('Aircraft %s not found in ADS-B data', icao);
        end

        ac_lat = T.lat(idx);
        ac_lon = T.lon(idx);
        ac_spd = T.speed_kt(idx);
        ts_raw = T.ts(idx);

        % Parse timestamps safely
        if iscell(ts_raw)
            ts_dt = datetime(ts_raw, 'InputFormat', 'yyyy-MM-dd HH:mm:ss');
        else
            ts_dt = datetime(cellstr(string(ts_raw)), 'InputFormat', 'yyyy-MM-dd HH:mm:ss');
        end
        t_sec = seconds(ts_dt - start_time);

        % Filter to simulation window (with offset)
        valid = t_sec >= time_offset_sec & t_sec <= time_offset_sec + duration_sec ...
                & ~isnan(ac_lat) & ~isnan(ac_lon);
        ac_lat = ac_lat(valid);
        ac_lon = ac_lon(valid);
        ac_spd = ac_spd(valid);
        t_sec = t_sec(valid);

        if sum(valid) < 3
            error('Aircraft %s has <3 valid points in simulation window', icao);
        end

        % Deduplicate by time
        [t_sec, ui] = unique(t_sec, 'stable');
        ac_lat = ac_lat(ui);
        ac_lon = ac_lon(ui);
        ac_spd = ac_spd(ui);

        % Sort by time
        [t_sec, si] = sort(t_sec);
        ac_lat = ac_lat(si);
        ac_lon = ac_lon(si);
        ac_spd = ac_spd(si);

        % Resample to simulation grid (times relative to offset)
        t_grid = (0:dt_sec:duration_sec)';
        t_sec_relative = t_sec - time_offset_sec;
        lat_grid = interp1(t_sec_relative, ac_lat, t_grid, 'linear', 'extrap');
        lon_grid = interp1(t_sec_relative, ac_lon, t_grid, 'linear', 'extrap');

        % Compute rates via central difference
        n = length(t_grid);
        lon_rate = zeros(n, 1);
        lat_rate = zeros(n, 1);
        for k = 2:n-1
            lon_rate(k) = (lon_grid(k+1) - lon_grid(k-1)) / (2*dt_sec);
            lat_rate(k) = (lat_grid(k+1) - lat_grid(k-1)) / (2*dt_sec);
        end
        if n >= 2
            lon_rate(1)   = (lon_grid(2) - lon_grid(1)) / dt_sec;
            lat_rate(1)   = (lat_grid(2) - lat_grid(1)) / dt_sec;
            lon_rate(end) = (lon_grid(end) - lon_grid(end-1)) / dt_sec;
            lat_rate(end) = (lat_grid(end) - lat_grid(end-1)) / dt_sec;
        end

        true_tracks{a} = [lon_grid, lat_grid, lon_rate, lat_rate, t_grid];
        labels{a} = label_list{a};
        speeds(a) = mean(ac_spd, 'omitnan') * 0.514444;

        fprintf('  %s(%s): %d ADS-B pts, resampled %d pts, avg spd %.0f m/s\n', ...
            label_list{a}, icao, sum(idx), n, speeds(a));
    end
end
