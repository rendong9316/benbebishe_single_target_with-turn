addpath(genpath('.'));
clear -file;
f = dir('results/simulation_multi_*.mat');
loaded = load(fullfile('results', f(end).name));
snapR1 = loaded.trackSnapshots_R1;

params = loaded.params;
way_A = [127.0, 31.0, 0; 130.0, 34.0, 0];
way_B = [126.5, 33.0, 0; 130.0, 31.0, 0];
way_C = [126.0, 32.5, 0; 131.0, 32.5, 0];
traj_A = aircraft_trajectory_create(way_A, params.aircraft_speed_ms, params.dt_sec);
traj_B = aircraft_trajectory_create(way_B, params.aircraft_speed_ms, params.dt_sec);
traj_C = aircraft_trajectory_create(way_C, params.aircraft_speed_ms, params.dt_sec);
ttA = aircraft_trajectory_interpolate('generate', traj_A);
ttB = aircraft_trajectory_interpolate('generate', traj_B);
ttC = aircraft_trajectory_interpolate('generate', traj_C);
t1_grid = params.time_offset_radar1_sec : params.dt_sec : 2039;

fprintf('Frame | ErrA(R1) | ErrB(R1) | ErrC(R1)\n');
for k = 1:length(t1_grid)
    [mi, ki] = min(abs(ttA(:,5) - t1_grid(k))); tlA = ttA(ki,1); tbA = ttA(ki,2);
    [mi, ki] = min(abs(ttB(:,5) - t1_grid(k))); tlB = ttB(ki,1); tbB = ttB(ki,2);
    [mi, ki] = min(abs(ttC(:,5) - t1_grid(k))); tlC = ttC(ki,1); tbC = ttC(ki,2);

    errA = NaN; errB = NaN; errC = NaN;
    snap = snapR1{k};
    for t = 1:length(snap.trackList)
        trk = snap.trackList{t};
        if trk.type == 7 || isnan(trk.lat), continue; end
        if isfield(trk, 'ac_idx')
            if trk.ac_idx == 1
                errA = sphere_utils_haversine_distance(trk.lon, trk.lat, tlA, tbA) / 1000;
            elseif trk.ac_idx == 2
                errB = sphere_utils_haversine_distance(trk.lon, trk.lat, tlB, tbB) / 1000;
            elseif trk.ac_idx == 3
                errC = sphere_utils_haversine_distance(trk.lon, trk.lat, tlC, tbC) / 1000;
            end
        end
    end
    fprintf('%5d | %8.2fkm | %8.2fkm | %8.2fkm\n', k, errA, errB, errC);
end
