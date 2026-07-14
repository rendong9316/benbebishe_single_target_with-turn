% diagnose_track3.m - 诊断3号航迹越界问题
% 不重新跑整个仿真，只统计 trackSnapshots_R1/R2 中每条航迹的 birth/death_frame

addpath(genpath('.'));

% 直接加载最新的仿真结果
d = dir('results/simulation_multi_*.mat');
if isempty(d)
    fprintf('没有找到 results/simulation_multi_*.mat，先跑一次 run_simulation_multi\n');
    return;
end
[~, idx] = max([d.datenum]);
f = fullfile('results', d(idx).name);
fprintf('加载: %s\n', f);
S = load(f);

truthTrajs = S.truthTrajs;
fprintf('\n=== 真值航迹长度 ===\n');
for i = 1:length(truthTrajs)
    t = truthTrajs{i};
    fprintf('  目标 %s: %d 点, t=[%.0f, %.0f]s\n', t.label, length(t.time_sec), t.time_sec(1), t.time_sec(end));
end

params = S.params;
dt = params.dt_sec;

for radar_name = {'R1', 'R2'}
    snaps = S.(sprintf('trackSnapshots_%s', radar_name{1}));
    n_frames = length(snaps);
    % 收集每条航迹的 birth_frame / death_frame / last_active_frame
    track_ids = [];
    birth_frame = [];
    last_active_frame = [];
    last_pos = [];
    for k = 1:n_frames
        if ~isfield(snaps{k}, 'trackList'), continue; end
        trks = snaps{k}.trackList;
        for t = 1:length(trks)
            trk = trks{t};
            if trk.type == 7, continue; end
            if isnan(trk.lat), continue; end
            tid = trk.id;
            fi = find(track_ids == tid, 1);
            if isempty(fi)
                track_ids(end+1) = tid;
                birth_frame(end+1) = k;
                last_active_frame(end+1) = k;
                last_pos(end+1, :) = [trk.lon, trk.lat];
            else
                last_active_frame(fi) = k;
                last_pos(fi, :) = [trk.lon, trk.lat];
            end
        end
    end
    fprintf('\n=== %s 航迹统计 (n_frames=%d, dt=%.0fs) ===\n', radar_name{1}, n_frames, dt);
    fprintf('  ID | birth | last_active | 寿命帧 | 寿命秒 | 末点(lon,lat)\n');
    for i = 1:length(track_ids)
        life = last_active_frame(i) - birth_frame(i) + 1;
        fprintf('  %2d | %4d  | %4d        | %4d   | %5.0f  | (%.2f, %.2f)\n', ...
            track_ids(i), birth_frame(i), last_active_frame(i), life, life*dt, ...
            last_pos(i,1), last_pos(i,2));
    end
end
