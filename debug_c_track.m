addpath(genpath('.'));
d=dir('results/simulation_multi_*.mat');
[~,i]=max([d.datenum]);
load(fullfile(d(i).folder,d(i).name));
snaps=trackSnapshots_R1;
tt=truthTrajs{3};
fprintf('Target C: lon=%.1f->%.1f lat=%.1f->%.1f\n', tt.lon(1), tt.lon(end), tt.lat(1), tt.lat(end));
fprintf('Time range: %.0f-%.0f s\n', tt.time_sec(1), tt.time_sec(end));
for k=1:min(34, length(snaps))
    tnow=(k-1)*params.dt_sec;
    tl=interp1(tt.time_sec,tt.lon,tnow,'linear','extrap');
    tb=interp1(tt.time_sec,tt.lat,tnow,'linear','extrap');
    trks=snaps{k}.trackList;
    best_id=0;
    best_d=inf;
    for t=1:length(trks)
        if trks{t}.type~=7 && ~isnan(trks{t}.lat)
            d=sphere_utils_haversine_distance(trks{t}.lon,trks{t}.lat,tl,tb)/1000;
            if d<best_d, best_d=d; best_id=trks{t}.id; end
        end
    end
    fprintf('Frame %2d t=%3d trk#%d dist=%.1fkm\n', k, tnow, best_id, best_d)
end
