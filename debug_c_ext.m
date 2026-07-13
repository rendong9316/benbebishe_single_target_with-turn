addpath(genpath('.'));
d=dir('results/simulation_multi_*.mat');
[~,i]=max([d.datenum]);
load(fullfile(d(i).folder,d(i).name));
fprintf('Loaded: %s\n', d(i).name);
fprintf('Target C time range: %.0f-%.0f s (n=%d frames)\n', ...
    truthTrajs{3}.time_sec(1), truthTrajs{3}.time_sec(end), ...
    length(truthTrajs{3}.time_sec));
fprintf('R1 #3 trajectory:\n');
snaps=trackSnapshots_R1;
for k=1:length(snaps)
    trks=snaps{k}.trackList;
    for t=1:length(trks)
        if trks{t}.id==3 && trks{t}.type~=7 && ~isnan(trks{t}.lat)
            tnow=(k-1)*params.dt_sec;
            fprintf('  f=%2d t=%3ds lon=%.3f lat=%.3f type=%d q=%d missed=%d life=%d\n', ...
                k, tnow, trks{t}.lon, trks{t}.lat, trks{t}.type, ...
                trks{t}.quality, trks{t}.missed, trks{t}.life);
            break;
        end
    end
end
