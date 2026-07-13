addpath(genpath('.'));
d=dir('results/simulation_multi_*.mat');
[~,i]=max([d.datenum]);
load(fullfile(d(i).folder,d(i).name));
fprintf('Loaded: %s\n', d(i).name);
errorStats_R1 = evaluate_all_multi('tracking_errors', trackSnapshots_R1, detList_R1, truthTrajs, length(trackSnapshots_R1), params.dt_sec, 'R1');
for a=1:length(errorStats_R1.summary)
    s = errorStats_R1.summary(a).ukf;
    fprintf('R1 target%d: n=%d median=%.1fkm mean=%.1fkm RMSE=%.1fkm\n', ...
        a, s.n, s.median, s.mean, s.rms);
end
errorStats_R2 = evaluate_all_multi('tracking_errors', trackSnapshots_R2, detList_R2, truthTrajs, length(trackSnapshots_R2), params.dt_sec, 'R2');
for a=1:length(errorStats_R2.summary)
    s = errorStats_R2.summary(a).ukf;
    fprintf('R2 target%d: n=%d median=%.1fkm mean=%.1fkm RMSE=%.1fkm\n', ...
        a, s.n, s.median, s.mean, s.rms);
end
