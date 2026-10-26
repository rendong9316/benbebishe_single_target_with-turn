function probe_adsb_run()
% 验证 ADS-B 场景能否跑通全流程
root = 'D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn';
cd(root); addpath(genpath(root));
fid = fopen(fullfile(root,'_audit','adsb_run.txt'),'w');
for sid = {'adsb_0002'}
    t0 = tic;
    try
        r = run(sid{1});
        el = toc(t0);
        fprintf(fid,'\n===== %s OK (%.0f s) =====\n', sid{1}, el);
        fprintf(fid,'targets=%d  frames=%d\n', r.scenario.n_targets, r.scenario.n_frames);
        fprintf(fid,'R1 n=%d rms=%.3f  R2 n=%d rms=%.3f\n', ...
            r.errorStats_R1.overall.ukf.n, r.errorStats_R1.overall.ukf.rms, ...
            r.errorStats_R2.overall.ukf.n, r.errorStats_R2.overall.ukf.rms);
        fprintf(fid,'matched pairs=%d\n', numel(r.matched_pairs));
        f = r.fusion_eval.overall;
        for m = 1:numel(f)
            fprintf(fid,'  %-10s n=%4d rms=%.3f\n', f(m).method, f(m).s.n, f(m).s.rms);
        end
    catch ME
        fprintf(fid,'\n===== %s FAILED =====\n%s\n', sid{1}, ME.message);
        for k=1:min(3,numel(ME.stack))
            fprintf(fid,'  at %s:%d\n', ME.stack(k).name, ME.stack(k).line);
        end
    end
end
fclose(fid);
disp(fileread(fullfile(root,'_audit','adsb_run.txt')));
end
