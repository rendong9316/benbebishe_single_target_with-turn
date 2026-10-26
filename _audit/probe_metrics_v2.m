function probe_metrics_v2()
root = 'D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn';
cd(root); addpath(genpath(root));
fid = fopen(fullfile(root,'_audit','metrics_v2.txt'),'w');

for sc = {'multi_cross','adsb_0002'}
    try
        r = run(sc{1});
        m = evaluate_paper_metrics(r);
        c = m.consistency;
        fprintf(fid,'\n### %s\n', sc{1});
        fprintf(fid,'  NEES R1=%.2f(n=%d) R2=%.2f(n=%d)\n', c.nees_R1, c.nees_R1_n, c.nees_R2, c.nees_R2_n);
        fprintf(fid,'  rho=%+.3f  indep=%.2f  opt=%.2f  actual=%.2f  gap=%.1f%%\n', ...
            c.rho, c.indep_rms, c.opt_rms, m.common_frame.fused, ...
            100*(m.common_frame.fused-c.opt_rms)/c.opt_rms);
    catch ME
        fprintf(fid,'\n### %s FAILED: %s\n', sc{1}, ME.message);
        for k=1:min(3,numel(ME.stack))
            fprintf(fid,'  at %s:%d\n', ME.stack(k).name, ME.stack(k).line);
        end
    end
end
fclose(fid);
disp(fileread(fullfile(root,'_audit','metrics_v2.txt')));
end
