function probe_metrics()
root = 'D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn';
cd(root); addpath(genpath(root));
fid = fopen(fullfile(root,'_audit','metrics_out.txt'),'w');

% 用缓存测试 adsb_0002
try
    r = load(fullfile(root,'_audit','adsb_0002_cache.mat'));
    m = evaluate_paper_metrics(r);
    fprintf(fid,'\n### adsb_0002 metrics computed OK\n');
    fprintf(fid,'track_len R1 real mean=%.1f false_rate=%.1f%%\n', ...
        m.track_length.real_mean, 100*m.track_length.false_rate);
    fprintf(fid,'match_rate=%.1f%% wrong_rate=%.1f%%\n', ...
        100*m.matching.match_rate, 100*m.matching.wrong_rate);
    fprintf(fid,'coverage=%.1f%%  common_frame n=%d R1=%.2f R2=%.2f fused=%.2f\n', ...
        100*m.coverage.coverage, m.common_frame.n, m.common_frame.R1, ...
        m.common_frame.R2, m.common_frame.fused);
catch ME
    fprintf(fid,'adsb FAILED: %s\n', ME.message);
    for k=1:min(3,numel(ME.stack))
        fprintf(fid,'  at %s:%d\n', ME.stack(k).name, ME.stack(k).line);
    end
end

% 合成场景
try
    r2 = run('multi_cross');
    m2 = evaluate_paper_metrics(r2);
    fprintf(fid,'\n### multi_cross metrics computed OK\n');
    fprintf(fid,'track_len R1 real mean=%.1f false_rate=%.1f%%\n', ...
        m2.track_length.real_mean, 100*m2.track_length.false_rate);
    fprintf(fid,'match_rate=%.1f%%  coverage=%.1f%%\n', ...
        100*m2.matching.match_rate, 100*m2.coverage.coverage);
    fprintf(fid,'common_frame n=%d R1=%.2f R2=%.2f fused=%.2f gain=%.1f%%\n', ...
        m2.common_frame.n, m2.common_frame.R1, m2.common_frame.R2, ...
        m2.common_frame.fused, ...
        100*(min(m2.common_frame.R1,m2.common_frame.R2)-m2.common_frame.fused)/min(m2.common_frame.R1,m2.common_frame.R2));
catch ME
    fprintf(fid,'multi_cross FAILED: %s\n', ME.message);
    for k=1:min(3,numel(ME.stack))
        fprintf(fid,'  at %s:%d\n', ME.stack(k).name, ME.stack(k).line);
    end
end

fclose(fid);
disp(fileread(fullfile(root,'_audit','metrics_out.txt')));
end
