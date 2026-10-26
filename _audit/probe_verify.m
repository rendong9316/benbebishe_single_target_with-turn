function probe_verify()
root = 'D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn';
cd(root); addpath(genpath(root));
r = run('multi_cross');
fid = fopen(fullfile(root,'_audit','verify.txt'),'w');
fprintf(fid,'R1=%.4f\nR2=%.4f\n', r.errorStats_R1.overall.ukf.rms, r.errorStats_R2.overall.ukf.rms);
fclose(fid);
disp(fileread(fullfile(root,'_audit','verify.txt')));
end
