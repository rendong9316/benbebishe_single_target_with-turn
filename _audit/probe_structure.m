% 公平口径审计：融合 vs 单站的帧集合是否一致
% 只读运行，不修改源码；run.m 的 Phase 10 保存已被注释，不产生结果文件
root = 'D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn';
cd(root);
addpath(genpath(root));

fprintf('\n===== RUN multi_cross =====\n');
r = run('multi_cross');

fprintf('\n===== R1 overall =====\n');
disp(r.errorStats_R1.overall);
fprintf('\n===== R1 perAircraft(1) =====\n');
disp(r.errorStats_R1.perAircraft(1));
fprintf('\n===== fusion overall =====\n');
disp(r.fusion_eval.overall);
fprintf('\n===== fusion method_names =====\n');
disp(r.method_names);

save(fullfile(root,'_audit','mc_current_geom.mat'), '-struct', 'r', '-v7.3');
fprintf('\nCACHE SAVED\n');
