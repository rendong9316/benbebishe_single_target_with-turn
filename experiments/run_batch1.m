function run_batch1()
% 第一批：小场景高种子数（multi_cross + adsb_0002），串行跑完四个消融
% 预计 50-60 分钟
root = 'D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn';
cd(root); addpath(genpath(root)); addpath(fullfile(root,'experiments'));

fprintf('\n########## BATCH 1 START %s ##########\n', datestr(now,31));

run_experiments('E1_small', 1:10);
run_experiments('E2_assoc_small', 1:5);
run_experiments('E3_t1', 1:5);
run_experiments('E4_m', 1:5);

fprintf('\n########## SUMMARIZE ##########\n');
summarize_paper({'E1_small','E2_assoc_small','E3_t1','E4_m'});

fprintf('\n########## BATCH 1 DONE %s ##########\n', datestr(now,31));
end
