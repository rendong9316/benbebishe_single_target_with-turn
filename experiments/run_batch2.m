function run_batch2()
% 第二批：补齐 NEES/rho 多 seed 分布 + CCF kappa 扫描 + 模型集消融
root = 'D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn';
cd(root); addpath(genpath(root)); addpath(fullfile(root,'experiments'));

fprintf('\n########## BATCH 2 START %s ##########\n', datestr(now,31));

% 1) 重跑 E1_small（新指标含 NEES/rho），覆盖旧结果
%    注意：需先删除旧结果，否则断点续跑会跳过
d = fullfile(root,'results','paper','E1_small');
if exist(d,'dir')
    old = dir(fullfile(d,'*.mat'));
    for i=1:numel(old)
        movefile(fullfile(d,old(i).name), fullfile(d,[old(i).name '.old']));
    end
end
run_experiments('E1_small', 1:10);

% 2) CCF 校准因子扫描
run_experiments('E6_kappa', 1:10);

% 3) 模型集消融（真实转弯率匹配）
run_experiments('E7_model', 1:10);

fprintf('\n########## SUMMARIZE ##########\n');
summarize_paper({'E1_small','E5_fusion','E6_kappa','E7_model'});

fprintf('\n########## BATCH 2 DONE %s ##########\n', datestr(now,31));
end
