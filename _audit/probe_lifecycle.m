% 航迹生命周期审计：航迹长度分布 + 滤波是否真的在收敛
root = 'D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn';
cd(root); addpath(genpath(root));

r = run('multi_cross');

fprintf('\n===== trackList_R1 字段 =====\n');
disp(fieldnames(r.trackList_R1{1})');
fprintf('\n===== trackList_R1 条数 = %d =====\n', numel(r.trackList_R1));

% 逐条航迹统计长度（用快照里出现该 ID 的帧数衡量）
snaps = r.trackSnapshots_R1; nf = numel(snaps);
ids = []; for i=1:numel(r.trackList_R1), ids(end+1) = r.trackList_R1{i}.id; end
ids = unique(ids);
len = zeros(size(ids)); firstf = nan(size(ids)); lastf = nan(size(ids));
for j = 1:numel(ids)
    cnt = 0; f0 = nan; f1 = nan;
    for k = 1:nf
        tl = snaps{k}.trackList;
        for t = 1:numel(tl)
            if tl{t}.id == ids(j) && tl{t}.type ~= 7
                cnt = cnt + 1; if isnan(f0), f0 = k; end; f1 = k;
            end
        end
    end
    len(j) = cnt; firstf(j) = f0; lastf(j) = f1;
end

fprintf('\n===== R1 航迹长度分布 =====\n');
fprintf('总航迹数=%d  长度均值=%.2f 中位=%.1f 最大=%d 最小=%d\n', ...
    numel(ids), mean(len), median(len), max(len), min(len));
edges = [0 2 5 10 20 40 60];
for e = 1:numel(edges)-1
    fprintf('  长度 %2d-%2d 帧: %3d 条\n', edges(e)+1, edges(e+1), sum(len>edges(e) & len<=edges(e+1)));
end
fprintf('  长度 >60 帧: %d 条\n', sum(len>60));
fprintf('  长度 <=2 帧: %d 条 (占 %.1f%%)\n', sum(len<=2), 100*sum(len<=2)/numel(len));

fprintf('\n===== 长航迹（>=20帧）明细 =====\n');
idx = find(len>=20);
for j = idx
    fprintf('  id=%3d 长度=%2d 帧区间=[%d,%d]\n', ids(j), len(j), firstf(j), lastf(j));
end

fprintf('\n===== diag_R1 =====\n');
try, disp(r.diag_R1); catch, fprintf('no diag\n'); end

save(fullfile(root,'_audit','mc_current_geom.mat'), '-struct', 'r', '-v7.3');
fprintf('\nCACHE SAVED\n');
