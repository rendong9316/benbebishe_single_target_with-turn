% 基于缓存的分析：航迹长度分布 + 融合源占比 + 模型概率
root = 'D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn';
cd(root);
S = load(fullfile(root,'_audit','mc_current_geom.mat'));
r = S;

fprintf('\n### 1. R1 航迹长度分布 ###\n');
snaps = r.trackSnapshots_R1; nf = numel(snaps);
ids = zeros(1, numel(r.trackList_R1));
for i = 1:numel(r.trackList_R1), ids(i) = r.trackList_R1{i}.id; end
ids = unique(ids);
len = zeros(size(ids));
for j = 1:numel(ids)
    cnt = 0;
    for k = 1:nf
        tl = snaps{k}.trackList;
        for t = 1:numel(tl)
            if tl{t}.id == ids(j) && tl{t}.type ~= 7, cnt = cnt + 1; end
        end
    end
    len(j) = cnt;
end
fprintf('总航迹数=%d  长度均值=%.2f  中位=%.1f  最大=%d\n', numel(ids), mean(len), median(len), max(len));
bins = [1 2 3 5 10 20 40 55];
for e = 1:numel(bins)-1
    fprintf('  %2d-%2d 帧: %3d 条\n', bins(e), bins(e+1)-1, sum(len>=bins(e) & len<bins(e+1)));
end
fprintf('  >=55 帧: %d 条\n', sum(len>=55));
fprintf('  <=3 帧: %d 条 (%.1f%%)\n', sum(len<=3), 100*sum(len<=3)/numel(len));

fprintf('\n### 2. 融合源占比（SCC 方法）###\n');
mi = find(strcmp(r.method_names, 'SCC'), 1);
if isempty(mi), mi = 1; end
fs = r.all_fused_snapshots;
nb = 0; n1 = 0; n2 = 0; n0 = 0;
for p = 1:size(fs,1)
    sn = fs{p, mi};
    for k = 1:numel(sn)
        tl = sn{k}.trackList;
        if isempty(tl), n0 = n0 + 1; continue; end
        s = tl{1}.source;
        if strcmp(s,'both'), nb = nb + 1;
        elseif strcmp(s,'R1_only'), n1 = n1 + 1;
        else, n2 = n2 + 1; end
    end
end
fprintf('both=%d  R1_only=%d  R2_only=%d  空=%d  总=%d\n', nb, n1, n2, n0, nb+n1+n2+n0);
fprintf('双源融合帧占比=%.1f%%\n', 100*nb/(nb+n1+n2));

fprintf('\n### 3. 融合航迹的 ukf 字段（看是否存 mu）###\n');
sn1 = fs{1, mi}{10};
if ~isempty(sn1.trackList)
    disp(fieldnames(sn1.trackList{1})');
end

fprintf('\n### 4. 单站航迹 type 分布（末帧）###\n');
tl = r.trackSnapshots_R1{end}.trackList;
for t = 1:numel(tl)
    fprintf('  id=%d type=%d lon=%.2f lat=%.2f\n', tl{t}.id, tl{t}.type, tl{t}.lon, tl{t}.lat);
end

fprintf('\n### 5. R1 每条航迹的 age（若存在）###\n');
if ~isempty(r.trackList_R1) && isfield(r.trackList_R1{1}, 'age')
    ages = cellfun(@(s) s.age, r.trackList_R1);
    fprintf('age: 均值=%.1f 中位=%.1f 最大=%d\n', mean(ages), median(ages), max(ages));
else
    fprintf('无 age 字段\n');
end
