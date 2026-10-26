function probe_gap_rank()
% 按 gap 排序看 10 个种子的特征，找"融合最需要改进"的规律
root = 'D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn';
cd(root);
fid = fopen(fullfile(root,'_audit','gap_rank.txt'),'w');
d = fullfile(root,'results','paper','E1_small');
files = dir(fullfile(d,'adsb_0002*.mat'));

R = [];
% 直接构造表
rows = [];
for i=1:numel(files)
    S = load(fullfile(d,files(i).name));
    if ~isfield(S,'m') || ~isfield(S.m,'consistency'), continue; end
    m = S.m; c = m.consistency;
    gap = NaN;
    if isfinite(c.opt_rms) && c.opt_rms>0
        gap = 100*(m.common_frame.fused - c.opt_rms)/c.opt_rms;
    end
    sd = NaN;
    if isfield(S,'meta'), sd = S.meta.seed; end
    rows(end+1,:) = [sd, m.rmse.R1_rms, m.rmse.R2_rms, getm(m,'SCC'), gap, c.rho, c.nees_R1, c.nees_R2, c.opt_rms];
end
[~,ord] = sort(rows(:,5),'descend');
rows = rows(ord,:);
fprintf(fid,'%-5s %7s %7s %7s %7s %8s %8s %8s %8s\n','seed','R1','R2','SCC','gap%%','rho','NEES1','NEES2','opt');
for i=1:size(rows,1)
    fprintf(fid,'%-5.0f %7.2f %7.2f %7.2f %+7.1f %+8.3f %8.2f %8.2f %8.2f\n', rows(i,:));
end
fprintf(fid,'\nmean %7.2f %7.2f %7.2f %+7.1f %+8.3f %8.2f %8.2f %8.2f\n', mean(rows,1,'omitnan'));
% 相关性：gap 与 |R1-R2|
dg = abs(rows(:,2)-rows(:,3));
cc = corrcoef(dg, rows(:,5));
fprintf(fid,'\ncorr(|R1-R2|, gap) = %.3f\n', cc(1,2));
cc2 = corrcoef(rows(:,6), rows(:,5));
fprintf(fid,'corr(rho, gap)     = %.3f\n', cc2(1,2));
fclose(fid);
disp(fileread(fullfile(root,'_audit','gap_rank.txt')));
end

function v = getm(m,nm)
v=NaN;
for i=1:numel(m.rmse.fusion)
    if strcmp(m.rmse.fusion(i).method,nm), v=m.rmse.fusion(i).rms; return; end
end
end
