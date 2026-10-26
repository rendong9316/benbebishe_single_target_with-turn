function probe_nees_dist()
% 多 seed 的 NEES / rho / gap 分布（E1_small）
root = 'D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn';
cd(root);
fid = fopen(fullfile(root,'_audit','nees_dist.txt'),'w');
d = fullfile(root,'results','paper','E1_small');
files = dir(fullfile(d,'*.mat'));

for sc = {'multi_cross','adsb_0002'}
    nr1=[]; nr2=[]; rh=[]; gp=[]; r1=[]; r2=[]; scc=[]; fz=[];
    for i=1:numel(files)
        if isempty(strfind(files(i).name, sc{1})), continue; end
        S = load(fullfile(d,files(i).name));
        if ~isfield(S,'m'), continue; end
        m = S.m;
        if ~isfield(m,'consistency'), continue; end
        c = m.consistency;
        nr1(end+1)=c.nees_R1; nr2(end+1)=c.nees_R2; rh(end+1)=c.rho;
        r1(end+1)=m.rmse.R1_rms; r2(end+1)=m.rmse.R2_rms;
        scc(end+1)=getm(m,'SCC');
        if isfinite(c.opt_rms) && c.opt_rms>0
            gp(end+1) = 100*(m.common_frame.fused - c.opt_rms)/c.opt_rms;
        end
    end
    n=numel(rh);
    fprintf(fid,'\n===== %s (n=%d) =====\n', sc{1}, n);
    if n==0, continue; end
    fprintf(fid,'  NEES_R1: mean=%.2f sd=%.2f  [%.2f, %.2f]\n', mean(nr1), std(nr1), min(nr1), max(nr1));
    fprintf(fid,'  NEES_R2: mean=%.2f sd=%.2f  [%.2f, %.2f]\n', mean(nr2), std(nr2), min(nr2), max(nr2));
    fprintf(fid,'  rho    : mean=%+.3f sd=%.3f  [%+.3f, %+.3f]\n', mean(rh), std(rh), min(rh), max(rh));
    fprintf(fid,'  R1     : mean=%.2f sd=%.2f\n', mean(r1), std(r1));
    fprintf(fid,'  R2     : mean=%.2f sd=%.2f\n', mean(r2), std(r2));
    fprintf(fid,'  SCC    : mean=%.2f sd=%.2f\n', mean(scc), std(scc));
    if ~isempty(gp)
        fprintf(fid,'  gap to opt: mean=%+.1f%% sd=%.1f  [%+.1f, %+.1f]\n', ...
            mean(gp), std(gp), min(gp), max(gp));
        fprintf(fid,'    gap<5%% seeds: %d/%d\n', sum(gp<5), numel(gp));
    end
    % 谁更准的统计
    fprintf(fid,'  R1 better than R2 in %d/%d seeds\n', sum(r1<r2), n);
end
fclose(fid);
disp(fileread(fullfile(root,'_audit','nees_dist.txt')));
end

function v = getm(m,nm)
v=NaN;
for i=1:numel(m.rmse.fusion)
    if strcmp(m.rmse.fusion(i).method,nm), v=m.rmse.fusion(i).rms; return; end
end
end
