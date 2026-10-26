function probe_e7_summary()
% E7 模型集实验的多 seed 统计 + 配对检验
root = 'D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn';
cd(root);
fid = fopen(fullfile(root,'_audit','e7_summary.txt'),'w');
d = fullfile(root,'results','paper','E7_model');
files = dir(fullfile(d,'*.mat'));

cfgs = {'w07_15','w02_07','w005_02'};
scenes = {'adsb_0002','multi_cross'};

for s = 1:numel(scenes)
    fprintf(fid,'\n########## %s ##########\n', scenes{s});
    fprintf(fid,'%-10s %5s %15s %15s %15s %15s %15s\n','cfg','n','R1','R2','SCC','rho','NEES_R1');
    D = struct();
    for c = 1:numel(cfgs)
        v = collect(d, files, scenes{s}, cfgs{c});
        if isempty(v.R1), continue; end
        D.(cfgs{c}) = v;
        fprintf(fid,'%-10s %5d %7.2f±%5.2f %7.2f±%5.2f %7.2f±%5.2f %+7.3f±%.3f %7.2f±%5.2f\n', ...
            cfgs{c}, numel(v.R1), mean(v.R1), std(v.R1), mean(v.R2), std(v.R2), ...
            mean(v.SCC), std(v.SCC), mean(v.rho), std(v.rho), mean(v.nees1), std(v.nees1));
    end
    % 增益
    fprintf(fid,'\n  gain (SCC vs best single):\n');
    for c = 1:numel(cfgs)
        if ~isfield(D,cfgs{c}), continue; end
        v = D.(cfgs{c});
        g = 100*(min(v.R1,v.R2)-v.SCC)./min(v.R1,v.R2);
        fprintf(fid,'    %-10s %+7.1f%% ± %4.1f   (n=%d, 负增益 %d)\n', ...
            cfgs{c}, mean(g), std(g), numel(g), sum(g<0));
    end
    % 配对检验：w005_02 vs w07_15
    if isfield(D,'w07_15') && isfield(D,'w005_02')
        a = D.w07_15; b = D.w005_02;
        n = min(numel(a.R1), numel(b.R1));
        if n >= 3
            [h1,p1] = ttest(a.R1(1:n), b.R1(1:n));
            [h2,p2] = ttest(a.SCC(1:n), b.SCC(1:n));
            [h3,p3] = ttest(a.rho(1:n), b.rho(1:n));
            [h4,p4] = ttest(a.nees1(1:n), b.nees1(1:n));
            fprintf(fid,'\n  paired t-test (0.7/1.5 vs 0.05/0.2, n=%d):\n', n);
            fprintf(fid,'    R1   : %+6.1f%%  p=%.4f  %s\n', 100*(mean(a.R1(1:n))-mean(b.R1(1:n)))/mean(a.R1(1:n)), p1, sig(h1));
            fprintf(fid,'    SCC  : %+6.1f%%  p=%.4f  %s\n', 100*(mean(a.SCC(1:n))-mean(b.SCC(1:n)))/mean(a.SCC(1:n)), p2, sig(h2));
            fprintf(fid,'    rho  : %+6.3f   p=%.4f  %s\n', mean(a.rho(1:n))-mean(b.rho(1:n)), p3, sig(h3));
            fprintf(fid,'    NEES : %+6.2f   p=%.4f  %s\n', mean(a.nees1(1:n))-mean(b.nees1(1:n)), p4, sig(h4));
        end
    end
end
fclose(fid);
disp(fileread(fullfile(root,'_audit','e7_summary.txt')));
end

function v = collect(d, files, scene, cfg)
v = struct('R1',[],'R2',[],'SCC',[],'rho',[],'nees1',[],'opt',[]);
for i=1:numel(files)
    nm = files(i).name;
    if isempty(strfind(nm, scene)) || isempty(strfind(nm, cfg)), continue; end
    S = load(fullfile(d,nm));
    if ~isfield(S,'m') || ~isfield(S.m,'consistency'), continue; end
    m = S.m; c = m.consistency;
    v.R1(end+1)=m.rmse.R1_rms; v.R2(end+1)=m.rmse.R2_rms;
    v.SCC(end+1)=getm(m,'SCC'); v.rho(end+1)=c.rho;
    v.nees1(end+1)=c.nees_R1; v.opt(end+1)=c.opt_rms;
end
end

function s = sig(h)
if h, s = '***'; else, s = 'n.s.'; end
end

function v = getm(m,nm)
v=NaN;
for i=1:numel(m.rmse.fusion)
    if strcmp(m.rmse.fusion(i).method,nm), v=m.rmse.fusion(i).rms; return; end
end
end
