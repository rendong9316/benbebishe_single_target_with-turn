function probe_mc_final()
% 最终统计：最优配置 15 种子（E7 w005_02 s1-5 + E8 opt s6-15）
root = 'D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn';
cd(root);
fid = fopen(fullfile(root,'_audit','mc_final.txt'),'w');

scenes = {'adsb_0002','multi_cross'};
for s = 1:numel(scenes)
    V = collect_all(root, scenes{s});
    n = numel(V.R1);
    fprintf(fid,'\n########## %s  最优配置 0.05/0.2  (n=%d) ##########\n', scenes{s}, n);
    if n==0, continue; end
    g = 100*(min(V.R1,V.R2)-V.SCC)./min(V.R1,V.R2);
    fprintf(fid,'  R1     : %.2f ± %.2f\n', mean(V.R1), std(V.R1));
    fprintf(fid,'  R2     : %.2f ± %.2f\n', mean(V.R2), std(V.R2));
    fprintf(fid,'  SCC    : %.2f ± %.2f\n', mean(V.SCC), std(V.SCC));
    fprintf(fid,'  rho    : %+.3f ± %.3f   [%+.3f, %+.3f]\n', mean(V.rho), std(V.rho), min(V.rho), max(V.rho));
    fprintf(fid,'  NEES_R1: %.2f ± %.2f\n', mean(V.nees1), std(V.nees1));
    fprintf(fid,'  gain   : %+.1f%% ± %.1f   [%+.1f, %+.1f]   负增益 %d/%d\n', ...
        mean(g), std(g), min(g), max(g), sum(g<0), n);
    % 95% CI
    if n>1
        ci = 1.96*std(g)/sqrt(n);
        fprintf(fid,'  gain 95%% CI: [%+.1f, %+.1f]\n', mean(g)-ci, mean(g)+ci);
    end
end

% 与基线对比（仅同种子 1-5）
fprintf(fid,'\n########## 配对对比（种子 1-5）##########\n');
for s = 1:numel(scenes)
    A = collect_cfg(root, scenes{s}, 'w07_15');
    B = collect_cfg(root, scenes{s}, 'w005_02');
    n = min(numel(A.R1), numel(B.R1));
    if n < 3, continue; end
    [~,p1] = ttest(A.R1(1:n), B.R1(1:n));
    [~,p2] = ttest(A.SCC(1:n), B.SCC(1:n));
    [~,p3] = ttest(A.rho(1:n), B.rho(1:n));
    [~,p4] = ttest(A.nees1(1:n), B.nees1(1:n));
    gA = 100*(min(A.R1(1:n),A.R2(1:n))-A.SCC(1:n))./min(A.R1(1:n),A.R2(1:n));
    gB = 100*(min(B.R1(1:n),B.R2(1:n))-B.SCC(1:n))./min(B.R1(1:n),B.R2(1:n));
    fprintf(fid,'  %s (n=%d):\n', scenes{s}, n);
    fprintf(fid,'    R1   %+.1f%% p=%.4f | SCC %+.1f%% p=%.4f | rho %+.3f p=%.4f | NEES %+.2f p=%.4f\n', ...
        100*(mean(A.R1(1:n))-mean(B.R1(1:n)))/mean(A.R1(1:n)), p1, ...
        100*(mean(A.SCC(1:n))-mean(B.SCC(1:n)))/mean(A.SCC(1:n)), p2, ...
        mean(A.rho(1:n))-mean(B.rho(1:n)), p3, ...
        mean(A.nees1(1:n))-mean(B.nees1(1:n)), p4);
    fprintf(fid,'    gain: %.1f%% -> %.1f%%  (%+.1f pp)\n', mean(gA), mean(gB), mean(gB)-mean(gA));
end
fclose(fid);
disp(fileread(fullfile(root,'_audit','mc_final.txt')));
end

function V = collect_all(root, scene)
V = struct('R1',[],'R2',[],'SCC',[],'rho',[],'nees1',[]);
A = collect_cfg(root, scene, 'w005_02');
B = collect_cfg(root, scene, 'opt');
fn = fieldnames(V);
for i=1:numel(fn)
    V.(fn{i}) = [A.(fn{i}), B.(fn{i})];
end
end

function V = collect_cfg(root, scene, cfg)
V = struct('R1',[],'R2',[],'SCC',[],'rho',[],'nees1',[]);
for e = {'E7_model','E8_optimal'}
    d = fullfile(root,'results','paper',e{1});
    if ~exist(d,'dir'), continue; end
    files = dir(fullfile(d,'*.mat'));
    for i=1:numel(files)
        nm = files(i).name;
        if isempty(strfind(nm, scene)) || isempty(strfind(nm, cfg)), continue; end
        S = load(fullfile(d,nm));
        if ~isfield(S,'m') || ~isfield(S.m,'consistency'), continue; end
        m = S.m; c = m.consistency;
        V.R1(end+1)=m.rmse.R1_rms; V.R2(end+1)=m.rmse.R2_rms;
        V.SCC(end+1)=getm(m,'SCC'); V.rho(end+1)=c.rho; V.nees1(end+1)=c.nees_R1;
    end
end
end

function v = getm(m,nm)
v=NaN;
for i=1:numel(m.rmse.fusion)
    if strcmp(m.rmse.fusion(i).method,nm), v=m.rmse.fusion(i).rms; return; end
end
end
