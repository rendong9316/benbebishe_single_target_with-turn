function probe_mc50()
% 50 种子大规模 MC 统计（E9_baseline vs E10_optimal，同种子配对）
root = 'D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn';
cd(root);
fid = fopen(fullfile(root,'_audit','mc50.txt'),'w');

scenes = {'adsb_0002','multi_cross'};
cfgA = 'w07_15';   cfgB = 'w005_02';
dirA = 'E9_baseline'; dirB = 'E10_optimal';

for s = 1:numel(scenes)
    A = collect(root, dirA, scenes{s}, cfgA);
    B = collect(root, dirB, scenes{s}, cfgB);
    fprintf(fid,'\n################ %s ################\n', scenes{s});
    fprintf(fid,'  baseline 0.7/1.5 : n=%d\n', numel(A.R1));
    fprintf(fid,'  optimal  0.05/0.2: n=%d\n', numel(B.R1));
    if numel(A.R1)==0 || numel(B.R1)==0, continue; end

    % 各配置统计
    show(fid, '0.7/1.5', A);
    show(fid, '0.05/0.2', B);

    % 配对（按种子对齐）
    common = intersect(A.seed, B.seed);
    n = numel(common);
    if n < 5, fprintf(fid,'  paired n too small\n'); continue; end
    ia = arrayfun(@(x) find(A.seed==x,1), common);
    ib = arrayfun(@(x) find(B.seed==x,1), common);
    gA = 100*(min(A.R1(ia),A.R2(ia))-A.SCC(ia))./min(A.R1(ia),A.R2(ia));
    gB = 100*(min(B.R1(ib),B.R2(ib))-B.SCC(ib))./min(B.R1(ib),B.R2(ib));

    fprintf(fid,'\n  --- paired t-test (n=%d) ---\n', n);
    [~,p1] = ttest(A.R1(ia), B.R1(ib));
    [~,p2] = ttest(A.SCC(ia), B.SCC(ib));
    [~,p3] = ttest(A.rho(ia), B.rho(ib));
    [~,p4] = ttest(A.nees1(ia), B.nees1(ib));
    [~,p5] = ttest(gA, gB);
    fprintf(fid,'    R1    : %+6.1f%%  p=%.2e\n', 100*(mean(A.R1(ia))-mean(B.R1(ib)))/mean(A.R1(ia)), p1);
    fprintf(fid,'    SCC   : %+6.1f%%  p=%.2e\n', 100*(mean(A.SCC(ia))-mean(B.SCC(ib)))/mean(A.SCC(ia)), p2);
    fprintf(fid,'    rho   : %+6.3f   p=%.2e\n', mean(A.rho(ia))-mean(B.rho(ib)), p3);
    fprintf(fid,'    NEES  : %+6.2f   p=%.2e\n', mean(A.nees1(ia))-mean(B.nees1(ib)), p4);
    fprintf(fid,'    gain  : %+6.1f pp  p=%.2e\n', mean(gB)-mean(gA), p5);

    % 效应量 Cohen dz
    dz = @(a,b) mean(a-b)/std(a-b);
    fprintf(fid,'    effect size dz: R1=%.2f  SCC=%.2f  rho=%.2f  gain=%.2f\n', ...
        dz(A.R1(ia),B.R1(ib)), dz(A.SCC(ia),B.SCC(ib)), dz(A.rho(ia),B.rho(ib)), dz(gA,gB));

    % 增益 CI
    ciA = 1.96*std(gA)/sqrt(n); ciB = 1.96*std(gB)/sqrt(n);
    fprintf(fid,'    gain 95%% CI: baseline [%.1f, %.1f]  optimal [%.1f, %.1f]\n', ...
        mean(gA)-ciA, mean(gA)+ciA, mean(gB)-ciB, mean(gB)+ciB);
    fprintf(fid,'    negative-gain seeds: baseline %d/%d  optimal %d/%d\n', ...
        sum(gA<0), n, sum(gB<0), n);
end
fclose(fid);
disp(fileread(fullfile(root,'_audit','mc50.txt')));
end

function show(fid, name, V)
g = 100*(min(V.R1,V.R2)-V.SCC)./min(V.R1,V.R2);
n = numel(V.R1);
fprintf(fid,'    %-9s R1=%.2f±%.2f  R2=%.2f±%.2f  SCC=%.2f±%.2f  rho=%+.3f±%.3f  NEES=%.2f±%.2f  gain=%+.1f%%±%.1f\n', ...
    name, mean(V.R1), std(V.R1), mean(V.R2), std(V.R2), mean(V.SCC), std(V.SCC), ...
    mean(V.rho), std(V.rho), mean(V.nees1), std(V.nees1), mean(g), std(g));
if n>1
    ci = 1.96*std(g)/sqrt(n);
    fprintf(fid,'              gain 95%% CI [%+.1f, %+.1f]  min=%+.1f max=%+.1f  负增益 %d/%d\n', ...
        mean(g)-ci, mean(g)+ci, min(g), max(g), sum(g<0), n);
end
end

function V = collect(root, dirname, scene, cfg)
V = struct('seed',[],'R1',[],'R2',[],'SCC',[],'rho',[],'nees1',[]);
d = fullfile(root,'results','paper',dirname);
if ~exist(d,'dir'), return; end
files = dir(fullfile(d,'*.mat'));
for i=1:numel(files)
    nm = files(i).name;
    if isempty(strfind(nm, scene)) || isempty(strfind(nm, cfg)), continue; end
    S = load(fullfile(d,nm));
    if ~isfield(S,'m') || ~isfield(S.m,'consistency'), continue; end
    m = S.m; c = m.consistency;
    sd = NaN; if isfield(S,'meta'), sd = S.meta.seed; end
    V.seed(end+1)=sd; V.R1(end+1)=m.rmse.R1_rms; V.R2(end+1)=m.rmse.R2_rms;
    V.SCC(end+1)=getm(m,'SCC'); V.rho(end+1)=c.rho; V.nees1(end+1)=c.nees_R1;
end
end

function v = getm(m,nm)
v=NaN;
for i=1:numel(m.rmse.fusion)
    if strcmp(m.rmse.fusion(i).method,nm), v=m.rmse.fusion(i).rms; return; end
end
end
