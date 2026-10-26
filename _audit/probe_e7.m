function probe_e7()
% E7 模型集实验：看 rho / NEES 是否随模型集变化
root = 'D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn';
cd(root);
fid = fopen(fullfile(root,'_audit','e7.txt'),'w');
d = fullfile(root,'results','paper','E7_model');
files = dir(fullfile(d,'*.mat'));
fprintf(fid,'%-26s %7s %7s %7s %8s %8s %8s %8s %8s\n', ...
    'file','R1','R2','SCC','gain%','rho','NEES1','NEES2','opt');
rows = {};
for i=1:numel(files)
    S = load(fullfile(d,files(i).name));
    if ~isfield(S,'m') || ~isfield(S.m,'consistency'), continue; end
    m = S.m; c = m.consistency;
    best = min(m.rmse.R1_rms, m.rmse.R2_rms);
    sc = getm(m,'SCC');
    g = 100*(best-sc)/best;
    fprintf(fid,'%-26s %7.2f %7.2f %7.2f %+8.1f %+8.3f %8.2f %8.2f %8.2f\n', ...
        strrep(files(i).name,'.mat',''), m.rmse.R1_rms, m.rmse.R2_rms, sc, g, ...
        c.rho, c.nees_R1, c.nees_R2, c.opt_rms);
    rows{end+1} = {files(i).name, m.rmse.R1_rms, m.rmse.R2_rms, sc, g, c.rho, c.nees_R1, c.opt_rms}; %#ok<AGROW>
end
fclose(fid);
disp(fileread(fullfile(root,'_audit','e7.txt')));
end

function v = getm(m,nm)
v=NaN;
for i=1:numel(m.rmse.fusion)
    if strcmp(m.rmse.fusion(i).method,nm), v=m.rmse.fusion(i).rms; return; end
end
end
