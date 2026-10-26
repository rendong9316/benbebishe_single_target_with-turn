function probe_e5_detail()
% 分析 E5 四个配置对各融合算法的影响
root = 'D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn';
cd(root);
d = fullfile(root,'results','paper','E5_fusion');
fid = fopen(fullfile(root,'_audit','e5_detail.txt'),'w');
files = dir(fullfile(d,'*.mat'));
fprintf(fid,'%-22s %7s %7s %7s %7s %7s %7s %8s\n','file','R1','R2','SCC','BC','CI','FCI','cf_gain');
for i=1:numel(files)
    S = load(fullfile(d,files(i).name));
    if ~isfield(S,'m'), continue; end
    m = S.m;
    f = m.rmse.fusion;
    g = @(nm) getf(f,nm);
    fprintf(fid,'%-22s %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f %8.1f\n', ...
        strrep(files(i).name,'.mat',''), m.rmse.R1_rms, m.rmse.R2_rms, ...
        g('SCC'), g('BC'), g('CI'), g('FCI'), m.common_frame.fused);
end
% 打印 NEES / rho
fprintf(fid,'\n%-22s %8s %8s %8s %8s %8s\n','file','NEES_R1','NEES_R2','rho','indep','opt');
for i=1:numel(files)
    S = load(fullfile(d,files(i).name));
    if ~isfield(S,'m') || ~isfield(S.m,'consistency'), continue; end
    c = S.m.consistency;
    fprintf(fid,'%-22s %8.2f %8.2f %+8.3f %8.2f %8.2f\n', ...
        strrep(files(i).name,'.mat',''), c.nees_R1, c.nees_R2, c.rho, c.indep_rms, c.opt_rms);
end
fclose(fid);
disp(fileread(fullfile(root,'_audit','e5_detail.txt')));
end

function v = getf(f, nm)
v = NaN;
for i=1:numel(f)
    if strcmp(f(i).method, nm), v = f(i).rms; return; end
end
end
