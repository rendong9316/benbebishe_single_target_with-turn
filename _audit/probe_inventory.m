% probe_inventory.m — 全量实验账目汇总
% 遍历 results/paper 下所有实验单元，提取关键指标，输出统一 CSV 与分组统计。
% 目的：为论文写作提供一份可核对的完整账目，避免引用过时或单种子的数字。
% 只读：不改动源码
r0 = fileparts(pwd);
addpath(r0);
for d = {'config','simulation','ukf','tracker','fusion','evaluation','utils','visualization'}
    addpath(fullfile(r0, d{1}));
end

base = fullfile(pwd, '..', 'results', 'paper');
dirs = dir(base);
dirs = dirs([dirs.isdir]);
dirs = dirs(~ismember({dirs.name}, {'.','..'}));

outCsv = 'inventory.csv';
outTxt = 'inventory.txt';
fidC = fopen(outCsv, 'w', 'n', 'UTF-8');
fprintf(fidC, ['exp,cfg,scene,seed,R1,R2,SCC,rho,NEES1,match,coverage,' ...
               'cf_R1,cf_R2,cf_fused,cf_gain,n_frames\n']);

rows = {};   % {exp,cfg,scene,seed,R1,R2,SCC,rho,NEES1,match,cov,cfR1,cfR2,cfF,cfG,nf}
nFail = 0;

for di = 1:numel(dirs)
    expName = dirs(di).name;
    files = dir(fullfile(base, expName, '*.mat'));
    if isempty(files), continue; end
    nok = 0;
    for fi = 1:numel(files)
        f = fullfile(base, expName, files(fi).name);
        try
            S = load(f, 'meta', 'm');
            m = S.m;
            cfg = ''; scene = ''; sd = NaN;
            if isfield(S, 'meta')
                mt = S.meta;
                if isfield(mt,'cfg'),   cfg = char(string(mt.cfg));     end
                if isfield(mt,'scene'), scene = char(string(mt.scene)); end
                if isfield(mt,'seed'),  sd = mt.seed;                   end
            end
            R1 = getf(m, 'rmse', 'R1_rms');
            R2 = getf(m, 'rmse', 'R2_rms');
            SCC = fusion_rms(m, 'SCC');
            rho = getf2(m, 'consistency', 'rho');
            nees = getf2(m, 'consistency', 'nees_R1');
            match = getf2(m, 'match', 'value');
            cov = getf2(m, 'coverage', 'coverage');
            cfR1 = getf2(m, 'common_frame', 'R1');
            cfR2 = getf2(m, 'common_frame', 'R2');
            cfF  = getf2(m, 'common_frame', 'fused');
            cfG  = getf2(m, 'common_frame', 'gain');
            nf   = getf2(m, 'meta', 'n_frames');
            rows(end+1,:) = {expName, cfg, scene, sd, R1, R2, SCC, rho, nees, ...
                             match, cov, cfR1, cfR2, cfF, cfG, nf}; %#ok<AGROW>
            fprintf(fidC, '%s,%s,%s,%g,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%g\n', ...
                expName, cfg, scene, sd, R1, R2, SCC, rho, nees, match, cov, ...
                cfR1, cfR2, cfF, cfG, nf);
            nok = nok + 1;
        catch
            nFail = nFail + 1;
        end
    end
    fprintf('  %-16s %4d 单元\n', expName, nok);
end
fclose(fidC);

% ---- 分组统计 ----
fid = fopen(outTxt, 'w', 'n', 'UTF-8');
pr = @(varargin) fprintf(fid, varargin{:});
pr('=== 全量实验账目（共 %d 单元，%d 个读取失败）===\n\n', size(rows,1), nFail);

keys = unique(cellfun(@(a,b,c) sprintf('%s|%s|%s', a, b, c), ...
    rows(:,1), rows(:,2), rows(:,3), 'UniformOutput', false), 'stable');

pr('%-16s %-14s %-12s %4s %16s %16s %16s %14s %8s\n', ...
    'exp','cfg','scene','n','R1','SCC','rho','NEES1','gain%');
pr('%s\n', repmat('-', 1, 116));
for ki = 1:numel(keys)
    sel = strcmp(cellfun(@(a,b,c) sprintf('%s|%s|%s', a, b, c), ...
        rows(:,1), rows(:,2), rows(:,3), 'UniformOutput', false), keys{ki});
    sub = rows(sel,:);
    R1 = cell2mat(sub(:,5));  SCC = cell2mat(sub(:,7));
    rho = cell2mat(sub(:,8)); NE = cell2mat(sub(:,9));
    gain = 100*(min(cell2mat(sub(:,5)), cell2mat(sub(:,6))) - SCC) ./ ...
        min(cell2mat(sub(:,5)), cell2mat(sub(:,6)));
    parts = strsplit(keys{ki}, '|');
    pr('%-16s %-14s %-12s %4d %7.3f±%5.3f %7.3f±%5.3f %6.3f±%5.3f %6.2f±%5.2f %7.2f\n', ...
        parts{1}, parts{2}, parts{3}, numel(R1), ...
        mean(R1), std(R1), mean(SCC), std(SCC), ...
        mean(rho), std(rho), mean(NE), std(NE), mean(gain));
end
fclose(fid);
fprintf('WROTE %s / %s\n', outCsv, outTxt);

% ---- 局部函数 ----
function v = getf(m, f1, f2)
v = NaN;
if isfield(m, f1) && isfield(m.(f1), f2), v = m.(f1).(f2); end
end

function v = getf2(m, f1, f2)
v = getf(m, f1, f2);
end

function v = fusion_rms(m, name)
v = NaN;
if ~isfield(m, 'rmse') || ~isfield(m.rmse, 'fusion'), return; end
F = m.rmse.fusion;
for i = 1:numel(F)
    if strcmp(F(i).method, name), v = F(i).rms; return; end
end
end
