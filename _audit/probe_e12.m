function probe_e12()
% E12_sweep 分析：模型集 6 档 × 15 种子
% 核心问题：转弯率是不是越小越好？如果单调递减，说明杠杆是"关掉 CT 模型"而非"匹配真实机动"

d    = fullfile(pwd,'..','results','paper','E12_sweep');
out  = 'e12.txt';
fid  = fopen(out, 'w', 'n', 'UTF-8');
pr = @(varargin) fprintf(fid, varargin{:});

cfgs = {'s07_15','s03_07','s02_04','s01_03','s005_02','s002_005'};
name = {'0.7/1.5','0.3/0.7','0.2/0.4','0.1/0.3','0.05/0.2','0.02/0.05'};
NS   = 15;

% 收集
R1 = nan(numel(cfgs),NS); R2 = R1; SC = R1; RH = R1; NE = R1;
for ci = 1:numel(cfgs)
    for s = 1:NS
        f = fullfile(d, sprintf('adsb_0002__%s__s%02d.mat', cfgs{ci}, s));
        if ~isfile(f), continue; end
        S = load(f);
        if ~isfield(S,'m'), continue; end
        m = S.m;
        R1(ci,s) = m.rmse.R1_rms;  R2(ci,s) = m.rmse.R2_rms;
        SC(ci,s) = getm(m,'SCC');
        RH(ci,s) = m.consistency.rho;
        NE(ci,s) = m.consistency.nees_R1;
    end
end

pr('################ E12_sweep: 模型集细扫 (adsb_0002, %d seeds) ################\n\n', NS);
pr('  %-10s %14s %14s %14s %14s %10s %8s\n', ...
   'CT set','R1','SCC','rho','NEES','gain','n');
pr('  %s\n', repmat('-',1,90));
for ci = 1:numel(cfgs)
    ok = isfinite(SC(ci,:));
    n = sum(ok);
    if n==0, continue; end
    best = min(R1(ci,ok), R2(ci,ok));
    g = 100*(best - SC(ci,ok))./best;
    pr('  %-10s %6.2f±%5.2f %6.2f±%5.2f %+6.3f±%5.3f %6.2f±%4.2f %+6.1f%% %8d\n', ...
       name{ci}, mean(R1(ci,ok)), std(R1(ci,ok)), mean(SC(ci,ok)), std(SC(ci,ok)), ...
       mean(RH(ci,ok)), std(RH(ci,ok)), mean(NE(ci,ok)), std(NE(ci,ok)), mean(g), n);
end

% 配对检验：每一档 vs 0.05/0.2
pr('\n  paired t-test vs 0.05/0.2 (SCC):\n');
ref = 5;
for ci = [1 2 3 4 6]
    ok = isfinite(SC(ci,:)) & isfinite(SC(ref,:));
    if sum(ok) < 3, continue; end
    a = SC(ci,ok); b = SC(ref,ok);
    [~,p] = ttest(a,b);
    d = mean(a-b)/std(a-b);
    pr('    %-10s vs 0.05/0.2 : diff=%+6.2f km  (%+5.1f%%)  p=%.2e  dz=%+.2f\n', ...
       name{ci}, mean(a-b), 100*mean(a-b)/mean(b), p, d);
end

% 最优档判定
mS = mean(SC,2,'omitnan');
[mv, mi] = min(mS);
pr('\n  best SCC = %.2f km at CT set %s\n', mv, name{mi});
pr('  monotonic decrease?  ');
pr('%s\n', mat2str(round(mS',2)));

% rho 单调性
pr('\n  rho by config: ');
for ci=1:numel(cfgs), pr('%.3f ', mean(RH(ci,:),'omitnan')); end
pr('\n');

fclose(fid);
fprintf('WROTE %s\n', out);
end

function v = getm(m, nm)
v = NaN;
for i=1:numel(m.rmse.fusion)
    if strcmp(m.rmse.fusion(i).method, nm), v = m.rmse.fusion(i).rms; return; end
end
end
