function probe_mc50_deep()
% 50 种子 MC 的深度分析
%  1) 增益分布的分位数（不只均值±std）
%  2) 极端离群种子的定位（multi_cross baseline 有 -226% 的种子）
%  3) 机理检验：逐种子 Δrho 与 Δgain 的相关性（因果证据，不是相关叙事）
%  4) 单站改善的效应量对比（修正"单站不显著"的说法）

root = 'D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn';
out  = fullfile(root, '_audit', 'mc50_deep.txt');
fid  = fopen(out, 'w', 'n', 'UTF-8');
pr = @(varargin) fprintf(fid, varargin{:});

exps   = {'E9_baseline','E10_optimal'};
cfgs   = {'w07_15','w005_02'};
scenes = {'adsb_0002','multi_cross'};
MAXS   = 200;

for si = 1:numel(scenes)
    sc = scenes{si};
    pr('\n################ %s ################\n', sc);

    % ---- 收集：按 seed 索引的数组 ----
    M = struct();
    for ci = 1:numel(cfgs)
        A = struct('seed',[], 'R1',[], 'R2',[], 'SCC',[], 'rho',[], 'NEES',[], 'opt',[]);
        A.R1 = nan(MAXS,1); A.R2 = A.R1; A.SCC = A.R1;
        A.rho = A.R1; A.NEES = A.R1; A.opt = A.R1;
        for ei = 1:numel(exps)
            d = fullfile(root,'results','paper',exps{ei});
            files = dir(fullfile(d, [sc '__' cfgs{ci} '__s*.mat']));
            for k = 1:numel(files)
                tok = regexp(files(k).name, [sc '__' cfgs{ci} '__s(\d+)\.mat'], 'tokens');
                if isempty(tok), continue; end
                sd = str2double(tok{1}{1});
                if sd < 1 || sd > MAXS, continue; end
                S = load(fullfile(d, files(k).name));
                if ~isfield(S,'m'), continue; end
                m = S.m;
                A.R1(sd) = m.rmse.R1_rms;  A.R2(sd) = m.rmse.R2_rms;
                A.SCC(sd) = getm(m,'SCC');
                A.rho(sd) = m.consistency.rho;
                A.NEES(sd) = m.consistency.nees_R1;
                A.opt(sd) = m.consistency.opt_rms;
            end
        end
        M.(cfgs{ci}) = A;
    end

    B = M.(cfgs{1});  O = M.(cfgs{2});
    seeds = find(isfinite(B.SCC) & isfinite(O.SCC))';
    n = numel(seeds);
    pr('  paired seeds: %d  (range %d-%d)\n', n, min(seeds), max(seeds));

    bestB = min(B.R1(seeds), B.R2(seeds));
    bestO = min(O.R1(seeds), O.R2(seeds));
    gB = 100*(bestB - B.SCC(seeds))./bestB;
    gO = 100*(bestO - O.SCC(seeds))./bestO;
    rB = B.rho(seeds);  rO = O.rho(seeds);

    % ---- 1) 分位数 ----
    pr('\n  [1] gain distribution (%% vs best single station)\n');
    pr('      %-10s %7s %7s %7s %7s %7s %7s %7s\n','config','p5','p25','median','p75','p95','mean','sd');
    pr('      %-10s %7.1f %7.1f %7.1f %7.1f %7.1f %7.1f %7.1f\n','0.7/1.5', ...
        prctile(gB,5),prctile(gB,25),median(gB),prctile(gB,75),prctile(gB,95),mean(gB),std(gB));
    pr('      %-10s %7.1f %7.1f %7.1f %7.1f %7.1f %7.1f %7.1f\n','0.05/0.2', ...
        prctile(gO,5),prctile(gO,25),median(gO),prctile(gO,75),prctile(gO,95),mean(gO),std(gO));
    pr('      optimal > baseline: %d/%d seeds;  optimal gain > 15%%: %d/%d;  > 20%%: %d/%d\n', ...
        sum(gO>gB), n, sum(gO>15), n, sum(gO>20), n);

    % ---- 2) 离群种子 ----
    pr('\n  [2] outliers\n');
    flag = find(abs(gB)>50 | abs(gO)>50 | B.SCC(seeds) > 2*median(B.SCC(seeds)));
    if isempty(flag)
        pr('      none\n');
    else
        pr('      %6s %8s %8s %8s %8s %8s %8s %6s\n','seed','SCC_b','SCC_o','rho_b','rho_o','gain_b','gain_o','n_b');
        for i = flag'
            pr('      %6d %8.2f %8.2f %8.3f %8.3f %8.1f %8.1f\n', ...
                seeds(i), B.SCC(seeds(i)), O.SCC(seeds(i)), rB(i), rO(i), gB(i), gO(i));
        end
    end

    % ---- 3) 机理检验 ----
    dRho = rO - rB;   dGain = gO - gB;
    ok  = isfinite(dRho) & isfinite(dGain);
    ok2 = ok & abs(dGain) < 50;
    pr('\n  [3] mechanism test: does dRho predict dGain across seeds?\n');
    pr('      corr(dRho,dGain) = %+.3f  (n=%d, all)\n', corr(dRho(ok),dGain(ok)), sum(ok));
    pr('      corr(dRho,dGain) = %+.3f  (n=%d, |dGain|>=50 removed)\n', corr(dRho(ok2),dGain(ok2)), sum(ok2));
    pr('      mean dRho=%+.4f  mean dGain=%+.1f pp\n', mean(dRho(ok)), mean(dGain(ok)));
    if sum(ok2) > 3
        pf = polyfit(dRho(ok2), dGain(ok2), 1);
        pr('      OLS: dGain = %.1f*dRho %+.1f   => rho -0.1 => gain %+.1f pp\n', ...
            pf(1), pf(2), -0.1*pf(1));
    end

    % ---- 4) 效应量 ----
    pr('\n  [4] effect size (paired, baseline vs optimal)\n');
    vars = {'R1','R2','SCC','rho','NEES'};
    for vi = 1:numel(vars)
        v = vars{vi};
        a = B.(v)(seeds);  b2 = O.(v)(seeds);
        okk = isfinite(a) & isfinite(b2);
        d = a(okk) - b2(okk);
        dz = mean(d)/std(d);
        [~,pv] = ttest(a(okk), b2(okk));
        pr('      %-6s base=%8.3f  opt=%8.3f  diff=%+8.3f  dz=%+6.2f  p=%.2e\n', ...
            v, mean(a(okk)), mean(b2(okk)), mean(d), dz, pv);
    end
end

fclose(fid);
fprintf('WROTE %s\n', out);
end

function v = getm(m, nm)
v = NaN;
for i=1:numel(m.rmse.fusion)
    if strcmp(m.rmse.fusion(i).method, nm), v = m.rmse.fusion(i).rms; return; end
end
end
