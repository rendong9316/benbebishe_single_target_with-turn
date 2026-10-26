function probe_final()
% 最终汇总：100 种子 MC + 中间档 + 高密度
d0  = fullfile(pwd,'..','results','paper');
fid = fopen('final.txt','w','n','UTF-8');
pr  = @(varargin) fprintf(fid, varargin{:});

%% ============ 1. 三档模型集，100 种子（adsb_0002）============
pr('################ 1. 模型集三档 · 100 种子 · adsb_0002 ################\n\n');
specs = {'E9_baseline','w07_15','0.7/1.5'; 'E11_mid','w02_07','0.2/0.7'; 'E10_optimal','w005_02','0.05/0.2'};
pr('  %-9s %6s %14s %14s %14s %12s %12s %10s\n', ...
   'CT set','n','R1','SCC','rho','NEES','gain','neg');
for i = 1:3
  V = collect(d0, specs{i,1}, 'adsb_0002', specs{i,2});
  best = min(V.R1, V.R2);
  g = 100*(best - V.SCC)./best;
  pr('  %-9s %6d %6.2f±%5.2f %6.2f±%5.2f %+6.3f±%5.3f %5.2f±%4.2f %+6.1f%%±%4.1f %6d/%d\n', ...
     specs{i,3}, numel(V.SCC), mean(V.R1),std(V.R1), mean(V.SCC),std(V.SCC), ...
     mean(V.rho),std(V.rho), mean(V.NEES),std(V.NEES), mean(g),std(g), sum(g<0), numel(g));
end

%% ============ 2. 高密度 adsb_0608（28 架）5 种子 ============
pr('\n################ 2. 高密度 adsb_0608（28 架并发）· 5 种子 ################\n\n');
pr('  %-9s %6s %14s %14s %14s %12s %10s %10s\n', ...
   'CT set','n','R1','SCC','rho','gain','match','precision');
for cf = {'w07_15','w005_02'}
  V = collect(d0, 'E13_dense', 'adsb_0608', cf{1});
  if isempty(V.SCC), continue; end
  best = min(V.R1, V.R2);
  g = 100*(best - V.SCC)./best;
  nm = '0.7/1.5'; if strcmp(cf{1},'w005_02'), nm='0.05/0.2'; end
  pr('  %-9s %6d %6.2f±%5.2f %6.2f±%5.2f %+6.3f±%5.3f %+6.1f%%±%4.1f %8.1f%% %8.1f%%\n', ...
     nm, numel(V.SCC), mean(V.R1),std(V.R1), mean(V.SCC),std(V.SCC), ...
     mean(V.rho),std(V.rho), mean(g),std(g), 100*mean(V.match), 100*mean(V.prec));
end

% 配对检验
A = collect(d0,'E13_dense','adsb_0608','w07_15');
B = collect(d0,'E13_dense','adsb_0608','w005_02');
n = min(numel(A.SCC), numel(B.SCC));
if n >= 2
  a = A.SCC(1:n); b = B.SCC(1:n);
  [~,p1] = ttest(a,b);
  [~,p2] = ttest(A.rho(1:n), B.rho(1:n));
  pr('\n  paired (n=%d): SCC %+6.2f km (%+5.1f%%) p=%.3f | rho %+.3f p=%.3f\n', ...
     n, mean(a-b), 100*mean(a-b)/mean(a), p1, mean(A.rho(1:n)-B.rho(1:n)), p2);
end

%% ============ 3. 密度效应：6 架 vs 28 架 ============
pr('\n################ 3. 密度效应（同为 0.05/0.2 配置）################\n\n');
V6  = collect(d0,'E10_optimal','adsb_0002','w005_02');
V28 = collect(d0,'E13_dense', 'adsb_0608','w005_02');
g6  = 100*(min(V6.R1,V6.R2)-V6.SCC)./min(V6.R1,V6.R2);
g28 = 100*(min(V28.R1,V28.R2)-V28.SCC)./min(V28.R1,V28.R2);
pr('  %-10s %6s %10s %10s %10s %10s %10s\n','scene','n','R1','SCC','rho','gain','coverage');
pr('  %-10s %6d %10.2f %10.2f %+10.3f %+9.1f%% %9.1f%%\n','6 架',numel(V6.SCC), ...
   mean(V6.R1), mean(V6.SCC), mean(V6.rho), mean(g6), 100*mean(V6.cov));
pr('  %-10s %6d %10.2f %10.2f %+10.3f %+9.1f%% %9.1f%%\n','28 架',numel(V28.SCC), ...
   mean(V28.R1), mean(V28.SCC), mean(V28.rho), mean(g28), 100*mean(V28.cov));

%% ============ 4. 场景间 ρ 与增益的一致性 ============
pr('\n################ 4. 机理跨场景一致性 ################\n\n');
pr('  若"模型集匹配→去相关→融合增益"成立，则各场景的 (rho, gain) 应落在同一趋势上\n');
pr('  %-14s %-9s %10s %10s %10s\n','scene','CT set','rho','gain','NEES');
rows = {
  'adsb_0002','w07_15','0.7/1.5'
  'adsb_0002','w005_02','0.05/0.2'
  'adsb_0608','w07_15','0.7/1.5'
  'adsb_0608','w005_02','0.05/0.2'
  'multi_cross','w07_15','0.7/1.5'
  'multi_cross','w005_02','0.05/0.2'
};
for i = 1:size(rows,1)
  V = collect(d0,'E9_baseline',rows{i,1},rows{i,2});
  if isempty(V.SCC), V = collect(d0,'E10_optimal',rows{i,1},rows{i,2}); end
  if isempty(V.SCC), V = collect(d0,'E13_dense',rows{i,1},rows{i,2}); end
  if isempty(V.SCC), continue; end
  g = 100*(min(V.R1,V.R2)-V.SCC)./min(V.R1,V.R2);
  pr('  %-14s %-9s %+10.3f %+9.1f%% %10.2f\n', rows{i,1}, rows{i,3}, mean(V.rho), mean(g), mean(V.NEES));
end

fclose(fid);
fprintf('WROTE final.txt\n');
end

% ---------------------------------------------------------------
function V = collect(d0, exp_id, scene, cfg)
V = struct('R1',[],'R2',[],'SCC',[],'rho',[],'NEES',[],'match',[],'prec',[],'cov',[]);
d = fullfile(d0, exp_id);
files = dir(fullfile(d, [scene '__' cfg '__s*.mat']));
for k = 1:numel(files)
  S = load(fullfile(d, files(k).name));
  if ~isfield(S,'m'), continue; end
  m = S.m;
  V.R1(end+1)    = m.rmse.R1_rms;
  V.R2(end+1)    = m.rmse.R2_rms;
  V.SCC(end+1)   = getm(m,'SCC');
  V.rho(end+1)   = m.consistency.rho;
  V.NEES(end+1)  = m.consistency.nees_R1;
  V.match(end+1) = m.matching.match_rate;
  V.prec(end+1)  = m.matching.n_correct / max(m.matching.n_pairs,1);
  V.cov(end+1)   = m.coverage.coverage;
end
end

function v = getm(m, nm)
v = NaN;
for i=1:numel(m.rmse.fusion)
  if strcmp(m.rmse.fusion(i).method, nm), v = m.rmse.fusion(i).rms; return; end
end
end
