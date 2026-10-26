function probe_e14()
% E14 配准三模式对比分析（adsb_0002，5 种子）
d0 = fullfile(pwd,'..','results','paper','E14_reg');
fid = fopen('e14.txt','w','n','UTF-8');
pr = @(varargin) fprintf(fid, varargin{:});

modes = {'reg_none','reg_mean','reg_mle3'};
label = {'none','mean','mle3'};

V = struct();
for i = 1:numel(modes)
  A = struct('R1',[],'R2',[],'SCC',[],'rho',[],'NEES',[],'match',[],'prec',[],'n',[]);
  files = dir(fullfile(d0, ['adsb_0002__' modes{i} '__s*.mat']));
  for k = 1:numel(files)
    S = load(fullfile(d0, files(k).name));
    if ~isfield(S,'m'), continue; end
    m = S.m;
    A.R1(end+1)   = m.rmse.R1_rms;
    A.R2(end+1)   = m.rmse.R2_rms;
    A.SCC(end+1)  = getm(m,'SCC');
    A.rho(end+1)  = m.consistency.rho;
    A.NEES(end+1) = m.consistency.nees_R1;
    A.match(end+1)= m.matching.match_rate;
    A.prec(end+1) = m.matching.n_correct/max(m.matching.n_pairs,1);
    A.n(end+1)    = m.rmse.R1_n;
  end
  V.(modes{i}) = A;
end

pr('################ E14 配准三模式对比 · adsb_0002 ################\n');
pr('真值虚高 320 km，标称 300 km（失配 20 km）\n\n');
pr('  %-6s %4s %14s %14s %14s %10s %10s\n', ...
   'mode','n','R1','R2','SCC','rho','NEES');
for i = 1:numel(modes)
  A = V.(modes{i});
  if isempty(A.R1), continue; end
  pr('  %-6s %4d %6.2f±%5.2f %6.2f±%5.2f %6.2f±%5.2f %+6.3f±%5.3f %5.2f±%4.2f\n', ...
     label{i}, numel(A.R1), mean(A.R1),std(A.R1), mean(A.R2),std(A.R2), ...
     mean(A.SCC),std(A.SCC), mean(A.rho),std(A.rho), mean(A.NEES),std(A.NEES));
end

% 配对改进（同种子）
A = V.('reg_mean');  B = V.('reg_mle3');
n = min(numel(A.SCC), numel(B.SCC));
if n >= 2
  pr('\n  配对对比 mean → mle3（n=%d，同种子）:\n', n);
  vars = {'R1','R2','SCC','rho','NEES'};
  for vi = 1:numel(vars)
    v = vars{vi};
    a = A.(v)(1:n); b = B.(v)(1:n);
    [~,p] = ttest(a,b);
    pr('    %-5s mean=%8.3f  mle3=%8.3f  改善 %+6.2f%%  p=%.3f\n', ...
       v, mean(a), mean(b), 100*(mean(a)-mean(b))/mean(a), p);
  end
  % 相对 none 的整体收益
  N = V.('reg_none');
  if ~isempty(N.R1)
    pr('\n  相对不校正（none）的总收益:\n');
    pr('    none  : R1=%.1f  SCC=%.1f\n', mean(N.R1), mean(N.SCC));
    pr('    mean  : R1=%.2f（收回 %.1f%%）  SCC=%.2f（收回 %.1f%%）\n', ...
       mean(A.R1), 100*(mean(N.R1)-mean(A.R1))/mean(N.R1), ...
       mean(A.SCC), 100*(mean(N.SCC)-mean(A.SCC))/mean(N.SCC));
    pr('    mle3  : R1=%.2f（收回 %.1f%%）  SCC=%.2f（收回 %.1f%%）\n', ...
       mean(B.R1), 100*(mean(N.R1)-mean(B.R1))/mean(N.R1), ...
       mean(B.SCC), 100*(mean(N.SCC)-mean(B.SCC))/mean(N.SCC));
  end
end

fclose(fid);
fprintf('WROTE e14.txt\n');
end

function v = getm(m, nm)
v = NaN;
for i=1:numel(m.rmse.fusion)
  if strcmp(m.rmse.fusion(i).method, nm), v = m.rmse.fusion(i).rms; return; end
end
end
