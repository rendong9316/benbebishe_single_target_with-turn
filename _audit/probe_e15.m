function probe_e15()
% E15 虚高失配敏感性：mean vs mle3 随失配增大的趋势
d0 = fullfile(pwd,'..','results','paper','E15_hsens');
fid = fopen('e15.txt','w','n','UTF-8');
pr = @(varargin) fprintf(fid, varargin{:});

hset = [310 320 340 380];
pr('################ E15 虚高失配敏感性 · adsb_0002 ################\n');
pr('标称虚高固定 300 km，真值从 310 变到 380 km\n');
pr('两参数均值标定（mean）vs 三参数 MLE + h 反馈（mle3）\n\n');
pr('  %-6s %6s %14s %14s %14s %12s %12s %8s\n', ...
   'h_true','失配','mean_SCC','mle3_SCC','改善','mean_R1','mle3_R1','改善');

for i = 1:numel(hset)
  hh = hset(i);
  A = collect(d0, sprintf('h%d_mean', hh));
  B = collect(d0, sprintf('h%d_mle3', hh));
  if isempty(A.SCC) || isempty(B.SCC), continue; end
  n = min(numel(A.SCC), numel(B.SCC));
  aS = A.SCC(1:n); bS = B.SCC(1:n);
  aR = A.R1(1:n);  bR = B.R1(1:n);
  pr('  %-6d %5d km %6.2f±%5.2f %6.2f±%5.2f %+6.2f%% %6.2f±%5.2f %6.2f±%5.2f %+6.2f%%\n', ...
     hh, hh-300, mean(aS),std(aS), mean(bS),std(bS), 100*(mean(aS)-mean(bS))/mean(aS), ...
     mean(aR),std(aR), mean(bR),std(bR), 100*(mean(aR)-mean(bR))/mean(aR));
end

pr('\n  逐种子配对（SCC 改善 %%，正= mle3 更好）:\n');
pr('  %-8s', 'h_true'); for s = 1:3, pr(' %8s', sprintf('s%d', s)); end; pr('\n');
for i = 1:numel(hset)
  hh = hset(i);
  A = collect(d0, sprintf('h%d_mean', hh));
  B = collect(d0, sprintf('h%d_mle3', hh));
  n = min(numel(A.SCC), numel(B.SCC));
  if n == 0, continue; end
  pr('  %-8d', hh);
  for s = 1:n
    pr(' %+7.2f%%', 100*(A.SCC(s)-B.SCC(s))/A.SCC(s));
  end
  pr('\n');
end

% NEES：协方差可信度
pr('\n  NEES_R1（一致时应约 2）:\n');
pr('  %-8s %14s %14s\n', 'h_true', 'mean', 'mle3');
for i = 1:numel(hset)
  hh = hset(i);
  A = collect(d0, sprintf('h%d_mean', hh));
  B = collect(d0, sprintf('h%d_mle3', hh));
  if isempty(A.NEES), continue; end
  pr('  %-8d %6.2f±%5.2f %6.2f±%5.2f\n', hh, mean(A.NEES),std(A.NEES), mean(B.NEES),std(B.NEES));
end

fclose(fid);
fprintf('WROTE e15.txt\n');
end

function A = collect(d0, tag)
A = struct('R1',[],'R2',[],'SCC',[],'rho',[],'NEES',[]);
files = dir(fullfile(d0, ['adsb_0002__' tag '__s*.mat']));
for k = 1:numel(files)
  S = load(fullfile(d0, files(k).name));
  if ~isfield(S,'m'), continue; end
  m = S.m;
  A.R1(end+1)   = m.rmse.R1_rms;
  A.R2(end+1)   = m.rmse.R2_rms;
  A.SCC(end+1)  = getm(m,'SCC');
  A.rho(end+1)  = m.consistency.rho;
  A.NEES(end+1) = m.consistency.nees_R1;
end
end

function v = getm(m, nm)
v = NaN;
for i=1:numel(m.rmse.fusion)
  if strcmp(m.rmse.fusion(i).method, nm), v = m.rmse.fusion(i).rms; return; end
end
end
