function probe_acc()
% 多目标跟踪精度 + 站站配对正确率 的完整指标
% 覆盖：adsb_0002（6 架）、adsb_0608（28 架）、multi_cross（3 架对照）

d0 = fullfile(pwd,'..','results','paper');
fid = fopen('acc.txt','w','n','UTF-8');
pr = @(varargin) fprintf(fid, varargin{:});

specs = {
  'adsb_0002', 'E9_baseline',  'w07_15',  1:50
  'adsb_0002', 'E10_optimal',  'w005_02', 1:50
  'adsb_0608', 'E13_dense',    'w07_15',  1:5
  'adsb_0608', 'E13_dense',    'w005_02', 1:5
  'multi_cross','E9_baseline', 'w07_15',  1:50
  'multi_cross','E10_optimal', 'w005_02', 1:50
};

pr('################ 多目标跟踪与配对精度 ################\n');
pr('口径说明：\n');
pr('  点-航迹关联 = Oracle（用真值 ID 选点），所以跟踪精度是"关联正确前提下"的条件精度\n');
pr('  站站航迹配对 = 真实算法（双门限/三判据），这才是要考核的部分\n\n');

for k = 1:size(specs,1)
  sc = specs{k,1}; ex = specs{k,2}; cf = specs{k,3}; seeds = specs{k,4};
  d = fullfile(d0, ex);
  % 收集
  A = struct('nT',[],'R1',[],'R2',[],'SCC',[],'rho',[],'gain',[], ...
             'match',[],'wrong',[],'corr',[],'mis',[],'npr',[], ...
             'len',[],'len_med',[],'falseR1',[],'ntrR1',[],'cov',[],'cf_gain',[]);
  nfound = 0;
  for s = seeds
    f = fullfile(d, sprintf('%s__%s__s%02d.mat', sc, cf, s));
    if ~isfile(f), continue; end
    S = load(f);  if ~isfield(S,'m'), continue; end
    m = S.m;  nfound = nfound + 1;
    A.nT(end+1)      = m.n_targets;
    A.R1(end+1)      = m.rmse.R1_rms;
    A.R2(end+1)      = m.rmse.R2_rms;
    A.SCC(end+1)     = getm(m,'SCC');
    A.rho(end+1)     = m.consistency.rho;
    A.match(end+1)   = m.matching.match_rate;
    A.wrong(end+1)   = m.matching.wrong_rate;
    A.corr(end+1)    = m.matching.n_correct;
    A.mis(end+1)     = m.matching.n_missed;
    A.npr(end+1)     = m.matching.n_pairs;
    A.len(end+1)     = m.track_length.real_mean;
    A.len_med(end+1) = m.track_length.real_median;
    A.falseR1(end+1) = 100*m.track_length.n_false/max(m.track_length.n_total,1);
    A.ntrR1(end+1)   = m.track_length.n_total;
    A.cov(end+1)     = 100*m.coverage.coverage;
    A.cf_gain(end+1) = 100*(min(m.common_frame.R1,m.common_frame.R2)-m.common_frame.fused) ...
                       /min(m.common_frame.R1,m.common_frame.R2);
  end
  if nfound == 0, continue; end
  n = nfound;
  best = min(A.R1, A.R2);
  g = 100*(best - A.SCC)./best;

  pr('\n================ %s / %s  (n=%d seed) ================\n', sc, cf, n);
  pr('  场景规模      : %d 条航迹（全时段），同时活跃均值见 meta\n', A.nT(1));
  pr('  单站 RMSE     : R1=%.2f±%.2f   R2=%.2f±%.2f\n', mean(A.R1), std(A.R1), mean(A.R2), std(A.R2));
  pr('  融合 SCC      : %.2f±%.2f   增益 %+.1f%%（全帧口径）\n', mean(A.SCC), std(A.SCC), mean(g));
  pr('  共同帧增益    : %+.1f%%（公平口径，只统计两站都有估计的帧）\n', mean(A.cf_gain));
  pr('  融合覆盖率    : %.1f%%（both 帧占比）\n', mean(A.cov));
  pr('  rho           : %+.3f±%.3f\n', mean(A.rho), std(A.rho));
  pr('  ---- 站站配对 ----\n');
  pr('    配对数      : %.1f±%.1f  /  真值航迹 %d\n', mean(A.npr), std(A.npr), A.nT(1));
  pr('    正确配对    : %.1f±%.1f\n', mean(A.corr), std(A.corr));
  pr('    正确匹配率  : %.1f%%  (正确配对 / 真值航迹数)\n', 100*mean(A.match));
  pr('    误配率      : %.2f%%\n', 100*mean(A.wrong));
  pr('    漏配数      : %.1f±%.1f\n', mean(A.mis), std(A.mis));
  pr('  ---- 航迹质量（R1）----\n');
  pr('    真实航迹长度: 均值 %.1f 帧，中位 %.1f 帧\n', mean(A.len), mean(A.len_med));
  pr('    虚假航迹率  : %.1f%%（总航迹数 %.1f）\n', mean(A.falseR1), mean(A.ntrR1));
end

fclose(fid);
fprintf('WROTE acc.txt\n');
end

function v = getm(m, nm)
v = NaN;
for i=1:numel(m.rmse.fusion)
  if strcmp(m.rmse.fusion(i).method, nm), v = m.rmse.fusion(i).rms; return; end
end
end
