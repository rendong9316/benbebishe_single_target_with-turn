function probe_robust()
% 均值 vs 最坏情况：四种融合算法的鲁棒性对比
% 动机：multi_cross seed=5 的 SCC=17.06 km，是最优单站的 3.3 倍（gain=-226.9%）
%       说明 SCC 在协方差失配时会发散。均值最优 != 最坏情况可控。
% 输出：每种方法 per-seed 的 ratio = fused / best_single
%       mean / median / p95 / max / 超阈值种子数

root = 'D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn';
out  = fullfile(root, '_audit', 'robust.txt');
fid  = fopen(out, 'w', 'n', 'UTF-8');
pr = @(varargin) fprintf(fid, varargin{:});

exps   = {'E9_baseline','E10_optimal'};
cfgs   = {'w07_15','w005_02'};
scenes = {'adsb_0002','multi_cross'};
MAXS   = 200;

for si = 1:numel(scenes)
  sc = scenes{si};
  for ci = 1:numel(cfgs)
    cf = cfgs{ci};
    % 收集所有种子的各方法 rms
    meth = {};  R = [];  B = [];
    for ei = 1:numel(exps)
      d = fullfile(root,'results','paper',exps{ei});
      files = dir(fullfile(d, [sc '__' cf '__s*.mat']));
      for k = 1:numel(files)
        tok = regexp(files(k).name, [sc '__' cf '__s(\d+)\.mat'], 'tokens');
        if isempty(tok), continue; end
        S = load(fullfile(d, files(k).name));
        if ~isfield(S,'m') || ~isfield(S.m,'rmse'), continue; end
        m = S.m;
        if isempty(meth)
          for i = 1:numel(m.rmse.fusion), meth{end+1} = m.rmse.fusion(i).method; end %#ok<AGROW>
        end
        r = nan(1,numel(meth));
        for i = 1:numel(meth)
          for j = 1:numel(m.rmse.fusion)
            if strcmp(m.rmse.fusion(j).method, meth{i}), r(i) = m.rmse.fusion(j).rms; end
          end
        end
        R(end+1,:) = r; %#ok<AGROW>
        B(end+1,1) = min(m.rmse.R1_rms, m.rmse.R2_rms); %#ok<AGROW>
      end
    end
    n = size(R,1);
    if n == 0, continue; end

    pr('\n################ %s / %s  (n=%d seeds) ################\n', sc, cf, n);
    pr('  ratio = fused_rmse / best_single_rmse   (>1 表示比最优单站差)\n\n');
    pr('  %-6s %8s %8s %8s %8s %8s %8s %6s %6s\n', ...
       'method','mean','median','p90','p95','max','min','>1.2x','>1.5x');
    for i = 1:numel(meth)
      q = R(:,i)./B;
      ok = isfinite(q);
      q = q(ok);
      if isempty(q), continue; end
      pr('  %-6s %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %6d %6d\n', ...
         meth{i}, mean(q), median(q), prctile(q,90), prctile(q,95), max(q), min(q), ...
         sum(q>1.2), sum(q>1.5));
    end

    % 最坏情况种子明细
    [qmax, imax] = max(R(:,1)./B);
    pr('\n  worst SCC seed: #%d  SCC=%.2f  best_single=%.2f  ratio=%.2f\n', ...
       imax, R(imax,1), B(imax), qmax);
    pr('    all methods on that seed: ');
    for i = 1:numel(meth), pr('%s=%.2f  ', meth{i}, R(imax,i)); end
    pr('\n');
  end
end

fclose(fid);
fprintf('WROTE %s\n', out);
end
