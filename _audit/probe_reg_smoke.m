function probe_reg_smoke()
% 配准三模式冒烟测试（multi_cross，快）
addpath(fullfile(pwd,'..'));
root = fullfile(pwd,'..');
for d = {'config','simulation','ukf','tracker','fusion','evaluation','utils','visualization'}
    addpath(fullfile(root, d{1}));
end

fid = fopen('reg_smoke.txt','w','n','UTF-8');
pr = @(varargin) fprintf(fid, varargin{:});

pr('################ 配准三模式冒烟测试（multi_cross, seed=1）################\n');
pr('真值虚高 = 320 km，标称虚高 = 300 km\n\n');

modes = {'none','mean','mle3'};
for i = 1:numel(modes)
  p = simulation_params_oracle();
  p.registration_mode = modes{i};
  p.random_seed = 1;
  pr('\n===== mode = %s =====\n', modes{i});
  try
    evalc('r = run(''multi_cross'', p);');
    m = evaluate_paper_metrics(r);
    pr('  R1 = %.3f km   R2 = %.3f km   SCC = %.3f km\n', ...
       m.rmse.R1_rms, m.rmse.R2_rms, getm(m,'SCC'));
    pr('  rho = %+.3f   NEES_R1 = %.2f   coverage = %.1f%%\n', ...
       m.consistency.rho, m.consistency.nees_R1, 100*m.coverage.coverage);
    pr('  match = %.1f%%   pairs = %d\n', 100*m.matching.match_rate, m.matching.n_pairs);
  catch ME
    pr('  FAIL: %s\n', ME.message);
  end
end

fclose(fid);
fprintf('WROTE reg_smoke.txt\n');
end

function v = getm(m, nm)
v = NaN;
for i=1:numel(m.rmse.fusion)
  if strcmp(m.rmse.fusion(i).method, nm), v = m.rmse.fusion(i).rms; return; end
end
end
