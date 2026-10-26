function probe_apply_h()
% 验证 registration_apply_h 是否真的把估计的 h 反馈到了反演/滤波
% 对比：mle3 + apply_h=true  vs  mle3 + apply_h=false
% 若两者结果完全相同，说明反馈链路断了
root = fullfile(pwd,'..');
addpath(root);
for d = {'config','simulation','ukf','tracker','fusion','evaluation','utils','visualization'}
    addpath(fullfile(root, d{1}));
end

fid = fopen('apply_h.txt','w','n','UTF-8');
pr = @(varargin) fprintf(fid, varargin{:});

cases = {true, 'apply_h=ON'; false, 'apply_h=OFF'};
res = zeros(2,3);
for i = 1:2
  p = simulation_params_oracle();
  p.registration_mode = 'mle3';
  p.registration_apply_h = cases{i,1};
  p.random_seed = 1;
  fprintf('running %s ...\n', cases{i,2});
  evalc('r = run(''adsb_0002'', p);');
  m = evaluate_paper_metrics(r);
  res(i,:) = [m.rmse.R1_rms, m.rmse.R2_rms, getm(m,'SCC')];
  pr('%s : R1=%.4f  R2=%.4f  SCC=%.4f  NEES=%.3f\n', cases{i,2}, ...
     res(i,1), res(i,2), res(i,3), m.consistency.nees_R1);
end

pr('\n差异: dR1=%.4f km  dR2=%.4f km  dSCC=%.4f km\n', ...
   res(1,1)-res(2,1), res(1,2)-res(2,2), res(1,3)-res(2,3));
if abs(res(1,1)-res(2,1)) < 1e-6
  pr('结论：完全相同 —— h 反馈链路未生效，需要排查\n');
else
  pr('结论：有差异 —— h 反馈已生效\n');
end
fclose(fid);
fprintf('WROTE apply_h.txt\n');
end

function v = getm(m, nm)
v = NaN;
for i=1:numel(m.rmse.fusion)
  if strcmp(m.rmse.fusion(i).method, nm), v = m.rmse.fusion(i).rms; return; end
end
end
