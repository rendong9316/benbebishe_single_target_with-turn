function probe_tracklen()
% 融合航迹长度对比（开题承诺"平均长度提升 50%"）
% 用当前最优配置（0.05/0.2）与基线（0.7/1.5）各跑 2 个种子

root = fullfile(pwd,'..');
addpath(root);
for d = {'config','simulation','ukf','tracker','fusion','evaluation','utils','visualization'}
    addpath(fullfile(root, d{1}));
end

fid = fopen('tracklen.txt','w','n','UTF-8');
pr = @(varargin) fprintf(fid, varargin{:});

cases = {
  'adsb_0002', 0.7,  1.5,  1
  'adsb_0002', 0.05, 0.2,  1
  'adsb_0002', 0.05, 0.2,  2
};

pr('################ 融合航迹长度（开题指标：平均长度提升 50%%）################\n\n');

for i = 1:size(cases,1)
  sc = cases{i,1};
  p  = simulation_params_oracle();
  p.imm_ct_omega_slow_deg = cases{i,2};
  p.imm_ct_omega_fast_deg = cases{i,3};
  p.random_seed = cases{i,4};
  fprintf('running %s  %.2f/%.2f  s%d ...\n', sc, cases{i,2}, cases{i,3}, cases{i,4});
  evalc('r = run(sc, p);');
  st = evaluate_fusion_track_length(r);

  pr('\n===== %s  CT=%.2f/%.2f  seed=%d =====\n', sc, cases{i,2}, cases{i,3}, cases{i,4});
  pr('  配对数           : %d\n', st.n_pairs);
  pr('  单站 R1 平均长度 : %.1f 帧\n', st.single_R1);
  pr('  单站 R2 平均长度 : %.1f 帧\n', st.single_R2);
  pr('  逐配对两站长度   : R1 %.1f 帧，R2 %.1f 帧，较长者 %.1f 帧\n', ...
     mean(st.pair_len_R1), mean(st.pair_len_R2), mean(max(st.pair_len_R1,st.pair_len_R2)));
  pr('\n  %-8s %10s %10s %10s %10s %10s %10s\n', ...
     'method','mean_len','both','R1_only','R2_only','gain_vs_best','both_ratio');
  for m = 1:numel(st.methods)
    pr('  %-8s %10.1f %10.1f %10.1f %10.1f %+9.1f%% %9.1f%%\n', ...
       st.methods(m).name, st.methods(m).mean_len, st.methods(m).both, ...
       st.methods(m).r1_only, st.methods(m).r2_only, ...
       st.pairwise(m).gain_vs_best, 100*st.pairwise(m).both_ratio);
  end
end

fclose(fid);
fprintf('WROTE tracklen.txt\n');
end
