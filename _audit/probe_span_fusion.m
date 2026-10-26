% probe_span_fusion.m — 距离分层场景下的融合验证
% 目的：确认 adsb_span 不只是配准验证场景，在最优模型集（0.05/0.2）下
%       也能复现 ~25% 的融合增益，即它是一个可用的通用实验平台。
% 只读：不改动源码
r0 = fileparts(pwd);
addpath(r0);
for d = {'config','simulation','ukf','tracker','fusion','evaluation','utils','visualization'}
    addpath(fullfile(r0, d{1}));
end

out = 'span_fusion.txt';
fid = fopen(out, 'w', 'n', 'UTF-8');
pr = @(varargin) fprintf(fid, varargin{:});

sets = {[0.7 1.5], [0.05 0.2]};
tags = {'orig_0.7_1.5', 'opt_0.05_0.2'};
scenes = {'adsb_span','adsb_span_d'};
seeds = [1 2 3];

pr('=== 距离分层场景的融合验证 ===\n');
pr('%-14s %-12s %-4s %8s %8s %8s %7s %7s %8s %7s\n', ...
    'scene','modelset','sd','R1','R2','SCC','rho','NEES','gain%','cover%');
pr('%s\n', repmat('-', 1, 88));

for si = 1:numel(scenes)
    sc = scenes{si};
    for mi = 1:numel(sets)
        for sd = seeds
            p = simulation_params_oracle();
            p.imm_ct_omega_slow_deg = sets{mi}(1);
            p.imm_ct_omega_fast_deg = sets{mi}(2);
            p.random_seed = sd;
            try
                evalc('r = run(sc, p);');
                m = evaluate_paper_metrics(r);
                scc = NaN;
                for i = 1:numel(m.rmse.fusion)
                    if strcmp(m.rmse.fusion(i).method, 'SCC')
                        scc = m.rmse.fusion(i).rms;
                    end
                end
                R1 = m.rmse.R1_rms;  R2 = m.rmse.R2_rms;
                gain = 100*(min(R1,R2) - scc)/min(R1,R2);
                cov  = 100*m.coverage.coverage;
                pr('%-14s %-12s %-4d %8.3f %8.3f %8.3f %7.3f %7.2f %8.2f %7.1f\n', ...
                    sc, tags{mi}, sd, R1, R2, scc, m.consistency.rho, ...
                    m.consistency.nees_R1, gain, cov);
                fprintf('  [%s %s s%d] R1=%.2f SCC=%.2f rho=%.3f gain=%.1f%%\n', ...
                    sc, tags{mi}, sd, R1, scc, m.consistency.rho, gain);
            catch ME
                pr('%-14s %-12s %-4d FAILED: %s\n', sc, tags{mi}, sd, ME.message);
                fprintf('  [%s %s s%d] FAILED: %s\n', sc, tags{mi}, sd, ME.message);
            end
        end
    end
    pr('\n');
end
fclose(fid);
fprintf('WROTE %s\n', out);
