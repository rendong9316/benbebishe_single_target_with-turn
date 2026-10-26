% probe_reg_span.m — 距离分层场景下的配准三模式对比
% 目的：验证"两参数均值标定依赖标校集与评估集的分布一致，三参数不依赖"
% 只读：不改动源码
r0 = fileparts(pwd);
addpath(r0);
for d = {'config','simulation','ukf','tracker','fusion','evaluation','utils','visualization'}
    addpath(fullfile(r0, d{1}));
end

out = 'reg_span.txt';
fid = fopen(out, 'w', 'n', 'UTF-8');
pr = @(varargin) fprintf(fid, varargin{:});

scenes = {'adsb_0002','adsb_span','adsb_span_d'};
modes  = {'none','mean','mle3'};
seeds  = [1 2];

pr('=== 配准三模式 x 距离跨度 ===\n');
pr('%-14s %-6s %-4s %9s %9s %9s %8s %8s %8s\n', ...
    'scene','mode','sd','R1','R2','SCC','rho','NEES1','h_hat');
pr('%s\n', repmat('-', 1, 84));

for si = 1:numel(scenes)
    sc = scenes{si};
    for mi = 1:numel(modes)
        md = modes{mi};
        for sd = seeds
            p = simulation_params_oracle();
            p.registration_mode = md;
            p.random_seed = sd;
            try
                evalc('r = run(sc, p);');
                m = evaluate_paper_metrics(r);
                g = r.registration;
                scc = NaN;
                for i = 1:numel(m.rmse.fusion)
                    if strcmp(m.rmse.fusion(i).method, 'SCC')
                        scc = m.rmse.fusion(i).rms;
                    end
                end
                rho  = m.consistency.rho;
                nees = m.consistency.nees_R1;
                if isfield(g,'h1_km'), hh = g.h1_km; else, hh = NaN; end
                pr('%-14s %-6s %-4d %9.3f %9.3f %9.3f %8.3f %8.3f %8.1f\n', ...
                    sc, md, sd, r.errorStats_R1.overall.ukf.rms, ...
                    r.errorStats_R2.overall.ukf.rms, scc, rho, nees, hh);
                fprintf('  [%s %s s%d] R1=%.3f SCC=%.3f rho=%.3f\n', sc, md, sd, ...
                    r.errorStats_R1.overall.ukf.rms, scc, rho);
            catch ME
                pr('%-14s %-6s %-4d  FAILED: %s\n', sc, md, sd, ME.message);
                fprintf('  [%s %s s%d] FAILED: %s\n', sc, md, sd, ME.message);
            end
        end
    end
    pr('\n');
end
fclose(fid);
fprintf('WROTE %s\n', out);
