addpath(genpath('.'));
f = dir('results/simulation_multi_*.mat');
loaded = load(fullfile('results', f(end).name));
fprintf('=== 多目标融合误差详情 ===\n\n');

% 看 fusion_eval
fe = loaded.fusion_eval;
fprintf('fusion_eval fields: ');
ef = fieldnames(fe);
for i = 1:length(ef), fprintf('%s ', ef{i}); end
fprintf('\n');

if isstruct(fe) && isfield(fe, 'overall')
    fprintf('overall:\n');
    for oi = 1:length(fe.overall)
        om = fe.overall(oi);
        fprintf('  %s: RMS=%.1f median=%.1f n=%d\n', om.method, om.s.rms, om.s.median, om.s.n);
    end
end

if isstruct(fe) && isfield(fe, 'summary')
    fprintf('\nsummary (per-method, per-aircraft):\n');
    for si = 1:length(fe.summary)
        sm = fe.summary(si);
        fprintf('  %s aircraft=%s RMS=%.1f n=%d\n', sm.method, sm.aircraft, sm.s.rms, sm.s.n);
    end
end
