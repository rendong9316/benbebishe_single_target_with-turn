addpath(genpath('.'));
f = dir('results/simulation_multi_*.mat');
loaded = load(fullfile('results', f(end).name));
ev = loaded.fusion_eval;

for m = 1:4
    for a = 1:3
        fe = ev.fusion_errors{m,a};
        fprintf('  [%d,%d] class=%s len=%d', m, a, class(fe), length(fe));
        if isstruct(fe) && length(fe) > 0
            fprintf(' fields=');
            sf = fieldnames(fe);
            for j = 1:length(sf), fprintf(' %s', sf{j}); end
        end
        fprintf('\n');
    end
end
