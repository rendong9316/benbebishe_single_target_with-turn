% =========================================================================
% audit_mc_stats.m -- Read-only audit of the IMM Monte-Carlo result files
% -------------------------------------------------------------------------
% Purpose: recompute the Default / Config51 / Sigmoid comparison directly
%          from results/mc_*.mat, de-duplicating overlapping seed batches.
% Usage:   matlab -batch "cd _audit; audit_mc_stats"
% Notes:   Does not modify any file. Legacy files without scenario_name,
%          seeds or configs (e.g. mc_turn180_*) are skipped.
% =========================================================================
function audit_mc_stats()
    root = fileparts(fileparts(mfilename('fullpath')));
    d = dir(fullfile(root, 'results', 'mc_*.mat'));
    rec = []; scen = {}; cfg = {};
    for i = 1:numel(d)
        M = load(fullfile(root, 'results', d(i).name));
        if ~isfield(M,'scenario_name') || ~isfield(M,'seeds') || ~isfield(M,'configs'), continue; end
        sd = M.seeds(:); R = M.rmse_matrix;
        if numel(sd) ~= size(R,1), continue; end
        sc = char(M.scenario_name);
        if ~any(strcmp(scen,sc)), scen{end+1} = sc; end %#ok<AGROW>
        for a = 1:numel(sd)
            for b = 1:numel(M.configs)
                c = M.configs{b};
                if contains(c,'Sig') || contains(c,'Sigmoid'), c = 'Sigmoid'; end
                if ~any(strcmp(cfg,c)), cfg{end+1} = c; end %#ok<AGROW>
                rec(end+1,:) = [sd(a), find(strcmp(scen,sc)), find(strcmp(cfg,c)), R(a,b)]; %#ok<AGROW>
            end
        end
    end
    seed = rec(:,1); scn = rec(:,2); cgf = rec(:,3); rm = rec(:,4);
    key = seed*10000 + scn*100 + cgf; uk = unique(key);
    val = arrayfun(@(k) mean(rm(key==k)), uk);
    Ks = floor(uk/10000); Kc = mod(floor(uk/100),100); Kg = mod(uk,100);
    iD = find(strcmp(cfg,'Default')); iS = find(strcmp(cfg,'Sigmoid'));
    fprintf('%-15s %4s %9s %9s %8s %7s %9s\n','scenario','n','Default','Sigmoid','imp%','win','p');
    T = []; nw = 0;
    for s = 1:numel(scen)
        gd = []; gs = [];
        for a = unique(Ks(Kc==s))'
            kd = find(Ks==a & Kc==s & Kg==iD); ks = find(Ks==a & Kc==s & Kg==iS);
            if ~isempty(kd) && ~isempty(ks), gd(end+1,1)=val(kd(1)); gs(end+1,1)=val(ks(1)); end %#ok<AGROW>
        end
        if numel(gd) < 3, continue; end
        imp = (gd-gs)./gd*100; [~,p] = ttest(gs,gd);
        fprintf('%-15s %4d %9.3f %9.3f %+8.2f %3d/%-2d %9.4f\n', scen{s}, numel(gd), mean(gd), mean(gs), mean(imp), sum(imp>0), numel(imp), p);
        T = [T; imp]; nw = nw + sum(imp>0); %#ok<AGROW>
    end
    if ~isempty(T)
        [~,p] = ttest(T);
        fprintf('%-15s %4d %9s %9s %+8.2f %3d/%-2d %9.4f\n','OVERALL',numel(T),'','',mean(T),nw,numel(T),p);
        fprintf('median=%+.2f%%  worst=%+.1f%%  Cohen dz=%.2f\n', median(T), min(T), mean(T)/std(T));
    end
end
