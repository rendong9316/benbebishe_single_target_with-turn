function summarize_paper(exp_ids)
%SUMMARIZE_PAPER 汇总批量实验结果，输出 CSV（论文表格直接引用）
%
%   summarize_paper({'E1_density','E2_assoc'})
%
%   读取 results/paper/<exp_id>/*.mat，提取关键指标，写成
%   results/paper/summary_<exp_id>.csv 与一份合并的 summary_all.csv

    if nargin < 1 || isempty(exp_ids)
        exp_ids = {'E1_density','E2_assoc','E3_t1','E4_m'};
    end
    root = 'D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn';
    cd(root);

    allrows = {};
    for e = 1:numel(exp_ids)
        eid = exp_ids{e};
        d = fullfile(root,'results','paper',eid);
        if ~exist(d,'dir'), continue; end
        files = dir(fullfile(d,'*.mat'));
        rows = {};
        for i = 1:numel(files)
            S = load(fullfile(d, files(i).name));
            if ~isfield(S,'m') || ~isfield(S,'meta'), continue; end
            m = S.m; mt = S.meta;
            r = struct();
            r.exp = eid;
            r.scene = mt.scene;
            r.cfg = mt.cfg;
            r.seed = mt.seed;
            r.n_targets = mt.n_targets;
            r.elapsed = mt.elapsed_sec;
            r.R1 = m.rmse.R1_rms;
            r.R2 = m.rmse.R2_rms;
            r.SCC = getm(m,'SCC');
            r.BC  = getm(m,'BC');
            r.CI  = getm(m,'CI');
            r.FCI = getm(m,'FCI');
            r.R1_only = getm(m,'R1_only');
            r.R2_only = getm(m,'R2_only');
            r.coverage = m.coverage.coverage;
            r.match_rate = m.matching.match_rate;
            r.wrong_rate = m.matching.wrong_rate;
            r.n_pairs = m.matching.n_pairs;
            r.real_len = m.track_length.real_mean;
            r.false_rate = m.track_length.false_rate;
            r.cf_n = m.common_frame.n;
            r.cf_R1 = m.common_frame.R1;
            r.cf_R2 = m.common_frame.R2;
            r.cf_fused = m.common_frame.fused;
            if isfinite(r.cf_fused) && isfinite(r.cf_R1) && isfinite(r.cf_R2)
                r.cf_gain = 100*(min(r.cf_R1,r.cf_R2)-r.cf_fused)/min(r.cf_R1,r.cf_R2);
            else
                r.cf_gain = NaN;
            end
            rows{end+1} = r; %#ok<AGROW>
        end
        if isempty(rows), continue; end
        T = struct2table([rows{:}]);
        out = fullfile(d, sprintf('summary_%s.csv', eid));
        writetable(T, out);
        fprintf('wrote %s (%d rows)\n', out, height(T));
        allrows = [allrows, rows]; %#ok<AGROW>
    end

    if ~isempty(allrows)
        T = struct2table([allrows{:}]);
        out = fullfile(root,'results','paper','summary_all.csv');
        writetable(T, out);
        fprintf('wrote %s (%d rows)\n', out, height(T));

        % 打印分组均值（按 exp+scene+cfg）
        fprintf('\n===== GROUP MEANS =====\n');
        fprintf('%-12s %-12s %-10s %5s %7s %7s %7s %7s %7s\n', ...
            'exp','scene','cfg','n','R1','R2','SCC','cov%','cf_gain%');
        g = unique(T(:,[1 2 3]),'rows');
        for i = 1:height(g)
            msk = strcmp(T.exp,g.exp{i}) & strcmp(T.scene,g.scene{i}) & strcmp(T.cfg,g.cfg{i});
            sub = T(msk,:);
            fprintf('%-12s %-12s %-10s %5d %7.2f %7.2f %7.2f %7.1f %7.1f\n', ...
                g.exp{i}, g.scene{i}, g.cfg{i}, height(sub), ...
                mean(sub.R1,'omitnan'), mean(sub.R2,'omitnan'), ...
                mean(sub.SCC,'omitnan'), 100*mean(sub.coverage,'omitnan'), ...
                mean(sub.cf_gain,'omitnan'));
        end
    end
end

function v = getm(m, name)
    v = NaN;
    for i = 1:numel(m.rmse.fusion)
        if strcmp(m.rmse.fusion(i).method, name), v = m.rmse.fusion(i).rms; return; end
    end
end
