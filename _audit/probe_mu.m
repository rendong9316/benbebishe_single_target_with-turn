% 模型概率 mu 的时间历程：验证 5 模型是否真的被激活
root = 'D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn';
cd(root);
S = load(fullfile(root,'_audit','mc_current_geom.mat'));

names = {'CV','CTL-slow','CTL-fast','CTR-fast','CTR-slow'};
snaps = S.trackSnapshots_R1; nf = numel(snaps);

% 按 truth_idx 收集每个目标的 mu 序列
for ac = 1:numel(S.truthTrajs)
    MU = nan(nf, 5);
    for k = 1:nf
        tl = snaps{k}.trackList;
        for t = 1:numel(tl)
            s = tl{t};
            if isfield(s,'truth_idx') && s.truth_idx == ac && s.type ~= 7 ...
               && isfield(s,'ukf') && isfield(s.ukf,'mu')
                MU(k,:) = s.ukf.mu(:)';
                break;
            end
        end
    end
    fprintf('\n===== 目标 %d (%s) =====\n', ac, S.truthTrajs{ac}.label);
    fprintf(' 帧    CV   CTLs  CTLf  CTRf  CTRs   | CT合计\n');
    for k = 1:2:nf
        if all(isnan(MU(k,:))), continue; end
        fprintf('%3d  %5.2f %5.2f %5.2f %5.2f %5.2f  | %5.2f\n', k-1, MU(k,:), sum(MU(k,2:5)));
    end
    ct = sum(MU(:,2:5), 2);
    fprintf('  CT合计: 均值=%.3f 最大=%.3f (帧%d)  超过0.3的帧数=%d/%d\n', ...
        mean(ct(~isnan(ct))), max(ct), find(ct==max(ct),1)-1, sum(ct>0.3), sum(~isnan(ct)));
    for m = 1:5
        fprintf('   %s: 均值=%.3f 最大=%.3f\n', names{m}, mean(MU(:,m),'omitnan'), max(MU(:,m)));
    end
end
