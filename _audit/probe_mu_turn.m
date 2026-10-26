% 转弯场景下 5 模型 IMM 的 mu 演化：验证 CT 模型是否被激活
root = 'D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn';
cd(root); addpath(genpath(root));

r = run('single_turn_right_sustained');
save(fullfile(root,'_audit','mc_turn_geom.mat'), '-struct', 'r', '-v7.3');

names = {'CV','CTL-slow','CTL-fast','CTR-fast','CTR-slow'};
snaps = r.trackSnapshots_R1; nf = numel(snaps);
fprintf('\n### 转弯场景 %s, 真值转弯率=%.2f deg/s ###\n', 'single_turn_right_sustained', ...
    r.params.truth_turn_rate_deg_per_sec);

for ac = 1:numel(r.truthTrajs)
    MU = nan(nf, 5);
    for k = 1:nf
        tl = snaps{k}.trackList;
        for t = 1:numel(tl)
            s = tl{t};
            if isfield(s,'truth_idx') && s.truth_idx == ac && s.type ~= 7 ...
               && isfield(s,'ukf') && isfield(s.ukf,'mu')
                MU(k,:) = s.ukf.mu(:)'; break;
            end
        end
    end
    ct = sum(MU(:,2:5), 2);
    fprintf('\n== 目标 %d ==\n', ac);
    fprintf(' 帧    CV   CTLs  CTLf  CTRf  CTRs | CT合计\n');
    for k = 1:2:nf
        if all(isnan(MU(k,:))), continue; end
        fprintf('%3d  %5.2f %5.2f %5.2f %5.2f %5.2f | %5.2f\n', k-1, MU(k,:), ct(k));
    end
    fprintf(' CT合计: 均值=%.3f 最大=%.3f  >0.3帧数=%d/%d\n', ...
        mean(ct(~isnan(ct))), max(ct), sum(ct>0.3), sum(~isnan(ct)));
end
fprintf('\n R1 RMSE = %.3f km,  n = %d\n', r.errorStats_R1.overall.ukf.rms, r.errorStats_R1.overall.ukf.n);
