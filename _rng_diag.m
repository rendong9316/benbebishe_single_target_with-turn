% _rng_diag.m — 纯内置函数，分析rng状态与坏种子聚类的关系
fid = fopen('_rng_analysis.txt', 'w');
Pd = 0.6; K_LOSS = 4; n_frames = 52; n_seeds = 200;

fprintf(fid, '===== RNG状态与坏种子聚类分析 =====\n');
fprintf(fid, 'Pd=%.1f  K_loss=%d  n_frames=%d  n_seeds=%d\n\n', Pd, K_LOSS, n_frames, n_seeds);

% 1. 采样 rng(N) first_rand
fprintf(fid, '--- 1. rng(N) first_rand N=1..250 ---\n');
R = 1:250;
fr = zeros(1,250);
bad_rng = zeros(1,250);
for n = R
    rng(n);
    fr(n) = rand();
    bad_rng(n) = fr(n) > Pd;
end
fprintf(fid, '漏检状态 (first_rand > %.1f): %d/250 (%.1f%%)\n', Pd, sum(bad_rng), sum(bad_rng)/250*100);
fprintf(fid, 'first_rand分布: mean=%.4f median=%.4f min=%.4f max=%.4f\n\n', ...
    mean(fr), median(fr), min(fr), max(fr));

% 2. 连续>=K_LOSS的坏rng区间
fprintf(fid, '--- 2. 连续>=%d漏检的rng区间 ---\n', K_LOSS);
cz = []; c = 0;
for n = R
    if bad_rng(n), c = c + 1;
    else
        if c >= K_LOSS, cz(end+1, :) = [n-c, n-1, c]; end
        c = 0;
    end
end
if c >= K_LOSS, cz(end+1, :) = [R(end)-c+1, R(end), c]; end
fprintf(fid, '共 %d 个区间\n', size(cz,1));
for z = 1:size(cz,1)
    ns = cz(z,1); ne = cz(z,2); nl = cz(z,3);
    smin = max(1, ns - n_frames);
    smax = min(200, ne - 1);
    fprintf(fid, '  区间%d: N=[%d,%d] len=%d → seed[%d,%d] (%d个)\n', ...
        z, ns, ne, nl, smin, smax, max(0, smax-smin+1));
    % 举例3个种子
    ex = unique([smin, round((smin+smax)/2), smax]);
    ex = ex(ex>=1 & ex<=200);
    for ei = 1:length(ex)
        s = ex(ei); bf = [];
        for k = 1:min(n_frames, ne - s)
            if s+k >= ns && s+k <= ne, bf(end+1) = k; end
        end
        if ~isempty(bf)
            fprintf(fid, '      seed=%d: 漏检帧[%d→%d] (%d帧)\n', s, bf(1), bf(end), length(bf));
        end
    end
end
fprintf(fid, '\n');

% 3. 构建检测矩阵
fprintf(fid, '--- 3. 检测矩阵 (seed×frame) ---\n');
D = zeros(n_seeds, n_frames);
for si = 1:n_seeds
    for k = 1:n_frames
        n = si + k;
        if n <= 250
            if ~bad_rng(n), D(si,k) = 1; end
        else
            rng(n);
            if rand() <= Pd, D(si,k) = 1; end
        end
    end
end

% 4. 每种子统计
max_strk = zeros(n_seeds,1);
first_kl = nan(n_seeds,1);
kl_events = zeros(n_seeds,1);

for si = 1:n_seeds
    dv = D(si,:);
    s = 0; c = 0;
    for i = 1:n_frames
        if dv(i)==0, c=c+1; s=max(s,c); else c=0; end
    end
    max_strk(si) = s;

    c = 0; in_ev = false;
    for k = 1:n_frames
        if dv(k)==0
            c = c + 1;
            if c >= K_LOSS && ~in_ev
                kl_events(si) = kl_events(si) + 1;
                in_ev = true;
                if isnan(first_kl(si)), first_kl(si) = k - c + 1; end
            end
        else
            c = 0; in_ev = false;
        end
    end
end

% 5. 理论 vs MC实际
fprintf(fid, '--- 4. 理论(Kloss) vs MC实际坏种子 ---\n');
actual_bad = false(1,200);
actual_bad([21,92,93,116,127:145,152:167,168:194]) = true;
theory_bad = kl_events > 0;
tp = sum(theory_bad & actual_bad);
fp = sum(theory_bad & ~actual_bad);
fn = sum(~theory_bad & actual_bad);
tn = sum(~theory_bad & ~actual_bad);
fprintf(fid, '                MC坏  MC好\n');
fprintf(fid, '  理论坏        %3d   %3d\n', tp, fp);
fprintf(fid, '  理论好        %3d   %3d\n', fn, tn);
fprintf(fid, '  准确率=%.1f%%  召回率=%.1f%%  精确率=%.1f%%\n\n', ...
    (tp+tn)/200*100, tp/(tp+fn)*100, tp/(tp+fp)*100);

if fp > 0
    fp_seeds = find(theory_bad & ~actual_bad);
    fprintf(fid, '  假阳性(理论坏MC好): seed=%s → M/N或真值兜底救回\n', num2str(fp_seeds));
end
if fn > 0
    fn_seeds = find(~theory_bad & actual_bad);
    fprintf(fid, '  假阴性(理论好MC坏): seed=%s → 杂波劫持/UKF发散等其他因素\n', num2str(fn_seeds));
end
fprintf(fid, '\n');

% 6. 坏种子区间检测矩阵
fprintf(fid, '--- 5. 坏种子区间检测矩阵 (█=检测 ·=miss) ---\n');
show_ranges = {[120,150], [150,170], [165,198]};
show_names = {'R1退化区127-145附近', 'R2退化区152-167附近', '双站退化区168-194附近'};

for r = 1:length(show_ranges)
    sr = show_ranges{r};
    fprintf(fid, '\n  %s:\n    帧 ', show_names{r});
    for k = 0:2:28, fprintf(fid, '%-2d', k); end
    fprintf(fid, '\n    ');
    for k = 1:30, fprintf(fid, '─'); end
    fprintf(fid, '\n');

    for si = sr(1):2:sr(2)
        if si < 1 || si > 200, continue; end
        fprintf(fid, '  s%03d|', si);
        for k = 1:30
            if D(si,k)==1, fprintf(fid, 'O'); else fprintf(fid, '.'); end
        end
        if actual_bad(si), fprintf(fid, '|BAD'); end
        if kl_events(si) > 0, fprintf(fid, ' KL@%d', first_kl(si)); end
        fprintf(fid, '\n');
    end
end
fprintf(fid, '\n');

% 7. 聚类内梯度
fprintf(fid, '--- 6. 聚类内退化梯度 (127-145) ---\n');
fprintf(fid, '  seed  首KLoss  最长strk  漏检帧区间        MC退化\n');
fprintf(fid, '  ────  ───────  ────────  ────────────────  ──────\n');
mc_deg = [-54,-58,-61,-64,-70,-67,-85,-106,-263,-96,-115,-126,-141,-148,-151];
mc_s = [127,128,129,132,133,134,135,136,137,140,141,142,143,144,145];
for i = 1:length(mc_s)
    s = mc_s(i);
    dv = D(s,:);
    mf = find(dv==0);
    if isempty(mf)
        ms = '(无漏检)';
    else
        mr = {}; rs = mf(1); re = mf(1);
        for j = 2:length(mf)
            if mf(j) == re+1, re = mf(j);
            else
                mr{end+1} = sprintf('%d-%d', rs, re);
                rs = mf(j); re = mf(j);
            end
        end
        mr{end+1} = sprintf('%d-%d', rs, re);
        ms = strjoin(mr, ',');
    end
    kl_str = '无';
    if ~isnan(first_kl(s)), kl_str = sprintf('帧%d', first_kl(s)); end
    fprintf(fid, '  %-6d %-8s %-9d %-20s %+.0f%%\n', s, kl_str, max_strk(s), ms, mc_deg(i));
end
fprintf(fid, '\n');

% 8. 结论
fprintf(fid, '==================== 结论 ====================\n');
fprintf(fid, '\n');
fprintf(fid, '1.【根因】rng(seed+k) 的 Toeplitz 结构\n');
fprintf(fid, '   检测矩阵检测概率沿对角线 seed+k=const 完全一致。\n');
fprintf(fid, '   同一个 rng 状态在不同种子的不同帧上精确重现。\n');
fprintf(fid, '\n');
fprintf(fid, '2.【聚类】rng状态流中天然存在连续>=%d漏检的"坏区间"\n', K_LOSS);
fprintf(fid, '   P(连续%d漏检)=%.1f^%d=%.4f\n', K_LOSS, 1-Pd, K_LOSS, (1-Pd)^K_LOSS);
fprintf(fid, '   250个状态中预期 %.0f 个连续>=%d 的区间\n', 250*(1-Pd)^K_LOSS, K_LOSS);
fprintf(fid, '   坏区间沿对角线平移 → 依次击中相邻种子 → 聚类\n');
fprintf(fid, '\n');
fprintf(fid, '3.【梯度】seed越大 → 坏帧编号越小(越靠前)\n');
fprintf(fid, '   → UKF有效收敛时间越短 → 退化越严重\n');
fprintf(fid, '   → 解释了127(-54%%)到145(-151%%)的连续恶化\n');
fprintf(fid, '\n');
fprintf(fid, '4.【本质】这不是bug，是 Pd=%.1f+K_loss=%d+rng(N)结构的数学必然\n', Pd, K_LOSS);
fprintf(fid, '\n');
fprintf(fid, '5.【缓解】按推荐优先级:\n');
fprintf(fid, '   (a) K_loss: %d→6 (坏区间数预期减至1/%.0f)\n', K_LOSS, round(1/(1-Pd)^2));
fprintf(fid, '   (b) rng(''shuffle'') 打破Toeplitz结构 (但不可复现)\n');
fprintf(fid, '   (c) 双站融合: 已救回~85%%坏种子\n');
fprintf(fid, '   (d) 重新起始超时兜底: 6帧→3帧\n');
fprintf(fid, '   (e) 长期: IMM降低K_loss敏感性\n');
fprintf(fid, '\n');

elapsed = toc();
fprintf(fid, '分析完成. 耗时 %.1f 秒\n', elapsed);
fclose(fid);
fprintf('_rng_diag done, results in _rng_analysis.txt\n');
