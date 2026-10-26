function probe_match3_test()
% 三判据关联升级验证脚本（只读源码，通过 params 覆盖注入开关）
%
% 测试 1（向后兼容）：multi_cross，三判据全关
%   - 新版匹配器（开关全关）应与原版 track_matcher_dualgate_orig 逐对一致
% 测试 2（真实数据）：adsb_0002，42 条真实 ADS-B 航迹
%   - A：仅双门限（use_maha=false, use_vel=false）
%   - B：三判据（use_maha=true, use_vel=true）
%   - 用快照里的 truth_idx 判定每对是正确配对还是误配
%
% 输出写入 _audit/probe_match3_test.txt（纯 ASCII，避免控制台乱码）

root = 'D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn';
cd(root);
addpath(genpath(root));

log_path = fullfile(root, '_audit', 'probe_match3_test.txt');
fid = fopen(log_path, 'w');
if fid < 0
    error('无法打开日志文件: %s', log_path);
end

fprintf(fid, '===== probe_match3_test =====\n');
fprintf(fid, 'MATLAB %s\n\n', version);

% ---------------------------------------------------------------
% 测试 1：multi_cross 向后兼容
% ---------------------------------------------------------------
fprintf(fid, '----- TEST 1: multi_cross (backward compatibility) -----\n');
p1 = simulation_params_oracle();
p1.dualgate_use_maha = false;
p1.dualgate_use_vel  = false;

t0 = tic;
r1 = run('multi_cross', p1);
el1 = toc(t0);

new_pairs  = r1.matched_pairs;
orig_pairs = track_matcher_dualgate_orig(r1.trackSnapshots_R1, r1.aligned_R2, p1);

fprintf(fid, 'run time           : %.1f s\n', el1);
fprintf(fid, 'new  (gates OFF)   : %d pairs\n', numel(new_pairs));
fprintf(fid, 'orig (backup file) : %d pairs\n', numel(orig_pairs));
same = compare_pairs(new_pairs, orig_pairs);
if same
    fprintf(fid, 'IDENTICAL          : YES (same ids, same match_count, same quality)\n');
else
    fprintf(fid, 'IDENTICAL          : NO\n');
end
print_pairs(fid, new_pairs);
fprintf(fid, '\n');

% ---------------------------------------------------------------
% 测试 2：adsb_0002 真实数据
% ---------------------------------------------------------------
fprintf(fid, '----- TEST 2: adsb_0002 (real ADS-B, 42 tracks) -----\n');

pA = simulation_params_oracle();
pA.dualgate_use_maha = false;
pA.dualgate_use_vel  = false;

pB = simulation_params_oracle();
pB.dualgate_use_maha = true;
pB.dualgate_use_vel  = true;

tA = tic;
rA = run('adsb_0002', pA);
elA = toc(tA);

tB = tic;
rB = run('adsb_0002', pB);
elB = toc(tB);

fprintf(fid, 'run time  A(dual only)=%.1f s   B(three gates)=%.1f s\n', elA, elB);
fprintf(fid, 'targets=%d  frames=%d\n', rA.scenario.n_targets, rA.scenario.n_frames);
fprintf(fid, '\n');

repA = eval_pairs(rA.matched_pairs, rA.trackSnapshots_R1, rA.aligned_R2, rA.scenario);
repB = eval_pairs(rB.matched_pairs, rB.trackSnapshots_R1, rB.aligned_R2, rB.scenario);

fprintf(fid, '[A] dual-gate only (var + no velocity)\n');
print_report(fid, repA);
fprintf(fid, '\n[B] three gates (mahalanobis + velocity)\n');
print_report(fid, repB);

% ---- 候选层评估：关闭 1v1 互斥，看判据本身的区分力 ----
% 互斥后每站每航迹只能留一对，会掩盖误配；候选层才能反映判据真假分辨能力。
pA1 = pA; pA1.dualgate_mutual_exclusion = false;
pB1 = pB; pB1.dualgate_mutual_exclusion = false;
candA = track_matcher_dualgate(rA.trackSnapshots_R1, rA.aligned_R2, pA1);
candB = track_matcher_dualgate(rA.trackSnapshots_R1, rA.aligned_R2, pB1);
repCA = eval_pairs(candA, rA.trackSnapshots_R1, rA.aligned_R2, rA.scenario);
repCB = eval_pairs(candB, rA.trackSnapshots_R1, rA.aligned_R2, rA.scenario);

fprintf(fid, '\n[CAND-A] candidates before 1v1 exclusion, dual-gate only\n');
print_report(fid, repCA);
fprintf(fid, '\n[CAND-B] candidates before 1v1 exclusion, three gates\n');
print_report(fid, repCB);
fprintf(fid, '\n----- pair detail A -----\n');
print_pairs(fid, rA.matched_pairs);
fprintf(fid, '\n----- pair detail B -----\n');
print_pairs(fid, rB.matched_pairs);

% 逐对对比（哪些对在 A 中正确、在 B 中被纠正）
fprintf(fid, '\n----- wrong pairs -----\n');
fprintf(fid, 'A wrong (%d) : %s\n', repA.n_wrong, strjoin(repA.wrong_str, ' '));
fprintf(fid, 'B wrong (%d) : %s\n', repB.n_wrong, strjoin(repB.wrong_str, ' '));
fprintf(fid, 'CAND-A wrong (%d) : %s\n', repCA.n_wrong, strjoin(repCA.wrong_str, ' '));
fprintf(fid, 'CAND-B wrong (%d) : %s\n', repCB.n_wrong, strjoin(repCB.wrong_str, ' '));
fprintf(fid, '\n===== done =====\n');
fclose(fid);

disp(fileread(log_path));
end


% =========================================================================
% eval_pairs — 用 truth_idx 评估配对正确性
% =========================================================================
function rep = eval_pairs(pairs, snaps_R1, snaps_R2, scenario)
    rep = struct();
    rep.n_pairs = numel(pairs);
    rep.n_correct = 0;
    rep.n_wrong = 0;
    rep.n_missed = 0;
    rep.avg_quality = NaN;
    rep.wrong_str = {};
    rep.missed_str = {};

    if rep.n_pairs == 0
        rep.wrong_str = {'-'};
        rep.missed_str = {'-'};
        return;
    end

    truth1 = id_truth_map(snaps_R1);
    truth2 = id_truth_map(snaps_R2);

    rep.avg_quality = mean([pairs.quality]);

    correct_truth = [];   % 被正确配对的 truth_idx 集合
    for i = 1:rep.n_pairs
        id1 = pairs(i).R1_track_id;
        id2 = pairs(i).R2_track_id;
        t1 = lookup_truth(truth1, id1);
        t2 = lookup_truth(truth2, id2);
        if isnan(t1) || isnan(t2) || t1 ~= t2
            rep.n_wrong = rep.n_wrong + 1;
            rep.wrong_str{end+1} = sprintf('(R1#%d/t%d<->R2#%d/t%d)', ...
                id1, t1, id2, t2);
        else
            rep.n_correct = rep.n_correct + 1;
            correct_truth(end+1) = t1; %#ok<AGROW>
        end
    end

    % 漏配：真实同时出现在 R1/R2 的目标，但没有被任何正确配对覆盖
    tset1 = unique_truth(truth1);
    tset2 = unique_truth(truth2);
    common = intersect(tset1, tset2);
    for q = 1:numel(common)
        t = common(q);
        if ~any(correct_truth == t)
            rep.n_missed = rep.n_missed + 1;
            rep.missed_str{end+1} = sprintf('t%d', t); %#ok<AGROW>
        end
    end

    rep.n_common_truth = numel(common);
    if isfield(scenario, 'n_targets')
        rep.n_targets = scenario.n_targets;
    end
end


% =========================================================================
% id_truth_map — 航迹 ID → truth_idx 映射（取首次出现的非 NaN 值）
% =========================================================================
function m = id_truth_map(snaps)
    max_id = 0;
    for k = 1:numel(snaps)
        if ~isfield(snaps{k}, 'trackList') || isempty(snaps{k}.trackList)
            continue;
        end
        for t = 1:numel(snaps{k}.trackList)
            max_id = max(max_id, double(snaps{k}.trackList{t}.id));
        end
    end
    m = nan(max_id, 1);
    for k = 1:numel(snaps)
        if ~isfield(snaps{k}, 'trackList') || isempty(snaps{k}.trackList)
            continue;
        end
        for t = 1:numel(snaps{k}.trackList)
            trk = snaps{k}.trackList{t};
            if trk.type == 7
                continue;
            end
            id = double(trk.id);
            if ~isfield(trk, 'truth_idx')
                continue;
            end
            ti = double(trk.truth_idx);
            if isnan(m(id))
                m(id) = ti;
            end
        end
    end
end


function t = lookup_truth(map, id)
    id = double(id);
    if id >= 1 && id <= numel(map)
        t = map(id);
    else
        t = NaN;
    end
end


function s = unique_truth(map)
    v = map(~isnan(map));
    s = unique(v(:))';
end


% =========================================================================
% compare_pairs — 两套配对结果是否逐项一致
% =========================================================================
function same = compare_pairs(a, b)
    same = false;
    if numel(a) ~= numel(b)
        return;
    end
    if isempty(a)
        same = true;
        return;
    end
    fa = {a.R1_track_id, a.R2_track_id, a.match_count, a.quality};
    fb = {b.R1_track_id, b.R2_track_id, b.match_count, b.quality};
    for k = 1:4
        x = double(fa{k}); y = double(fb{k});
        if numel(x) ~= numel(y)
            return;
        end
        if any(abs(x(:) - y(:)) > 1e-9)
            return;
        end
    end
    same = true;
end


% =========================================================================
% print_report / print_pairs
% =========================================================================
function print_report(fid, rep)
    fprintf(fid, '  pairs      : %d\n', rep.n_pairs);
    fprintf(fid, '  correct    : %d\n', rep.n_correct);
    fprintf(fid, '  wrong      : %d\n', rep.n_wrong);
    fprintf(fid, '  missed     : %d  (truth targets in both radars but unmatched)\n', rep.n_missed);
    fprintf(fid, '  avg quality: %.2f\n', rep.avg_quality);
    fprintf(fid, '  common truth targets: %d / total %d\n', rep.n_common_truth, rep.n_targets);
end

function print_pairs(fid, pairs)
    if isempty(pairs)
        fprintf(fid, '  (none)\n');
        return;
    end
    fprintf(fid, '  R1id  R2id  nMatch  dist_km   var   quality  maha_r  vel_d  vel_tol\n');
    for i = 1:numel(pairs)
        p = pairs(i);
        mr = NaN; vd = NaN; vt = NaN;
        if isfield(p, 'maha_ratio'),   mr = p.maha_ratio;   end
        if isfield(p, 'vel_diff_kmps'), vd = p.vel_diff_kmps; end
        if isfield(p, 'vel_tol_kmps'),  vt = p.vel_tol_kmps;  end
        fprintf(fid, '  %4d  %4d  %6d  %7.2f  %6.2f  %7.2f  %6.3f  %6.4f  %7.4f\n', ...
            p.R1_track_id, p.R2_track_id, p.match_count, p.mean_dist_km, ...
            p.var_dist, p.quality, mr, vd, vt);
    end
end
