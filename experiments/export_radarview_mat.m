function out = export_radarview_mat(varargin)
%EXPORT_RADARVIEW_MAT 全帧跑主链路，按阶段导出 5 个 RadarView 可读的 .mat
%
%   out = export_radarview_mat()
%   out = export_radarview_mat('scenario','adsb_ns_fr24','outdir',p,'method','SCC')
%
%   导出 5 个文件（变量名统一为 trackList，格式对齐
%   D:/Desktop/RadarAgent/runs/tracks-shx/trackList.mat）：
%
%     1  R1_raw_meas.mat        R1 原始量测点（Pd=0.6，含漏检空洞）
%                               RadarView 用「导入雷达原始测量数据...」
%     2  R2_raw_meas.mat        R2 原始量测点（精度低于 R1：20 km / 0.60°）
%     3  R1_track_filtered.mat  R1 单站 IMM-UKF 输出（逐帧外推，30 s 无空洞）
%                               RadarView 用「导入雷达数据...」
%     4  R2_track_filtered.mat  R2 单站 IMM-UKF 输出
%     5  fused_track.mat        系统航迹（两站互补融合，覆盖两站并集）
%
%   trackList 字段（与示例完全一致）：
%     BatchNo         double   批号；原始量测文件里是目标真值编号
%     Type            double   1
%     asscPointList   1xM struct(time,lat,lon)  原始/关联点
%     smoothPointList 1xM struct(time,lat,lon)  平滑点
%   time 为 MATLAB datenum（天），lat/lon 为 WGS84 度。
%
%   时间基准取场景 CSV 的最早时间戳（adsb_ns_fr24 → 2026-10-24），
%   因此 RadarView 的回放时间轴就是真实日期，而不是从 0 起算的秒数。

    opts = parse_opts(varargin);

    if ~exist(opts.outdir, 'dir'), mkdir(opts.outdir); end

    fprintf('=== 全帧跑主链路：%s ===\n', opts.scenario);
    result = run(opts.scenario, opts.params, struct('verbose', false));

    base_dn = scene_base_datenum(opts.scenario);
    fprintf('时间基准 datenum = %.6f（%s）\n', base_dn, datestr(base_dn, 'yyyy-mm-dd HH:MM:SS'));

    % ---------------- 1/2 原始量测（按目标真值编号分组）----------------
    rec_raw1 = collect_raw(result.detList_R1, result.t1_grid, base_dn);
    rec_raw2 = collect_raw(result.detList_R2, result.t2_grid, base_dn);
    f1 = fullfile(opts.outdir, 'R1_raw_meas.mat');
    f2 = fullfile(opts.outdir, 'R2_raw_meas.mat');
    write_radarview(f1, rec_raw1, 'assc');
    write_radarview(f2, rec_raw2, 'assc');

    % ---------------- 3/4 单站跟踪输出（按局部航迹批号分组）----------------
    rec_trk1 = collect_tracks(result.trackSnapshots_R1, result.t1_grid, base_dn);
    rec_trk2 = collect_tracks(result.trackSnapshots_R2, result.t2_grid, base_dn);
    f3 = fullfile(opts.outdir, 'R1_track_filtered.mat');
    f4 = fullfile(opts.outdir, 'R2_track_filtered.mat');
    write_radarview(f3, rec_trk1, 'smooth');
    write_radarview(f4, rec_trk2, 'smooth');

    % ---------------- 5 融合系统航迹 ----------------
    % 系统航迹取 ctSnap（融合中心的系统航迹状态），不取 fused_snapshots：
    % fused 快照在"本帧末新建系统航迹"之前生成，新航迹的出生帧不在里面，
    % 会比长度指标的 ct_len 少 1 帧（8 条几何受限目标会从 1.000 掉到 0.983）。
    % ctSnap 的 n_src>0 口径与 evaluate_online_track_length 完全一致。
    names = result.method_names;
    if ~strcmp(opts.method, names{1})
        warning('export:method', ['系统航迹状态只保存默认融合算法（%s）的结果，' ...
            '请求的 %s 无法单独导出'], names{1}, opts.method);
    end
    rec_fus = collect_fused(result.ctSnap, result.t1_grid, base_dn);
    f5 = fullfile(opts.outdir, 'fused_track.mat');
    write_radarview(f5, rec_fus, 'smooth');

    % ---------------- 自检：按 RadarView 的解析规则逐项核对 ----------------
    files = {f1, f2, f3, f4, f5};
    tags  = {'R1 原始量测', 'R2 原始量测', 'R1 跟踪输出', 'R2 跟踪输出', '系统航迹'};
    report = cell(numel(files), 1);
    for i = 1:numel(files)
        report{i} = verify_radarview(files{i});
        fprintf('[%d] %-12s %s\n', i, tags{i}, report{i}.note);
    end

    % ---------------- 长度对比：系统航迹 vs 单站 ----------------
    cmp = compare_length(rec_trk1, rec_trk2, rec_raw1, rec_raw2, rec_fus, opts.outdir);

    out = struct('files', {files}, 'tags', {tags}, 'report', {report}, ...
        'length_cmp', cmp, 'base_datenum', base_dn, 'method', names{1});
    fprintf('\n导出目录：%s\n', opts.outdir);
end

% =====================================================================
%  参数
% =====================================================================
function opts = parse_opts(args)
    opts = struct('scenario', 'adsb_ns_fr24', 'params', [], ...
        'outdir', fullfile(fileparts(fileparts(mfilename('fullpath'))), ...
                           'results', 'radarview_export'), 'method', 'SCC');
    if isempty(args), return; end
    if isstruct(args{1})
        f = fieldnames(args{1});
        for i = 1:numel(f), opts.(f{i}) = args{1}.(f{i}); end
        return;
    end
    for i = 1:2:numel(args)
        opts.(args{i}) = args{i+1};
    end
    if isempty(opts.params), opts.params = []; end
end

% =====================================================================
%  时间基准：场景 CSV 最早时间戳
% =====================================================================
function dn = scene_base_datenum(scenario_name)
    dn = datenum(2000, 1, 1);
    fr24_map = {'adsb_ns_fr24', 'scene_fr24.csv';
                'adsb_ns_all',  'scene_fr24_all.csv'};
    is_fr24 = any(strcmp(char(scenario_name), fr24_map(:, 1)));
    if ~is_fr24, return; end
    root = fileparts(fileparts(mfilename('fullpath')));
    fn = fr24_map{strcmp(char(scenario_name), fr24_map(:, 1)), 2};
    p = fullfile(root, '..', 'data', 'adsb_ns', fn);
    if ~isfile(p)
        warning('export:noScene', '场景文件不存在，时间基准退化为 %s', datestr(dn));
        return;
    end
    fid = fopen(p, 'r');
    ts = {};
    while true
        ln = fgetl(fid);
        if ~ischar(ln), break; end
        c = strsplit(ln, ',', 'CollapseDelimiters', false);
        if numel(c) >= 11, ts{end+1, 1} = c{11}; end %#ok<AGROW>
    end
    fclose(fid);
    vals = nan(numel(ts), 1);
    for i = 1:numel(ts)
        s = ts{i};
        if numel(s) >= 19
            vals(i) = datenum(s(1:19), 'yyyy-mm-dd HH:MM:SS');
        end
    end
    vals = vals(isfinite(vals));
    if isempty(vals)
        warning('export:noTime', '场景无有效时间戳，时间基准退化为 %s', datestr(dn));
        return;
    end
    dn = min(vals);
end

% =====================================================================
%  原始量测：按 aircraft_id 分组（不含杂波）
% =====================================================================
function recs = collect_raw(detCell, tgrid, base_dn)
    n = numel(detCell);
    max_id = 0;
    for k = 1:n
        d = detCell{k};
        if isempty(d), continue; end
        for i = 1:numel(d)
            if ~d(i).is_clutter
                max_id = max(max_id, double(d(i).aircraft_id));
            end
        end
    end
    T = cell(1, max_id);  L = cell(1, max_id);  N = cell(1, max_id);
    for k = 1:n
        d = detCell{k};
        if isempty(d), continue; end
        tk = base_dn + tgrid(k) / 86400;
        for i = 1:numel(d)
            if d(i).is_clutter, continue; end
            id = double(d(i).aircraft_id);
            if ~isfinite(id) || id < 1, continue; end
            T{id}(end+1) = tk;
            L{id}(end+1) = d(i).lat;
            N{id}(end+1) = d(i).lon;
        end
    end
    recs = struct('batch', {}, 't', {}, 'lat', {}, 'lon', {}, 'truth', {});
    for id = 1:max_id
        if isempty(T{id}), continue; end
        recs(end+1) = struct('batch', id, 't', T{id}, 'lat', L{id}, 'lon', N{id}, ...
            'truth', id); %#ok<AGROW>
    end
end

% =====================================================================
%  单站跟踪输出：按局部航迹 id 分组（逐帧都有点，含纯外推帧）
% =====================================================================
function recs = collect_tracks(snapCell, tgrid, base_dn)
    n = numel(snapCell);
    ids = zeros(1, 0);
    T = {};  L = {};  N = {};  R = {};
    for k = 1:n
        s = snapCell{k};
        if ~isstruct(s) || ~isfield(s, 'trackList'), continue; end
        tl = s.trackList;
        tk = base_dn + tgrid(k) / 86400;
        for i = 1:numel(tl)
            tr = tl{i};
            id = double(tr.id);
            j = find(ids == id, 1);
            if isempty(j)
                ids(end+1) = id;              %#ok<AGROW>
                T{end+1} = zeros(1, 0);       %#ok<AGROW>
                L{end+1} = zeros(1, 0);       %#ok<AGROW>
                N{end+1} = zeros(1, 0);       %#ok<AGROW>
                R{end+1} = NaN;               %#ok<AGROW>
                j = numel(ids);
            end
            T{j}(end+1) = tk;
            L{j}(end+1) = tr.lat;
            N{j}(end+1) = tr.lon;
            if isfinite(double(tr.truth_idx)), R{j} = double(tr.truth_idx); end
        end
    end
    recs = struct('batch', {}, 't', {}, 'lat', {}, 'lon', {}, 'truth', {});
    for j = 1:numel(ids)
        recs(end+1) = struct('batch', ids(j), 't', T{j}, 'lat', L{j}, 'lon', N{j}, ...
            'truth', R{j}); %#ok<AGROW>
    end
    [~, ord] = sort([recs.batch]);
    recs = recs(ord);
end

% =====================================================================
%  系统航迹：取指定融合算法的逐帧输出（只含本帧有源的帧）
% =====================================================================
function recs = collect_fused(ctCell, tgrid, base_dn)
    n = numel(ctCell);
    ids = zeros(1, 0);
    T = {};  L = {};  N = {};  R = {};
    for k = 1:n
        CTk = ctCell{k};
        if isempty(CTk), continue; end
        tk = base_dn + tgrid(k) / 86400;
        for i = 1:numel(CTk)
            ct = CTk(i);
            if ct.n_src <= 0 || numel(ct.x) < 3, continue; end
            id = double(ct.id);
            j = find(ids == id, 1);
            if isempty(j)
                ids(end+1) = id;              %#ok<AGROW>
                T{end+1} = zeros(1, 0);       %#ok<AGROW>
                L{end+1} = zeros(1, 0);       %#ok<AGROW>
                N{end+1} = zeros(1, 0);       %#ok<AGROW>
                R{end+1} = NaN;               %#ok<AGROW>
                j = numel(ids);
            end
            T{j}(end+1) = tk;
            L{j}(end+1) = ct.x(3);   % 状态向量 x(1)=经度、x(2)=纬度速度、x(3)=纬度
            N{j}(end+1) = ct.x(1);
            if isfinite(double(ct.truth_idx)), R{j} = double(ct.truth_idx); end
        end
    end
    recs = struct('batch', {}, 't', {}, 'lat', {}, 'lon', {}, 'truth', {});
    for j = 1:numel(ids)
        recs(end+1) = struct('batch', ids(j), 't', T{j}, 'lat', L{j}, 'lon', N{j}, ...
            'truth', R{j}); %#ok<AGROW>
    end
    [~, ord] = sort([recs.batch]);
    recs = recs(ord);
end

% =====================================================================
%  写 RadarView 格式
% =====================================================================
function write_radarview(path, recs, mode)
    trackList = struct('BatchNo', {}, 'Type', {}, 'asscPointList', {}, 'smoothPointList', {});
    empty_pts = struct('time', {}, 'lat', {}, 'lon', {});
    for i = 1:numel(recs)
        pts = struct('time', {}, 'lat', {}, 'lon', {});
        for j = 1:numel(recs(i).t)
            pts(end+1) = struct('time', recs(i).t(j), ...
                'lat', recs(i).lat(j), 'lon', recs(i).lon(j)); %#ok<AGROW>
        end
        if isempty(pts), continue; end
        if strcmp(mode, 'assc')
            trackList(end+1) = struct('BatchNo', recs(i).batch, 'Type', 1, ...
                'asscPointList', pts, 'smoothPointList', empty_pts); %#ok<AGROW>
        else
            trackList(end+1) = struct('BatchNo', recs(i).batch, 'Type', 1, ...
                'asscPointList', empty_pts, 'smoothPointList', pts); %#ok<AGROW>
        end
    end
    save(path, 'trackList', '-v7');
end

% =====================================================================
%  自检：完全按 RadarView 的解析规则核对
% =====================================================================
function r = verify_radarview(path)
    r = struct('path', path, 'n_tracks', 0, 'n_points', 0, 'n_bad', 0, ...
        'max_gap_sec', 0, 'median_dt_sec', NaN, 'note', '');
    S = load(path);
    if ~isfield(S, 'trackList')
        r.note = 'FAIL 缺变量 trackList'; return;
    end
    L = S.trackList;
    if ~isstruct(L)
        r.note = 'FAIL trackList 不是结构体'; return;
    end
    n_bad = 0; n_pts = 0; gaps = []; dts = [];
    for i = 1:numel(L)
        e = L(i);
        ok = isfield(e, 'BatchNo') && isfield(e, 'Type');
        if ~ok, n_bad = n_bad + 1; continue; end
        pf = {'asscPointList', 'smoothPointList'};
        pts = [];
        for q = 1:numel(pf)
            if isfield(e, pf{q}) && isstruct(e.(pf{q})) && numel(e.(pf{q})) > 0
                pts = e.(pf{q}); break;
            end
        end
        if isempty(pts), n_bad = n_bad + 1; continue; end
        t = [pts.time]; la = [pts.lat]; lo = [pts.lon];
        n_pts = n_pts + numel(t);
        bad = sum(~isfinite(t) | t <= 0 | ~isfinite(la) | ~isfinite(lo) ...
            | abs(la) > 90 | abs(lo) > 180);
        n_bad = n_bad + bad;
        if numel(t) > 1
            d = diff(t) * 86400;
            dts = [dts, d]; %#ok<AGROW>
            gaps = [gaps, max(d)]; %#ok<AGROW>
        end
    end
    r.n_tracks = numel(L);
    r.n_points = n_pts;
    r.n_bad = n_bad;
    if ~isempty(gaps), r.max_gap_sec = max(gaps); end
    if ~isempty(dts), r.median_dt_sec = median(dts); end
    if n_bad == 0
        r.note = sprintf('OK  %2d 条 / %5d 点 / 中位帧间隔 %.0f s / 最大空洞 %.0f s', ...
            r.n_tracks, r.n_points, r.median_dt_sec, r.max_gap_sec);
    else
        r.note = sprintf('FAIL %d 个非法点', n_bad);
    end
end

% =====================================================================
%  长度对比：系统航迹 vs 单站（按真值编号对齐）
% =====================================================================
function cmp = compare_length(trk1, trk2, raw1, raw2, fus, outdir)
    % 基线必须用"单站跟踪帧数"，不能用原始量测点数：Pd=0.6 会把量测点数
    % 压到跟踪帧数的六成，拿它当分母会把长度比虚高到 1.5 以上。
    % 系统航迹的批号是 CT 编号而非真值编号，按 truth 字段归并；
    % 同一目标若被拆成两条 CT，取较长的那条（与长度指标口径一致）。
    % 数组长度要同时覆盖批号与真值编号：批号是 CT 编号（radar*100000+loc.id），
    % 与真值编号不同源；一个目标被拆成多条 CT 时，最大批号会小于最大真值编号，
    % 只按批号定长会越界。
    ti = [trk1.truth, trk2.truth, raw1.truth, raw2.truth, fus.truth];
    ti = ti(isfinite(ti));
    n = max([trk1.batch, trk2.batch, ti(:)']);
    n1 = zeros(n, 1); n2 = zeros(n, 1); nf = zeros(n, 1);
    d1 = zeros(n, 1); d2 = zeros(n, 1);
    for i = 1:numel(trk1)
        if isfinite(trk1(i).truth), n1(trk1(i).truth) = max(n1(trk1(i).truth), numel(trk1(i).t)); end
    end
    for i = 1:numel(trk2)
        if isfinite(trk2(i).truth), n2(trk2(i).truth) = max(n2(trk2(i).truth), numel(trk2(i).t)); end
    end
    for i = 1:numel(raw1), d1(raw1(i).truth) = numel(raw1(i).t); end
    for i = 1:numel(raw2), d2(raw2(i).truth) = numel(raw2(i).t); end
    for i = 1:numel(fus)
        if ~isfinite(fus(i).truth), continue; end
        t = fus(i).truth;
        nf(t) = max(nf(t), numel(fus(i).t));
    end
    single = max(n1, n2);
    ok = single > 0;
    cmp = struct('n_targets', sum(ok), ...
        'n_fused_ge_single', sum(nf(ok) >= single(ok)), ...
        'mean_single', mean(single(ok)), 'mean_fused', mean(nf(ok)), ...
        'min_ratio', min(nf(ok) ./ max(single(ok), 1)));

    fid = fopen(fullfile(outdir, 'length_compare.csv'), 'w');
    fprintf(fid, 'truth,n_R1_det,n_R2_det,n_R1_trk,n_R2_trk,single_max,n_fused,ratio\n');
    for i = 1:n
        fprintf(fid, '%d,%d,%d,%d,%d,%d,%d,%.4f\n', i, d1(i), d2(i), n1(i), n2(i), ...
            single(i), nf(i), nf(i) / max(single(i), 1));
    end
    fclose(fid);

    fprintf('\n[长度对比] 目标 %d 条：系统航迹 ≥ 较长单站 %d 条（均值 单站 %.1f → 系统 %.1f 帧，最小比 %.3f）\n', ...
        cmp.n_targets, cmp.n_fused_ge_single, cmp.mean_single, cmp.mean_fused, cmp.min_ratio);
end
