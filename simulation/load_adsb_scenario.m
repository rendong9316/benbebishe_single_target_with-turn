function scenario = load_adsb_scenario(scene_id, params)
%LOAD_ADSB_SCENARIO 从真实 ADS-B 航迹构建仿真场景
%
%   scenario = load_adsb_scenario(scene_id, params)
%
%   支持两种输入格式：
%     1) 派生场景 CSV（data/adsb/<scene_id>.csv），列序
%        icao, seg, frame, t_sec, lat, lon, alt_ft, spd_kt, hdg_deg, callsign
%     2) FR24 原始 19 列 CSV（data/adsb_ns/scene_fr24.csv），场景名固定为
%        'adsb_ns_fr24'。该文件由 _analysis/adsb_scene_ns.py 从 FR24 原始报文
%        提取南北向航迹后导出，列序见 FR24 官方格式：
%        1 ICAO, 2 纬度, 3 经度, 4 航向, 5 高度ft, 6 地速kt, 8 数据源,
%        11 时间戳, 17 呼号
%
%   输出的 scenario 结构体与 build_truth_scenario 完全一致，可直接喂给
%   run.m / run_online.m 的下游（检测生成、跟踪、分配、融合）。
%
%   与合成场景的区别：
%     1. 航迹来自真实民航 ADS-B 报告（参考真值，位置精度约 30 m）
%     2. 目标数由真实空中交通决定（场景内每帧活跃数不同）
%     3. 航路交叉、转弯、速度波动均为真实行为
%     4. 只取巡航段（高度 >= 20000 ft），故为 2D 平面模型
%
%   可选参数（params）：
%     .adsb_scene_include   — cellstr，仅保留这些 ICAO（空表示全部）
%     .adsb_scene_max_targets — 最多保留的目标数（空表示不限）

    if nargin < 2 || isempty(params)
        params = simulation_params_oracle();
    end

    dt = params.dt_sec;
    root = fileparts(fileparts(mfilename('fullpath')));   % simulation/ -> 项目根

    % FR24 原始 19 列场景：场景名 -> 文件名
    fr24_map = {'adsb_ns_fr24', 'scene_fr24.csv';    % 37 条走廊内航迹
                'adsb_ns_all',  'scene_fr24_all.csv'}; % 57 条 = 走廊 37 + 远距 20
    is_fr24 = any(strcmp(char(scene_id), fr24_map(:, 1)));
    if is_fr24
        fn = fr24_map{strcmp(char(scene_id), fr24_map(:, 1)), 2};
        csv_path = fullfile(root, '..', 'data', 'adsb_ns', fn);
    else
        csv_path = fullfile(root, 'data', 'adsb', [char(scene_id) '.csv']);
    end
    if ~isfile(csv_path)
        error('load_adsb_scenario:notFound', ...
            '场景文件不存在：%s\n派生场景请运行 _analysis/build_adsb_scene.py；FR24 场景请运行 _analysis/adsb_scene_ns.py', ...
            csv_path);
    end

    if is_fr24
        [icao_all, seg_all, frame_all, lat_all, lon_all, spd_all, cs_all] = ...
            read_fr24_raw(csv_path, dt);
    else
        [icao_all, seg_all, frame_all, lat_all, lon_all, spd_all, cs_all] = ...
            read_derived_csv(csv_path);
    end

    % ---- 目标筛选（可选）----
    keep = true(size(frame_all));
    if isfield(params, 'adsb_scene_include') && ~isempty(params.adsb_scene_include)
        want = params.adsb_scene_include;
        if ischar(want), want = {want}; end
        keep = false(size(frame_all));
        for i = 1:numel(want)
            keep = keep | strcmp(icao_all, want{i});
        end
    end
    icao_all = icao_all(keep);
    seg_all = seg_all(keep);
    frame_all = frame_all(keep);
    lat_all = lat_all(keep);
    lon_all = lon_all(keep);
    spd_all = spd_all(keep);
    cs_all = cs_all(keep);
    if isempty(frame_all)
        error('load_adsb_scenario:empty', '场景 %s 筛选后没有数据行', char(scene_id));
    end

    key = strcat(icao_all, '|', seg_all);
    % 手动去重，保持首次出现顺序。
    % unique（string 与 cellstr 两种）和 containers.Map 在本机 MATLAB 上
    % 处理这个规模的键都会触发访问违例，逐条 strcmp 是唯一稳定的写法。
    ukey = cell(0, 1);
    gidx = zeros(numel(key), 1);
    for i = 1:numel(key)
        s = key{i};
        hit = 0;
        for j = 1:numel(ukey)
            if strcmp(ukey{j}, s), hit = j; break; end
        end
        if hit == 0
            ukey{end+1, 1} = s; %#ok<AGROW>
            hit = numel(ukey);
        end
        gidx(i) = hit;
    end

    truth_all   = {};
    truthTrajs  = {};
    meta_labels = {};

    for i = 1:numel(ukey)
        idx = find(gidx == i);
        [f, ord] = sort(frame_all(idx));
        idx = idx(ord);

        % 按帧缺口拆段（保证 interp1 不在缺口处凭空造目标）
        brk = [0; find(diff(f) > 1); numel(f)];
        for s = 1:numel(brk)-1
            a = brk(s) + 1;  b = brk(s+1);
            if b - a + 1 < 10, continue; end    % 少于 10 帧（5 分钟）的段丢弃

            ii  = idx(a:b);
            t   = f(a:b) * dt;                  % 秒
            lon = lon_all(ii);
            lat = lat_all(ii);
            spd_kt = mean(spd_all(ii));

            % 速度（度/秒）：用中心差分，端点用单侧差分
            lon_rate = gradient(lon, t);
            lat_rate = gradient(lat, t);

            % 标签：优先呼号，缺失则用 ICAO
            cs = cs_all{ii(1)};
            if isempty(cs)
                lbl = icao_all{ii(1)};
            else
                lbl = cs;
            end
            if numel(brk) > 2
                lbl = sprintf('%s#%d', lbl, s);
            end

            truth_all{end+1, 1} = [lon, lat, lon_rate, lat_rate, t]; %#ok<AGROW>
            truthTrajs{end+1, 1} = struct( ...
                'label',      lbl, ...
                'speed_ms',   spd_kt * 0.514444, ...
                'time_sec',   t, ...
                'lat',        lat, ...
                'lon',        lon, ...
                'lon_rate',   lon_rate, ...
                'lat_rate',   lat_rate); %#ok<AGROW>
            meta_labels{end+1, 1} = lbl; %#ok<AGROW>
        end
    end

    if isempty(truth_all)
        error('load_adsb_scenario:empty', '场景 %s 未解析出任何有效航迹', char(scene_id));
    end

    % ---- 按起始时间排序（保证场景顺序与真实时序一致，可复现）----
    start_t = zeros(1, numel(truth_all));
    for i = 1:numel(truth_all)
        start_t(i) = truth_all{i}(1, 5);
    end
    [~, ord_t] = sort(start_t);
    truth_all = truth_all(ord_t);
    truthTrajs = truthTrajs(ord_t);
    meta_labels = meta_labels(ord_t);

    % ---- 目标数量上限（可选，便于快速实验）----
    if isfield(params, 'adsb_scene_max_targets') && ~isempty(params.adsb_scene_max_targets)
        nmax = min(numel(truth_all), round(params.adsb_scene_max_targets));
        truth_all = truth_all(1:nmax);
        truthTrajs = truthTrajs(1:nmax);
        meta_labels = meta_labels(1:nmax);
    end

    % ---- 全局时间网格（与 build_truth_scenario 完全一致的口径）----
    max_duration = 0;
    for ac = 1:numel(truth_all)
        max_duration = max(max_duration, truth_all{ac}(end, 5));
    end
    t1_grid = params.time_offset_radar1_sec : dt : max_duration;
    t2_grid = params.time_offset_radar2_sec : dt : max_duration;
    n_frames = min(numel(t1_grid), numel(t2_grid));
    t1_grid = t1_grid(1:n_frames);
    t2_grid = t2_grid(1:n_frames);

    scenario = struct();
    scenario.name      = char(scene_id);
    scenario.n_targets = numel(truth_all);
    scenario.truth_all = truth_all;
    scenario.truthTrajs = truthTrajs;
    scenario.t1_grid   = t1_grid;
    scenario.t2_grid   = t2_grid;
    scenario.n_frames  = n_frames;
    scenario.source    = 'adsb';
    scenario.labels    = meta_labels;
end

% =====================================================================
% 派生场景 CSV（10 列，带表头）
% =====================================================================
function [icao, seg, frame, lat, lon, spd, cs] = read_derived_csv(csv_path)
    fid = fopen(csv_path, 'r');
    if fid < 0
        error('load_adsb_scenario:openFail', '无法打开场景文件：%s', csv_path);
    end
    fgetl(fid);   % 跳过表头
    C = textscan(fid, '%s %s %f %f %f %f %f %f %f %s', 'Delimiter', ',');
    fclose(fid);
    if isempty(C{1})
        error('load_adsb_scenario:empty', '场景文件没有数据行：%s', csv_path);
    end
    icao = C{1};   seg = C{2};   frame = C{3};
    % 列序：icao, seg, frame, t_sec, lat, lon, alt_ft, spd_kt, hdg_deg, callsign
    lat = C{5};  lon = C{6};  spd = C{8};  cs = C{10};
end

% =====================================================================
% FR24 原始 19 列 CSV（无表头）
%   1 ICAO, 2 纬度, 3 经度, 4 航向, 5 高度ft, 6 地速kt, 8 数据源,
%   11 时间戳 'YYYY-MM-DD HH:MM:SS.mmm', 17 呼号
% =====================================================================
function [icao, seg, frame, lat, lon, spd, cs] = read_fr24_raw(csv_path, dt)
    fid = fopen(csv_path, 'r');
    if fid < 0
        error('load_adsb_scenario:openFail', '无法打开 FR24 文件：%s', csv_path);
    end
    fmt = repmat('%s', 1, 19);
    C = textscan(fid, fmt, 'Delimiter', ',');
    fclose(fid);
    if isempty(C{1})
        error('load_adsb_scenario:empty', 'FR24 文件没有数据行：%s', csv_path);
    end

    icao = C{1};
    lat  = str2double(C{2});
    lon  = str2double(C{3});
    spd  = str2double(C{6});
    ts   = C{11};
    cs   = C{17};

    valid = isfinite(lat) & isfinite(lon) & isfinite(spd) & ~cellfun(@isempty, ts);
    icao = icao(valid);  lat = lat(valid);  lon = lon(valid);
    spd = spd(valid);  ts = ts(valid);  cs = cs(valid);

    % 时间戳 → 全局绝对秒；帧号 = 相对最早时刻的 30 s 栅格
    t_abs = zeros(numel(ts), 1);
    for i = 1:numel(ts)
        s = ts{i};
        if numel(s) >= 19, s = s(1:19); end
        t_abs(i) = (datenum(s, 'yyyy-mm-dd HH:MM:SS') - ...
                    datenum('2000-01-01 00:00:00')) * 86400;
    end
    t0 = min(t_abs);
    frame = round((t_abs - t0) / dt);

    % 同一 ICAO 内部按帧缺口切段，段号用于区分同一架飞机的多段
    seg = repmat({'0'}, numel(frame), 1);
    [uicao, ~, g] = unique(icao, 'stable');
    for gi = 1:numel(uicao)
        idx = find(g == gi);
        [f, ord] = sort(frame(idx));
        idx = idx(ord);
        brk = [0; find(diff(f) > 1); numel(f)];
        for s = 1:numel(brk)-1
            a = brk(s) + 1;  b = brk(s+1);
            seg(idx(a:b)) = repmat({sprintf('%d', s)}, b - a + 1, 1);
        end
    end
end
