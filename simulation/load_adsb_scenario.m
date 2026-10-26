function scenario = load_adsb_scenario(scene_id, params)
%LOAD_ADSB_SCENARIO 从真实 ADS-B 航迹构建仿真场景
%
%   scenario = load_adsb_scenario(scene_id, params)
%
%   读取 data/adsb/<scene_id>.csv（由 _analysis/build_adsb_scene.py 生成），
%   构造与 build_truth_scenario 完全兼容的 scenario 结构体，可直接喂给
%   run.m / prepare_oracle_tracking_inputs 的下游（检测生成、跟踪、融合）。
%
%   与合成场景的区别：
%     1. 航迹来自真实民航 ADS-B 报告（参考真值，位置精度约 30 m）
%     2. 目标数由真实空中交通决定（场景内每帧活跃数不同）
%     3. 航路交叉、转弯、速度波动均为真实行为
%     4. 只取巡航段（高度 >= 20000 ft），故为 2D 平面模型
%
%   输入：
%     scene_id — 场景标识，如 'adsb_0002'
%     params   — 全局参数（需含 dt_sec / time_offset_radar1_sec /
%                time_offset_radar2_sec）；缺省时自动加载
%
%   输出：
%     scenario — 结构体，字段与 build_truth_scenario 一致：
%                name / n_targets / truth_all / truthTrajs /
%                t1_grid / t2_grid / n_frames
%
%   注意：本函数只读 CSV，不修改任何源码；接入主链路需在
%        build_truth_scenario 中增加分支（待批准）。

    if nargin < 2 || isempty(params)
        params = simulation_params_oracle();
    end

    dt = params.dt_sec;
    root = fileparts(fileparts(mfilename('fullpath')));   % simulation/ -> 项目根
    csv_path = fullfile(root, 'data', 'adsb', [scene_id '.csv']);
    if ~isfile(csv_path)
        error('load_adsb_scenario:notFound', ...
            '场景文件不存在：%s\n请先运行 _analysis/build_adsb_scene.py', csv_path);
    end

    % 显式指定列类型：ICAO 形如 T01586，readtable 会误判为 datetime（T 是 ISO 时间分隔符）
    opts = detectImportOptions(csv_path, 'TextType', 'string');
    opts = setvartype(opts, {'icao', 'callsign'}, 'string');
    opts = setvartype(opts, {'seg', 'frame'}, 'double');
    T = readtable(csv_path, opts);
    T.icao     = string(T.icao);
    T.seg      = string(T.seg);
    T.callsign = string(T.callsign);
    key = T.icao + "|" + T.seg;
    ukey = unique(key, 'stable');

    truth_all   = {};
    truthTrajs  = {};
    meta_labels = {};

    for i = 1:numel(ukey)
        sub = T(key == ukey(i), :);
        [~, ord] = sort(sub.frame);
        sub = sub(ord, :);

        f = double(sub.frame);
        % 按帧缺口拆段（保证 interp1 不在缺口处凭空造目标）
        brk = [0; find(diff(f) > 1); numel(f)];
        for s = 1:numel(brk)-1
            a = brk(s) + 1;  b = brk(s+1);
            if b - a + 1 < 10, continue; end    % 少于 10 帧（5 分钟）的段丢弃

            t   = f(a:b) * dt;                  % 秒
            lon = double(sub.lon(a:b));
            lat = double(sub.lat(a:b));
            spd_kt = mean(double(sub.spd_kt(a:b)));

            % 速度（度/秒）：用中心差分，端点用单侧差分
            lon_rate = gradient(lon, t);
            lat_rate = gradient(lat, t);

            % 标签：优先呼号，缺失则用 ICAO
            cs = sub.callsign(a);
            if ismissing(cs) || strlength(cs) == 0
                lbl = char(sub.icao(a));
            else
                lbl = char(cs);
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
        error('load_adsb_scenario:empty', '场景 %s 未解析出任何有效航迹', scene_id);
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
