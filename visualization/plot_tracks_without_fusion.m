% =========================================================================
% plot_tracks_without_fusion.m — 无融合模式单站航迹可视化
% =========================================================================
% 【功能】
%   绘制 Oracle 单站航迹维护结果，包含以下图层：
%     1. 真值航迹（黄/品红/青色虚线）
%     2. R1/R2 校准点迹连线（蓝色系/红色系）
%     3. R1/R2 航迹提取点（asscPointList 中的关联点迹，菱形标记）
%     4. R1/R2 UKF 滤波航迹（深蓝/深红色实线）
%     5. 雷达站标记（接收站方块 + 发射站三角）
%
%   右侧提供图层控制复选框，可独立切换各图层的可见性。
%
% 【输入】
%   truth_all       — 真值航迹 cell 数组
%   detList_R1/R2   — 两站点迹 cell 数组
%   trackSnapshots_R1/R2 — 两站航迹快照 cell 数组
%   trackList_R1/R2   — 两站最终航迹列表
%   params            — 仿真参数结构体
% =========================================================================
function plot_tracks_without_fusion(truth_all, detList_R1, detList_R2, ...
        trackSnapshots_R1, trackSnapshots_R2, trackList_R1, trackList_R2, params, varargin)

    % 创建图窗，设置标题和尺寸
    % 图窗分为两部分：左侧 70% 为地图区域，右侧 30% 为图层控制面板
    fig = figure('Name', 'Figure 4 - Oracle 单站航迹维护', 'Position', [50, 50, 1400, 750]);
    try
        % 尝试使用 darkwater 暗色底图
        ax = geoaxes('Units', 'normalized', 'Position', [0.04, 0.10, 0.70, 0.88]);
        ax.Basemap = 'darkwater';
    catch
        % 降级为默认底图
        ax = geoaxes('Units', 'normalized', 'Position', [0.04, 0.10, 0.70, 0.88]);
    end
    hold(ax, 'on');  % 保持坐标系，允许后续多层叠加绘图

    h_layers = {};    % 存储所有绘图句柄，供图层控制使用
    layer_names = {}; % 存储图层名称，用于复选框显示
    study = struct();
    if ~isempty(varargin), study = varargin{1}; end
    truth_colors = {[1 1 0], [1 0 1], [0 1 1]};  % 真值航迹颜色：黄、品红、青

    % ---- 真值航迹 ----
    % 遍历所有真值航迹，用不同颜色绘制
    for ac = 1:length(truth_all)
        tt = truth_all{ac};                  % 取出第 ac 架飞机的真值航迹
        color = truth_colors{min(ac, length(truth_colors))};  % 循环取色
        % 绘制虚线方块航迹，--s 表示 dashed line with square marker
        % tt(:,2)=lat 作为 y 轴，tt(:,1)=lon 作为 x 轴
        h = geoplot(ax, tt(:, 2), tt(:, 1), '--s', 'Color', color, ...
            'LineWidth', 1.5, 'MarkerSize', 5, 'MarkerFaceColor', color, ...
            'DisplayName', sprintf('真值%c', char('A' + ac - 1)));
        % 将绘图句柄和图层名添加到管理列表
        [h_layers, layer_names] = add_layer(h_layers, layer_names, h, ...
            sprintf('真值%c', char('A' + ac - 1)));
    end

    % ---- R1 校准点迹连线 ----
    % plot_calibrated_detection_tracks 将同一 aircraft_id 的连续检测点迹连成线
    % [0.30 0.65 1.00] 是淡蓝色，与 R1 接收站的蓝色标记呼应
    h = plot_calibrated_detection_tracks(ax, detList_R1, [0.30 0.65 1.00], 'R1校准点迹');
    [h_layers, layer_names] = add_layer(h_layers, layer_names, h, 'R1校准点迹连线');
    % plot_consumed_detection_tracks 从 trackList 中提取 asscPointList 历史
    % [0.00 0.25 0.85] 是中蓝色，比校准点迹更深
    h = plot_consumed_detection_tracks(ax, trackList_R1, [0.00 0.25 0.85], 'R1航迹提取点');
    [h_layers, layer_names] = add_layer(h_layers, layer_names, h, 'R1航迹提取点连线');
    % plot_filter_tracks 从 trackSnapshots 中提取 UKF 估计位置
    % [0.00 0.00 0.55] 是深蓝色，代表最终滤波输出的航迹
    h = plot_filter_tracks(ax, trackSnapshots_R1, [0.00 0.00 0.55], 'R1 UKF');
    [h_layers, layer_names] = add_layer(h_layers, layer_names, h, 'R1 UKF航迹');

    % ---- R2 校准点迹连线 ----
    % R2 使用红色系配色，与 R1 的蓝色系区分
    h = plot_calibrated_detection_tracks(ax, detList_R2, [1.00 0.55 0.40], 'R2校准点迹');
    [h_layers, layer_names] = add_layer(h_layers, layer_names, h, 'R2校准点迹连线');
    h = plot_consumed_detection_tracks(ax, trackList_R2, [0.85 0.15 0.05], 'R2航迹提取点');
    [h_layers, layer_names] = add_layer(h_layers, layer_names, h, 'R2航迹提取点连线');
    h = plot_filter_tracks(ax, trackSnapshots_R2, [0.55 0.00 0.00], 'R2 UKF');
    [h_layers, layer_names] = add_layer(h_layers, layer_names, h, 'R2 UKF航迹');

    [h_layers, layer_names] = plot_study_layers(ax, study, h_layers, layer_names);

    % ---- 雷达站标记 ----
    % 接收站用蓝色/红色方块(bs/rs)，发射站用蓝色/红色上三角(b^/r^)
    geoplot(ax, params.radar1_lat, params.radar1_lon, 'bs', ...
        'MarkerSize', 14, 'MarkerFaceColor', 'b', 'DisplayName', 'R1 Rx');
    geoplot(ax, params.radar2_lat, params.radar2_lon, 'rs', ...
        'MarkerSize', 14, 'MarkerFaceColor', 'r', 'DisplayName', 'R2 Rx');
    geoplot(ax, params.radar1_tx_lat, params.radar1_tx_lon, 'b^', ...
        'MarkerSize', 10, 'DisplayName', 'R1 Tx');
    geoplot(ax, params.radar2_tx_lat, params.radar2_tx_lon, 'r^', ...
        'MarkerSize', 10, 'DisplayName', 'R2 Tx');

    title(ax, 'Oracle单站航迹维护：校准点迹、航迹提取点与UKF输出');
    legend(ax, 'Location', 'northeastoutside');  % 图例放在地图区域外右上角
    % 安装图层控制复选框（右侧面板）
    install_layer_controls(fig, h_layers, layer_names, trackList_R1, trackList_R2, params);
    drawnow;  % 强制刷新图形显示
end

% =========================================================================
% plot_calibrated_detection_tracks — 校准点迹连线绘制
% =========================================================================
% 按 aircraft_id 分组，将同一目标的连续检测点迹用线连接。
% 断帧处插入 NaN 断开连线（避免跨帧连线造成视觉误导）。
function h = plot_calibrated_detection_tracks(ax, detList, color, label)
    h = gobjects(0);  % 初始化空句柄数组
    aircraft_ids = collect_detection_aircraft_ids(detList);  % 收集所有出现过的目标ID
    for ac = aircraft_ids
        % 提取该目标 ID 的连续点迹坐标序列（含 NaN 断帧标记）
        [lat, lon] = detection_line(detList, ac);
        % 如果没有有效点迹，跳过该目标
        if sum(~isnan(lat)) == 0
            continue;
        end
        % 绘制带圆形标记的连线
        % '-o' 表示实线加圆形标记
        hp = geoplot(ax, lat, lon, '-o', 'Color', color, 'LineWidth', 1.6, ...
            'MarkerSize', 5, 'MarkerFaceColor', color, ...
            'DisplayName', sprintf('%s T%d', label, ac));  % 图例标签含目标编号
        h(end+1) = hp;  % 累积句柄
    end
end

% =========================================================================
% collect_detection_aircraft_ids — 收集检测中的 aircraft_id
% =========================================================================
% 遍历所有帧的检测数据，提取真实目标的 aircraft_id（排除杂波和无效点）
function ids = collect_detection_aircraft_ids(detList)
    ids = [];  % 初始化空数组
    for k = 1:length(detList)
        dets = detList{k};  % 第 k 帧的所有检测
        for i = 1:length(dets)
            % 只收集真实目标的 aircraft_id（非杂波且 id > 0）
            % is_clutter=false 表示这是真实目标的检测
            % aircraft_id>0 排除了未关联的孤立检测
            if ~dets(i).is_clutter && isfield(dets(i), 'aircraft_id') && dets(i).aircraft_id > 0
                ids(end+1) = double(dets(i).aircraft_id);  % 转为 double 类型
            end
        end
    end
    ids = unique(ids);  % 去重排序，得到唯一的 ID 集合
end

% =========================================================================
% detection_line — 按 aircraft_id 提取连续点迹坐标
% =========================================================================
% 遍历所有帧，提取该 aircraft_id 的检测点迹。
% 如果两帧之间不连续（k > previous_frame + 1），插入 NaN 断开连线。
% NaN 在 plot/geoplot 中会自动断开线段，避免跨帧连线造成视觉误导。
function [lat, lon] = detection_line(detList, aircraft_id)
    lat = [];  % 初始化 lat 数组
    lon = [];  % 初始化 lon 数组
    previous_frame = NaN;  % 记录上一帧的帧号，用于检测断帧
    for k = 1:length(detList)
        dets = detList{k};  % 第 k 帧的所有检测
        idx = 0;  % 初始化索引，0 表示本帧未找到该目标
        for i = 1:length(dets)
            % 查找当前帧中属于该 aircraft_id 的检测点
            % 条件：非杂波、ID 匹配、经纬度有效
            if ~dets(i).is_clutter && double(dets(i).aircraft_id) == aircraft_id ...
                    && ~isnan(dets(i).lat) && ~isnan(dets(i).lon)
                idx = i;  % 找到了，记录索引
                break;    % 跳出内层循环
            end
        end
        if idx == 0
            continue;  % 本帧没有找到该目标，跳过
        end
        % 断帧检测：如果当前帧与上一帧不连续（中间有缺失帧），插入 NaN 断开
        % 例如：上一帧是第5帧，当前帧是第8帧，说明第6、7帧丢失了数据
        if ~isnan(previous_frame) && k > previous_frame + 1
            lat(end+1) = NaN;  % 插入 NaN 断开连线
            lon(end+1) = NaN;
        end
        lat(end+1) = dets(idx).lat;  % 累加有效经度
        lon(end+1) = dets(idx).lon;  % 累加有效纬度
        previous_frame = k;          % 更新上一帧号
    end
end

% =========================================================================
% plot_consumed_detection_tracks — 航迹消耗点迹连线绘制
% =========================================================================
% 从 trackList 中提取每条航迹的 asscPointList（关联点迹历史），
% 按 frameID 排序后连成线。asscPointList 记录了该航迹在各个帧中
% 被分配到的原始检测点。
function h = plot_consumed_detection_tracks(ax, trackList, color, label)
    h = gobjects(0);  % 初始化空句柄数组
    for i = 1:length(trackList)
        trk = trackList{i};  % 取出第 i 条航迹
        % 检查航迹是否有 asscPointList 字段
        if ~isfield(trk, 'asscPointList') || isempty(trk.asscPointList)
            continue;  % 没有关联点迹数据则跳过
        end
        % point_cell_line 从 asscPointList cell 中提取有序坐标序列
        [lat, lon] = point_cell_line(trk.asscPointList);
        % 如果没有有效点迹，跳过
        if sum(~isnan(lat)) == 0
            continue;
        end
        % 绘制带菱形标记的连线，'-d' 表示 dashed diamond marker
        hp = geoplot(ax, lat, lon, '-d', 'Color', color, 'LineWidth', 1.8, ...
            'MarkerSize', 6, 'MarkerFaceColor', color, ...
            'DisplayName', sprintf('%s #%d/T%d', label, trk.id, trk.truth_idx));
        h(end+1) = hp;  % 累积句柄
    end
end

% =========================================================================
% point_cell_line — 从 asscPointList cell 中提取有序坐标序列
% =========================================================================
% 按 frameID 排序，去重（同帧只保留一个点），断帧处插入 NaN。
% asscPointList 可能包含重复帧号（如同一帧有多个候选点被关联），
% 需要用 unique 去重；也可能存在跳帧（目标短暂丢失后恢复），
% 需要插入 NaN 断开连线。
function [lat, lon] = point_cell_line(pointList)
    frames = [];  % 存储帧号
    points = {};  % 存储对应的检测点结构体
    for i = 1:length(pointList)
        dp = pointList{i};  % 第 i 个检测点
        % 跳过无效条目：空、非结构体、缺少必要字段、经纬度为 NaN
        if isempty(dp) || ~isstruct(dp) || ~isfield(dp, 'frameID') ...
                || ~isfield(dp, 'lat') || ~isfield(dp, 'lon') ...
                || isnan(dp.lat) || isnan(dp.lon)
            continue;
        end
        frames(end+1) = double(dp.frameID);  % 提取帧号
        points{end+1} = dp;                   % 保存完整结构体
    end
    if isempty(frames)
        lat = [];  % 没有有效数据则返回空数组
        lon = [];
        return;
    end
    % 按 frameID 排序，order 是排序后的索引
    % 排序后才能按时间顺序连接点迹
    [frames, order] = sort(frames);
    points = points(order);  % 按排序索引重新排列点
    % 去重（stable 保留第一个出现的）
    % 同帧可能有多个候选点被关联，只取第一个
    [frames, unique_idx] = unique(frames, 'stable');
    points = points(unique_idx);
    lat = [];  % 初始化输出数组
    lon = [];
    for i = 1:length(points)
        % 断帧检测：如果当前帧与前一个帧不连续，插入 NaN
        if i > 1 && frames(i) > frames(i-1) + 1
            lat(end+1) = NaN;  % 插入 NaN 断开连线
            lon(end+1) = NaN;
        end
        lat(end+1) = points{i}.lat;  % 累加有效纬度
        lon(end+1) = points{i}.lon;  % 累加有效经度
    end
end

% =========================================================================
% plot_filter_tracks — UKF 滤波航迹绘制
% =========================================================================
% 从 trackSnapshots 中提取所有活跃航迹的 UKF 估计位置，
% 按 ID 分组后连成线。UKF 航迹是经过 UKF（无迹卡尔曼滤波）平滑后的
% 最终输出，代表了滤波器的状态估计。
function h = plot_filter_tracks(ax, snapshots, color, label)
    h = gobjects(0);  % 初始化空句柄数组
    ids = collect_snapshot_track_ids(snapshots);  % 收集所有出现过的航迹ID
    for id = ids
        % 提取该 ID 航迹的连续坐标序列
        [lat, lon] = snapshot_track_line(snapshots, id);
        % 至少需要 2 个点才能画线，少于 2 个点跳过
        if sum(~isnan(lat)) < 2
            continue;
        end
        % 绘制实线，'-' 表示连续实线
        % UKF 航迹通常比原始检测更平滑，所以用更粗的线宽
        hp = geoplot(ax, lat, lon, '-', 'Color', color, 'LineWidth', 2.2, ...
            'DisplayName', sprintf('%s #%d', label, id));
        h(end+1) = hp;  % 累积句柄
    end
end

% =========================================================================
% collect_snapshot_track_ids — 收集快照中出现的所有航迹 ID
% =========================================================================
% 遍历所有帧的快照，提取每条航迹的 ID，去重后返回唯一 ID 列表
function ids = collect_snapshot_track_ids(snapshots)
    ids = [];  % 初始化空数组
    for k = 1:length(snapshots)
        snap = snapshots{k};  % 第 k 帧的快照
        % 检查快照是否为空且有 trackList 字段
        if isempty(snap) || ~isfield(snap, 'trackList')
            continue;
        end
        for i = 1:length(snap.trackList)
            ids(end+1) = snap.trackList{i}.id;  % 累加航迹ID
        end
    end
    ids = unique(ids);  % 去重排序
end

% =========================================================================
% snapshot_track_line — 从快照中提取指定 ID 航迹的连续坐标
% =========================================================================
% 遍历所有帧的快照，找到指定 track_id 的航迹，提取其 lat/lon。
% 断帧处插入 NaN 断开连线。type==7 表示该航迹已被删除/终止，需要跳过。
function [lat, lon] = snapshot_track_line(snapshots, track_id)
    lat = [];  % 初始化输出数组
    lon = [];
    previous_frame = NaN;  % 记录上一帧号，用于检测断帧
    for k = 1:length(snapshots)
        snap = snapshots{k};  % 第 k 帧的快照
        trk = [];  % 初始化航迹变量
        % 在快照的 trackList 中查找匹配的航迹 ID
        if ~isempty(snap) && isfield(snap, 'trackList')
            for i = 1:length(snap.trackList)
                candidate = snap.trackList{i};
                if candidate.id == track_id  % 找到匹配的 ID
                    trk = candidate;  % 保存该航迹
                    break;
                end
            end
        end
        % 跳过无效航迹：不存在、已终止(type==7)、经纬度为NaN
        if isempty(trk) || trk.type == 7 || isnan(trk.lat) || isnan(trk.lon)
            continue;
        end
        % 断帧检测：如果当前帧与上一帧不连续，插入 NaN 断开
        if ~isnan(previous_frame) && k > previous_frame + 1
            lat(end+1) = NaN;  % 插入 NaN 断开连线
            lon(end+1) = NaN;
        end
        lat(end+1) = trk.lat;  % 累加有效纬度
        lon(end+1) = trk.lon;  % 累加有效经度
        previous_frame = k;    % 更新上一帧号
    end
end

function [h_layers, names] = plot_study_layers(ax, study, h_layers, names)
    if isempty(fieldnames(study)) || ~isfield(study, 'segments'), return; end
    colors = {[0.15 0.55 1.00], [1.00 0.35 0.20]};
    for i = 1:numel(study.segments)
        seg = study.segments(i); color = colors{seg.radar_id};
        idx = ismember(seg.raw_frames, seg.effective_frames);
        if sum(idx) >= 2
            h = geoplot(ax, seg.lats(idx), seg.lons(idx), '-', 'Color', color, 'LineWidth', 2.6, ...
                'DisplayName', sprintf('R%d段%d #%d有效', seg.radar_id, seg.segment_id, seg.track_id));
            [h_layers, names] = add_layer(h_layers, names, h, sprintf('R%d段%d有效', seg.radar_id, seg.segment_id));
        end
        idx = ismember(seg.raw_frames, seg.tail_frames);
        if sum(idx) >= 2
            h = geoplot(ax, seg.lats(idx), seg.lons(idx), '--', 'Color', 0.65*color+0.35, 'LineWidth', 1.8, ...
                'DisplayName', sprintf('R%d段%d coasting tail', seg.radar_id, seg.segment_id));
            [h_layers, names] = add_layer(h_layers, names, h, sprintf('R%d段%d tail', seg.radar_id, seg.segment_id));
        end
    end
    if isfield(study, 'published')
        for i = 1:numel(study.published)
            pub = study.published(i); [lat, lon] = fused_line(pub.snapshots);
            if sum(isfinite(lat)) < 2, continue; end
            h = geoplot(ax, lat, lon, '-', 'Color', [0.20 0.90 0.35], 'LineWidth', 3.4, ...
                'DisplayName', sprintf('Group%d %s RMSE %.2fkm', pub.group_id, pub.method, pub.rmse_km));
            [h_layers, names] = add_layer(h_layers, names, h, sprintf('G%d最佳%s %.2fkm', pub.group_id, pub.method, pub.rmse_km));
        end
    end
end

function [lat, lon] = fused_line(snapshots)
    lat = []; lon = []; previous = NaN;
    for k = 1:numel(snapshots)
        if isempty(snapshots{k}) || isempty(snapshots{k}.trackList), continue; end
        trk = snapshots{k}.trackList{1};
        if ~isnan(previous) && k > previous + 1, lat(end+1)=NaN; lon(end+1)=NaN; end %#ok<AGROW>
        lat(end+1)=trk.lat; lon(end+1)=trk.lon; previous=k; %#ok<AGROW>
    end
end

% =========================================================================
% 辅助函数：图层管理和 UI 控制
% =========================================================================

% =========================================================================
% add_layer — 将绘图句柄添加到图层列表
% =========================================================================
% 将 geoplot/plot3 返回的绘图句柄追加到 h_layers 和 names 元胞数组中
% 用于后续的图层可见性控制
function [h_layers, names] = add_layer(h_layers, names, h, name)
    % 如果绘图句柄为空（没有实际绘制内容），直接返回
    if isempty(h)
        return;
    end
    h_layers{end+1} = h;  % 将句柄追加到图层列表末尾
    names{end+1} = name;  % 将图层名追加到名称列表末尾
end

% =========================================================================
% install_layer_controls — 安装右侧图层控制面板
% =========================================================================
% 在图窗右侧创建：
%   1. 复选框列表：每个图层一个，可独立切换可见性
%   2. 全部隐藏/全部显示按钮：批量操作
%   3. 底部状态栏：显示 R1/R2 航迹数量和仿真参数
function install_layer_controls(fig, h_layers, names, trackList_R1, trackList_R2, params)
    panel = uipanel('Parent', fig, 'Units', 'normalized', 'Position', [0.75 0.11 0.24 0.84], ...
        'BackgroundColor', [1 1 1], 'BorderType', 'none');
    cb = gobjects(1, length(names));
    rows = max(length(names), 1);
    row_h = min(0.055, 0.92 / rows);
    for i = 1:length(names)
        ypos = 0.97 - i * row_h;
        cb(i) = uicontrol('Parent', panel, 'Style', 'checkbox', 'String', names{i}, ...
            'Value', 1, 'Units', 'normalized', 'Position', [0.02, ypos, 0.96, row_h], ...
            'FontSize', 8, 'BackgroundColor', [1 1 1], ...
            'Callback', @(src, ~) set_layer_visibility(h_layers{i}, src.Value));
    end
    % 全部隐藏按钮：点击后将所有复选框设为 0，所有图层设为不可见
    uicontrol('Parent', fig, 'Style', 'pushbutton', 'String', '全部隐藏', ...
        'Units', 'normalized', 'Position', [0.76, 0.055, 0.10, 0.04], ...
        'Callback', @(~, ~) set_all_layers(cb, h_layers, 0));
    % 全部显示按钮：点击后将所有复选框设为 1，所有图层设为可见
    uicontrol('Parent', fig, 'Style', 'pushbutton', 'String', '全部显示', ...
        'Units', 'normalized', 'Position', [0.87, 0.055, 0.10, 0.04], ...
        'Callback', @(~, ~) set_all_layers(cb, h_layers, 1));
    % 底部状态栏：显示 R1/R2 航迹数量和仿真参数
    uicontrol('Parent', fig, 'Style', 'text', 'Units', 'normalized', ...
        'Position', [0.76, 0.005, 0.22, 0.04], ...
        'String', sprintf('R1:%d航迹 R2:%d航迹 | Pd=%.0f%% Pfa=%.3f', ...
        length(trackList_R1), length(trackList_R2), ...
        params.detection_probability*100, params.false_alarm_rate), ...
        'FontSize', 8, 'BackgroundColor', [1 1 1]);
end

% =========================================================================
% set_layer_visibility — 设置图层可见性
% =========================================================================
% 遍历绘图句柄数组，将 Visible 属性设为 'on' 或 'off'
% isgraphics 检查句柄是否指向有效的图形对象（避免对复合句柄报错）
function set_layer_visibility(h, value)
    % 根据 value 决定可见性字符串
    if value
        visibility = 'on';  % 值为 1 时表示显示
    else
        visibility = 'off';  % 值为 0 时表示隐藏
    end
    for i = 1:length(h)
        % isgraphics 检查句柄是否有效（未被删除的对象返回 false）
        if isgraphics(h(i))
            h(i).Visible = visibility;  % 直接设置句柄的 Visible 属性
        end
    end
end

% =========================================================================
% set_all_layers — 一键设置所有图层可见性
% =========================================================================
% 批量操作：同时更新所有复选框的状态和所有图层的可见性
function set_all_layers(cb, h_layers, value)
    for i = 1:length(h_layers)
        % 同步更新复选框的 Value 属性（如果存在且有效）
        if i <= length(cb) && isgraphics(cb(i))
            cb(i).Value = value;  % 0=未选中, 1=选中
        end
        % 设置对应图层的可见性
        set_layer_visibility(h_layers{i}, value);
    end
end
