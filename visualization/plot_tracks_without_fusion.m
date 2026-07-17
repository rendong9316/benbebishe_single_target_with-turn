function plot_tracks_without_fusion(truth_all, detList_R1, detList_R2, ...
        trackSnapshots_R1, trackSnapshots_R2, trackList_R1, trackList_R2, params)

    fig = figure('Name', 'Figure 4 - Oracle 单站航迹维护', 'Position', [50, 50, 1400, 750]);
    try
        ax = geoaxes('Units', 'normalized', 'Position', [0.04, 0.10, 0.70, 0.88]);
        ax.Basemap = 'darkwater';
    catch
        ax = geoaxes('Units', 'normalized', 'Position', [0.04, 0.10, 0.70, 0.88]);
    end
    hold(ax, 'on');

    h_layers = {};
    layer_names = {};
    truth_colors = {[1 1 0], [1 0 1], [0 1 1]};

    for ac = 1:length(truth_all)
        tt = truth_all{ac};
        color = truth_colors{min(ac, length(truth_colors))};
        h = geoplot(ax, tt(:, 2), tt(:, 1), '--s', 'Color', color, ...
            'LineWidth', 1.5, 'MarkerSize', 5, 'MarkerFaceColor', color, ...
            'DisplayName', sprintf('真值%c', char('A' + ac - 1)));
        [h_layers, layer_names] = add_layer(h_layers, layer_names, h, ...
            sprintf('真值%c', char('A' + ac - 1)));
    end

    h = plot_calibrated_detection_tracks(ax, detList_R1, [0.30 0.65 1.00], 'R1校准点迹');
    [h_layers, layer_names] = add_layer(h_layers, layer_names, h, 'R1校准点迹连线');
    h = plot_consumed_detection_tracks(ax, trackList_R1, [0.00 0.25 0.85], 'R1航迹提取点');
    [h_layers, layer_names] = add_layer(h_layers, layer_names, h, 'R1航迹提取点连线');
    h = plot_filter_tracks(ax, trackSnapshots_R1, [0.00 0.00 0.55], 'R1 UKF');
    [h_layers, layer_names] = add_layer(h_layers, layer_names, h, 'R1 UKF航迹');

    h = plot_calibrated_detection_tracks(ax, detList_R2, [1.00 0.55 0.40], 'R2校准点迹');
    [h_layers, layer_names] = add_layer(h_layers, layer_names, h, 'R2校准点迹连线');
    h = plot_consumed_detection_tracks(ax, trackList_R2, [0.85 0.15 0.05], 'R2航迹提取点');
    [h_layers, layer_names] = add_layer(h_layers, layer_names, h, 'R2航迹提取点连线');
    h = plot_filter_tracks(ax, trackSnapshots_R2, [0.55 0.00 0.00], 'R2 UKF');
    [h_layers, layer_names] = add_layer(h_layers, layer_names, h, 'R2 UKF航迹');

    geoplot(ax, params.radar1_lat, params.radar1_lon, 'bs', ...
        'MarkerSize', 14, 'MarkerFaceColor', 'b', 'DisplayName', 'R1 Rx');
    geoplot(ax, params.radar2_lat, params.radar2_lon, 'rs', ...
        'MarkerSize', 14, 'MarkerFaceColor', 'r', 'DisplayName', 'R2 Rx');
    geoplot(ax, params.radar1_tx_lat, params.radar1_tx_lon, 'b^', ...
        'MarkerSize', 10, 'DisplayName', 'R1 Tx');
    geoplot(ax, params.radar2_tx_lat, params.radar2_tx_lon, 'r^', ...
        'MarkerSize', 10, 'DisplayName', 'R2 Tx');

    title(ax, 'Oracle单站航迹维护：校准点迹、航迹提取点与UKF输出');
    legend(ax, 'Location', 'northeastoutside');
    install_layer_controls(fig, h_layers, layer_names, trackList_R1, trackList_R2, params);
    drawnow;
end

function h = plot_calibrated_detection_tracks(ax, detList, color, label)
    h = gobjects(0);
    aircraft_ids = collect_detection_aircraft_ids(detList);
    for ac = aircraft_ids
        [lat, lon] = detection_line(detList, ac);
        if sum(~isnan(lat)) == 0
            continue;
        end
        hp = geoplot(ax, lat, lon, '-o', 'Color', color, 'LineWidth', 1.6, ...
            'MarkerSize', 5, 'MarkerFaceColor', color, ...
            'DisplayName', sprintf('%s T%d', label, ac));
        h(end+1) = hp;
    end
end

function ids = collect_detection_aircraft_ids(detList)
    ids = [];
    for k = 1:length(detList)
        dets = detList{k};
        for i = 1:length(dets)
            if ~dets(i).is_clutter && isfield(dets(i), 'aircraft_id') && dets(i).aircraft_id > 0
                ids(end+1) = double(dets(i).aircraft_id);
            end
        end
    end
    ids = unique(ids);
end

function [lat, lon] = detection_line(detList, aircraft_id)
    lat = [];
    lon = [];
    previous_frame = NaN;
    for k = 1:length(detList)
        dets = detList{k};
        idx = 0;
        for i = 1:length(dets)
            if ~dets(i).is_clutter && double(dets(i).aircraft_id) == aircraft_id ...
                    && ~isnan(dets(i).lat) && ~isnan(dets(i).lon)
                idx = i;
                break;
            end
        end
        if idx == 0
            continue;
        end
        if ~isnan(previous_frame) && k > previous_frame + 1
            lat(end+1) = NaN;
            lon(end+1) = NaN;
        end
        lat(end+1) = dets(idx).lat;
        lon(end+1) = dets(idx).lon;
        previous_frame = k;
    end
end

function h = plot_consumed_detection_tracks(ax, trackList, color, label)
    h = gobjects(0);
    for i = 1:length(trackList)
        trk = trackList{i};
        if ~isfield(trk, 'asscPointList') || isempty(trk.asscPointList)
            continue;
        end
        [lat, lon] = point_cell_line(trk.asscPointList);
        if sum(~isnan(lat)) == 0
            continue;
        end
        hp = geoplot(ax, lat, lon, '-d', 'Color', color, 'LineWidth', 1.8, ...
            'MarkerSize', 6, 'MarkerFaceColor', color, ...
            'DisplayName', sprintf('%s #%d/T%d', label, trk.id, trk.truth_idx));
        h(end+1) = hp;
    end
end

function [lat, lon] = point_cell_line(pointList)
    frames = [];
    points = {};
    for i = 1:length(pointList)
        dp = pointList{i};
        if isempty(dp) || ~isstruct(dp) || ~isfield(dp, 'frameID') ...
                || ~isfield(dp, 'lat') || ~isfield(dp, 'lon') ...
                || isnan(dp.lat) || isnan(dp.lon)
            continue;
        end
        frames(end+1) = double(dp.frameID);
        points{end+1} = dp;
    end
    if isempty(frames)
        lat = [];
        lon = [];
        return;
    end
    [frames, order] = sort(frames);
    points = points(order);
    [frames, unique_idx] = unique(frames, 'stable');
    points = points(unique_idx);
    lat = [];
    lon = [];
    for i = 1:length(points)
        if i > 1 && frames(i) > frames(i-1) + 1
            lat(end+1) = NaN;
            lon(end+1) = NaN;
        end
        lat(end+1) = points{i}.lat;
        lon(end+1) = points{i}.lon;
    end
end

function h = plot_filter_tracks(ax, snapshots, color, label)
    h = gobjects(0);
    ids = collect_snapshot_track_ids(snapshots);
    for id = ids
        [lat, lon] = snapshot_track_line(snapshots, id);
        if sum(~isnan(lat)) < 2
            continue;
        end
        hp = geoplot(ax, lat, lon, '-', 'Color', color, 'LineWidth', 2.2, ...
            'DisplayName', sprintf('%s #%d', label, id));
        h(end+1) = hp;
    end
end

function ids = collect_snapshot_track_ids(snapshots)
    ids = [];
    for k = 1:length(snapshots)
        snap = snapshots{k};
        if isempty(snap) || ~isfield(snap, 'trackList')
            continue;
        end
        for i = 1:length(snap.trackList)
            ids(end+1) = snap.trackList{i}.id;
        end
    end
    ids = unique(ids);
end

function [lat, lon] = snapshot_track_line(snapshots, track_id)
    lat = [];
    lon = [];
    previous_frame = NaN;
    for k = 1:length(snapshots)
        snap = snapshots{k};
        trk = [];
        if ~isempty(snap) && isfield(snap, 'trackList')
            for i = 1:length(snap.trackList)
                candidate = snap.trackList{i};
                if candidate.id == track_id
                    trk = candidate;
                    break;
                end
            end
        end
        if isempty(trk) || trk.type == 7 || isnan(trk.lat) || isnan(trk.lon)
            continue;
        end
        if ~isnan(previous_frame) && k > previous_frame + 1
            lat(end+1) = NaN;
            lon(end+1) = NaN;
        end
        lat(end+1) = trk.lat;
        lon(end+1) = trk.lon;
        previous_frame = k;
    end
end

function [h_layers, names] = add_layer(h_layers, names, h, name)
    if isempty(h)
        return;
    end
    h_layers{end+1} = h;
    names{end+1} = name;
end

function install_layer_controls(fig, h_layers, names, trackList_R1, trackList_R2, params)
    cb = gobjects(1, length(names));
    for i = 1:length(names)
        ypos = 0.92 - (i-1) * 0.045;
        if ypos < 0.12
            break;
        end
        cb(i) = uicontrol('Parent', fig, 'Style', 'checkbox', 'String', names{i}, ...
            'Value', 1, 'Units', 'normalized', 'Position', [0.76, ypos, 0.22, 0.040], ...
            'FontSize', 9, 'BackgroundColor', [1 1 1], ...
            'Callback', @(src, ~) set_layer_visibility(h_layers{i}, src.Value));
    end
    uicontrol('Parent', fig, 'Style', 'pushbutton', 'String', '全部隐藏', ...
        'Units', 'normalized', 'Position', [0.76, 0.055, 0.10, 0.04], ...
        'Callback', @(~, ~) set_all_layers(cb, h_layers, 0));
    uicontrol('Parent', fig, 'Style', 'pushbutton', 'String', '全部显示', ...
        'Units', 'normalized', 'Position', [0.87, 0.055, 0.10, 0.04], ...
        'Callback', @(~, ~) set_all_layers(cb, h_layers, 1));
    uicontrol('Parent', fig, 'Style', 'text', 'Units', 'normalized', ...
        'Position', [0.76, 0.005, 0.22, 0.04], ...
        'String', sprintf('R1:%d航迹 R2:%d航迹 | Pd=%.0f%% Pfa=%.3f', ...
        length(trackList_R1), length(trackList_R2), ...
        params.detection_probability*100, params.false_alarm_rate), ...
        'FontSize', 8, 'BackgroundColor', [1 1 1]);
end

function set_layer_visibility(h, value)
    if value
        visibility = 'on';
    else
        visibility = 'off';
    end
    for i = 1:length(h)
        if isgraphics(h(i))
            h(i).Visible = visibility;
        end
    end
end

function set_all_layers(cb, h_layers, value)
    for i = 1:length(h_layers)
        if i <= length(cb) && isgraphics(cb(i))
            cb(i).Value = value;
        end
        set_layer_visibility(h_layers{i}, value);
    end
end
