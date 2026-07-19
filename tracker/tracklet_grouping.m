function result = tracklet_grouping(snapshots_R1, snapshots_R2, params)
% TRACKLET_GROUPING 通过运动学相容性将同站/跨站航迹段凝聚为 M:N 组。
% 匹配过程不读取 truth_idx，真值标签只允许由外部评估使用。

segments = [extract_segments(snapshots_R1, 1), extract_segments(snapshots_R2, 2)];
edges = build_edges(segments, params);
groups = constrained_components(segments, edges);
fused_snapshots = fuse_groups(groups, segments, snapshots_R1, snapshots_R2, params);

result = struct();
result.segments = segments;
result.edges = edges;
result.groups = groups;
result.fused_snapshots = fused_snapshots;
end

function segments = extract_segments(snapshots, radar_id)
segments = struct('segment_id', {}, 'radar_id', {}, 'track_id', {}, ...
    'frames', {}, 'start_frame', {}, 'end_frame', {}, 'states', {}, ...
    'covariances', {}, 'lats', {}, 'lons', {});

active_ids = [];
active_segment_idx = [];
for k = 1:numel(snapshots)
    snap = snapshots{k};
    if isempty(snap) || ~isfield(snap, 'trackList') || isempty(snap.trackList)
        continue;
    end
    for t = 1:numel(snap.trackList)
        trk = snap.trackList{t};
        if isempty(trk) || trk.type == 7 || ~isfield(trk, 'ukf') || isempty(trk.ukf.x)
            continue;
        end
        track_id = double(trk.id);
        idx = find(active_ids == track_id, 1);
        if isempty(idx) || segments(active_segment_idx(idx)).end_frame ~= k - 1
            segments(end+1) = new_segment(numel(segments) + 1, radar_id, track_id, k, trk); %#ok<AGROW>
            if isempty(idx)
                active_ids(end+1) = track_id; %#ok<AGROW>
                active_segment_idx(end+1) = numel(segments); %#ok<AGROW>
            else
                active_segment_idx(idx) = numel(segments);
            end
        else
            s = active_segment_idx(idx);
            segments(s).frames(end+1) = k;
            segments(s).end_frame = k;
            segments(s).states(:, end+1) = trk.ukf.x;
            segments(s).covariances(:, :, end+1) = trk.ukf.P;
            segments(s).lats(end+1) = trk.lat;
            segments(s).lons(end+1) = trk.lon;
        end
    end
end
end

function seg = new_segment(segment_id, radar_id, track_id, frame_id, trk)
seg = struct();
seg.segment_id = segment_id;
seg.radar_id = radar_id;
seg.track_id = track_id;
seg.frames = frame_id;
seg.start_frame = frame_id;
seg.end_frame = frame_id;
seg.states = trk.ukf.x;
seg.covariances = trk.ukf.P;
seg.lats = trk.lat;
seg.lons = trk.lon;
end

function edges = build_edges(segments, params)
edges = struct('a', {}, 'b', {}, 'edge_type', {}, 'score', {}, ...
    'mean_distance_km', {}, 'gap_frames', {}, 'heading_diff_deg', {});
for i = 1:numel(segments)-1
    for j = i+1:numel(segments)
        [valid, edge] = segment_compatibility(segments(i), segments(j), params);
        if valid
            edge.a = i;
            edge.b = j;
            edges(end+1) = edge; %#ok<AGROW>
        end
    end
end
end

function [valid, edge] = segment_compatibility(a, b, params)
edge = struct('a', 0, 'b', 0, 'edge_type', '', 'score', 0, ...
    'mean_distance_km', inf, 'gap_frames', 0, 'heading_diff_deg', 180);
valid = false;

common = intersect(a.frames, b.frames);
if ~isempty(common)
    if a.radar_id == b.radar_id
        return;
    end
    distances = zeros(1, numel(common));
    for q = 1:numel(common)
        ia = find(a.frames == common(q), 1);
        ib = find(b.frames == common(q), 1);
        distances(q) = haversine_km(a.lats(ia), a.lons(ia), b.lats(ib), b.lons(ib));
    end
    mean_distance = mean(distances);
    heading_diff = overlap_heading_difference(a, b, common);
    min_overlap = min(3, min(numel(a.frames), numel(b.frames)));
    distance_gate = matcher_param(params, 'tracklet_overlap_distance_km', 45);
    heading_gate = matcher_param(params, 'tracklet_heading_gate_deg', 75);
    valid = numel(common) >= min_overlap && mean_distance <= distance_gate && heading_diff <= heading_gate;
    edge.edge_type = 'overlap';
    edge.mean_distance_km = mean_distance;
    edge.heading_diff_deg = heading_diff;
    edge.score = exp(-mean_distance / distance_gate) * exp(-heading_diff / heading_gate) * ...
        min(1, numel(common) / 8);
    return;
end

if a.end_frame < b.start_frame
    earlier = a; later = b;
elseif b.end_frame < a.start_frame
    earlier = b; later = a;
else
    return;
end

gap = later.start_frame - earlier.end_frame - 1;
max_gap = matcher_param(params, 'tracklet_max_gap_frames', 6);
if gap > max_gap
    return;
end

[x_pred, P_pred] = propagate_cv(earlier.states(:, end), earlier.covariances(:, :, end), ...
    (later.start_frame - earlier.end_frame) * params.dt_sec, params);
delta = later.states(:, 1) - x_pred;
position_distance = haversine_km(x_pred(3), x_pred(1), later.states(3, 1), later.states(1, 1));
S = P_pred + later.covariances(:, :, 1);
mahal = delta' * (S \ delta);
heading_diff = heading_difference(earlier, later);
distance_gate = matcher_param(params, 'tracklet_endpoint_distance_km', 80);
mahal_gate = matcher_param(params, 'tracklet_mahal_gate', 25);
heading_gate = matcher_param(params, 'tracklet_heading_gate_deg', 75);
valid = position_distance <= distance_gate && mahal <= mahal_gate && heading_diff <= heading_gate;
edge.edge_type = 'successor';
edge.mean_distance_km = position_distance;
edge.gap_frames = gap;
edge.heading_diff_deg = heading_diff;
edge.score = exp(-position_distance / distance_gate) * exp(-mahal / mahal_gate) * ...
    exp(-heading_diff / heading_gate) * exp(-gap / max(max_gap, 1));
end

function groups = constrained_components(segments, edges)
parent = 1:numel(segments);
[~, order] = sort([edges.score], 'descend');
for q = order
    a = edges(q).a;
    b = edges(q).b;
    ra = root(parent, a);
    rb = root(parent, b);
    if ra == rb
        continue;
    end
    members_a = find(arrayfun(@(x) root(parent, x) == ra, 1:numel(segments)));
    members_b = find(arrayfun(@(x) root(parent, x) == rb, 1:numel(segments)));
    merged_members = [members_a, members_b];
    if group_is_consistent(segments(merged_members)) && ...
            groups_have_cross_support(members_a, members_b, edges)
        parent(rb) = ra;
    end
end

roots = arrayfun(@(x) root(parent, x), 1:numel(segments));
unique_roots = unique(roots);
groups = struct('group_id', {}, 'segment_indices', {}, 'start_frame', {}, 'end_frame', {});
for g = 1:numel(unique_roots)
    members = find(roots == unique_roots(g));
    if numel(members) < 2
        continue;
    end
    groups(end+1).group_id = numel(groups) + 1; %#ok<AGROW>
    groups(end).segment_indices = members;
    groups(end).start_frame = min([segments(members).start_frame]);
    groups(end).end_frame = max([segments(members).end_frame]);
end
end

function ok = groups_have_cross_support(members_a, members_b, edges)
ok = false;
for i = members_a
    for j = members_b
        for e = 1:numel(edges)
            if (edges(e).a == i && edges(e).b == j) || (edges(e).a == j && edges(e).b == i)
                ok = true;
                return;
            end
        end
    end
end
end

function ok = group_is_consistent(group_segments)
ok = true;
for radar_id = 1:2
    idx = find([group_segments.radar_id] == radar_id);
    for i = 1:numel(idx)-1
        for j = i+1:numel(idx)
            if ~isempty(intersect(group_segments(idx(i)).frames, group_segments(idx(j)).frames))
                ok = false;
                return;
            end
        end
    end
end
end

function fused_snapshots = fuse_groups(groups, segments, snapshots_R1, snapshots_R2, params)
n_frames = max(numel(snapshots_R1), numel(snapshots_R2));
fused_snapshots = cell(n_frames, 1);
for k = 1:n_frames
    fused_snapshots{k} = struct('frameID', k, 'trackList', {{}});
end

for g = 1:numel(groups)
    group_segments = segments(groups(g).segment_indices);
    last_x = [];
    last_P = [];
    last_frame = [];
    for k = groups(g).start_frame:groups(g).end_frame
        estimates = estimates_at_frame(group_segments, k);
        if numel(estimates) >= 2
            [x, P, w] = track_fusion_algorithms('CI', estimates(1).x, estimates(1).P, ...
                estimates(2).x, estimates(2).P);
            source = 'both';
        elseif numel(estimates) == 1
            x = estimates(1).x;
            P = estimates(1).P;
            w = NaN;
            source = sprintf('R%d_only', estimates(1).radar_id);
        elseif ~isempty(last_x) && k - last_frame <= matcher_param(params, 'tracklet_max_prediction_frames', 2)
            [x, P] = propagate_cv(last_x, last_P, (k - last_frame) * params.dt_sec, params);
            w = NaN;
            source = 'predicted';
        else
            continue;
        end
        trk = struct('id', groups(g).group_id, 'group_id', groups(g).group_id, ...
            'lat', x(3), 'lon', x(1), 'ukf', struct('x', x, 'P', P), ...
            'source', source, 'w', w, 'segment_ids', [group_segments.segment_id]);
        fused_snapshots{k}.trackList{end+1} = trk;
        last_x = x;
        last_P = P;
        last_frame = k;
    end
end
end

function estimates = estimates_at_frame(segments, frame_id)
estimates = struct('radar_id', {}, 'x', {}, 'P', {});
for i = 1:numel(segments)
    idx = find(segments(i).frames == frame_id, 1);
    if isempty(idx)
        continue;
    end
    estimates(end+1) = struct('radar_id', segments(i).radar_id, ...
        'x', segments(i).states(:, idx), 'P', segments(i).covariances(:, :, idx)); %#ok<AGROW>
end
end

function [x_new, P_new] = propagate_cv(x, P, dt, params)
F = [1 dt 0 0; 0 1 0 0; 0 0 1 dt; 0 0 0 1];
Q = eye(4) * matcher_param(params, 'tracklet_prediction_q', 1e-8) * max(abs(dt), 1);
x_new = F * x;
P_new = F * P * F' + Q;
P_new = (P_new + P_new') / 2;
end

function d = overlap_heading_difference(a, b, common)
ha = heading_on_frames(a, common);
hb = heading_on_frames(b, common);
if ~isfinite(ha) || ~isfinite(hb)
    d = 180;
else
    d = abs(mod(ha - hb + 180, 360) - 180);
end
end

function h = heading_on_frames(seg, frames)
idx = find(ismember(seg.frames, frames));
if isempty(idx)
    h = NaN;
    return;
end
v_lon = median(seg.states(2, idx));
v_lat = median(seg.states(4, idx));
if hypot(v_lon, v_lat) < eps
    h = NaN;
else
    h = mod(atan2d(v_lon, v_lat), 360);
end
end

function d = heading_difference(a, b)
ha = segment_heading(a);
hb = segment_heading(b);
if ~isfinite(ha) || ~isfinite(hb)
    d = 180;
    return;
end
d = abs(mod(ha - hb + 180, 360) - 180);
end

function h = segment_heading(seg)
if size(seg.states, 2) < 1
    h = NaN;
    return;
end
v_lon = median(seg.states(2, :));
v_lat = median(seg.states(4, :));
if hypot(v_lon, v_lat) < eps
    h = NaN;
else
    h = mod(atan2d(v_lon, v_lat), 360);
end
end

function value = matcher_param(params, name, default_value)
if isfield(params, name)
    value = params.(name);
else
    value = default_value;
end
end

function r = root(parent, x)
r = x;
while parent(r) ~= r
    r = parent(r);
end
end

function d = haversine_km(lat1, lon1, lat2, lon2)
R = 6371.0088;
dlat = deg2rad(lat2 - lat1);
dlon = deg2rad(lon2 - lon1);
a = sin(dlat/2).^2 + cos(deg2rad(lat1)) .* cos(deg2rad(lat2)) .* sin(dlon/2).^2;
d = 2 * R * atan2(sqrt(a), sqrt(max(0, 1-a)));
end
