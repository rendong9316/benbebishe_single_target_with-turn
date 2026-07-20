function fused_snapshots = run_track_fusion(matched_pairs, trackSnapshots_R1, ...
    aligned_R2, params, method)
% RUN_TRACK_FUSION Backward-compatible pair adapter to the group engine.

if numel(matched_pairs) ~= 1
    error('run_track_fusion:singlePairRequired', ...
        'The legacy adapter accepts exactly one matched pair');
end
method = upper(char(method));

r1_segments = build_faded_track_segments('extract', trackSnapshots_R1, [], 1);
r2_segments = build_faded_track_segments('extract', aligned_R2, [], 2);
r1_id = double(matched_pairs.R1_track_id);
r2_id = double(matched_pairs.R2_track_id);
r1_segments = r1_segments([r1_segments.track_id] == r1_id);
r2_segments = r2_segments([r2_segments.track_id] == r2_id);
segments = [r1_segments, r2_segments];
for i = 1:numel(segments), segments(i).segment_id = i; end

if isempty(segments)
    fused_snapshots = empty_snapshots(max(numel(trackSnapshots_R1), numel(aligned_R2)));
    return;
end
group = struct('group_id', 1, 'segment_indices', 1:numel(segments));
result = fuse_estimate_sequence(group, segments, params);
idx = find(strcmp({result.methods.method}, method), 1);
if isempty(idx)
    error('run_track_fusion:unknownMethod', 'Unknown fusion method: %s', method);
end
fused_snapshots = result.methods(idx).snapshots;
target_length = max(numel(trackSnapshots_R1), numel(aligned_R2));
for k = numel(fused_snapshots)+1:target_length
    fused_snapshots{k,1} = struct('frameID', k, 'trackList', {{}});
end
end

function snapshots = empty_snapshots(n_frames)
snapshots = cell(n_frames, 1);
for k = 1:n_frames
    snapshots{k} = struct('frameID', k, 'trackList', {{}});
end
end
