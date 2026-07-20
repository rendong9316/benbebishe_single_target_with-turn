function fused_snapshots = run_track_fusion(matched_pairs, trackSnapshots_R1, ...
    aligned_R2, params, method)
% RUN_TRACK_FUSION 向后兼容的旧 pair matcher 到新版 group engine 的适配器。
%
% 【作用】
%   旧的融合接口是一对一对地融合（一个 R1 航迹 + 一个 R2 航迹）。
%   新的融合接口是基于 group 的（多个片段聚合后再融合）。
%   本函数将旧的 pair 接口转换为新的 group 接口，让旧代码无需修改即可工作。
%
% 【流程】
%   1. 从 pair 中提取 R1 和 R2 的航迹片段
%   2. 将两个片段组装为一个 group（2 个片段）
%   3. 调用 fuse_estimate_sequence 执行融合
%   4. 返回指定算法的融合快照

if numel(matched_pairs) ~= 1
    error('run_track_fusion:singlePairRequired', ...
        'The legacy adapter accepts exactly one matched pair');
end
method = upper(char(method));  % 统一为大写

r1_segments = build_faded_track_segments('extract', trackSnapshots_R1, [], 1);  % 提取 R1 片段
r2_segments = build_faded_track_segments('extract', aligned_R2, [], 2);  % 提取 R2 片段
r1_id = double(matched_pairs.R1_track_id);  % R1 航迹 ID
r2_id = double(matched_pairs.R2_track_id);  % R2 航迹 ID
r1_segments = r1_segments([r1_segments.track_id] == r1_id);  % 筛选 R1 片段
r2_segments = r2_segments([r2_segments.track_id] == r2_id);  % 筛选 R2 片段
segments = [r1_segments, r2_segments];  % 合并为 2 个片段的组
for i = 1:numel(segments), segments(i).segment_id = i; end  % 重新编号

if isempty(segments)  % 没有片段，返回空快照
    fused_snapshots = empty_snapshots(max(numel(trackSnapshots_R1), numel(aligned_R2)));
    return;
end
group = struct('group_id', 1, 'segment_indices', 1:numel(segments));  % 构造 group
result = fuse_estimate_sequence(group, segments, params);  % 执行四算法融合
idx = find(strcmp({result.methods.method}, method), 1);  % 找到指定算法
if isempty(idx)
    error('run_track_fusion:unknownMethod', 'Unknown fusion method: %s', method);
end
fused_snapshots = result.methods(idx).snapshots;  % 返回融合快照
target_length = max(numel(trackSnapshots_R1), numel(aligned_R2));
for k = numel(fused_snapshots)+1:target_length  % 补齐空帧
    fused_snapshots{k,1} = struct('frameID', k, 'trackList', {{}});
end
end

function snapshots = empty_snapshots(n_frames)
% empty_snapshots 创建全空快照 cell 数组（每帧 trackList 为空）
snapshots = cell(n_frames, 1);
for k = 1:n_frames
    snapshots{k} = struct('frameID', k, 'trackList', {{}});
end
end
