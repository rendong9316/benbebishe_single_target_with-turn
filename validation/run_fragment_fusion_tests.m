function run_fragment_fusion_tests(run_integration)
% RUN_FRAGMENT_FUSION_TESTS 动态片段分组/融合测试套件。
%
% 测试覆盖：
%   1. 未知目标数分组（4 片段 → 2 组）
%   2. 跨站交接边（handoff edge）
%   3. 真值泄漏被拒绝（truth_idx 字段检测）
%   4. 四算法单源融合
%   5. RTS 桥接有界空洞
%   6. IMM 桥接转弯方向
%   7. 低置信度检测
%   8. [可选] 精确 K 片段实验（单目标 + 多目标）

if nargin < 1, run_integration = false; end  % 默认不跑集成测试
addpath(genpath('.'));
test_unknown_target_count_grouping();  % 测试片段分组
test_cross_handoff();  % 测试交接边
test_truth_leak_rejected();  % 测试真值泄漏检测
test_four_method_single_source();  % 测试四算法单源融合
test_bridge_bounded_gap();  % 测试 RTS/IMM 桥接空洞
test_bridge_turn_direction();  % 测试 IMM 转弯方向判断
test_bridge_low_confidence();  % 测试低置信度标记
if run_integration
    test_exact_k_single_and_multi();  % 集成测试：精确 K 片段
end
disp('fragment fusion tests ok');
end
function test_bridge_bounded_gap()
% test_bridge_bounded_gap 测试 RTS/IMM 桥接有界空洞。
% 构造 8 帧快照，第 2 帧和第 6 帧有数据，中间 3 帧为空。
% 桥接器应填补 3 帧空洞，且开头/结尾的空帧保持不变。
params = simulation_params_oracle();
snapshots = cell(8,1);  % 8 帧
P = diag([1e-5 1e-9 1e-5 1e-9]);  % 协方差
snapshots{2} = fixture_snapshot(2, [128; 0.001; 31; 0.0005], P);  % 左锚点
snapshots{6} = fixture_snapshot(6, [128.12; 0.001; 31.06; 0.0005], P);  % 右锚点
result = bridge_smoother(snapshots, params);
assert(result.rts.bridge_frame_count == 3);  % RTS 填补 3 帧
assert(result.imm.bridge_frame_count == 3);  % IMM 填补 3 帧
assert(isempty(result.reconstructed_snapshots{1}));  % 开头空帧不变
assert(isempty(result.reconstructed_snapshots{7}));  % 结尾空帧不变
assert(isempty(result.reconstructed_snapshots{8}));
for k = 3:5  % 空洞帧 3,4,5
    trk = result.reconstructed_snapshots{k}.trackList{1};
    assert(trk.is_virtual && ~trk.has_measurement_support);  % 虚拟航迹
    assert(strcmp(trk.source, 'bridge_smoothed'));  % 来源标记正确
    assert(all(isfinite(trk.ukf.x)) && all(isfinite(trk.ukf.P(:))));  % 状态有限
    assert(min(eig((trk.ukf.P + trk.ukf.P')/2)) > 0);  % 协方差正定
end
assert(~result.reconstructed_snapshots{2}.trackList{1}.is_virtual);  % 锚点帧非虚拟
end

function test_bridge_turn_direction()
% test_bridge_turn_direction 测试 IMM 桥接能否正确判断转弯方向。
% 构造 5 帧快照，两端锚点之间有一个左转（正角速度），
% 验证 IMM 模式概率中左转模式 > 右转模式。
params = simulation_params_oracle();
params.bridge.accel_std_mps2 = 0.05;  % 降低加速度噪声，使转弯更明显
lat0 = 31; lon0 = 128; radius = params.bridge.earth_radius_m;
meters_lon = radius * cosd(lat0) * pi / 180;  % 经度方向米/度
meters_lat = radius * pi / 180;  % 纬度方向米/度
dt = params.dt_sec * 4;  % 时间步长
omega = params.bridge.turn_rate_rad_per_sec;  % 转弯角速度
s = sin(omega*dt); c = cos(omega*dt); omc = 1-c;
F = [1 s/omega 0 -omc/omega; 0 c 0 -s; ...  % 恒转弯转移矩阵
     0 omc/omega 1 s/omega; 0 s 0 c];
x0e = [0; 250; 0; 0];  % 初始 ENU 状态（北向速度 250m/s）
x1e = F*x0e;  % 转弯后的 ENU 状态
x0 = [lon0; x0e(2)/meters_lon; lat0; x0e(4)/meters_lat];  % 转回经纬度
x1 = [lon0+x1e(1)/meters_lon; x1e(2)/meters_lon; ...
      lat0+x1e(3)/meters_lat; x1e(4)/meters_lat];
P = diag([1e-8 1e-11 1e-8 1e-11]);  % 小协方差
snapshots = cell(5,1);
snapshots{1} = fixture_snapshot(1, x0, P);  % 左锚点
snapshots{5} = fixture_snapshot(5, x1, P);  % 右锚点
result = bridge_smoother(snapshots, params);
prob = result.imm.bridge_snapshots{3}.trackList{1}.bridge_mode_probabilities;  % 中间帧的模式概率
assert(abs(sum(prob)-1) < 1e-10);  % 概率和为 1
assert(prob(2) > prob(3));  % 左转模式概率 > 右转模式概率
end

function test_bridge_low_confidence()
% test_bridge_low_confidence 测试低置信度检测。
% 将马氏距离门限设为 0，使所有桥接帧都低于门限，
% 验证低置信度计数和标记正确。
params = simulation_params_oracle();
params.bridge.confidence_mahal_gate = 0;  % 门限=0 → 所有帧低置信度
P = diag([1e-7 1e-10 1e-7 1e-10]);
snapshots = cell(4,1);
snapshots{1} = fixture_snapshot(1, [128;0;31;0], P);  % 左锚点
snapshots{4} = fixture_snapshot(4, [130;0;34;0], P);  % 右锚点（距离很远）
result = bridge_smoother(snapshots, params);
assert(result.imm.bridge_frame_count == 2);  % 填补 2 帧
assert(result.imm.low_confidence_bridge_count == 2);  % 2 帧都低置信度
assert(strcmp(result.imm.bridge_snapshots{2}.trackList{1}.bridge_confidence, 'low'));  % 标记正确
end

function test_unknown_target_count_grouping()
% test_unknown_target_count_grouping 测试片段分组能否自动发现目标数。
% 构造 4 个片段（R1 2 个 + R2 2 个），期望分成 2 组（每组跨 2 雷达）
params = simulation_params_oracle();
frames = 1:10;
segments = [fixture_segment(1,1,11,frames,128.0,31.0), ...  % R1 片段1（目标A）
    fixture_segment(2,1,12,frames,130.0,33.0), ...         % R1 片段2（目标B）
    fixture_segment(3,2,21,frames,128.03,31.02), ...       % R2 片段1（目标A）
    fixture_segment(4,2,22,frames,130.03,33.02)];          % R2 片段2（目标B）
result = tracklet_grouping('segments', segments, params);
assert(strcmp(result.status, 'SUCCESS'));  % 分组成功
assert(numel(result.groups) == 2);  % 分成 2 组
% 每组包含 2 个片段
assert(all(sort(cellfun(@numel, {result.groups.segment_indices})) == [2,2]));
% 每组都包含 R1 和 R2 的片段（跨雷达）
assert(all(arrayfun(@(g) numel(unique([segments(g.segment_indices).radar_id])) == 2, ...
    result.groups)));
end

function test_cross_handoff()
% test_cross_handoff 测试跨站交接边（handoff edge）的构建。
% 构造 R1 片段 [1,8] 和 R2 片段 [10,17]，时间不重叠但有空间接近，
% 应构建 handoff 类型的边，并将两个片段分到同一组。
params = simulation_params_oracle();
a = fixture_segment(1,1,1,1:8,128.0,31.0);  % R1 片段
b = fixture_segment(2,2,2,10:17,128.0,31.0);  % R2 片段（时间不重叠）
result = tracklet_grouping('segments', [a,b], params);
assert(strcmp(result.status, 'SUCCESS'));  % 分组成功
assert(numel(result.edges) == 1);  % 只有 1 条 handoff 边
assert(strcmp(result.edges(1).edge_type, 'handoff'));  % 边类型是 handoff
assert(numel(result.groups) == 1);  % 两个片段分到同一组
end

function test_truth_leak_rejected()
% test_truth_leak_rejected 测试真值泄漏检测。
% 如果片段结构体中包含 truth_idx 字段，分组算法应拒绝并抛出错误。
% 这是为了确保分组算法是"盲"的——不能利用真值信息做决策。
params = simulation_params_oracle();
seg = fixture_segment(1,1,1,1:8,128.0,31.0);
seg.truth_idx = 1;  % 注入真值泄漏
failed = false;
try
    tracklet_grouping('segments', seg, params);
catch exception
    failed = strcmp(exception.identifier, 'tracklet_grouping:truthLeak');
end
assert(failed);  % 确认抛出了正确的错误
end

function test_four_method_single_source()
% test_four_method_single_source 测试四算法在单源片段上的行为。
% 构造 1 个 R1 片段，调用 fuse_estimate_sequence 执行四种融合算法，
% 验证每种算法都能正常输出且覆盖帧数正确。
params = simulation_params_oracle();
seg = fixture_segment(1,1,1,1:8,128.0,31.0);  % 1 个 R1 片段
group = struct('group_id',1,'segment_indices',1);  % 单片段组
result = fuse_estimate_sequence(group, seg, params);
assert(numel(result.methods) == 4);  % 四种算法
assert(isequal({result.methods.method}, {'SCC','BC','CI','FCI'}));  % 算法名称正确
assert(all([result.methods.coverage_frames] == 8));  % 每种算法覆盖 8 帧
% 所有帧都是 R1_only（单源）
assert(all(arrayfun(@(m) m.source_stats.R1_only == 8, result.methods)));
end

function test_exact_k_single_and_multi()
% test_exact_k_single_and_multi 集成测试：精确 K 片段实验。
% 分别在单目标和多目标场景下运行可控衰落实验，验证：
%   1. 衰落方案构建成功
%   2. 每个目标每雷达恰好产生 2 个片段
%   3. 多目标场景下分组数 = 目标数
%   4. 无混合组、无未匹配真值
cfg = struct('scenario_name','single_turn','show_figures',false, ...
    'save_result',false,'verbose',false);
single = run_random_fade_fragment_fusion(cfg);  % 单目标实验
assert(strcmp(single.status, 'SUCCESS'));  % 实验成功
assert(all([single.fragment_validation.R1.actual_segments] == 2));  % R1 每目标 2 片段
assert(all([single.fragment_validation.R2.actual_segments] == 2));  % R2 每目标 2 片段

cfg.scenario_name = 'multi_cross';  % 切换到多目标场景
multi = run_random_fade_fragment_fusion(cfg);  % 多目标实验
assert(strcmp(multi.status, 'SUCCESS'));  % 实验成功
assert(all([multi.fragment_validation.R1.actual_segments] == 2));  % R1 每目标 2 片段
assert(all([multi.fragment_validation.R2.actual_segments] == 2));  % R2 每目标 2 片段
assert(numel(multi.grouping.groups) == multi.scenario.n_targets);  % 分组数 = 目标数
assert(multi.evaluation.mixed_group_count == 0);  % 无混合组
assert(isempty(multi.evaluation.unmatched_truth));  % 无未匹配真值
end

function seg = fixture_segment(segment_id, radar_id, track_id, frames, lon, lat)
% fixture_segment 构造测试用的片段结构体。
% 所有片段在固定经纬度上，状态为恒速（速度=0），协方差固定。
n = numel(frames);  % 帧数
x = repmat([lon;0;lat;0], 1, n);  % 状态：[经度; 经度速率; 纬度; 纬度速率]
P = repmat(eye(4)*1e-3, 1, 1, n);  % 协方差
Q = repmat(eye(4)*1e-8, 1, 1, n);  % 过程噪声
seg = struct('segment_id',segment_id,'radar_id',radar_id, ...
    'track_id',track_id,'raw_frames',frames,'effective_frames',frames, ...
    'support_frames',frames,'tail_frames',[], ...
    'support_mask',true(1,n),'coast_mask',false(1,n), ...
    'first_support_frame',frames(1),'last_support_frame',frames(end), ...
    'online_end_frame',frames(end),'start_frame',frames(1), ...
    'end_frame',frames(end),'states',x,'covariances',P, ...
    'pred_covariances',P,'process_noises',Q, ...
    'lats',lat*ones(1,n),'lons',lon*ones(1,n));
end


function snap = fixture_snapshot(frame, x, P)
% fixture_snapshot 构造测试用的航迹快照结构体。
% 用于 bridge_smoother 测试。
trk = struct('id',1,'group_id',1,'r1_id',1,'r2_id',2, ...
    'lat',x(3),'lon',x(1),'ukf',struct('x',x,'P',P), ...
    'ukf_x',x,'ukf_P',P,'source','both','is_virtual',false, ...
    'has_measurement_support',true,'segment_ids',[1 2]);
snap = struct('frameID',frame,'trackList',{{trk}});
end
