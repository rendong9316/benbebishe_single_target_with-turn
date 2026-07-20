function run_fragment_fusion_tests(run_integration)
% RUN_FRAGMENT_FUSION_TESTS Dynamic fragment grouping/fusion tests.
if nargin < 1, run_integration = false; end
addpath(genpath('.'));
test_unknown_target_count_grouping();
test_cross_handoff();
test_truth_leak_rejected();
test_four_method_single_source();
test_bridge_bounded_gap();
test_bridge_turn_direction();
test_bridge_low_confidence();
if run_integration
    test_exact_k_single_and_multi();
end
disp('fragment fusion tests ok');
end

function test_bridge_bounded_gap()
params = simulation_params_oracle();
snapshots = cell(8,1);
P = diag([1e-5 1e-9 1e-5 1e-9]);
snapshots{2} = fixture_snapshot(2, [128; 0.001; 31; 0.0005], P);
snapshots{6} = fixture_snapshot(6, [128.12; 0.001; 31.06; 0.0005], P);
result = bridge_smoother(snapshots, params);
assert(result.rts.bridge_frame_count == 3);
assert(result.imm.bridge_frame_count == 3);
assert(isempty(result.reconstructed_snapshots{1}));
assert(isempty(result.reconstructed_snapshots{7}));
assert(isempty(result.reconstructed_snapshots{8}));
for k = 3:5
    trk = result.reconstructed_snapshots{k}.trackList{1};
    assert(trk.is_virtual && ~trk.has_measurement_support);
    assert(strcmp(trk.source, 'bridge_smoothed'));
    assert(all(isfinite(trk.ukf.x)) && all(isfinite(trk.ukf.P(:))));
    assert(min(eig((trk.ukf.P + trk.ukf.P')/2)) > 0);
end
assert(~result.reconstructed_snapshots{2}.trackList{1}.is_virtual);
end

function test_bridge_turn_direction()
params = simulation_params_oracle();
params.bridge.accel_std_mps2 = 0.05;
lat0 = 31; lon0 = 128; radius = params.bridge.earth_radius_m;
meters_lon = radius * cosd(lat0) * pi / 180;
meters_lat = radius * pi / 180;
dt = params.dt_sec * 4;
omega = params.bridge.turn_rate_rad_per_sec;
s = sin(omega*dt); c = cos(omega*dt); omc = 1-c;
F = [1 s/omega 0 -omc/omega; 0 c 0 -s; ...
     0 omc/omega 1 s/omega; 0 s 0 c];
x0e = [0; 250; 0; 0];
x1e = F*x0e;
x0 = [lon0; x0e(2)/meters_lon; lat0; x0e(4)/meters_lat];
x1 = [lon0+x1e(1)/meters_lon; x1e(2)/meters_lon; ...
      lat0+x1e(3)/meters_lat; x1e(4)/meters_lat];
P = diag([1e-8 1e-11 1e-8 1e-11]);
snapshots = cell(5,1);
snapshots{1} = fixture_snapshot(1, x0, P);
snapshots{5} = fixture_snapshot(5, x1, P);
result = bridge_smoother(snapshots, params);
prob = result.imm.bridge_snapshots{3}.trackList{1}.bridge_mode_probabilities;
assert(abs(sum(prob)-1) < 1e-10);
assert(prob(2) > prob(3));
end

function test_bridge_low_confidence()
params = simulation_params_oracle();
params.bridge.confidence_mahal_gate = 0;
P = diag([1e-7 1e-10 1e-7 1e-10]);
snapshots = cell(4,1);
snapshots{1} = fixture_snapshot(1, [128;0;31;0], P);
snapshots{4} = fixture_snapshot(4, [130;0;34;0], P);
result = bridge_smoother(snapshots, params);
assert(result.imm.bridge_frame_count == 2);
assert(result.imm.low_confidence_bridge_count == 2);
assert(strcmp(result.imm.bridge_snapshots{2}.trackList{1}.bridge_confidence, 'low'));
end

function test_unknown_target_count_grouping()
params = simulation_params_oracle();
frames = 1:10;
segments = [fixture_segment(1,1,11,frames,128.0,31.0), ...
    fixture_segment(2,1,12,frames,130.0,33.0), ...
    fixture_segment(3,2,21,frames,128.03,31.02), ...
    fixture_segment(4,2,22,frames,130.03,33.02)];
result = tracklet_grouping('segments', segments, params);
assert(strcmp(result.status, 'SUCCESS'));
assert(numel(result.groups) == 2);
assert(all(sort(cellfun(@numel, {result.groups.segment_indices})) == [2,2]));
assert(all(arrayfun(@(g) numel(unique([segments(g.segment_indices).radar_id])) == 2, ...
    result.groups)));
end

function test_cross_handoff()
params = simulation_params_oracle();
a = fixture_segment(1,1,1,1:8,128.0,31.0);
b = fixture_segment(2,2,2,10:17,128.0,31.0);
result = tracklet_grouping('segments', [a,b], params);
assert(strcmp(result.status, 'SUCCESS'));
assert(numel(result.edges) == 1);
assert(strcmp(result.edges(1).edge_type, 'handoff'));
assert(numel(result.groups) == 1);
end

function test_truth_leak_rejected()
params = simulation_params_oracle();
seg = fixture_segment(1,1,1,1:8,128.0,31.0);
seg.truth_idx = 1;
failed = false;
try
    tracklet_grouping('segments', seg, params);
catch exception
    failed = strcmp(exception.identifier, 'tracklet_grouping:truthLeak');
end
assert(failed);
end

function test_four_method_single_source()
params = simulation_params_oracle();
seg = fixture_segment(1,1,1,1:8,128.0,31.0);
group = struct('group_id',1,'segment_indices',1);
result = fuse_estimate_sequence(group, seg, params);
assert(numel(result.methods) == 4);
assert(isequal({result.methods.method}, {'SCC','BC','CI','FCI'}));
assert(all([result.methods.coverage_frames] == 8));
assert(all(arrayfun(@(m) m.source_stats.R1_only == 8, result.methods)));
end

function test_exact_k_single_and_multi()
cfg = struct('scenario_name','single_turn','show_figures',false, ...
    'save_result',false,'verbose',false);
single = run_random_fade_fragment_fusion(cfg);
assert(strcmp(single.status, 'SUCCESS'));
assert(all([single.fragment_validation.R1.actual_segments] == 2));
assert(all([single.fragment_validation.R2.actual_segments] == 2));

cfg.scenario_name = 'multi_cross';
multi = run_random_fade_fragment_fusion(cfg);
assert(strcmp(multi.status, 'SUCCESS'));
assert(all([multi.fragment_validation.R1.actual_segments] == 2));
assert(all([multi.fragment_validation.R2.actual_segments] == 2));
assert(numel(multi.grouping.groups) == multi.scenario.n_targets);
assert(multi.evaluation.mixed_group_count == 0);
assert(isempty(multi.evaluation.unmatched_truth));
end

function seg = fixture_segment(segment_id, radar_id, track_id, frames, lon, lat)
n = numel(frames);
x = repmat([lon;0;lat;0], 1, n);
P = repmat(eye(4)*1e-3, 1, 1, n);
Q = repmat(eye(4)*1e-8, 1, 1, n);
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
trk = struct('id',1,'group_id',1,'r1_id',1,'r2_id',2, ...
    'lat',x(3),'lon',x(1),'ukf',struct('x',x,'P',P), ...
    'ukf_x',x,'ukf_P',P,'source','both','is_virtual',false, ...
    'has_measurement_support',true,'segment_ids',[1 2]);
snap = struct('frameID',frame,'trackList',{{trk}});
end
