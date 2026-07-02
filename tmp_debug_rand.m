addpath(genpath('.'));
params = simulation_params();
rng(params.random_seed + 2e7);
fprintf('detection_probability = %g\n', params.detection_probability);

% 模拟 generate_frame_detections_multi 中的检测判断
for ac = 1:3
    r = rand();
    passed = r <= params.detection_probability;
    fprintf('  Target %d: rand()=%f Pd=%f passed=%d\n', ac, r, params.detection_probability, passed);
end
