clear; close all; clc;
addpath(genpath('.'));
fprintf('Test 1: simulation_params\n');
params = simulation_params();
fprintf('OK: seed=%d\n', params.random_seed);

fprintf('Test 2: aircraft_trajectory_create\n');
fprintf('  (if fails, aircraft_trajectory_create.m has encoding issues)\n');
[traj, turn_waypoints] = aircraft_trajectory_create('gradual_turn', params);
fprintf('OK: traj dur=%.0fs\n', traj.duration_sec);

fprintf('Test 3: ukf_jichu\n');
fprintf('  (if fails, ukf_jichu.m has encoding issues)\n');
ukf0 = ukf_jichu('create', params, 113.0, 33.5, 109.0, 33.5, 30.0);
fprintf('OK: ukf model_type=%s\n', ukf0.model_type);

fprintf('Test 4: imm_tracker\n');
[snaps, ft] = imm_tracker(cell(1,1), ukf0, ukf0, params, 1, [], 0:30:100);
fprintf('OK: imm_tracker loaded\n');

fprintf('All tests passed\n');
