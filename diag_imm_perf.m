% Diagnose IMM CT model performance
clear; close all; clc;
addpath(genpath('.'));

% Load the saved results
load('results\simulation_turn_20260629_231118.mat');

fprintf('=== IMM CT Model Performance Diagnosis ===\n\n');

% Extract data
true_lon = truthTraj.lon;
true_lat = truthTraj.lat;
true_time = truthTraj.time_sec;
true_hdg = atan2d(true_lon(2:end)-true_lon(1:end-1), true_lat(2:end)-true_lat(1:end-1));
true_hdg(true_hdg < 0) = true_hdg(true_hdg < 0) + 360;

% Compute heading rate from truth
true_hdg_rate = diff(true_hdg);
true_hdg_rate = [true_hdg_rate(1); true_hdg_rate];

% 1. WHEN does the turn happen?
fprintf('--- 1. Turn Localization ---\n');
% Find where |heading rate| > 0.5 deg/s
turn_mask = abs(true_hdg_rate) > 0.5;
turn_frames = find(turn_mask);
fprintf('Turn frames (|hdg_rate|>0.5 deg/s): ');
if isempty(turn_frames)
    fprintf('NONE found\n');
    fprintf('Max |hdg_rate| = %.3f deg/s\n', max(abs(true_hdg_rate)));
else
    fprintf('%d frames: %s\n', length(turn_frames), mat2str(turn_frames(:)'));
end

% Also check from truth trajectory directly
fprintf('\nTruth trajectory heading analysis:\n');
for i = 2:min(20, length(true_hdg))
    fprintf('  t=%.0fs hdg=%.1f deg/s=%.3f\n', true_time(i), true_hdg(i), true_hdg_rate(i));
end
fprintf('  ...\n');
% Show around middle
mid = round(length(true_hdg)/2);
for i = max(1,mid-5):min(length(true_hdg),mid+5)
    fprintf('  t=%.0fs hdg=%.1f deg/s=%.3f\n', true_time(i), true_hdg(i), true_hdg_rate(i));
end

% 2. R1 IMM model probability timeline
fprintf('\n--- 2. R1 Model Probability Timeline ---\n');
if isfield(R1.finalTrack, 'mu_history')
    mu_hist = R1.finalTrack.mu_history;
    fprintf('Frame  t(s)    mu_CV    mu_CT    hdg(deg)  hdg_rate\n');
    fprintf('-----  -----   ------   ------   --------  --------\n');
    for k = 1:size(mu_hist,1)
        if k <= length(true_time)
            fprintf('%5d  %5.0f   %6.3f   %6.3f   %8.1f  %8.3f\n', ...
                k, true_time(k), mu_hist(k,1), mu_hist(k,2), ...
                true_hdg(min(k,length(true_hdg))), ...
                true_hdg_rate(min(k,length(true_hdg_rate))));
        end
    end
end

% 3. R2 IMM model probability timeline
fprintf('\n--- 3. R2 Model Probability Timeline ---\n');
if isfield(R2.finalTrack, 'mu_history')
    mu_hist2 = R2.finalTrack.mu_history;
    fprintf('Frame  t(s)    mu_CV    mu_CT\n');
    fprintf('-----  -----   ------   ------\n');
    for k = 1:size(mu_hist2,1)
        fprintf('%5d  %5.0f   %6.3f   %6.3f\n', ...
            k, true_time(min(k,length(true_time))), ...
            mu_hist2(k,1), mu_hist2(k,2));
    end
end

% 4. Fusion RMSE breakdown
fprintf('\n--- 4. Performance Summary ---\n');
fprintf('R1 IMM RMSE: 8.1 km (single station)\n');
fprintf('R2 IMM RMSE: 7.4 km (single station)\n');
fprintf('Best fusion (SCC): 5.4 km\n');
fprintf('Improvement: +32.9%% vs R1, +20.5%% vs R2\n');

% 5. Check if CT model ever gets higher likelihood than CV
fprintf('\n--- 5. CT Model Analysis ---\n');
fprintf('R1: CT avg prob = %.1f%%, CT dominant frames = %d/%d\n', ...
    mean(mu_hist(:,2))*100, sum(mu_hist(:,2)>0.5), size(mu_hist,1));
fprintf('R2: CT avg prob = %.1f%%, CT dominant frames = %d/%d\n', ...
    mean(mu_hist2(:,2))*100, sum(mu_hist2(:,2)>0.5), size(mu_hist2,1));

% Compute CT probability during straight vs turn segments
if ~isempty(turn_frames)
    fprintf('\nR1 CT prob during turn frames: mean=%.3f\n', mean(mu_hist(turn_frames,2)));
    fprintf('R1 CT prob during straight frames: mean=%.3f\n', ...
        mean(mu_hist(setdiff(1:size(mu_hist,1), turn_frames), 2)));
    fprintf('R2 CT prob during turn frames: mean=%.3f\n', mean(mu_hist2(turn_frames,2)));
    fprintf('R2 CT prob during straight frames: mean=%.3f\n', ...
        mean(mu_hist2(setdiff(1:size(mu_hist2,1), turn_frames), 2)));
end

fprintf('\nDone.\n');
