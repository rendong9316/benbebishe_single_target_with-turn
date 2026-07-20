function result = tune_ukf_parameters(mode)
%TUNE_UKF_PARAMETERS Staged automatic search for UKF/IMM parameters.
%   result = tune_ukf_parameters('quick') runs the development grid.
%   result = tune_ukf_parameters('full') uses denser candidate sets.

if nargin < 1 || isempty(mode), mode = 'quick'; end
if ~ismember(mode, {'quick', 'full'})
    error('tune_ukf_parameters:badMode', 'mode must be quick or full');
end

scenario_names = {'single_turn', 'single_uturn', 'multi_cross'};
if strcmp(mode, 'full')
    seeds = [42, 142, 242];
else
    seeds = 42;
end
prepared = cell(1, numel(scenario_names) * numel(seeds));
input_index = 0;
for seed = seeds
    for i = 1:numel(scenario_names)
        input_index = input_index + 1;
        prepared{input_index} = prepare_oracle_tracking_inputs( ...
            scenario_names{i}, struct('random_seed', seed));
    end
end
defaults = prepared{1}.params;
best = struct();
records = empty_records_local();
[best_report, records] = run_candidate_local( ...
    'baseline', best, prepared, defaults, records);

if strcmp(mode, 'full')
    accel_values = [0.005, 0.01, 0.02, 0.03, 0.08, 0.15, 0.3, ...
        0.5, 0.9, 1.5, 3.0, 4.0, 6.0];
    dwell_values = [450, 90; 750, 120; 1200, 180; 1800, 240; 2400, 360];
    ct_scale_values = [0.7, 1.0, 1.3, 1.8, 2.5, 3.5, 4.5];
    transient_values = [1.5, 2.5, 3.5, 5.0, 7.0, 9.0, 12.0];
else
    accel_values = [0.05, 0.15, 0.5, 1.5, 4.0];
    dwell_values = [600, 120; 1200, 180; 1800, 240; 2400, 360];
    ct_scale_values = [0.8, 1.2, 1.8, 2.8];
    transient_values = [2.0, 3.5, 5.0];
end

for value = accel_values
    trial = best;
    trial.ukf_process_accel_psd_m2_s3 = value;
    [report, records] = run_candidate_local('accel_psd', trial, ...
        prepared, defaults, records);
    [best, best_report] = accept_if_better_local(trial, report, best, best_report);
end
for row = 1:size(dwell_values, 1)
    trial = best;
    trial.imm_cv_dwell_time_sec = dwell_values(row, 1);
    trial.imm_ct_dwell_time_sec = dwell_values(row, 2);
    [report, records] = run_candidate_local('dwell_time', trial, ...
        prepared, defaults, records);
    [best, best_report] = accept_if_better_local(trial, report, best, best_report);
end
for value = ct_scale_values
    trial = best;
    trial.imm_ct_fixed_Q_scale = value;
    [report, records] = run_candidate_local('ct_q_scale', trial, ...
        prepared, defaults, records);
    [best, best_report] = accept_if_better_local(trial, report, best, best_report);
end
for value = transient_values
    trial = best;
    trial.imm_transient_gain_max = value;
    [report, records] = run_candidate_local('transient_gain', trial, ...
        prepared, defaults, records);
    [best, best_report] = accept_if_better_local(trial, report, best, best_report);
end

initialization_sets = [6000, 10000, 50, 70; ...
                       8000, 12000, 70, 90; ...
                       10000, 16000, 90, 120; ...
                       12000, 20000, 110, 150; ...
                       14000, 24000, 130, 180];
for row = 1:size(initialization_sets, 1)
    trial = best;
    trial.radar1_ukf_init_pos_std_m = initialization_sets(row, 1);
    trial.radar2_ukf_init_pos_std_m = initialization_sets(row, 2);
    trial.radar1_ukf_init_vel_std_ms = initialization_sets(row, 3);
    trial.radar2_ukf_init_vel_std_ms = initialization_sets(row, 4);
    [report, records] = run_candidate_local('initial_covariance', trial, ...
        prepared, defaults, records);
    [best, best_report] = accept_if_better_local(trial, report, best, best_report);
end

result = struct('mode', mode, 'best_overrides', best, ...
    'best_report', best_report, 'records', records);
if ~exist('results', 'dir'), mkdir('results'); end
timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
save(fullfile('results', ['ukf_tuning_' timestamp '.mat']), 'result');
writetable(struct2table(records), ...
    fullfile('results', ['ukf_tuning_' timestamp '.csv']));
fprintf('Best score %.4f: pos %.3f km, speed %.2f m/s, NIS %.2f, NEES %.2f\n', ...
    best_report.score, best_report.position_rmse_km, ...
    best_report.speed_rmse_ms, best_report.nis_mean, best_report.nees_mean);
disp(best);
end


function [report, records] = run_candidate_local(stage, overrides, ...
        prepared, defaults, records)
report = evaluate_ukf_configuration(overrides, prepared, false);
effective = apply_overrides_local(defaults, overrides);
record = struct('candidate_id', numel(records) + 1, 'stage', stage, ...
    'score', report.score, 'position_rmse_km', report.position_rmse_km, ...
    'speed_rmse_ms', report.speed_rmse_ms, 'nis_mean', report.nis_mean, ...
    'nees_mean', report.nees_mean, ...
    'accel_psd', effective.ukf_process_accel_psd_m2_s3, ...
    'cv_dwell_sec', effective.imm_cv_dwell_time_sec, ...
    'ct_dwell_sec', effective.imm_ct_dwell_time_sec, ...
    'ct_q_scale', effective.imm_ct_fixed_Q_scale, ...
    'transient_gain', effective.imm_transient_gain_max, ...
    'r1_pos_std_m', effective.radar1_ukf_init_pos_std_m, ...
    'r2_pos_std_m', effective.radar2_ukf_init_pos_std_m, ...
    'r1_vel_std_ms', effective.radar1_ukf_init_vel_std_ms, ...
    'r2_vel_std_ms', effective.radar2_ukf_init_vel_std_ms);
records(end+1) = record;
fprintf('[%02d] %-18s score=%.4f pos=%.3fkm NIS=%.2f NEES=%.2f\n', ...
    record.candidate_id, stage, record.score, record.position_rmse_km, ...
    record.nis_mean, record.nees_mean);
end


function [best, best_report] = accept_if_better_local( ...
        trial, report, best, best_report)
if report.score < best_report.score
    best = trial;
    best_report = report;
end
end


function params = apply_overrides_local(params, overrides)
names = fieldnames(overrides);
for i = 1:numel(names), params.(names{i}) = overrides.(names{i}); end
end


function records = empty_records_local()
records = struct('candidate_id', {}, 'stage', {}, 'score', {}, ...
    'position_rmse_km', {}, 'speed_rmse_ms', {}, 'nis_mean', {}, ...
    'nees_mean', {}, 'accel_psd', {}, 'cv_dwell_sec', {}, ...
    'ct_dwell_sec', {}, 'ct_q_scale', {}, 'transient_gain', {}, ...
    'r1_pos_std_m', {}, 'r2_pos_std_m', {}, 'r1_vel_std_ms', {}, ...
    'r2_vel_std_ms', {});
end
