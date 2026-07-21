function result = run_ukf_monte_carlo(mode, options)
%RUN_UKF_MONTE_CARLO Calibrate, validate, or regress the UKF/IMM chain.
% All target identities are used only after tracking for truth evaluation.

if nargin < 1 || isempty(mode), mode = 'validate'; end
if nargin < 2, options = struct(); end
options = monte_carlo_defaults_local(mode, options);
addpath(genpath('.'));
root = fullfile('results', 'monte_carlo');
if ~exist(root, 'dir'), mkdir(root); end

switch mode
    case {'calibrate', 'smoke'}
        result = calibrate_staged_local(options, root);
    case 'calibrate_joint'
        result = calibrate_joint_local(options, root);
    case 'validate'
        result = validate_local(options, root);
    case 'full_chain'
        result = full_chain_local(options, root);
    otherwise
        error('run_ukf_monte_carlo:badMode', ...
            ['mode must be calibrate, calibrate_joint, validate, ' ...
             'full_chain, or smoke']);
end
end


function result = calibrate_staged_local(options, root)
% First stabilize the UKF core, then tune IMM behavior around that core.
ukf_stage = calibrate_stage_local('ukf', struct(), options, root);
imm_stage = calibrate_stage_local( ...
    'imm', ukf_stage.selected_overrides, options, root);

timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
output_dir = fullfile(root, ['calibration_staged_' timestamp]);
mkdir(output_dir);
result = struct('mode', 'calibrate_staged', 'options', options, ...
    'ukf_stage', ukf_stage, 'imm_stage', imm_stage, ...
    'best_overrides', imm_stage.selected_overrides, ...
    'best_report', imm_stage.selected_report, ...
    'best_violation', imm_stage.selected_item.violation, ...
    'best_objective', imm_stage.selected_item.objective, ...
    'ukf_stage_accepted', ukf_stage.accepted, ...
    'imm_stage_accepted', imm_stage.accepted, ...
    'output_dir', output_dir);
save(fullfile(output_dir, 'calibration_result.mat'), 'result');
if options.publish_calibration
    save(fullfile(root, 'latest_calibration.mat'), 'result');
end
fprintf(['Staged calibration: UKF accepted=%d, IMM accepted=%d, ' ...
    'final pos=%.3f km\n'], ukf_stage.accepted, imm_stage.accepted, ...
    result.best_report.position_rmse_km);
disp(result.best_overrides);
end


function stage_result = calibrate_stage_local(stage, base_overrides, options, root)
specs = staged_specs_local(stage);
prepared = prepare_set_local(options.calibration_seeds, specs, options);
screen_count = min(options.screen_seed_count, numel(options.calibration_seeds));
screen_prepared = prepared(1:screen_count * numel(specs));
work_dir = checkpoint_dir_local(root, ['calibration_' stage], options);
baseline_checkpoint = fullfile(work_dir, 'baseline_reports.mat');
spec_signature = strjoin({specs.label}, '|');
baseline_valid = false;
if exist(baseline_checkpoint, 'file')
    loaded = load(baseline_checkpoint, 'baseline_screen', 'baseline_full', ...
        'checkpoint_base_overrides', 'checkpoint_spec_signature');
    baseline_valid = isfield(loaded, 'checkpoint_base_overrides') && ...
        isequaln(loaded.checkpoint_base_overrides, base_overrides) && ...
        isfield(loaded, 'checkpoint_spec_signature') && ...
        strcmp(loaded.checkpoint_spec_signature, spec_signature);
end
if baseline_valid
    baseline_screen = loaded.baseline_screen;
    baseline_full = loaded.baseline_full;
else
    baseline_screen = evaluate_ukf_configuration( ...
        base_overrides, screen_prepared, false);
    baseline_full = evaluate_ukf_configuration( ...
        base_overrides, prepared, false);
    checkpoint_base_overrides = base_overrides; %#ok<NASGU>
    checkpoint_spec_signature = spec_signature; %#ok<NASGU>
    save(baseline_checkpoint, 'baseline_screen', 'baseline_full', ...
        'checkpoint_base_overrides', 'checkpoint_spec_signature');
end

candidate_count = options.staged_ukf_candidate_count;
if strcmp(stage, 'imm')
    candidate_count = options.staged_imm_candidate_count;
end
candidates = staged_lhs_candidates_local(stage, candidate_count, base_overrides);
screen = repmat(empty_staged_item_local(), 1, numel(candidates));
for i = 1:numel(candidates)
    checkpoint = fullfile(work_dir, sprintf('screen_%03d.mat', i));
    valid = false;
    if exist(checkpoint, 'file')
        loaded_item = load(checkpoint, 'item');
        valid = isfield(loaded_item, 'item') && ...
            isequaln(loaded_item.item.overrides, candidates(i));
    end
    if valid
        screen(i) = loaded_item.item;
    else
        report = evaluate_ukf_configuration( ...
            candidates(i), screen_prepared, false);
        item = staged_candidate_result_local( ...
            stage, candidates(i), report, baseline_screen);
        save(checkpoint, 'item');
        screen(i) = item;
        clear report item;
    end
    fprintf('%s screen %d/%d complete\n', stage, i, numel(candidates));
end

[~, order] = sortrows([[screen.violation]', [screen.objective]'], [1, 2]);
top_count = min(options.staged_top_candidate_count, numel(order));
top_indices = order(1:top_count);
full_results = repmat(empty_staged_item_local(), 1, top_count);
for i = 1:top_count
    candidate = candidates(top_indices(i));
    checkpoint = fullfile(work_dir, ...
        sprintf('full_candidate_%03d.mat', top_indices(i)));
    valid = false;
    if exist(checkpoint, 'file')
        loaded_item = load(checkpoint, 'item');
        valid = isfield(loaded_item, 'item') && ...
            isequaln(loaded_item.item.overrides, candidate);
    end
    if valid
        full_results(i) = loaded_item.item;
    else
        report = evaluate_ukf_configuration(candidate, prepared, false);
        item = staged_candidate_result_local( ...
            stage, candidate, report, baseline_full);
        save(checkpoint, 'item');
        full_results(i) = item;
        clear report item;
    end
    fprintf('%s full %d/%d complete\n', stage, i, top_count);
end

best = choose_best_candidate_local(full_results);
local_history = empty_staged_item_local();
local_history = local_history([]);
fields = staged_fields_local(stage);
for pass = 1:options.staged_local_refinement_passes
    for field_index = 1:numel(fields)
        for direction_index = 1:2
            direction = [-1, 1];
            source_overrides = best.overrides;
            name = fields{field_index};
            trial = source_overrides;
            trial.(name) = staged_bounded_parameter_local( ...
                name, trial.(name) * ...
                (1 + options.staged_local_step_fraction * ...
                direction(direction_index)));
            checkpoint = fullfile(work_dir, sprintf( ...
                'local_pass_%02d_field_%02d_direction_%d.mat', ...
                pass, field_index, direction_index));
            valid = false;
            if exist(checkpoint, 'file')
                loaded_item = load(checkpoint, 'item', 'checkpoint_source');
                valid = isfield(loaded_item, 'item') && ...
                    isfield(loaded_item, 'checkpoint_source') && ...
                    isequaln(loaded_item.checkpoint_source, source_overrides) && ...
                    isequaln(loaded_item.item.overrides, trial);
            end
            if valid
                item = loaded_item.item;
            else
                report = evaluate_ukf_configuration(trial, prepared, false);
                item = staged_candidate_result_local( ...
                    stage, trial, report, baseline_full);
                checkpoint_source = source_overrides; %#ok<NASGU>
                save(checkpoint, 'item', 'checkpoint_source');
                clear report checkpoint_source;
            end
            local_history(end+1) = item; %#ok<AGROW>
            if candidate_less_local(item, best), best = item; end
            fprintf('%s local pass %d/%d field %d/%d direction %d/2 complete\n', ...
                stage, pass, options.staged_local_refinement_passes, ...
                field_index, numel(fields), direction_index);
        end
    end
end

baseline_item = staged_candidate_result_local( ...
    stage, candidates(1), baseline_full, baseline_full);
accepted = staged_acceptance_local(stage, best);
if accepted
    selected_item = best;
else
    selected_item = baseline_item;
end
stage_result = struct('stage', stage, 'base_overrides', base_overrides, ...
    'baseline_report', baseline_full, 'screen_results', screen, ...
    'full_results', full_results, 'local_results', local_history, ...
    'search_best_item', best, 'accepted', accepted, ...
    'selected_item', selected_item, ...
    'selected_overrides', selected_item.overrides, ...
    'selected_report', selected_item.report, 'work_dir', work_dir);
save(fullfile(work_dir, 'stage_result.mat'), 'stage_result');
writetable(staged_candidate_table_local( ...
    [screen, full_results, local_history], fields), ...
    fullfile(work_dir, 'candidate_summary.csv'));
fprintf('%s stage best: accepted=%d violation=%.4f objective=%.4f pos=%.3f km\n', ...
    upper(stage), accepted, best.violation, best.objective, ...
    best.report.position_rmse_km);
end


function specs = staged_specs_local(stage)
guard = [scenario_spec_local('single_straight', 'single_straight', 1.0), ...
    scenario_spec_local('single_turn_left_short', 'left_short', 1.0), ...
    scenario_spec_local('single_turn_right_short', 'right_short', 1.0)];
if strcmp(stage, 'ukf')
    specs = guard;
else
    specs = [guard, ...
        scenario_spec_local('single_turn_left_sustained', ...
            'left_sustained', 1.0), ...
        scenario_spec_local('single_turn_right_sustained', ...
            'right_sustained', 1.0)];
end
end


function candidates = staged_lhs_candidates_local(stage, count, base_overrides)
fields = staged_fields_local(stage);
ranges = parameter_ranges_local();
seed = 71001;
if strcmp(stage, 'imm'), seed = 72001; end
rng(seed, 'twister');
unit = zeros(count, numel(fields));
for column = 1:numel(fields)
    unit(:, column) = (randperm(count)' - rand(count, 1)) / count;
end
defaults = merge_overrides_local( ...
    params_to_field_overrides_local(simulation_params_oracle(), fields), ...
    base_overrides);
candidates = repmat(defaults, 1, count);
for row = 1:count
    candidate = base_overrides;
    for column = 1:numel(fields)
        name = fields{column};
        bounds = ranges.(name);
        if strcmp(name, 'ukf_process_accel_psd_m2_s3')
            value = exp(log(bounds(1)) + unit(row, column) * ...
                (log(bounds(2)) - log(bounds(1))));
        else
            value = bounds(1) + unit(row, column) * ...
                (bounds(2) - bounds(1));
        end
        candidate.(name) = value;
    end
    candidates(row) = merge_overrides_local(defaults, candidate);
end
candidates(1) = defaults;
end


function item = staged_candidate_result_local(stage, overrides, report, baseline)
item = empty_staged_item_local();
item.overrides = overrides;
item.report = report;
case_position_ratio = [report.cases.position_rmse_km] ./ ...
    [baseline.cases.position_rmse_km];
case_speed_ratio = [report.cases.speed_rmse_ms] ./ ...
    [baseline.cases.speed_rmse_ms];
scenario_names = {report.cases.scenario};
sustained = contains(scenario_names, 'sustained');
guard = ~sustained;
metrics = struct('position_ratio', mean(case_position_ratio), ...
    'speed_ratio', mean(case_speed_ratio), ...
    'guard_max_position_regression', ...
        max_or_zero_local(case_position_ratio(guard) - 1), ...
    'guard_max_speed_regression', ...
        max_or_zero_local(case_speed_ratio(guard) - 1), ...
    'sustained_position_ratio', mean_or_one_local(case_position_ratio(sustained)), ...
    'sustained_speed_ratio', mean_or_one_local(case_speed_ratio(sustained)), ...
    'turn_direction_accuracy', NaN, 'turn_detection_delay_frames', NaN, ...
    'straight_false_ct_rate', NaN);
straight = strcmp(scenario_names, 'single_straight');
if any(straight)
    metrics.straight_false_ct_rate = mean( ...
        [report.cases(straight).straight_false_ct_rate], 'omitnan');
end
if any(sustained)
    metrics.turn_direction_accuracy = mean( ...
        [report.cases(sustained).turn_direction_accuracy], 'omitnan');
    metrics.turn_detection_delay_frames = median( ...
        [report.cases(sustained).turn_detection_delay_frames], 'omitnan');
end
if strcmp(stage, 'ukf')
    item.objective = 0.75 * metrics.position_ratio + ...
        0.25 * metrics.speed_ratio;
else
    baseline_accuracy = mean( ...
        [baseline.cases(sustained).turn_direction_accuracy], 'omitnan');
    denominator = max(0.05, 1 - baseline_accuracy);
    direction_ratio = (1 - metrics.turn_direction_accuracy) / denominator;
    item.objective = 0.75 * metrics.sustained_position_ratio + ...
        0.15 * metrics.sustained_speed_ratio + 0.10 * direction_ratio;
end
item.metrics = metrics;
item.violation = staged_violation_local(stage, report, metrics);
end


function violation = staged_violation_local(stage, report, metrics)
violations = [interval_violation_local(report.nis_mean / 3, 0.40, 1.40), ...
    interval_violation_local(report.nees_mean / 4, 0.40, 1.40), ...
    interval_violation_local(report.nis_coverage95, 0.70, 0.99), ...
    interval_violation_local(report.nees_coverage95, 0.70, 0.99)];
if strcmp(stage, 'ukf')
    violations(end+1) = max(0, metrics.guard_max_position_regression - 0.03);
    violations(end+1) = max(0, metrics.guard_max_speed_regression - 0.05);
else
    violations(end+1) = max(0, metrics.guard_max_position_regression - 0.02);
    violations(end+1) = max(0, metrics.guard_max_speed_regression - 0.05);
    violations(end+1) = max(0, metrics.straight_false_ct_rate - 0.02);
    violations(end+1) = max(0, 0.65 - metrics.turn_direction_accuracy);
    violations(end+1) = max(0, metrics.turn_detection_delay_frames - 5) / 5;
end
if any(~isfinite(violations))
    violation = inf;
else
    violation = max(violations) + 0.01 * sum(violations);
end
end


function accepted = staged_acceptance_local(stage, item)
if strcmp(stage, 'ukf')
    accepted = item.violation <= 1e-12 && ...
        item.metrics.position_ratio <= 0.995 && item.objective < 0.997;
else
    accepted = item.violation <= 1e-12 && ...
        item.metrics.sustained_position_ratio <= 0.98 && ...
        item.objective < 0.99;
end
end


function fields = staged_fields_local(stage)
if strcmp(stage, 'ukf')
    fields = {'ukf_process_accel_psd_m2_s3', ...
        'radar1_ukf_init_pos_std_m', 'radar2_ukf_init_pos_std_m', ...
        'radar1_ukf_init_vel_std_ms', 'radar2_ukf_init_vel_std_ms'};
else
    fields = {'imm_cv_dwell_time_sec', 'imm_ct_dwell_time_sec', ...
        'imm_ct_fixed_Q_scale', 'imm_transient_gain_max', ...
        'imm_transient_nis_start', 'imm_transient_nis_full', ...
        'imm_transient_ewma_alpha'};
end
end


function value = staged_bounded_parameter_local(name, value)
ranges = parameter_ranges_local();
value = min(ranges.(name)(2), max(ranges.(name)(1), value));
end


function result = merge_overrides_local(base, extra)
result = base;
names = fieldnames(extra);
for i = 1:numel(names), result.(names{i}) = extra.(names{i}); end
end


function overrides = params_to_field_overrides_local(params, fields)
overrides = struct();
for i = 1:numel(fields), overrides.(fields{i}) = params.(fields{i}); end
end


function value = max_or_zero_local(values)
if isempty(values), value = 0; else, value = max(values); end
end


function value = mean_or_one_local(values)
if isempty(values), value = 1; else, value = mean(values); end
end


function item = empty_staged_item_local()
item = struct('overrides', struct(), 'report', struct(), ...
    'metrics', struct(), 'violation', inf, 'objective', inf);
end


function table_value = staged_candidate_table_local(items, fields)
if isempty(items), table_value = table(); return; end
row_cells = cell(1, numel(items));
for i = 1:numel(items)
    row = struct('violation', items(i).violation, ...
        'objective', items(i).objective, ...
        'position_rmse_km', items(i).report.position_rmse_km, ...
        'speed_rmse_ms', items(i).report.speed_rmse_ms, ...
        'nis_mean', items(i).report.nis_mean, ...
        'nees_mean', items(i).report.nees_mean);
    metric_names = fieldnames(items(i).metrics);
    for j = 1:numel(metric_names)
        row.(metric_names{j}) = items(i).metrics.(metric_names{j});
    end
    for j = 1:numel(fields)
        row.(fields{j}) = items(i).overrides.(fields{j});
    end
    row_cells{i} = row;
end
rows = [row_cells{:}];
table_value = struct2table(rows);
end


function result = calibrate_joint_local(options, root)
ensure_parallel_pool_local(options);
specs = calibration_specs_local();
prepared = prepare_set_local(options.calibration_seeds, specs, options);
screen_count = min(options.screen_seed_count, numel(options.calibration_seeds));
screen_prepared = prepared(1:screen_count * numel(specs));
work_dir = checkpoint_dir_local(root, 'calibration', options);
if ~exist(work_dir, 'dir'), mkdir(work_dir); end
baseline_checkpoint = fullfile(work_dir, 'baseline_reports.mat');
if exist(baseline_checkpoint, 'file')
    loaded_baseline = load(baseline_checkpoint, 'baseline_screen', 'baseline_full');
    baseline_screen = loaded_baseline.baseline_screen;
    baseline_full = loaded_baseline.baseline_full;
else
    baseline_screen = evaluate_ukf_configuration(struct(), screen_prepared, false);
    baseline_full = evaluate_ukf_configuration(struct(), prepared, false);
    save(baseline_checkpoint, 'baseline_screen', 'baseline_full');
end

candidates = lhs_candidates_local(options.candidate_count);
default_params = simulation_params_oracle();
candidates(1) = params_to_overrides_local(default_params);
screen = repmat(empty_candidate_result_local(), 1, numel(candidates));
if options.use_parallel
    parfor i = 1:numel(candidates)
        report = evaluate_ukf_configuration(candidates(i), screen_prepared, false);
        screen(i) = candidate_result_local(candidates(i), report, baseline_screen);
    end
else
    for i = 1:numel(candidates)
        checkpoint = fullfile(work_dir, sprintf('screen_%03d.mat', i));
        if exist(checkpoint, 'file')
            loaded = load(checkpoint, 'item');
            screen(i) = loaded.item;
        else
            report = evaluate_ukf_configuration(candidates(i), screen_prepared, false);
            item = candidate_result_local(candidates(i), report, baseline_screen);
            save(checkpoint, 'item');
            screen(i) = item;
            clear report item;
        end
        fprintf('screen %d/%d complete\n', i, numel(candidates));
    end
end
fprintf('screening candidates complete: %d\n', numel(candidates));
[~, order] = sortrows([[screen.violation]', [screen.objective]'], [1, 2]);
top_count = min(options.top_candidate_count, numel(order));
top_indices = order(1:top_count);
full_results = repmat(empty_candidate_result_local(), 1, top_count);
if options.use_parallel
    parfor i = 1:top_count
        report = evaluate_ukf_configuration( ...
            candidates(top_indices(i)), prepared, false);
        full_results(i) = candidate_result_local( ...
            candidates(top_indices(i)), report, baseline_full);
    end
else
    for i = 1:top_count
        checkpoint = fullfile(work_dir, ...
            sprintf('full_candidate_%03d.mat', top_indices(i)));
        if exist(checkpoint, 'file')
            loaded = load(checkpoint, 'item');
            full_results(i) = loaded.item;
        else
            report = evaluate_ukf_configuration( ...
                candidates(top_indices(i)), prepared, false);
            item = candidate_result_local( ...
                candidates(top_indices(i)), report, baseline_full);
            save(checkpoint, 'item');
            full_results(i) = item;
            clear report item;
        end
        fprintf('full %d/%d complete\n', i, top_count);
    end
end
fprintf('full-seed candidates complete: %d\n', top_count);
best = choose_best_candidate_local(full_results);

local_history = empty_candidate_result_local();
local_history = local_history([]);
fields = tunable_fields_local();
for pass = 1:options.local_refinement_passes
    for field_index = 1:numel(fields)
        directions = [-1, 1];
        for direction_index = 1:numel(directions)
            checkpoint = fullfile(work_dir, sprintf( ...
                'local_pass_%02d_field_%02d_direction_%d.mat', ...
                pass, field_index, direction_index));
            if exist(checkpoint, 'file')
                loaded = load(checkpoint, 'item');
                item = loaded.item;
            else
                trial = best.overrides;
                name = fields{field_index};
                trial.(name) = bounded_parameter_local( ...
                    name, trial.(name) * ...
                    (1 + 0.25 * directions(direction_index)));
                report = evaluate_ukf_configuration(trial, prepared, false);
                item = candidate_result_local(trial, report, baseline_full);
                save(checkpoint, 'item');
                clear report trial;
            end
            local_history(end+1) = item; %#ok<AGROW>
            if candidate_less_local(item, best)
                best = item;
            end
            fprintf('local pass %d/%d field %d/%d direction %d/2 complete\n', ...
                pass, options.local_refinement_passes, field_index, ...
                numel(fields), direction_index);
        end
    end
end

timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
output_dir = fullfile(root, ['calibration_' timestamp]);
mkdir(output_dir);
result = struct('mode', 'calibrate', 'options', options, ...
    'baseline_report', baseline_full, 'screen_results', screen, ...
    'full_results', full_results, 'local_results', local_history, ...
    'best_overrides', best.overrides, 'best_report', best.report, ...
    'best_violation', best.violation, 'best_objective', best.objective, ...
    'output_dir', output_dir);
save(fullfile(output_dir, 'calibration_result.mat'), 'result');
if options.publish_calibration
    save(fullfile(root, 'latest_calibration.mat'), 'result');
end
writetable(candidate_table_local([screen, full_results, local_history]), ...
    fullfile(output_dir, 'candidate_summary.csv'));
fprintf('Calibration best: violation=%.4f objective=%.4f pos=%.3f km\n', ...
    best.violation, best.objective, best.report.position_rmse_km);
disp(best.overrides);
end


function result = validate_local(options, root)
ensure_parallel_pool_local(options);
overrides = resolve_calibration_overrides_local(options, root);
specs = validation_specs_local();
prepared = prepare_set_local(options.validation_seeds, specs, options);
records = empty_validation_records_local();
pair_results = cell(1, numel(prepared));
work_dir = checkpoint_dir_local(root, 'validation', options);
if options.use_parallel
    parfor i = 1:numel(prepared)
        pair_results{i} = {evaluate_ukf_configuration(struct(), prepared(i), false), ...
            evaluate_ukf_configuration(overrides, prepared(i), false)};
    end
else
    for i = 1:numel(prepared)
        checkpoint = fullfile(work_dir, sprintf('pair_%05d.mat', i));
        if exist(checkpoint, 'file')
            loaded = load(checkpoint, 'pair', 'candidate_overrides', ...
                'prepared_path');
        else
            loaded = struct();
        end
        if isfield(loaded, 'candidate_overrides') && ...
                isequaln(loaded.candidate_overrides, overrides) && ...
                isfield(loaded, 'prepared_path') && ...
                strcmp(loaded.prepared_path, prepared{i})
            pair_results{i} = loaded.pair;
        else
            pair = {evaluate_ukf_configuration(struct(), prepared(i), false), ...
                evaluate_ukf_configuration(overrides, prepared(i), false)};
            candidate_overrides = overrides; %#ok<NASGU>
            prepared_path = prepared{i}; %#ok<NASGU>
            save(checkpoint, 'pair', 'candidate_overrides', 'prepared_path');
            pair_results{i} = pair;
            clear pair candidate_overrides prepared_path;
        end
        if mod(i, 20) == 0
            fprintf('validation %d/%d evaluations complete\n', ...
                i, numel(prepared));
        end
    end
end
for i = 1:numel(prepared)
    baseline = pair_results{i}{1};
    candidate = pair_results{i}{2};
    label = baseline.cases(1).scenario;
    for case_index = 1:numel(baseline.cases)
        records(end+1) = validation_record_local( ...
            label, baseline.cases(case_index), 'baseline'); %#ok<AGROW>
        records(end+1) = validation_record_local( ...
            label, candidate.cases(case_index), 'candidate'); %#ok<AGROW>
    end
    if mod(i, 20) == 0
        fprintf('validation %d/%d prepared cases complete\n', i, numel(prepared));
    end
end
summary = validation_summary_local(records, options);
timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
output_dir = fullfile(root, ['validation_' timestamp]);
mkdir(output_dir);
result = struct('mode', 'validate', 'options', options, ...
    'candidate_overrides', overrides, 'records', records, ...
    'summary', summary, 'output_dir', output_dir);
save(fullfile(output_dir, 'validation_result.mat'), 'result');
writetable(struct2table(records), fullfile(output_dir, 'run_metrics.csv'));
writetable(struct2table(summary.groups), fullfile(output_dir, 'group_summary.csv'));
write_validation_figures_local(records, summary, output_dir);
fprintf('Validation accepted=%d, paired improvement=%.2f%%, CI=[%.3f %.3f] km\n', ...
    summary.accepted, 100 * summary.position_improvement_fraction, ...
    summary.bootstrap_ci_km(1), summary.bootstrap_ci_km(2));
end


function result = full_chain_local(options, root)
overrides = resolve_calibration_overrides_local(options, root);
records = empty_chain_records_local();
work_dir = checkpoint_dir_local(root, 'full_chain', options);
for seed = options.full_chain_seeds
    for configuration = 1:2
        if configuration == 1
            name = 'baseline';
            filter_overrides = struct();
        else
            name = 'candidate';
            filter_overrides = overrides;
        end
        filter_overrides.random_seed = seed;
        config = struct('scenario_name', 'multi_cross', 'show_figures', false, ...
            'save_result', false, 'verbose', false, 'print_summary', false, ...
            'param_overrides', filter_overrides, ...
            'fragment_seed_r1', seed + 110000, ...
            'fragment_seed_r2', seed + 220000);
        checkpoint = fullfile(work_dir, sprintf( ...
            'seed_%d_configuration_%s.mat', seed, name));
        if exist(checkpoint, 'file')
            loaded = load(checkpoint, 'record', 'checkpoint_overrides');
        else
            loaded = struct();
        end
        if isfield(loaded, 'checkpoint_overrides') && ...
                isequaln(loaded.checkpoint_overrides, filter_overrides)
            record = loaded.record;
        else
            item = run_random_fade_fragment_fusion(config);
            record = chain_record_local(seed, name, item);
            checkpoint_overrides = filter_overrides; %#ok<NASGU>
            save(checkpoint, 'record', 'checkpoint_overrides');
            clear item checkpoint_overrides;
        end
        records(end+1) = record; %#ok<AGROW>
    end
    fprintf('full-chain seed %d complete\n', seed);
end
summary = chain_summary_local(records);
timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
output_dir = fullfile(root, ['full_chain_' timestamp]);
mkdir(output_dir);
result = struct('mode', 'full_chain', 'options', options, ...
    'candidate_overrides', overrides, 'records', records, ...
    'summary', summary, 'output_dir', output_dir);
save(fullfile(output_dir, 'full_chain_result.mat'), 'result');
writetable(struct2table(records), fullfile(output_dir, 'full_chain_metrics.csv'));
fprintf('Full-chain success %.1f%%, wrong merges=%d, group success %.1f%%\n', ...
    100 * summary.candidate_success_rate, summary.candidate_wrong_merges, ...
    100 * summary.candidate_group_success_rate);
end


function prepared = prepare_set_local(seeds, specs, options)
prepared = cell(1, numel(seeds) * numel(specs));
task_seeds = zeros(1, numel(prepared));
task_specs = repmat(specs(1), 1, numel(prepared));
index = 0;
for seed = seeds
    for spec_index = 1:numel(specs)
        index = index + 1;
        task_seeds(index) = seed;
        task_specs(index) = specs(spec_index);
    end
end
if options.use_parallel
    ensure_parallel_pool_local(options);
    parfor task = 1:numel(prepared)
        prepared{task} = prepare_cached_local(task_seeds(task), task_specs(task), options);
    end
else
    for task = 1:numel(prepared)
        prepared{task} = prepare_cached_local(task_seeds(task), task_specs(task), options);
    end
end
end


function path = prepare_cached_local(seed, spec, options)
cache_dir = fullfile('results', 'mc_cache', options.cache_version);
if ~exist(cache_dir, 'dir'), mkdir(cache_dir); end
rate_token = strrep(sprintf('%.3f', spec.turn_rate), '.', 'p');
path = fullfile(cache_dir, sprintf('seed_%d_%s_rate_%s.mat', ...
    seed, spec.label, rate_token));
if exist(path, 'file')
    return;
end
overrides = struct('random_seed', seed, ...
    'truth_turn_rate_deg_per_sec', spec.turn_rate); %#ok<NASGU>
evalc('inputs = prepare_oracle_tracking_inputs(spec.scenario, overrides);');
inputs.scenario.name = spec.label;
save(path, 'inputs');
clear inputs;
end


function specs = calibration_specs_local()
specs = [scenario_spec_local('single_straight', 'single_straight', 1.0), ...
    scenario_spec_local('single_turn_left_short', 'left_short', 1.0), ...
    scenario_spec_local('single_turn_right_short', 'right_short', 1.0), ...
    scenario_spec_local('single_turn_left_sustained', 'left_sustained', 1.0), ...
    scenario_spec_local('single_turn_right_sustained', 'right_sustained', 1.0), ...
    scenario_spec_local('multi_cross', 'multi_cross', 1.0)];
end


function specs = validation_specs_local()
specs = calibration_specs_local();
specs = [specs, ...
    scenario_spec_local('single_turn_left_sustained', 'left_rate_0p7', 0.7), ...
    scenario_spec_local('single_turn_right_sustained', 'right_rate_0p7', 0.7), ...
    scenario_spec_local('single_turn_left_sustained', 'left_rate_1p3', 1.3), ...
    scenario_spec_local('single_turn_right_sustained', 'right_rate_1p3', 1.3)];
end


function spec = scenario_spec_local(scenario, label, rate)
spec = struct('scenario', scenario, 'label', label, 'turn_rate', rate);
end


function candidates = lhs_candidates_local(count)
ranges = parameter_ranges_local();
fields = tunable_fields_local();
rng(70001, 'twister');
unit = zeros(count, numel(fields));
for column = 1:numel(fields)
    unit(:, column) = (randperm(count)' - rand(count, 1)) / count;
end
candidates = repmat(params_to_overrides_local(simulation_params_oracle()), 1, count);
for row = 1:count
    candidate = struct();
    for column = 1:numel(fields)
        bounds = ranges.(fields{column});
        if strcmp(fields{column}, 'ukf_process_accel_psd_m2_s3')
            value = exp(log(bounds(1)) + unit(row, column) * ...
                (log(bounds(2)) - log(bounds(1))));
        else
            value = bounds(1) + unit(row, column) * (bounds(2) - bounds(1));
        end
        candidate.(fields{column}) = value;
    end
    candidates(row) = candidate;
end
end


function item = candidate_result_local(overrides, report, baseline)
item = empty_candidate_result_local();
item.overrides = overrides;
item.report = report;
position_ratio = mean([report.cases.position_rmse_km] ./ ...
    [baseline.cases.position_rmse_km]);
speed_ratio = mean([report.cases.speed_rmse_ms] ./ ...
    [baseline.cases.speed_rmse_ms]);
item.objective = 0.8 * position_ratio + 0.2 * speed_ratio;
item.violation = consistency_violation_local(report);
end


function violation = consistency_violation_local(report)
violations = [interval_violation_local(report.nis_mean / 3, 0.8, 1.2), ...
    interval_violation_local(report.nees_mean / 4, 0.8, 1.2), ...
    interval_violation_local(report.nis_coverage95, 0.90, 0.99), ...
    interval_violation_local(report.nees_coverage95, 0.90, 0.99)];
names = unique({report.cases.scenario});
for name_index = 1:numel(names)
    for radar_id = 1:2
        mask = strcmp({report.cases.scenario}, names{name_index}) & ...
            [report.cases.radar_id] == radar_id;
        if any(mask)
            violations(end+1) = interval_violation_local( ...
                mean([report.cases(mask).nis_mean]) / 3, 0.6, 1.4); %#ok<AGROW>
            violations(end+1) = interval_violation_local( ...
                mean([report.cases(mask).nees_mean]) / 4, 0.6, 1.4); %#ok<AGROW>
        end
    end
end
sustained = report.cases(contains({report.cases.scenario}, 'sustained'));
straight = report.cases(strcmp({report.cases.scenario}, 'single_straight'));
turn_accuracy = mean([sustained.turn_direction_accuracy], 'omitnan');
turn_delay = median([sustained.turn_detection_delay_frames], 'omitnan');
false_ct = mean([straight.straight_false_ct_rate], 'omitnan');
violations(end+1) = max(0, 0.90 - turn_accuracy);
violations(end+1) = max(0, false_ct - 0.10);
violations(end+1) = max(0, turn_delay - 2) / 2;
violation = max(violations) + 0.01 * sum(violations);
end


function value = interval_violation_local(value, lower, upper)
if ~isfinite(value), value = inf; return; end
value = max([0, lower - value, value - upper]);
end


function best = choose_best_candidate_local(items)
best = items(1);
for i = 2:numel(items)
    if candidate_less_local(items(i), best), best = items(i); end
end
end


function yes = candidate_less_local(a, b)
yes = a.violation < b.violation - 1e-12 || ...
    (abs(a.violation - b.violation) <= 1e-12 && a.objective < b.objective);
end


function fields = tunable_fields_local()
fields = {'ukf_process_accel_psd_m2_s3', 'imm_cv_dwell_time_sec', ...
    'imm_ct_dwell_time_sec', 'imm_ct_fixed_Q_scale', ...
    'imm_transient_gain_max', 'radar1_ukf_init_pos_std_m', ...
    'radar2_ukf_init_pos_std_m', 'radar1_ukf_init_vel_std_ms', ...
    'radar2_ukf_init_vel_std_ms'};
end


function ranges = parameter_ranges_local()
ranges = struct('ukf_process_accel_psd_m2_s3', [0.002, 0.2], ...
    'imm_cv_dwell_time_sec', [900, 3600], ...
    'imm_ct_dwell_time_sec', [120, 600], ...
    'imm_ct_fixed_Q_scale', [1, 6], ...
    'imm_transient_gain_max', [1, 12], ...
    'imm_transient_nis_start', [1.5, 5.0], ...
    'imm_transient_nis_full', [6.0, 15.0], ...
    'imm_transient_ewma_alpha', [0.30, 0.85], ...
    'radar1_ukf_init_pos_std_m', [6000, 20000], ...
    'radar2_ukf_init_pos_std_m', [12000, 32000], ...
    'radar1_ukf_init_vel_std_ms', [50, 180], ...
    'radar2_ukf_init_vel_std_ms', [80, 240]);
end


function value = bounded_parameter_local(name, value)
bounds = parameter_ranges_local();
value = min(bounds.(name)(2), max(bounds.(name)(1), value));
end


function overrides = params_to_overrides_local(params)
fields = tunable_fields_local();
overrides = struct();
for i = 1:numel(fields), overrides.(fields{i}) = params.(fields{i}); end
end


function item = empty_candidate_result_local()
item = struct('overrides', struct(), 'report', struct(), ...
    'violation', inf, 'objective', inf);
end


function table_value = candidate_table_local(items)
fields = tunable_fields_local();
if isempty(items)
    table_value = table();
    return;
end
rows = repmat(candidate_row_local(items(1), fields), 1, numel(items));
for i = 1:numel(items)
    rows(i) = candidate_row_local(items(i), fields);
end
table_value = struct2table(rows);
end


function row = candidate_row_local(item, fields)
row = struct('violation', item.violation, 'objective', item.objective, ...
    'position_rmse_km', item.report.position_rmse_km, ...
    'speed_rmse_ms', item.report.speed_rmse_ms, ...
    'nis_mean', item.report.nis_mean, 'nees_mean', item.report.nees_mean);
for j = 1:numel(fields), row.(fields{j}) = item.overrides.(fields{j}); end
end


function overrides = resolve_calibration_overrides_local(options, root)
if ~isempty(fieldnames(options.candidate_overrides))
    overrides = options.candidate_overrides;
    return;
end
path = fullfile(root, 'latest_calibration.mat');
if ~exist(path, 'file')
    warning('run_ukf_monte_carlo:noCalibration', ...
        'No calibration result found; validating current defaults.');
    overrides = params_to_overrides_local(simulation_params_oracle());
    return;
end
loaded = load(path, 'result');
overrides = loaded.result.best_overrides;
end


function record = validation_record_local(label, item, configuration)
record = struct('random_seed', item.random_seed, 'scenario', label, ...
    'radar_id', item.radar_id, 'configuration', configuration, ...
    'position_rmse_km', item.position_rmse_km, ...
    'speed_rmse_ms', item.speed_rmse_ms, 'nis_mean', item.nis_mean, ...
    'nees_mean', item.nees_mean, 'nis_coverage95', item.nis_coverage95, ...
    'nees_coverage95', item.nees_coverage95, ...
    'turn_direction_accuracy', item.turn_direction_accuracy, ...
    'straight_false_ct_rate', item.straight_false_ct_rate, ...
    'turn_detection_delay_frames', item.turn_detection_delay_frames, ...
    'track_count', item.track_count, 'sample_count', item.sample_count, ...
    'nis_count', item.nis_count, 'nees_count', item.nees_count);
end


function records = empty_validation_records_local()
prototype = validation_record_local('', struct('random_seed', 0, 'radar_id', 0, ...
    'position_rmse_km', NaN, 'speed_rmse_ms', NaN, 'nis_mean', NaN, ...
    'nees_mean', NaN, 'nis_coverage95', NaN, 'nees_coverage95', NaN, ...
    'turn_direction_accuracy', NaN, 'straight_false_ct_rate', NaN, ...
    'turn_detection_delay_frames', NaN, 'track_count', 0, 'sample_count', 0, ...
    'nis_count', 0, 'nees_count', 0), '');
records = prototype([]);
end


function summary = validation_summary_local(records, options)
candidate = records(strcmp({records.configuration}, 'candidate'));
baseline = records(strcmp({records.configuration}, 'baseline'));
groups = summarize_groups_local(records);
candidate_pos = mean([candidate.position_rmse_km]);
baseline_pos = mean([baseline.position_rmse_km]);
candidate_speed = mean([candidate.speed_rmse_ms]);
baseline_speed = mean([baseline.speed_rmse_ms]);
[ci, seed_differences] = bootstrap_difference_local( ...
    baseline, candidate, options.bootstrap_repetitions);
group_regression = max_group_regression_local(groups);
overall_nis_ratio = weighted_mean_local(candidate, 'nis_mean', 'nis_count') / 3;
overall_nees_ratio = weighted_mean_local(candidate, 'nees_mean', 'nees_count') / 4;
nis_coverage = weighted_mean_local(candidate, 'nis_coverage95', 'nis_count');
nees_coverage = weighted_mean_local(candidate, 'nees_coverage95', 'nees_count');
coverage_ok = nis_coverage >= 0.90 && nis_coverage <= 0.99 && ...
    nees_coverage >= 0.90 && nees_coverage <= 0.99;
candidate_groups = groups(strcmp({groups.configuration}, 'candidate'));
group_consistency_ok = all([candidate_groups.nis_ratio] >= 0.6 & ...
    [candidate_groups.nis_ratio] <= 1.4 & ...
    [candidate_groups.nees_ratio] >= 0.6 & ...
    [candidate_groups.nees_ratio] <= 1.4);
sustained = candidate(contains({candidate.scenario}, 'sustained') | ...
    contains({candidate.scenario}, 'rate_'));
straight = candidate(strcmp({candidate.scenario}, 'single_straight'));
turn_accuracy = mean([sustained.turn_direction_accuracy], 'omitnan');
turn_delay = median([sustained.turn_detection_delay_frames], 'omitnan');
straight_false_ct = mean([straight.straight_false_ct_rate], 'omitnan');
model_ok = turn_accuracy >= 0.90 && turn_delay <= 2 && straight_false_ct <= 0.10;
summary = struct('baseline_position_rmse_km', baseline_pos, ...
    'candidate_position_rmse_km', candidate_pos, ...
    'position_improvement_fraction', (baseline_pos - candidate_pos) / baseline_pos, ...
    'baseline_speed_rmse_ms', baseline_speed, ...
    'candidate_speed_rmse_ms', candidate_speed, ...
    'bootstrap_ci_km', ci, 'seed_differences_km', seed_differences, ...
    'overall_nis_ratio', overall_nis_ratio, ...
    'overall_nees_ratio', overall_nees_ratio, ...
    'nis_coverage95', nis_coverage, 'nees_coverage95', nees_coverage, ...
    'turn_direction_accuracy', turn_accuracy, ...
    'turn_detection_delay_frames', turn_delay, ...
    'straight_false_ct_rate', straight_false_ct, ...
    'group_consistency_ok', group_consistency_ok, 'model_ok', model_ok, ...
    'groups', groups, ...
    'accepted', candidate_pos <= 0.98 * baseline_pos && ci(2) < 0 && ...
        candidate_speed <= 1.05 * baseline_speed && group_regression <= 0.05 && ...
        overall_nis_ratio >= 0.8 && overall_nis_ratio <= 1.2 && ...
        overall_nees_ratio >= 0.8 && overall_nees_ratio <= 1.2 && ...
        coverage_ok && group_consistency_ok && model_ok);
end


function groups = summarize_groups_local(records)
scenarios = unique({records.scenario});
groups = struct('scenario', {}, 'radar_id', {}, 'configuration', {}, ...
    'position_rmse_km', {}, 'speed_rmse_ms', {}, 'nis_ratio', {}, ...
    'nees_ratio', {}, 'nis_coverage95', {}, 'nees_coverage95', {});
for scenario_index = 1:numel(scenarios)
    for radar_id = 1:2
        for configuration = {'baseline', 'candidate'}
            mask = strcmp({records.scenario}, scenarios{scenario_index}) & ...
                [records.radar_id] == radar_id & ...
                strcmp({records.configuration}, configuration{1});
            selected = records(mask);
            groups(end+1) = struct('scenario', scenarios{scenario_index}, ...
                'radar_id', radar_id, 'configuration', configuration{1}, ...
                'position_rmse_km', mean([selected.position_rmse_km]), ...
                'speed_rmse_ms', mean([selected.speed_rmse_ms]), ...
                'nis_ratio', mean([selected.nis_mean]) / 3, ...
                'nees_ratio', mean([selected.nees_mean]) / 4, ...
                'nis_coverage95', mean([selected.nis_coverage95]), ...
                'nees_coverage95', mean([selected.nees_coverage95])); %#ok<AGROW>
        end
    end
end
end


function regression = max_group_regression_local(groups)
regression = -inf;
scenarios = unique({groups.scenario});
for scenario_index = 1:numel(scenarios)
    for radar_id = 1:2
        mask = strcmp({groups.scenario}, scenarios{scenario_index}) & ...
            [groups.radar_id] == radar_id;
        selected = groups(mask);
        baseline = selected(strcmp({selected.configuration}, 'baseline')).position_rmse_km;
        candidate = selected(strcmp({selected.configuration}, 'candidate')).position_rmse_km;
        regression = max(regression, (candidate - baseline) / baseline);
    end
end
end


function value = weighted_mean_local(records, value_field, weight_field)
values = [records.(value_field)];
weights = [records.(weight_field)];
mask = isfinite(values) & isfinite(weights) & weights > 0;
value = sum(values(mask) .* weights(mask)) / sum(weights(mask));
end


function [ci, differences] = bootstrap_difference_local(baseline, candidate, repetitions)
seeds = unique([baseline.random_seed]);
differences = zeros(size(seeds));
for i = 1:numel(seeds)
    differences(i) = mean([candidate([candidate.random_seed] == seeds(i)).position_rmse_km]) - ...
        mean([baseline([baseline.random_seed] == seeds(i)).position_rmse_km]);
end
rng(90001, 'twister');
bootstrap = zeros(repetitions, 1);
for i = 1:repetitions
    indices = randi(numel(seeds), 1, numel(seeds));
    bootstrap(i) = mean(differences(indices));
end
bootstrap = sort(bootstrap);
ci = [bootstrap(max(1, round(0.025 * repetitions))), ...
    bootstrap(min(repetitions, round(0.975 * repetitions)))];
end


function write_validation_figures_local(records, summary, output_dir)
figure_handle = figure('Visible', 'off', 'Color', 'w');
candidate = records(strcmp({records.configuration}, 'candidate'));
baseline = records(strcmp({records.configuration}, 'baseline'));
boxchart([ones(1, numel(baseline)), 2 * ones(1, numel(candidate))], ...
    [[baseline.position_rmse_km], [candidate.position_rmse_km]]);
set(gca, 'XTick', [1, 2], 'XTickLabel', {'Baseline', 'Candidate'});
ylabel('Position RMSE (km)'); grid on;
exportgraphics(figure_handle, fullfile(output_dir, 'position_rmse_boxplot.png'));
close(figure_handle);

figure_handle = figure('Visible', 'off', 'Color', 'w');
bar([summary.overall_nis_ratio, summary.overall_nees_ratio]);
hold on; yline(1, 'k--'); ylim([0, max(1.5, 1.1 * max(ylim))]);
set(gca, 'XTickLabel', {'ANIS/3', 'ANEES/4'}); grid on;
exportgraphics(figure_handle, fullfile(output_dir, 'consistency_ratios.png'));
close(figure_handle);
end


function record = chain_record_local(seed, configuration, result)
purity = NaN; group_count = 0; mixed = NaN; unmatched = NaN;
rmse = NaN; bridge_rmse = NaN; reconstructed_rmse = NaN;
if isfield(result, 'grouping') && isfield(result.grouping, 'groups')
    group_count = numel(result.grouping.groups);
end
if isfield(result, 'evaluation') && ~isempty(result.evaluation)
    mixed = result.evaluation.mixed_group_count;
    unmatched = numel(result.evaluation.unmatched_truth);
    if ~isempty(result.evaluation.groups), purity = mean([result.evaluation.groups.purity]); end
    if ~isempty(result.evaluation.overall)
        methods = {result.evaluation.overall.method};
        index = find(strcmp(methods, 'FCI'), 1);
        if isempty(index), index = 1; end
        rmse = result.evaluation.overall(index).rmse_km;
        bridge_rmse = result.evaluation.overall(index).bridge_rmse_km;
        reconstructed_rmse = result.evaluation.overall(index).reconstructed_rmse_km;
    end
end
record = struct('random_seed', seed, 'configuration', configuration, ...
    'status', result.status, 'success', strcmp(result.status, 'SUCCESS'), ...
    'target_count', result.scenario.n_targets, 'group_count', group_count, ...
    'group_count_correct', group_count == result.scenario.n_targets, ...
    'mixed_group_count', mixed, 'unmatched_truth_count', unmatched, ...
    'purity', purity, 'fci_rmse_km', rmse, 'bridge_rmse_km', bridge_rmse, ...
    'reconstructed_rmse_km', reconstructed_rmse);
end


function records = empty_chain_records_local()
prototype = struct('random_seed', 0, 'configuration', '', 'status', '', ...
    'success', false, 'target_count', 0, 'group_count', 0, ...
    'group_count_correct', false, 'mixed_group_count', NaN, ...
    'unmatched_truth_count', NaN, 'purity', NaN, 'fci_rmse_km', NaN, ...
    'bridge_rmse_km', NaN, 'reconstructed_rmse_km', NaN);
records = prototype([]);
end


function summary = chain_summary_local(records)
candidate = records(strcmp({records.configuration}, 'candidate'));
baseline = records(strcmp({records.configuration}, 'baseline'));
summary = struct('candidate_success_rate', mean([candidate.success]), ...
    'baseline_success_rate', mean([baseline.success]), ...
    'candidate_wrong_merges', sum([candidate.mixed_group_count], 'omitnan'), ...
    'candidate_group_success_rate', mean([candidate.group_count_correct]), ...
    'candidate_purity', mean([candidate.purity], 'omitnan'), ...
    'baseline_fci_rmse_km', mean([baseline.fci_rmse_km], 'omitnan'), ...
    'candidate_fci_rmse_km', mean([candidate.fci_rmse_km], 'omitnan'));
summary.accepted = summary.candidate_success_rate == 1 && ...
    summary.candidate_wrong_merges == 0 && ...
    summary.candidate_group_success_rate >= 0.98 && ...
    summary.candidate_purity >= 0.99 && ...
    summary.candidate_fci_rmse_km <= 1.05 * summary.baseline_fci_rmse_km;
end


function options = monte_carlo_defaults_local(mode, options)
defaults = struct('calibration_seeds', 1001:1030, ...
    'screen_seed_count', 10, 'candidate_count', 64, ...
    'top_candidate_count', 8, 'local_refinement_passes', 2, ...
    'staged_ukf_candidate_count', 48, ...
    'staged_imm_candidate_count', 64, ...
    'staged_top_candidate_count', 6, ...
    'staged_local_refinement_passes', 2, ...
    'staged_local_step_fraction', 0.25, ...
    'validation_seeds', 10001:10200, 'full_chain_seeds', 10001:10050, ...
    'bootstrap_repetitions', 10000, 'candidate_overrides', struct(), ...
    'cache_version', 'v1', 'checkpoint_version', 'v3', ...
    'checkpoint_label', '', ...
    'publish_calibration', true, ...
    'use_parallel', false, ...
    'parallel_workers', 4);
if strcmp(mode, 'smoke')
    defaults.calibration_seeds = 1001:1002;
    defaults.screen_seed_count = 1;
    defaults.candidate_count = 4;
    defaults.top_candidate_count = 2;
    defaults.local_refinement_passes = 0;
    defaults.staged_ukf_candidate_count = 4;
    defaults.staged_imm_candidate_count = 4;
    defaults.staged_top_candidate_count = 2;
    defaults.staged_local_refinement_passes = 0;
    defaults.checkpoint_label = 'smoke';
    defaults.publish_calibration = false;
end
names = fieldnames(defaults);
for i = 1:numel(names)
    if ~isfield(options, names{i}), options.(names{i}) = defaults.(names{i}); end
end
end


function path = checkpoint_dir_local(root, stage, options)
suffix = ['_' options.checkpoint_version];
if ~isempty(options.checkpoint_label)
    suffix = [suffix '_' options.checkpoint_label];
end
path = fullfile(root, [stage '_work' suffix]);
if ~exist(path, 'dir'), mkdir(path); end
end


function ensure_parallel_pool_local(options)
if ~options.use_parallel, return; end
pool = gcp('nocreate');
if isempty(pool)
    parpool('local', options.parallel_workers);
end
end
