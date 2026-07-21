function run_monte_carlo_tests()
%RUN_MONTE_CARLO_TESTS Deterministic tests for MC scenarios and diagnostics.
params = simulation_params_oracle();
left = build_truth_scenario('single_turn_left_sustained', params);
right = build_truth_scenario('single_turn_right_sustained', params);
assert(left.n_frames == right.n_frames);
left_truth = left.truth_all{1};
right_truth = right.truth_all{1};
assert(abs(left_truth(1, 1) - right_truth(end, 1)) < 0.1);
assert(abs(left_truth(end, 1) - right_truth(1, 1)) < 0.1);
left_sign = dominant_turn_sign_local(left_truth);
right_sign = dominant_turn_sign_local(right_truth);
assert(left_sign == -1 && right_sign == 1);

inputs = prepare_oracle_tracking_inputs( ...
    'single_straight', struct('random_seed', 91234));
report = evaluate_ukf_configuration(struct(), {inputs}, false);
assert(all([report.cases.nis_count] > 0));
assert(all(isfinite([report.cases.nis_mean])));
assert(all([report.cases.nis_coverage95] >= 0 & ...
    [report.cases.nis_coverage95] <= 1));
assert(all([report.cases.nees_coverage95] >= 0 & ...
    [report.cases.nees_coverage95] <= 1));
disp('monte carlo tests ok');
end


function sign_value = dominant_turn_sign_local(truth)
heading = atan2d(truth(:, 3) .* cosd(truth(:, 2)), truth(:, 4));
delta = mod(diff(heading) + 180, 360) - 180;
delta = delta(abs(delta) > 1);
sign_value = sign(median(delta));
end
