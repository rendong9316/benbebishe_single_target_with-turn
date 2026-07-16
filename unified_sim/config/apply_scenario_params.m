function params = apply_scenario_params(params)
% apply_scenario_params — 按场景统一派生轨迹、目标数和算法参数

    if ~isfield(params, 'scenario') || isempty(params.scenario)
        if isfield(params, 'n_targets') && params.n_targets > 1
            params.scenario = 'multi';
        elseif isfield(params, 'trajectory_mode') && ~isempty(params.trajectory_mode)
            params.scenario = params.trajectory_mode;
        else
            params.scenario = 'straight';
        end
    end

    scenario = lower(params.scenario);
    if strcmp(scenario, 'turn')
        scenario = 'gradual_turn';
    elseif strcmp(scenario, 'multi_cross')
        scenario = 'multi';
    end

    params.scenario = scenario;

    switch scenario
        case 'straight'
            params.trajectory_mode = 'straight';
            params.n_targets = 1;
            params.ukf_backend = 'zishiying';
            params.detection_probability = 0.6;
            params.track_matcher_method = 'direct_single';
            params.multi_single_use_truth_labels = true;
            params.multi_single_lock_one_track = true;
            params.multi_single_assoc_mode = 'nn_pda';
            params.multi_truth_init_perfect_measurement = true;
            params.jpda_geo_gate_m_initial = 220000;
            params.jpda_geo_gate_m_stable = 220000;
            params.motion_gate_margin_m = 180000;
            params.motion_gate_max_m = 220000;
            params.multi_fallback_geo_gate_m = 220000;

        case 'gradual_turn'
            params.trajectory_mode = 'gradual_turn';
            params.n_targets = 1;
            params.ukf_backend = 'imm_3in1';
            params.detection_probability = 0.6;
            params.track_matcher_method = 'direct_single';
            params.multi_single_use_truth_labels = true;
            params.multi_single_lock_one_track = true;
            params.multi_single_assoc_mode = 'nn_pda';
            params.multi_truth_init_perfect_measurement = true;
            params.jpda_geo_gate_m_initial = 220000;
            params.jpda_geo_gate_m_stable = 220000;
            params.motion_gate_margin_m = 180000;
            params.motion_gate_max_m = 220000;
            params.multi_fallback_geo_gate_m = 220000;
            params.imm_turn_rate_rad_per_sec = 1.0 * pi / 180.0;

        case 'uturn'
            params.trajectory_mode = 'uturn';
            params.n_targets = 1;
            params.ukf_backend = 'imm_3in1';
            params.detection_probability = 0.6;
            params.track_matcher_method = 'direct_single';
            params.multi_single_use_truth_labels = true;
            params.multi_single_lock_one_track = true;
            params.multi_single_assoc_mode = 'nn_pda';
            params.multi_truth_init_perfect_measurement = true;
            params.jpda_geo_gate_m_initial = 220000;
            params.jpda_geo_gate_m_stable = 220000;
            params.motion_gate_margin_m = 180000;
            params.motion_gate_max_m = 220000;
            params.multi_fallback_geo_gate_m = 220000;
            params.imm_turn_rate_rad_per_sec = 1.0 * pi / 180.0;

        case 'multi'
            params.trajectory_mode = 'multi_cross';
            if ~isfield(params, 'n_targets') || params.n_targets <= 1
                params.n_targets = 3;
            end
            params.ukf_backend = 'imm_3in1';
            params.detection_probability = 0.6;
            params.track_matcher_method = 'dualgate';
            params.imm_Pi_CV_to_CT = 0.001;
            params.imm_Pi_CT_to_CV = 0.001;
            params.multi_fallback_use_vr_gate = false;
            params.multi_truth_reinit_enable = true;
            params.multi_truth_init_perfect_measurement = true;
            params.tracker_K_loss = 20;
            params.jpda_geo_gate_m_missed_step = 40000;
            params.motion_gate_margin_m = 60000;
            params.motion_gate_max_m = 150000;
            params.imm_turn_rate_rad_per_sec = 1.0 * pi / 180.0;

        otherwise
            error('apply_scenario_params: unknown scenario "%s"', params.scenario);
    end
end
