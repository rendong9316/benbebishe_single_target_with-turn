function params = simulation_params_oracle()
    params = simulation_params_multi();

    params.detection_probability = 0.6;
    params.false_alarm_rate = 0.001;

    params.RELIABLE_TRACK = 1;
    params.MAINTAIN_TRACK = 2;
    params.TEMPORARY_TRACK = 6;
    params.HISTORY_TRACK = 7;

    params.oracle_QUALIFY_NUM = 3;
    params.oracle_TOLERANT_NUM = 7;
    params.oracle_init_quality = 5;
    params.oracle_confirm_quality = 8;
    params.oracle_maintain_quality = 4;
    params.oracle_max_quality = 15;
    params.oracle_loss_quality_penalty = 1;
    params.oracle_truth_terminate_enable = true;

    params.tracker_K_loss = 8;
    params.radar1_tracker_K_loss = 8;
    params.radar2_tracker_K_loss = 8;

    params.multi_single_assoc_mode = 'oracle';
    params.multi_truth_init_enable = false;
    params.multi_truth_reinit_enable = false;
    params.multi_truth_terminate_enable = true;

    params.track_matcher_method = 'dualgate';
    params.dualgate_T1_km = 35;
    params.dualgate_M = 8;
    params.dualgate_var_km2 = 50;
    params.dualgate_coexist_thresh = 5;
    params.dualgate_mutual_exclusion = true;
end
