function analyze_ct_dwell_v3()
    close all;
    addpath(genpath('.'));
    rng(42);

    base_params = simulation_params_oracle();
    base_params.imm_cv_dwell_time_sec = 2500;
    base_params.imm_ct_dwell_time_sec = 660;
    base_params.imm_ct_fixed_Q_scale = 5.3;
    base_params.imm_transient_gain_max = 11.0;

    s1_name = 'single_straight'; s1_label = 'straight(0s)'; s1_turn = 0;
    s2_name = 'single_turn_right_short'; s2_label = 'right_short(47s)'; s2_turn = 47;
    s3_name = 'multi_cross'; s3_label = 'multi_cross(0s)'; s3_turn = 0;

    fprintf('========== Observed Dwell Analysis V3 ==========%s', newline);

    % 场景1: straight
    fprintf('%s--- %s ---%s', newline, s1_label, newline);
    try
        r1 = run(s1_name);
        dump_ct_stats(r1, r1.errorStats_R1.overall.ukf.rms, s1_turn);
    catch ME
        fprintf('ERROR: %s\n', ME.message);
    end

    % 场景2: right_short
    fprintf('%s--- %s ---%s', newline, s2_label, newline);
    try
        r2 = run(s2_name);
        dump_ct_stats(r2, r2.errorStats_R1.overall.ukf.rms, s2_turn);
    catch ME
        fprintf('ERROR: %s\n', ME.message);
    end

    % 场景3: multi_cross
    fprintf('%s--- %s ---%s', newline, s3_label, newline);
    try
        r3 = run(s3_name);
        dump_ct_stats(r3, r3.errorStats_R1.overall.ukf.rms, s3_turn);
    catch ME
        fprintf('ERROR: %s\n', ME.message);
    end
end

function dump_ct_stats(result, rmse_r1, expected_turn)
    % 从 trackList 中提取 mu_history CT 占比
    ct_pcts = [];

    for ti = 1:min(length(result.trackList_R1), 3)
        trk = result.trackList_R1{ti};
        if isfield(trk, 'ukf') && isfield(trk.ukf, 'mu_history')
            mh = trk.ukf.mu_history;
            if ~isempty(mh)
                cts = sum(mh(:,2:3), 2) * 100;
                valid = ~isnan(cts(:));
                ct_pcts = [ct_pcts; cts(valid)];
            end
        end
    end

    for ti = 1:min(length(result.trackList_R2), 3)
        trk = result.trackList_R2{ti};
        if isfield(trk, 'ukf') && isfield(trk.ukf, 'mu_history')
            mh = trk.ukf.mu_history;
            if ~isempty(mh)
                cts = sum(mh(:,2:3), 2) * 100;
                valid = ~isnan(cts(:));
                ct_pcts = [ct_pcts; cts(valid)];
            end
        end
    end

    avg_ct = mean(ct_pcts)*100;
    max_ct = max(ct_pcts)*100;
    active = sum(ct_pcts > 50);
    n_valid = length(ct_pcts);

    fprintf('RMSE=%.2f avg_CT=%.1f%% max_CT=%.1f%% active_frames=%d/%d\n', ...
        rmse_r1, avg_ct, max_ct, active, n_valid);

    % 连续段
    binary_vec = ct_pcts > 50;
    runs = [];
    cnt = 0; in_run = false;
    for i = 1:length(binary_vec)
        if binary_vec(i)
            if ~in_run, in_run=true; cnt=0; end
            cnt=cnt+1;
        else
            if in_run, runs(end+1)=cnt; in_run=false; end
        end
    end
    if in_run, runs(end+1)=cnt; end

    if length(runs)>0
        med_run_s = median(runs)*30;
        max_run_s = max(runs)*30;
        avg_run_s = mean(runs)*30;
        if expected_turn > 0
            if med_run_s >= expected_turn*0.8
                flag = '[CT够用]';
            elseif med_run_s < expected_turn*0.5
                flag = '[CT不足!]';
            else
                flag = '[边缘]';
            end
        else
            flag = '(无转弯预期)';
        end
        fprintf('runs: n=%d avg=%.0fs med=%.0fs max=%.0fs %s\n', ...
            length(runs), avg_run_s, med_run_s, max_run_s, flag);
    end
end
