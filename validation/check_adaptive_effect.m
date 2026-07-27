function check_adaptive_effect()
% 验证adaptive_p_ctcv是否真的被计算了
    close all;
    addpath(genpath('.'));
    rng(42);

    base_params = simulation_params_oracle();
    base_params.imm_cv_dwell_time_sec = 2500;
    base_params.imm_ct_dwell_time_sec = 660;
    base_params.imm_ct_fixed_Q_scale = 5.3;
    base_params.imm_transient_gain_max = 11.0;

    % 先测uturn（转弯180s，mu(CT)应该比较高）
    fprintf('=== U-Turn with k=0 ===%s', newline);
    params1 = base_params;
    r1 = run('single_uturn');
    fprintf('k=0: RMSE_R1=%.2f%s', r1.errorStats_R1.overall.ukf.rms, newline);

    fprintf('%s=== U-Turn with k=2.0 ===%s', newline, newline);
    params2 = base_params;
    params2.k_turnrate = 2.0;
    r2 = run('single_uturn');
    fprintf('k=2.0: RMSE_R1=%.2f%s', r2.errorStats_R1.overall.ukf.rms, newline);

    % 检查R1 trackList的Pi值
    fprintf('%s=== Checking Pi values in R1 trackList ===%s', newline, newline);
    for si = 1:length(r1.trackList_R1)
        trk = r1.trackList_R1{si};
        if isfield(trk, 'ukf') && isfield(trk.ukf, 'Pi')
            pi_val = trk.ukf.Pi(2,1);  % CT->CV probability
            printf 'Track %d (k=0): Pi_ct_cv=%.4f%s', si, pi_val, newline;
        end
    end
    for si = 1:length(r2.trackList_R1)
        trk = r2.trackList_R1{si};
        if isfield(trk, 'ukf') && isfield(trk.ukf, 'Pi')
            pi_val = trk.ukf.Pi(2,1);
            printf 'Track %d (k=2): Pi_ct_cv=%.4f%s', si, pi_val, newline;
        end
    end
end
