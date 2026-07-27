function mc_fixed_sanity(scene, s1, s2, out_suffix)
% 修复后 sanity check: 跑 Default / Config51 / Sig_k3_p1 / Sig_k7_p05 对比
    close all;
    addpath(genpath('.'));
    if nargin < 3, out_suffix = '_fixed'; end
    seeds = s1:s2;
    configs = {'Default', 'Config51', 'Sig_k3_p1', 'Sig_k7_p05'};
    rmse_matrix = NaN(numel(seeds), 4);
    fprintf('========== FIXED-SIGMOID SANITY: %s seeds %d-%d ==========%s', scene, s1, s2, newline);
    for si = 1:numel(seeds)
        seed = seeds(si);
        for ci = 1:4
            params = simulation_params_oracle();
            params.random_seed = seed;
            switch configs{ci}
                case 'Config51'
                    params.imm_cv_dwell_time_sec = 2500;
                    params.imm_ct_dwell_time_sec = 660;
                    params.imm_ct_fixed_Q_scale = 5.3;
                    params.imm_transient_gain_max = 11.0;
                case 'Sig_k3_p1'
                    params.imm_cv_dwell_time_sec = 2500;
                    params.imm_ct_dwell_time_sec = 660;
                    params.imm_ct_fixed_Q_scale = 5.3;
                    params.imm_transient_gain_max = 11.0;
                    params.k_turnrate = 3;
                    params.imm_adaptive_mode = 'sigmoid';
                    params.threshold = 0.3;
                    params.power = 1.0;
                case 'Sig_k7_p05'
                    params.imm_cv_dwell_time_sec = 2500;
                    params.imm_ct_dwell_time_sec = 660;
                    params.imm_ct_fixed_Q_scale = 5.3;
                    params.imm_transient_gain_max = 11.0;
                    params.k_turnrate = 7;
                    params.imm_adaptive_mode = 'sigmoid';
                    params.threshold = 0.3;
                    params.power = 0.5;
            end
            try
                r = run(scene, params);
                v = r.errorStats_R1.overall.ukf.rms;
                rmse_matrix(si, ci) = v;
            catch ME
                fprintf('  seed=%d %s ERROR %s%s', seed, configs{ci}, ME.message(1:40), newline);
            end
        end
        fprintf('  seed=%d  Def=%.2f  Cfg51=%.2f  Sigk3=%.2f  Sigk7=%.2f%s', ...
            seed, rmse_matrix(si,1), rmse_matrix(si,2), rmse_matrix(si,3), rmse_matrix(si,4), newline);
    end
    means = mean(rmse_matrix, 1, 'omitnan');
    fprintf('  MEAN:  Def=%.2f  Cfg51=%.2f  Sigk3=%.2f  Sigk7=%.2f%s', means(1), means(2), means(3), means(4), newline);
    imp3 = (means(1)-means(3))/means(1)*100;
    imp7 = (means(1)-means(4))/means(1)*100;
    fprintf('  Sigk3 vs Def: %+.1f%%   Sigk7 vs Def: %+.1f%%%s', imp3, imp7, newline);
    save_file = sprintf('results/mc_%s_%dto%d%s.mat', scene, s1, s2, out_suffix);
    save(save_file, 'rmse_matrix', 'seeds', 'configs', 'scene');
    fprintf('  Saved: %s%s', save_file, newline);
end
