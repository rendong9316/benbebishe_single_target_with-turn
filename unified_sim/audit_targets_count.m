function audit_targets_count()
% audit_targets_count — 验证不同目标数下统一入口的稳定性（Pd=0.6 真实雷达参数）

    fprintf('\n=== 不同目标数 (直线交叉, Pd=0.6, seed=94) ===\n');
    fprintf('n_targets,frames,R1_km,R2_km,best_fusion_km,best_method,pairs,R1_trk,R2_trk\n');
    for n = [1, 2, 3, 4, 5]
        params = simulation_params();
        if n == 1
            params.scenario = 'straight';
        else
            params.scenario = 'multi';
            params.n_targets = n;
        end
        params.detection_probability = 0.6;
        params.random_seed = 94;
        result = run_unified_once(params);
        fprintf('%d,%d,%.3f,%.3f,%.3f,%s,%d,%d,%d\n', ...
            n, result.n_frames, result.rmse_R1, result.rmse_R2, ...
            result.best_fusion_rmse, result.method_names{result.best_m}, ...
            length(result.matched_pairs), result.n_tracks_R1, result.n_tracks_R2);
    end
end
