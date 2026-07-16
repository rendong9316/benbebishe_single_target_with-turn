function audit_extended()
% audit_extended — 在 Pd=0.6 真实雷达参数下做 seed 扫描审查

    fprintf('\n=== A. 单目标 × 轨迹类型 × seed (Pd=0.6) ===\n');
    fprintf('scenario,seed,frames,R1_km,R2_km,best_fusion_km,best_method,pairs,R1_trk,R2_trk\n');
    for sc = {'straight', 'gradual_turn', 'uturn'}
        for seed = [94, 7, 42, 137, 211]
            params = simulation_params();
            params.scenario = sc{1};
            params.detection_probability = 0.6;
            params.random_seed = seed;
            result = run_unified_once(params);
            fprintf('%s,%d,%d,%.3f,%.3f,%.3f,%s,%d,%d,%d\n', ...
                result.params.scenario, seed, result.n_frames, ...
                result.rmse_R1, result.rmse_R2, ...
                result.best_fusion_rmse, result.method_names{result.best_m}, ...
                length(result.matched_pairs), result.n_tracks_R1, result.n_tracks_R2);
        end
    end

    fprintf('\n=== B. 多目标 n=3 × seed (Pd=0.6) ===\n');
    fprintf('scenario,seed,frames,R1_km,R2_km,best_fusion_km,best_method,pairs,R1_trk,R2_trk\n');
    for seed = [94, 7, 42, 137, 211]
        params = simulation_params();
        params.scenario = 'multi';
        params.detection_probability = 0.6;
        params.random_seed = seed;
        result = run_unified_once(params);
        fprintf('multi,%d,%d,%.3f,%.3f,%.3f,%s,%d,%d,%d\n', ...
            seed, result.n_frames, ...
            result.rmse_R1, result.rmse_R2, ...
            result.best_fusion_rmse, result.method_names{result.best_m}, ...
            length(result.matched_pairs), result.n_tracks_R1, result.n_tracks_R2);
    end
end
