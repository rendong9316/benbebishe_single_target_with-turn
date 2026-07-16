function audit_extended()
% audit_extended — 扩展审查：轨迹类型 × 检测概率 × 种子

    fprintf('\n=== A. 单目标 × 轨迹类型 × Pd ===\n');
    fprintf('scenario,Pd,seed,frames,R1_km,R2_km,best_fusion_km,best_method,pairs,R1_trk,R2_trk\n');
    for sc = {'straight', 'gradual_turn', 'uturn'}
        for pd = [1.0, 0.8, 0.6]
            for seed = [94, 7, 42]
                params = simulation_params();
                params.scenario = sc{1};
                params.detection_probability = pd;
                params.random_seed = seed;
                result = run_unified_once(params);
                fprintf('%s,%.1f,%d,%d,%.3f,%.3f,%.3f,%s,%d,%d,%d\n', ...
                    result.params.scenario, pd, seed, result.n_frames, ...
                    result.rmse_R1, result.rmse_R2, ...
                    result.best_fusion_rmse, result.method_names{result.best_m}, ...
                    length(result.matched_pairs), result.n_tracks_R1, result.n_tracks_R2);
            end
        end
    end

    fprintf('\n=== B. 多目标 n=3 × Pd × 种子 ===\n');
    fprintf('scenario,Pd,seed,frames,R1_km,R2_km,best_fusion_km,best_method,pairs,R1_trk,R2_trk\n');
    for pd = [1.0, 0.8, 0.6]
        for seed = [94, 7, 42]
            params = simulation_params();
            params.scenario = 'multi';
            params.detection_probability = pd;
            params.random_seed = seed;
            result = run_unified_once(params);
            fprintf('multi,%.1f,%d,%d,%.3f,%.3f,%.3f,%s,%d,%d,%d\n', ...
                pd, seed, result.n_frames, ...
                result.rmse_R1, result.rmse_R2, ...
                result.best_fusion_rmse, result.method_names{result.best_m}, ...
                length(result.matched_pairs), result.n_tracks_R1, result.n_tracks_R2);
        end
    end
end
