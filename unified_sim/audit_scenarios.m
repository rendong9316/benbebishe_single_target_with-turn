function audit_scenarios()
% audit_scenarios — 统一入口四类场景单次审查

    scenarios = {'straight', 'gradual_turn', 'uturn', 'multi'};
    fprintf('scenario,n_targets,frames,R1_km,R2_km,best_fusion_km,best_method,pairs,R1_tracks,R2_tracks\n');
    for i = 1:length(scenarios)
        params = simulation_params();
        params.scenario = scenarios{i};
        params.random_seed = 94;
        result = run_unified_once(params);
        fprintf('%s,%d,%d,%.3f,%.3f,%.3f,%s,%d,%d,%d\n', ...
            result.params.scenario, result.params.n_targets, result.n_frames, ...
            result.rmse_R1, result.rmse_R2, result.best_fusion_rmse, ...
            result.method_names{result.best_m}, length(result.matched_pairs), ...
            result.n_tracks_R1, result.n_tracks_R2);
    end
end
