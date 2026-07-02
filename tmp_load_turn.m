addpath(genpath('.'));
f = dir('results/simulation_turn180_*.mat');
loaded = load(fullfile('results', f(end).name));
fd = loaded.fus_data;

methods_list = {'jichu'; 'zishiying'; 'imm'};

fprintf('============================================================\n');
fprintf('  单目标回头弯 逐级误差对比 (UKF滤波 -> 融合)\n');
fprintf('============================================================\n\n');

for i = 1:numel(fd)
    s = fd{i};
    method_name = methods_list{i};
    fprintf('--- %s ---\n', method_name);

    ev = s.eval;
    fprintf('  单站UKF滤波:\n');
    for oi = 1:length(ev.overall)
        om = ev.overall(oi);
        if contains(om.method, 'R1')
            fprintf('    R1: RMSE=%.1f 中位=%.1f 均值=%.1f 95%%=%.1f\n', ...
                om.s.rms, om.s.median, om.s.mean, om.s.pct95);
        elseif contains(om.method, 'R2')
            fprintf('    R2: RMSE=%.1f 中位=%.1f 均值=%.1f 95%%=%.1f\n', ...
                om.s.rms, om.s.median, om.s.mean, om.s.pct95);
        end
    end

    fprintf('  融合算法:\n');
    for oi = 1:length(ev.overall)
        om = ev.overall(oi);
        if ~contains(om.method, 'R1') && ~contains(om.method, 'R2')
            fprintf('    %s: RMSE=%.1f 中位=%.1f 均值=%.1f 95%%=%.1f\n', ...
                om.method, om.s.rms, om.s.median, om.s.mean, om.s.pct95);
        end
    end

    % 计算融合vsR1改善
    r1_rmse = []; r2_rmse = [];
    for oi = 1:length(ev.overall)
        om = ev.overall(oi);
        if contains(om.method, 'R1'), r1_rmse = om.s.rms; end
        if contains(om.method, 'R2'), r2_rmse = om.s.rms; end
    end
    best_rmse = inf; best_name = '';
    for oi = 1:length(ev.overall)
        om = ev.overall(oi);
        if ~contains(om.method, 'R1') && ~contains(om.method, 'R2') && om.s.rms < best_rmse
            best_rmse = om.s.rms;
            best_name = om.method;
        end
    end
    if r1_rmse > 0
        improvement_vs_r1 = (1 - best_rmse/r1_rmse)*100;
        improvement_vs_r2 = (1 - best_rmse/r2_rmse)*100;
        fprintf('  最优融合: %s RMSE=%.1fkm\n', best_name, best_rmse);
        fprintf('  vs R1: %+.1f%%  vs R2: %+.1f%%\n\n', improvement_vs_r1, improvement_vs_r2);
    end
end
