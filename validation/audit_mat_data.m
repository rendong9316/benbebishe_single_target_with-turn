function audit_mat_data()
    % 审计报告：从已有 .mat MC 结果中重算所有统计量
    % 目标：对比 Phase 3/4 报告中的数字 vs 实际 .mat 数据
    %
    % 策略：扫描 results/mc_*rate*.mat，加载 rmse_matrix，计算改善、win rate 等
    %
    % 注意：这些 .mat 是用修复后的代码跑的（Phase 3/4 实验）
    % Sigmoid k=3/p=1.0 对应 col 3 (Cfg51+Sigmoid)
    % Default 对应 col 1

    addpath(genpath('.'));

    fprintf('=== Audit: Extract all stats from MC .mat files ===\n\n');

    mcFiles = dir('results/mc_*rate*_*.mat');

    if isempty(mcFiles)
        fprintf('No MC rate scenario files found!\n');
        return;
    end

    fprintf('Found %d MC batch files\n', length(mcFiles));
    fprintf('%-45s %-8s %-16s %-16s %-16s %-8s %-10s\n', ...
        'File', 'Batch', 'Default', 'Sig(k3)', 'Improvement', 'Win%', 'Notes');
    fprintf('%s\n', repmat('-', 1, 120));

    all_improvements = [];  % [n_scenarios x n_seeds] per config pair
    scene_summary = struct();

    for fi = 1:length(mcFiles)
        fname = mcFiles(fi).name;
        S = load(fullfile('results', fname));

        if ~isfield(S, 'rmse_matrix')
            fprintf('  [SKIP] %s — no rmse_matrix field\n', fname);
            continue;
        end

        R = S.rmse_matrix;  % [n_seeds x 4]
        seeds = S.seeds;
        cfgs = S.configs;
        scName = S.scenario_name;

        nSeeds = size(R, 1);

        % Parse seed batch label
        [~, ~, rest] = fileparts(fname);
        batchLabel = '';
        tokens = strsplit(rest, '_');
        for ti = 1:length(tokens)
            if contains(tokens{ti}, 'to')
                batchLabel = tokens{ti};
                break;
            end
        end
        if isempty(batchLabel), batchLabel = '?'; end

        % Col 1 = Default, Col 3 = Sig_k3_p1 (both use Config#51 base params)
        defRmse = R(:, 1);   % [nSeeds x 1]
        sigRmse = R(:, 3);   % [nSeeds x 1]

        valid = ~isnan(defRmse) & ~isnan(sigRmse);
        nValid = sum(valid);

        improvements = (defRmse(valid) - sigRmse(valid)) ./ defRmse(valid) * 100;
        wins = sum(improvements > 0);
        ties = sum(abs(improvements) <= 0.01);
        loses = sum(improvements < 0);
        worstIdx = find(improvements == min(improvements));
        worstImp = min(improvements);
        bestImp = max(improvements);
        meanImp = mean(improvements);
        stdImp = std(improvements);

        sigMeans = mean(R(:, 3), 'omitnan');
        stds = std(R(:, 3), 0, 'omitnan');

        scene_data = struct(
            'scene', scName,
            'batch', batchLabel,
            'seeds', seeds(valid)',
            'n_seeds', nValid,
            'mean_imp', meanImp,
            'std_imp', stdImp,
            'min_imp', min(improvements),
            'max_imp', max(improvements),
            'wins', wins,
            'ties', ties,
            'loses', loses,
            'improvements', improvements
        );

        scene_summary(scName) = scene_data;

        fprintf('%-45s %-8s %-16.2f %-16.2f %+.1f%% +/- %.1f  %d/%d %s\n', ...
            fname(1:44), batchLabel, ...
            mean(R(:,1),'omitnan'), mean(R(:,3),'omitnan'), ...
            meanImp, stdImp, wins+ties, nValid, ...
            sprintf('[Worst:%+.1f%%]', worstImp));

        % Collect overall
        if isfield(scene_summary, scName)
            % Will aggregate later
        end
    end

    % Aggregate by scene
    fprintf('\n\n=== Per-Scene Aggregate (all batches) ===\n');
    scenes = unique({mcFiles.name});
    % Group by scenario_name from filename patterns
    sceneGroups = struct();

    for fi = 1:length(mcFiles)
        fname = mcFiles(fi).name;
        S = load(fullfile('results', fname));
        if ~isfield(S, 'scenario_name'), continue; end
        scName = S.scenario_name;
        if isempty(sceneGroups), sceneGroups.names = {}; end
        idx = find(strcmp(sceneGroups.names, scName));
        if isempty(idx)
            sceneGroups.names{end+1} = scName;
            newIdx = numel(sceneGroups.names);
            sceneGroups.improvements{newIdx} = [];
        else
            newIdx = idx(1);
        end

        R = S.rmse_matrix;
        defRmse = R(:, 1);
        sigRmse = R(:, 3);
        valid = ~isnan(defRmse) & ~isnan(sigRmse);
        imps = (defRmse(valid) - sigRmse(valid)) ./ defRmse(valid) * 100;
        sceneGroups.improvements{newIdx} = [sceneGroups.improvements{newIdx}, imps];
    end

    for si = 1:length(sceneGroups.names)
        name = sceneGroups.names{si};
        imps = sceneGroups.improvements{si};
        n = numel(imps);
        if n == 0, continue; end
        meanImp = mean(imps);
        stdImp = std(imps);
        winRate = sum(imps > 0) / n * 100;
        worstImp = min(imps);
        severe = sum(imps < -10);

        fprintf('%-25s: n=%2d  mean=%+.1f%%  std=%.1f  win=%3.0f%%  worst=%+.1f%%  severe(-10%%)=%d\n', ...
            name, n, meanImp, stdImp, winRate, worstImp, severe);
    end
end
