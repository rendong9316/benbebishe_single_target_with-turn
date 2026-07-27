function fix_reports_from_mat()
    % 从 .mat MC 结果中提取真实统计数据

    close all;
    addpath(genpath('.'));

    fprintf('=== Extracting True Stats from MC .mat Files ===\n\n');

    mcFiles = dir('results/mc_*rate*_*.mat');

    if isempty(mcFiles)
        warning('No MC rate scenario files found!');
        return;
    end

    % Collect all batch results as a growing cell array of structs
    allBatches = {};

    for fi = 1:length(mcFiles)
        fname = mcFiles(fi).name;
        S = load(fullfile('results', fname));

        if ~isfield(S, 'rmse_matrix'), continue; end

        R = S.rmse_matrix;
        if isfield(S, 'scenario_name')
            scName = char(S.scenario_name);
        else
            tokens = strsplit(fname, '_');
            scName = '';
            for ti = 3:(length(tokens)-1)
                tok = lower(char(tokens(ti)));
                if ~isempty(scName), scName = [scName '_']; end
                scName = [scName tok];
            end
            if isempty(scName), scName = fname(1:end-4); end
        end

        defRmse = R(:, 1);
        sigRmse = R(:, 3);

        valid = ~isnan(defRmse) & ~isnan(sigRmse);
        imps = (defRmse(valid) - sigRmse(valid)) ./ defRmse(valid) * 100;

        allBatches{end+1} = struct(...
            'scene', scName, ...
            'file', fname, ...
            'imps', imps(:)', ...
            'n', length(imps) ...
        );
    end

    % Print per-batch summary
    fprintf('%-22s | %-22s | %4s | %8s | %6s | %6s | Wins\n', ...
        'Scene', 'File', 'N', 'Mean%', 'Min%', 'Max%', '');
    fprintf('%s\n', repmat('-', 1, 100));

    for bi = 1:length(allBatches)
        b = allBatches{bi};
        imps = b.imps;
        m = mean(imps);
        s = std(imps);
        scStr = char(b.scene);
        fnStr = char(b.file);
        fnShort = fnStr(1:end-4);
        if numel(fnShort)>22, fnShort = fnShort(1:22); end
        fprintf('%-22s | %-22s | %4d | %+7.1f%% | %+6.1f%% | %+6.1f%% | %d/%d\n', ...
            scStr(1:min(numel(scStr),22)), fnShort, b.n, m, min(imps), max(imps), sum(imps>0), b.n);
    end

    % Aggregate by scene using cell arrays
    sceneImps = {};
    sceneNames = {};
    sceneNs = cell(1, 20);  % preallocate enough space
    sceneWins = cell(1, 20);
    sceneSevere = cell(1, 20);

    for bi = 1:length(allBatches)
        b = allBatches{bi};
        sc = char(b.scene);
        imps = b.imps;

        found = false;
        for si = 1:length(sceneNames)
            if strcmp(sceneNames{si}, sc)
                sceneImps{si} = [sceneImps{si}, imps];
                sceneNs{si} = sceneNs{si} + b.n;
                sceneWins{si} = sum(sceneImps{si} > 0);
                found = true;
                break;
            end
        end

        if ~found
            newIdx = length(sceneNames) + 1;
            sceneNames{newIdx} = sc;
            sceneImps{newIdx} = imps;
            sceneNs{newIdx} = b.n;
            sceneWins{newIdx} = sum(imps > 0);
            sceneSevere{newIdx} = sum(imps < -10);
        end
    end

    % Print per-batch (already done above, skip to aggregate)

    % Compute aggregates
    allImmps = [];
    fprintf('\n=== PER-SCENE AGGREGATE ===\n');
    fprintf('%-25s  %4s  %-10s  %-8s  %-8s  %-8s  %-6s  %-8s\n', ...
        'Scene', 'N', 'MeanImp', 'StdImp', 'MinImp', 'Worst', 'Wins', 'Severe');
    fprintf('%s\n', repmat('-', 1, 90));

    for si = 1:length(sceneNames)
        imps = sceneImps{si};
        m = mean(imps);
        s = std(imps);
        mn = min(imps);
        mx = max(imps);
        w = sceneWins{si};
        sv = sceneSevere{si};

        nVal = str2double(sceneNs{si});
        fprintf('%-25s  %4d  %+9.1f%%  %+7.1f%%  %+8.1f%%   %+.1f%%  %d/%d  %d\n', ...
            sceneNames{si}, nVal, m, s, mn, mx, w, nVal, sv);

        allImmps = [allImmps, imps];
    end

    % ---- Global summary ----
    fprintf('\n\n=== GLOBAL SUMMARY (%d total cases) ===\n', length(allImmps));
    gn = length(allImmps);
    gm = mean(allImmps);
    gs = std(allImmps);
    gw = sum(allImmps > 0);
    gt = sum(abs(allImmps) <= 0.01);
    gl = sum(allImmps < 0);
    gsv = sum(allImmps < -10);
    gworst = min(allImmps);

    fprintf('  Total cases:          %d\n', gn);
    fprintf('  Mean improvement:     %+0.2f%%\n', gm);
    fprintf('  Std deviation:        %+0.2f%%\n', gs);
    fprintf('  Win rate:             %.1f%% (%d wins + %d ties out of %d)\n', (gw+gt)/gn*100, gw, gt, gn);
    fprintf('  Lose rate:            %.1f%%\n', gl/gn*100);
    fprintf('  Worst case:           %+.2f%%\n', gworst);
    fprintf('  Severe (>10%% worse):  %d\n', gsv);

    % Distribution
    fprintf('\n=== IMPROVEMENT DISTRIBUTION ===\n');
    binRanges = [-Inf, -10, -5, 0, 5, 10, Inf];
    binLabels = {'<-10%%', '-10~%%0%%', '-5~0%%', '0~5%%', '5~10%%', '+>10%%'};
    for di = 1:6
        lo = binRanges(di); hi = binRanges(di+1);
        counts = sum(allImmps >= lo & allImmps < hi);
        pct = counts/gn*100;
        label = binLabels{di};
        label = strrep(strrep(label, '%%%%', '%'), '%%0', '10');
        fprintf('  %-12s: %2d (%5.1f%%)\n', label, counts, pct);
    end

    % t-test per scene
    fprintf('\n=== PAIRED T-TESTS (one-sided, H1: improvement > 0) ===\n');
    for si = 1:length(sceneNames)
        imps = sceneImps{si} / 100;
        n = numel(imps);
        if n < 3, continue; end
        m = mean(imps);
        s = std(imps);
        se = s / sqrt(n);
        tVal = m / se;
        df = n - 1;

        % Approximate one-tailed p-value using normal approx for large n
        % For small samples use t-table values as reference
        pApprox = 1 - normcdf(tVal);  % Normal approx
        if pApprox < 0.001
            sig = ' ***';
        elseif pApprox < 0.01
            sig = ' **';
        elseif pApprox < 0.05
            sig = ' *';
        else
            sig = ' (n.s.)';
        end

        fprintf('  %-25s: t=%+.2f, df=%d, p=%.4f%s (normal approx)\n', ...
            sceneNames{si}, tVal, df, pApprox, sig);
    end

    % Save final stats
    save('results/audit_extracted_stats.mat', ...
        'gn', 'gm', 'gs', 'gw', 'gt', 'gl', 'gsv', 'gworst', ...
        'allImmps', 'sceneNames', 'sceneImps', 'sceneNs', 'sceneWins', 'sceneSevere', '-v7');

    fprintf('\nSaved: results/audit_extracted_stats.mat\n');
end
