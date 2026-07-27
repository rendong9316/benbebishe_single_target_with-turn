function check_mu_history_raw()
    close all;
    addpath(genpath('.'));
    rng(42);

    r = run('single_turn_right_short');
    fprintf('=== Checking R1 trackList ===%s', newline);
    for ti = 1:length(r.trackList_R1)
        trk = r.trackList_R1{ti};
        if isfield(trk, 'ukf') && isfield(trk.ukf, 'mu_history')
            mh = trk.ukf.mu_history;
            fprintf('Track %d: mu_history size=[%d x %d]%s', ti, size(mh,1), size(mh,2), newline);
            if ~isempty(mh)
                fprintf('  First 5 rows:%s', newline);
                disp(mh(1:min(5,size(mh,1)), :));
                fprintf('  Last 3 rows:%s', newline);
                disp(mh(max(1,size(mh,1)-2):end, :));
                fprintf('  Sum of each row (should be 1.0):%s', newline);
                row_sums = sum(mh, 2);
                fprintf('  min=%.4f max=%.4f mean=%.4f%s', min(row_sums), max(row_sums), ...
                    mean(row_sums(~isnan(row_sums(:)|isinf(row_sums)|isnan(row_sums)))), newline);
            end
        else
            fprintf('Track %d: no mu_history%s', ti, newline);
        end
    end

    fprintf('%s=== Checking R2 trackList ===%s', newline, newline);
    for ti = 1:length(r.trackList_R2)
        trk = r.trackList_R2{ti};
        if isfield(trk, 'ukf') && isfield(trk.ukf, 'mu_history')
            mh = trk.ukf.mu_history;
            fprintf('Track %d: mu_history size=[%d x %d]%s', ti, size(mh,1), size(mh,2), newline);
            if ~isempty(mh)
                fprintf('  First 3 rows:%s', newline);
                disp(mh(1:min(3,size(mh,1)), :));
            end
        end
    end
end
