function sweep_simple_test()
    % Simple turn_rate sweep - minimal logic, maximum reliability
    close all;
    addpath(genpath(pwd));

    % CRITICAL: Disable figure creation for batch mode
    set(0, 'DefaultFigureVisible', 'off');

    seeds = 1:10;  % Quick verification with 10 seeds
    candidates = [0.5, 0.7, 1.0, 1.3, 1.5, 2.0];
    scenarios = {'right_rate_0p7', 'left_rate_0p7', 'right_rate_1p3', 'left_rate_1p3'};

    n_seeds = length(seeds);
    n_cand = length(candidates);
    n_sc = length(scenarios);

    fprintf('Starting sweep: %d seeds x %d scenarios x %d candidates = %d runs\n', ...
        n_seeds, n_sc, n_cand, n_seeds*n_sc*n_cand);

    t0 = tic;

    for si = 1:n_seeds
        seed = seeds(si);
        fprintf('\n--- Seed %d ---\n', seed);

        for sci = 1:n_sc
            sname = scenarios{sci};

            for ci = 1:n_cand
                cand = candidates(ci);
                params = simulation_params_oracle();
                params.random_seed = seed;
                params.imm_turn_rate_rad_per_sec = cand * pi / 180.0;

                try
                    r = run(sname, params);
                    val = r.errorStats_R1.overall.ukf.rms;
                    fprintf('  %s %.1f°/s -> %.3f km\n', sname, cand, val);
                catch ME
                    fprintf('  %s %.1f°/s -> ERROR: %s\n', sname, cand, ME.message);
                end
            end
        end

        % Save seed results
        S.seed = seed;
        S.elapsed = toc(t0);
        save(sprintf('seed%d.mat', seed));
    end

    fprintf('\nTotal: %.0f seconds\n', toc(t0));
    fprintf('All seeds done. Check seed*.mat for results.\n');
end
