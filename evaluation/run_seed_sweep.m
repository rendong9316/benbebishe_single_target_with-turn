function T = run_seed_sweep(scene, seeds, outdir)
%RUN_SEED_SWEEP 用多个随机种子跑同一场景，统计鲁棒性（RMSE 均值/标准差等）。
%
%   run_seed_sweep('adsb_ns_all', 1:10)
%
%   每个种子只改 params.random_seed（点迹噪声、漏检、杂波共用同一随机流），
%   场景、部署、模型集、门限全部不变。
%
%   结果每跑完一个种子就追加一行到 <outdir>/<scene>_seeds.csv，
%   中途崩了也不会丢掉已完成的种子。同时把该种子的 metrics 存成
%   <outdir>/<scene>_seed%02d.mat。

    if nargin < 3 || isempty(outdir)
        outdir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results', 'seed_sweep');
    end
    if ~exist(outdir, 'dir'), mkdir(outdir); end
    csv_path = fullfile(outdir, sprintf('%s_seeds.csv', scene));

    cols = {'seed', 'wall_sec', 'R1_rms', 'R2_rms', ...
            'SCC', 'BC', 'CI', 'FCI', 'gain_pct', ...
            'n_pairs', 'n_correct', 'n_wrong', 'n_missed', 'precision', 'recall', ...
            'coverage', 'r1_only', 'r2_only', ...
            'ratio_mean', 'ratio_median', 'n_longer', 'n_shorter', ...
            'nees_R1', 'nees_R2', 'rho', 'indep_rms', 'opt_rms'};

    if ~isfile(csv_path)
        fid = fopen(csv_path, 'w');
        fprintf(fid, '%s\n', strjoin(cols, ','));
        fclose(fid);
    end

    T = zeros(numel(seeds), numel(cols));

    for ii = 1:numel(seeds)
        s = seeds(ii);
        p = params_coverage_online();
        p.random_seed = s;
        t0 = tic;
        fprintf('\n########## seed %d (%d/%d) ##########\n', s, ii, numel(seeds));
        r = run(scene, p, struct('verbose', false));
        wall = toc(t0);
        m = r.metrics;

        row = nan(1, numel(cols));
        row(1) = s;
        row(2) = wall;
        row(3) = m.rmse.R1_rms;
        row(4) = m.rmse.R2_rms;
        for k = 1:numel(m.rmse.fusion)
            switch upper(m.rmse.fusion(k).method)
                case 'SCC', row(5) = m.rmse.fusion(k).rms;
                case 'BC',  row(6) = m.rmse.fusion(k).rms;
                case 'CI',  row(7) = m.rmse.fusion(k).rms;
                case 'FCI', row(8) = m.rmse.fusion(k).rms;
            end
        end
        cf = m.common_frame;
        row(9)  = 100 * (min(cf.R1, cf.R2) - cf.fused) / min(cf.R1, cf.R2);
        row(10) = m.matching.n_pairs;
        row(11) = m.matching.n_correct;
        row(12) = m.matching.n_wrong;
        row(13) = m.matching.n_missed;
        row(14) = m.matching.precision;
        row(15) = m.matching.recall;
        row(16) = m.coverage.coverage;
        row(17) = m.coverage.r1_only;
        row(18) = m.coverage.r2_only;
        row(19) = m.length.ratio_mean;
        row(20) = m.length.ratio_median;
        row(21) = m.length.n_longer;
        row(22) = m.length.n_shorter;
        row(23) = m.consistency.nees_R1;
        row(24) = m.consistency.nees_R2;
        row(25) = m.consistency.rho;
        row(26) = m.consistency.indep_rms;
        row(27) = m.consistency.opt_rms;

        T(ii, :) = row;
        fid = fopen(csv_path, 'a');
        fprintf(fid, '%.6g,', row(1:end-1));
        fprintf(fid, '%.6g\n', row(end));
        fclose(fid);

        save(fullfile(outdir, sprintf('%s_seed%02d.mat', scene, s)), 'm', 's', 'wall');
        fprintf('seed %d 完成 %.0f s | R1 %.2f R2 %.2f SCC %.2f gain %.1f%% | P %.0f%% R %.0f%% | 比 %.3f\n', ...
            s, wall, row(3), row(4), row(5), row(9), row(14), row(15), row(19));
    end

    % ---- 汇总 ----
    fprintf('\n========== %s  %d 个种子汇总 ==========\n', scene, numel(seeds));
    stat = @(x) sprintf('%.2f ± %.2f  [%.2f, %.2f]', mean(x), std(x), min(x), max(x));
    fprintf('R1 RMSE      %s\n', stat(T(:, 3)));
    fprintf('R2 RMSE      %s\n', stat(T(:, 4)));
    fprintf('SCC RMSE     %s\n', stat(T(:, 5)));
    fprintf('BC  RMSE     %s\n', stat(T(:, 6)));
    fprintf('CI  RMSE     %s\n', stat(T(:, 7)));
    fprintf('FCI RMSE     %s\n', stat(T(:, 8)));
    fprintf('融合增益%%    %s\n', stat(T(:, 9)));
    fprintf('精度%%        %s\n', stat(T(:, 14)));
    fprintf('查全%%        %s\n', stat(T(:, 15)));
    fprintf('覆盖率%%      %s\n', stat(T(:, 16) * 100));
    fprintf('长度比       %s\n', stat(T(:, 19)));
    fprintf('NEES R1/R2   %s / %s\n', stat(T(:, 23)), stat(T(:, 24)));
    fprintf('rho          %s\n', stat(T(:, 25)));
    fprintf('\n结果表：%s\n', csv_path);
end
