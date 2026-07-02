addpath(genpath('D:/Desktop/single_target_with-turn'));
cd('D:/Desktop/single_target_with-turn');

Q_vals = [500, 1000, 3000, 10000, 30000, 100000, 300000, 1e6, 3e6];
UKF_NAMES = {'jichu','zishiying','imm'};

% For each Q, find best UKF fusion RMSE and best method for each UKF
fprintf('gradual_best_ukf_fusion:\n');
for qi = 1:length(Q_vals)
    q = Q_vals(qi);
    d = load(sprintf('results/gradual_N100_Q%g.mat', q));
    s = d.s;
    for uu = 1:3
        mean_rmse = nanmean(s(uu).rmse_fus_best);
        fprintf('  %s Q=%9g: fus_RMSE=%.2f\n', s(uu).name, q, mean_rmse);
    end
end

fprintf('\nuturn_best_ukf_fusion:\n');
for qi = 1:length(Q_vals)
    q = Q_vals(qi);
    d = load(sprintf('results/uturn_N100_Q%g.mat', q));
    s = d.s;
    for uu = 1:3
        mean_rmse = nanmean(s(uu).rmse_fus_best);
        fprintf('  %s Q=%9g: fus_RMSE=%.2f\n', s(uu).name, q, mean_rmse);
    end
end
