function simulate_sigmoid_behavior()
    % 模拟 Sigmoid bug vs fix 的行为对比
    % 计算不同 ct_mu 值下，各模式产生的 P_CT→CV 转移概率

    addpath(genpath('.'));

    fprintf('=== Sigmoid Bug vs Fix Behavior Analysis ===\n\n');

    % 典型参数
    base_p = 0.006;   % 1 - exp(-4/660) — Config#51 CT dwell=660
    adaptive_k = 3;     % k_turnrate=3 (bug: kp=9 with *3; fix: same)

    fprintf('Parameters: base_p=%.5f, adaptive_k=%d (kp=%d for sigmoid)\n', ...
        base_p, adaptive_k, adaptive_k*3);
    fprintf('ct_mu ranges from 0 (no turn) to 1 (strongly turning)\n\n');

    % 各分支的 adjusted P_CT→CV 函数
    f_linear = @(ctmu) base_p ./ (1 + adaptive_k * ctmu);
    f_exp_decay = @(ctmu) base_p * exp(-adaptive_k * ctmu);
    f_sigmoid_bug = @(ctmu) base_p ./ (1 + exp(-adaptive_k*3 * (ctmu - 0.3)));
    f_sigmoid_fix = @(ctmu) base_p ./ (1 + exp(+adaptive_k*3 * (ctmu - 0.3)));

    fprintf('%-10s | %-12s | %-12s | %-15s | %-15s | Clipped?\n', ...
        'ct_mu', 'linear', 'exp_decay', 'sigmoid_bug', 'sigmoid_fix', '');
    fprintf('%s\n', repmat('-', 1, 100));

    for ctmu = 0:0.1:1.0
        lin = f_linear(ctmu);
        exp_d = f_exp_decay(ctmu);
        sig_b = f_sigmoid_bug(ctmu);
        sig_f = f_sigmoid_fix(ctmu);

        clip_lin = lin < 0.05*base_p || lin > base_p;
        clip_exp = exp_d < 0.05*base_p || exp_d > base_p;
        clip_sb = sig_b < 0.05*base_p || sig_b > base_p;
        clip_sf = sig_f < 0.05*base_p || sig_f > base_p;

        clip_str = '';
        if any([clip_lin, clip_exp, clip_sb, clip_sf])
            clip_str = 'YES';
        end

        fprintf('%-10.1f | %-12.6f | %-12.6f | %-15.6f | %-15.6f | %s\n', ...
            ctmu, lin, exp_d, sig_b, sig_f, clip_str);
    end

    % Key comparison at low and high ct_mu
    fprintf('\n\n=== Key Behavior Differences ===\n\n');

    test_ctmu = [0.0, 0.2, 0.4, 0.6, 0.8, 1.0];

    for i = 1:length(test_ctmu)
        ctmu = test_ctmu(i);
        lin = f_linear(ctmu);
        exp_d = f_exp_decay(ctmu);
        sig_b = f_sigmoid_bug(ctmu);
        sig_f = f_sigmoid_fix(ctmu);

        fprintf('ct_mu = %.1f:\n', ctmu);

        if ctmu < 0.3
            fprintf('  → 直线段: P(CT→CV) 越高越好（快速切回 CV）\n');
            vals = [lin, exp_d, sig_b, sig_f];
            best = find(vals == min(vals), 1);
            names = {'linear', 'exp_decay', 'sigmoid_bug', 'sigmoid_fix'};
            fprintf('  → 实际: linear=%.5f, exp_decay=%.5f, sigmoid_bug=%.5f, sigmoid_fix=%.5f\n', lin, exp_d, sig_b, sig_f);
            fprintf('  → 最低值 = %s (最慢切换回CV) = ❌ 对直线不利\n', names{best});
            fprintf('  → 最高值 = %s (最快切换回CV) = ✓ 对直线有利\n', names{find(vals == max(vals,[],'all'), 1)});
        else
            fprintf('  → 转弯段: P(CT→CV) 越低越好（让CT留住）\n');
            vals = [lin, exp_d, sig_b, sig_f];
            best = find(vals == min(vals, [], 'all'), 1);
            names = {'linear', 'exp_decay', 'sigmoid_bug', 'sigmoid_fix'};
            fprintf('  → 实际: linear=%.5f, exp_decay=%.5f, sigmoid_bug=%.5f, sigmoid_fix=%.5f\n', lin, exp_d, sig_b, sig_f);
            fprintf('  → 最低值 = %s (最佳转弯留住) = ✓\n', names{best});
            fprintf('  → 最高值 = %s (最差转弯留住) = ❌\n', names{find(vals == max(vals, [], 'all'), 1)});
        end
        fprintf('\n');
    end

    % Summary table: improvement over linear
    fprintf('=== Relative Comparison (lower P_CT→CV = better for turns) ===\n');
    fprintf('%-10s | %-15s | %-15s | %-15s | %-15s\n', ...
        'ct_mu', 'linear', 'exp_decay', 'sigmoid_bug', 'sigmoid_fix');
    for ctmu = 0:0.2:1.0
        lin = f_linear(ctmu);
        exp_d = f_exp_decay(ctmu);
        sig_b = f_sigmoid_bug(ctmu);
        sig_f = f_sigmoid_fix(ctmu);
        fprintf('%-10.1f | %-15.6f | %-15.6f | %-15.6f | %-15.6f\n', ...
            ctmu, lin, exp_d, sig_b, sig_f);
    end
end
