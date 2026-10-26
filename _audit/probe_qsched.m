% Q 调度效果对比：A0-原始(3in1) / A0-纯净 / A1-Q调度
root = 'D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn';
cd(root); addpath(genpath(root));

base = simulation_params_oracle();
C = cell(1,3);
C{1} = base; C{1}.imm_q_schedule_enable = false; C{1}.use_fuzzy_adaptive = true;  C{1}.imm_adapt_mode = '3in1';
C{2} = base; C{2}.imm_q_schedule_enable = false; C{2}.use_fuzzy_adaptive = false; C{2}.imm_adapt_mode = 'none';
C{3} = base; C{3}.imm_q_schedule_enable = true;  C{3}.use_fuzzy_adaptive = false; C{3}.imm_adapt_mode = 'none';
names = {'A0-orig(3in1)', 'A0-clean    ', 'A1-Q调度    '};

scenes = {'multi_cross', 'single_turn_right_sustained'};
RES = nan(numel(scenes), 3, 4);   % [R1rmse, R2rmse, SCCrmse, CIrmse]

for si = 1:numel(scenes)
    fprintf('\n================ %s ================\n', scenes{si});
    for ci = 1:3
        try
            r = run(scenes{si}, C{ci});
            r1 = r.errorStats_R1.overall.ukf.rms;
            r2 = r.errorStats_R2.overall.ukf.rms;
            % 融合
            f = r.fusion_eval.overall;
            v = nan(1,4);
            for m = 1:numel(f)
                if strcmp(f(m).method,'SCC'), v(1)=f(m).s.rms; end
                if strcmp(f(m).method,'CI'),  v(2)=f(m).s.rms; end
            end
            RES(si,ci,:) = [r1, r2, v(1), v(2)];
            fprintf('  %s  R1=%.3f  R2=%.3f  SCC=%.3f  CI=%.3f  (n=%d)\n', ...
                names{ci}, r1, r2, v(1), v(2), r.errorStats_R1.overall.ukf.n);
        catch ME
            fprintf('  %s  ERROR: %s\n', names{ci}, ME.message);
        end
    end
end

fprintf('\n===== 汇总（RMSE km）=====\n');
fprintf('%-32s %8s %8s %8s %8s\n','配置','R1','R2','SCC','CI');
for si = 1:numel(scenes)
    fprintf('-- %s --\n', scenes{si});
    for ci = 1:3
        v = squeeze(RES(si,ci,:));
        fprintf('%-32s %8.3f %8.3f %8.3f %8.3f\n', names{ci}, v(1), v(2), v(3), v(4));
    end
    if all(isfinite(squeeze(RES(si,:,1))))
        b = RES(si,2,1); q = RES(si,3,1);
        fprintf('   Q调度 vs 纯净基线: R1 %+.1f%%\n', 100*(b-q)/b);
    end
end
