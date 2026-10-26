function probe_qcheck()
% 1) 检查参数传递链  2) 极端参数验证生效性  3) 可重复性
root = 'D:/Desktop/毕设相关/AAA毕业论文撰写/文稿撰写/single_target_with-turn';
cd(root); addpath(genpath(root));
fid = fopen(fullfile(root,'_audit','qcheck.txt'),'w');

p = simulation_params_oracle();
pr = radar_params(p, 1);
fprintf(fid,'--- param chain ---\n');
fprintf(fid,'global imm_q_schedule_enable = %d\n', p.imm_q_schedule_enable);
fprintf(fid,'radar1 has field = %d, value = %d\n', isfield(pr,'imm_q_schedule_enable'), ...
    getfield_def(pr,'imm_q_schedule_enable',-1));
fprintf(fid,'stage1 scale = %g, frames = %g\n', p.imm_q_stage1_scale, p.imm_q_stage1_frames);

% 创建 IMM 检查内部字段
imm = ukf_imm('create', pr, p.radar1_lon, p.radar1_lat, p.radar1_tx_lon, p.radar1_tx_lat, p.dt_sec);
fprintf(fid,'imm.params has q_sched = %d\n', isfield(imm.params,'imm_q_schedule_enable'));
fprintf(fid,'imm.age = %g\n', getfield_def(imm,'age',NaN));
fprintf(fid,'imm.ukf_cv.Q_ema = %g\n', getfield_def(imm.ukf_cv,'Q_ema',NaN));

% ---- 极端参数验证 ----
base = p; base.imm_q_schedule_enable = false; base.use_fuzzy_adaptive=false; base.imm_adapt_mode='none';
ext  = p; ext.imm_q_schedule_enable = true; ext.use_fuzzy_adaptive=false; ext.imm_adapt_mode='none';
ext.imm_q_stage1_scale = 1000; ext.imm_q_stage2_scale = 1000; ext.imm_q_stage3_scale = 100;

fprintf(fid,'\n--- run A0 (no sched) twice ---\n');
r1 = run('multi_cross', base);
r2 = run('multi_cross', base);
fprintf(fid,'A0 run1 R1=%.4f  run2 R1=%.4f  diff=%.4f\n', ...
    r1.errorStats_R1.overall.ukf.rms, r2.errorStats_R1.overall.ukf.rms, ...
    r1.errorStats_R1.overall.ukf.rms - r2.errorStats_R1.overall.ukf.rms);

fprintf(fid,'\n--- run A1-extreme (Q x1000) ---\n');
r3 = run('multi_cross', ext);
fprintf(fid,'A0 R1=%.4f   A1-extreme R1=%.4f   change=%.1f%%\n', ...
    r2.errorStats_R1.overall.ukf.rms, r3.errorStats_R1.overall.ukf.rms, ...
    100*(r2.errorStats_R1.overall.ukf.rms - r3.errorStats_R1.overall.ukf.rms)/r2.errorStats_R1.overall.ukf.rms);

% 快照里 ukf 有哪些字段
sn = r3.trackSnapshots_R1{20}.trackList;
if ~isempty(sn)
    fprintf(fid,'\nsnapshot ukf fields: %s\n', strjoin(fieldnames(sn{1}.ukf)', ' '));
end
fclose(fid);
disp(fileread(fullfile(root,'_audit','qcheck.txt')));
end

function v = getfield_def(s,f,d)
if isfield(s,f), v = s.(f); else, v = d; end
end
