% =========================================================================
% diag_adaptive_q.m — 诊断自适应 Q 在直线/拐弯场景的行为
% =========================================================================

clear; close all; clc;
addpath(genpath('.'));

fprintf('============================================================\n');
fprintf('  诊断 1: 直线场景自适应 Q 行为\n');
fprintf('============================================================\n\n');

nis_R1_straight = 1.589;
nis_R2_straight = 1.111;

show_q_factor(nis_R1_straight, 'R1 (NIS=1.589)');
show_q_factor(nis_R2_straight, 'R2 (NIS=1.111)');

fprintf('============================================================\n');
fprintf('  诊断 2: 拐弯场景自适应 Q 行为\n');
fprintf('============================================================\n\n');

nis_R1_turn = 1.90;
nis_R2_turn = 1.20;
nis_R1_imm  = 2.00;
nis_R2_imm  = 1.30;

show_q_factor(nis_R1_turn, 'zishiying R1 (NIS=1.90)');
show_q_factor(nis_R2_turn, 'zishiying R2 (NIS=1.20)');
show_q_factor(nis_R1_imm, 'imm R1 (NIS=2.00)');
show_q_factor(nis_R2_imm, 'imm R2 (NIS=1.30)');

fprintf('============================================================\n');
fprintf('  诊断 3: CT 模型加自适应 Q 会发生什么\n');
fprintf('============================================================\n\n');

fprintf('  CT 模型的运动学特性：\n');
fprintf('  - 状态转移 F_CT(omega) 包含 sin(omega*dt)/omega, (1-cos(omega*dt))/omega\n');
fprintf('  - omega=1 deg/s, dt=30s -> omega*dt=30 deg -> sin(30)=0.5, 1-cos(30)=0.134\n');
fprintf('  - CT 预测的 P_zz 行列式 > CV 预测的 P_zz 行列式\n');
fprintf('    (CT 引入了额外的不确定性：sin/cos 项的协方差传播)\n');
fprintf('  - 因此 CT 的归一化似然度 N(z;Hx,P_zz) 天然被行列式惩罚\n');
fprintf('\n');

fprintf('  如果给 CT 也加自适应 Q：\n');
fprintf('  1. CT 的 NIS 通常比 CV 更低（转弯段）-> Q 因子更小 -> Q 缩小\n');
fprintf('  2. Q 缩小 -> P_pred 缩小 -> K 缩小 -> 更信任模型而非量测\n');
fprintf('  3. 在转弯段：模型是对的，这步是对的\n');
fprintf('  4. 但在直线段：CT 模型是错的，Q 缩小会让模型更固执，误差更大\n');
fprintf('\n');

fprintf('  关键问题：CT 的 Q_base 和 CV 的 Q_base 相同\n');
fprintf('  但 CT 的预测协方差 P_pred_ct > P_pred_cv\n');
fprintf('  因为 F_CT 包含三角函数，协方差传播 P_pred = F*P*F^T + Q 中\n');
fprintf('  F 的条件数 > 1 -> CT 的 NIS 在直线段会更高 -> 模糊推理提升 Q\n');
fprintf('  -> 但提升幅度受 EMA(eta=0.2) 平滑，响应慢\n');
fprintf('\n');

fprintf('  结论：CT 加自适应 Q 后：\n');
fprintf('  - 直线段：NIS 高 -> Q 提升 -> 部分抵消 CT 模型误差（好事）\n');
fprintf('  - 转弯段：NIS 低 -> Q 缩小 -> CT 更信任自己（好事）\n');
fprintf('  - 但 CT 的似然度计算中 P_zz 行列式惩罚仍然存在\n');
fprintf('  - 且 Q 提升会增大 P_pred，进一步增大 P_zz，反而降低似然度\n');
fprintf('  -> 自适应 Q 可能帮倒忙：提升 Q -> 增大 P_zz -> 似然度下降 -> mu_ct 更低\n');
fprintf('\n');

fprintf('============================================================\n');
fprintf('  诊断 4: 自适应 Q 因子 vs NIS 的完整曲线\n');
fprintf('============================================================\n\n');

for nis_v = 0:0.1:4.0
    factor = compute_fuzzy_factor(nis_v);
    q_ema = 0.2 * factor + 0.8 * 1.0;
    if abs(q_ema - 1.0) < 0.05
        q_final = 1.0;
    else
        q_final = q_ema;
    end
    fprintf('  NIS=%.1f -> factor=%.3f -> Q_ema=%.3f -> Q/Q_base=%.3f\n', ...
        nis_v, factor, q_ema, q_final);
end

fprintf('\n============================================================\n');
fprintf('  诊断 5: 直线场景下 Q 变化的实际影响\n');
fprintf('============================================================\n\n');

fprintf('  直线场景 NIS = 1.6 (R1) / 1.1 (R2)\n');
fprintf('  -> NIS ratio = 0.8 / 0.55\n');
fprintf('  -> 隶属函数峰值在 S/M 区间\n');
fprintf('  -> 模糊因子 = 0.9~1.0\n');
fprintf('  -> Q_ema(eta=0.2) 从 1.0 收敛到 0.98~1.02\n');
fprintf('  -> Q 变化 < 2%%，对滤波效果影响微乎其微\n');
fprintf('  -> 真正的收益来自：机动检测（NIS 趋势比较）+ EMA 平滑\n');
fprintf('\n');

fprintf('  直线场景的 3%% 坏种子消失来自哪里？\n');
fprintf('  1. 自适应 Q 在 NIS 突增时（量测异常）自动提升 Q -> 防发散\n');
fprintf('  2. 机动检测在 NIS 短时突增时触发 -> Q 提升到 1.5~3.5\n');
fprintf('  3. 两者叠加，形成安全网\n');
fprintf('  4. 旧代码（固定 Q）遇到 NIS 突增 -> K 过大 -> 状态跳变 -> 发散\n');
fprintf('\n');

fprintf('============================================================\n');
fprintf('Done.\n');
fprintf('============================================================\n');


%% =========================================================================
% show_q_factor — 显示单个 NIS 值对应的自适应 Q 因子
% =========================================================================
function show_q_factor(nis_val, label)
    nis_ratio = nis_val / 2.0;

    mu_VS = trimf_val(nis_ratio, 0.0, 0.0, 0.4);
    mu_S  = trimf_val(nis_ratio, 0.2, 0.5, 0.8);
    mu_M  = trimf_val(nis_ratio, 0.6, 1.0, 1.5);
    mu_L  = trimf_val(nis_ratio, 1.3, 2.0, 3.0);
    mu_VL = trimf_val(nis_ratio, 2.5, 4.0, 4.0);

    total = mu_VS + mu_S + mu_M + mu_L + mu_VL;
    if total < 1e-10
        factor = 1.0;
    else
        factor = (mu_VS*0.6 + mu_S*0.8 + mu_M*1.0 + mu_L*1.8 + mu_VL*3.0) / total;
    end

    q_ema = 0.2 * factor + 0.8 * 1.0;
    if abs(q_ema - 1.0) < 0.05
        q_final = 1.0;
    else
        q_final = q_ema;
    end

    fprintf('  %s:\n', label);
    fprintf('    NIS = %.3f, nis_ratio = %.3f\n', nis_val, nis_ratio);
    fprintf('    隶属度: VS=%.3f S=%.3f M=%.3f L=%.3f VL=%.3f\n', mu_VS, mu_S, mu_M, mu_L, mu_VL);
    fprintf('    模糊因子 = %.4f\n', factor);
    fprintf('    EMA(eta=0.2) 收敛值 = %.4f\n', q_ema);
    fprintf('    最终 Q/Q_base = %.4f\n', q_final);
    fprintf('\n');
end


%% =========================================================================
% compute_fuzzy_factor — 计算模糊自适应因子
% =========================================================================
function f = compute_fuzzy_factor(nis_val)
    nis_ratio = nis_val / 2.0;
    mu_VS = trimf_val(nis_ratio, 0.0, 0.0, 0.4);
    mu_S  = trimf_val(nis_ratio, 0.2, 0.5, 0.8);
    mu_M  = trimf_val(nis_ratio, 0.6, 1.0, 1.5);
    mu_L  = trimf_val(nis_ratio, 1.3, 2.0, 3.0);
    mu_VL = trimf_val(nis_ratio, 2.5, 4.0, 4.0);
    total = mu_VS + mu_S + mu_M + mu_L + mu_VL;
    if total < 1e-10
        f = 1.0;
    else
        f = (mu_VS*0.6 + mu_S*0.8 + mu_M*1.0 + mu_L*1.8 + mu_VL*3.0) / total;
    end
    f = max(0.5, min(4.0, f));
end


%% =========================================================================
% trimf_val — 三角形隶属函数
% =========================================================================
function mu = trimf_val(x, a, b, c)
    if x <= a || x >= c
        mu = 0;
    elseif x < b
        mu = (x - a) / (b - a);
    else
        mu = (c - x) / (c - b);
    end
end
