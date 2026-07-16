% =========================================================================
% simulation_params_multi.m — 多目标交叉航迹仿真专用参数配置
% =========================================================================
%
% 【功能概述】
%   多目标跟踪仿真（三目标交叉航迹）的参数配置入口。
%   继承自 simulation_params.m（单目标/回头弯场景），仅覆盖
%   多目标场景下需要差异化的参数。
%
% 【与单目标场景的参数差异】
%   - detection_probability = 0.6  （雷达特性硬约束，与单目标一致，不作弊）
%   - radar1_ukf_Q_scale = 1e5    （与单目标一致，保证交叉区域滤波稳定）
%   - radar2_ukf_Q_scale = 2e5    （与单目标一致）
%   - 新增 imm_Pi_CV_to_CT/CT_to_CV （IMM 双模型转移概率）
%
% 【调用关系】
%   被调用方: run_simulation_multi.m
% =========================================================================

function params = simulation_params_multi()
    % 先加载单目标场景的全部默认参数
    params = simulation_params();

    % =====================================================================
    % 多目标差异化参数
    % =====================================================================

    % 检测概率：雷达特性硬约束，不可调整
    params.detection_probability = 0.6;

    % IMM 双模型转移概率（CV=常速, CT=恒转弯）
    % 0.001 表示平均每1000帧才会发生一次模型切换，适合直线交叉场景
    params.imm_Pi_CV_to_CT = 0.001;
    params.imm_Pi_CT_to_CV = 0.001;

    % 多目标 M/N 起始参数
    params.multi_truth_init_enable = true;
    params.multi_truth_init_gate_m = 120000;
    params.multi_truth_init_quality = 12;
    params.multi_start_M = 3;
    params.multi_start_N = 5;
    params.multi_start_max_gap_frames = 2;
    params.multi_start_max_misses = 2;
    params.multi_start_min_speed_ms = 80;
    params.multi_start_max_speed_ms = 380;
    params.multi_start_heading_gate_deg = 60;
    params.multi_start_initial_quality = 5;
    params.multi_start_used_prob_threshold = 0.35;
    params.multi_duplicate_gate_m = 50000;
    params.multi_prune_duplicate_gate_m = 10000;
    params.multi_prune_protect_life = 8;
    params.multi_fallback_geo_gate_m = 90000;

    % 多目标 JPDA 参数
    params.jpda_geo_gate_m_initial = 160000;
    params.jpda_geo_gate_m_stable = 90000;
    params.jpda_geo_gate_m_missed_step = 20000;
    params.jpda_max_hypotheses = 5000;
    params.jpda_min_update_prob = 0.05;
    params.jpda_vr_gate_ms = 60;
    params.multi_fallback_use_vr_gate = false;
    % JPDA* 置换剪枝（Blom & Bloem 2006）：防 track coalescence
    params.jpda_star_enable = true;

    % 运动一致性硬门：禁止航迹跳到物理不可达的远距离检测
    % 基于航迹当前速度计算可达半径，即使协方差膨胀也不会误关联
    params.motion_gate_margin_m = 25000;
    params.motion_gate_max_m = 60000;

    % 双门限航迹关联参数（开题报告）
    params.track_matcher_method = 'dualgate';  % 'dualgate' 或 'legacy'
    params.dualgate_T1_km = 35;            % 第一门限：距离粗筛
    params.dualgate_M = 8;                 % 第二门限：连续帧数
    params.dualgate_var_km2 = 50;          % 方差校验阈值
    params.dualgate_coexist_thresh = 5;    % 最少共现帧数
    params.dualgate_mutual_exclusion = true;  % 互斥后处理：每条航迹只保留最佳匹配

    % 多目标航迹质量参数
    params.multi_confirm_quality = 8;
    params.multi_maintain_quality = 4;
    params.tracker_K_loss = 15;
    params.multi_truth_reinit_enable = false;
    % 真值辅助终止：truth-init 航迹在对应真值结束后立即转 HISTORY，
    % 防止纯预测外推越过真值终点（修复短航迹结束后续命问题）
    params.multi_truth_terminate_enable = true;
end
