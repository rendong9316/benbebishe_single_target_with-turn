% =========================================================================
% radar_params.m — 雷达专属参数选择器
% =========================================================================
% 【功能】
%   从全局参数结构体中提取指定雷达的专属参数，映射到 UKF 模块通用的
%   字段名（ukf_*、gate_*、tracker_K_loss）。
%
% 【为什么需要这个函数？】
%   全局参数中两台雷达的噪声/Q/门限等按 radar1_*/radar2_* 前缀分别
%   存放。但 UKF 创建函数（ukf_imm/ukf_jichu）只认通用字段名 ukf_* /
%   gate_* / tracker_K_loss。本函数充当"参数适配层"，把雷达专属的数
%   据抄到通用字段名下。
%
%   使用方式：
%       params_r = radar_params(simulation_params_oracle(), 1);
%       ukf_imm('create', params_r, radar_lon, radar_lat, tx_lon, tx_lat, dt);
%
% 【映射字段（共 8 个）】
%     ukf_range_std_m       ← radar{N}_range_noise_std_m
%     ukf_azimuth_std_deg   ← radar{N}_azimuth_noise_std_deg
%     ukf_Q_scale           ← radar{N}_ukf_Q_scale
%     ukf_P_pos_std         ← radar{N}_ukf_P_pos_std
%     ukf_P_vel_std         ← radar{N}_ukf_P_vel_std
%     gate_sigma            ← radar{N}_gate_sigma
%     gate_vr_ms            ← radar{N}_gate_vr_ms
%     tracker_K_loss        ← radar{N}_tracker_K_loss
%
% 【透传字段】
%   其他共用字段（ukf_alpha/beta/kappa、ukf_rv_std_ms、IMM 参数、
%   模糊自适应参数等）原样透传，不作修改。
%
% 【输入】
%   params     - 全局参数结构体（由 simulation_params_oracle 产生）
%   radar_id   - 雷达编号（1 或 2）
%
% 【输出】
%   params_r   - 适配后的参数结构体，包含通用字段名 + 透传的共用字段
% =========================================================================
function params_r = radar_params(params, radar_id)
    % 参数校验：radar_id 必须是 1 或 2
    % 防止传入非法雷达编号导致后续索引错误
    if radar_id ~= 1 && radar_id ~= 2
        error('radar_params:badId', 'radar_id 必须是 1 或 2，收到 %g', radar_id);
    end

    % 深拷贝全局参数（MATLAB 的结构体赋值是浅拷贝，需要显式复制）
    params_r = params;
    % 构造雷达前缀，如 "radar1_" 或 "radar2_"
    prefix = sprintf('radar%d_', radar_id);

    % 按映射关系将雷达专属参数抄到通用字段名下
    % 使用动态字段名 params.([prefix 'range_noise_std_m']) 避免硬编码
    params_r.ukf_range_std_m     = params.([prefix 'range_noise_std_m']);
    params_r.ukf_azimuth_std_deg = params.([prefix 'azimuth_noise_std_deg']);
    params_r.ukf_Q_scale         = params.([prefix 'ukf_Q_scale']);
    params_r.ukf_P_pos_std       = params.([prefix 'ukf_P_pos_std']);
    params_r.ukf_P_vel_std       = params.([prefix 'ukf_P_vel_std']);
    params_r.ukf_init_pos_std_m  = params.([prefix 'ukf_init_pos_std_m']);
    params_r.ukf_init_vel_std_ms = params.([prefix 'ukf_init_vel_std_ms']);
    params_r.gate_sigma          = params.([prefix 'gate_sigma']);
    params_r.gate_vr_ms          = params.([prefix 'gate_vr_ms']);
    params_r.tracker_K_loss      = params.([prefix 'tracker_K_loss']);
end
