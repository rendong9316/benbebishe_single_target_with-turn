function params_r = radar_params(params, radar_id)
%RADAR_PARAMS 按雷达编号挑选该雷达的 UKF/门限/K_loss 参数。
%   params_r = radar_params(params, radar_id)
%
%   作用：config 里两台雷达的噪声/Q/门限等按 radar1_*/radar2_* 分别存放，
%   但 UKF 创建函数（ukf_imm/ukf_jichu）只认通用字段名 ukf_* / gate_* /
%   tracker_K_loss。本函数把指定雷达的数值抄到通用字段名下。
%
%   数值全部来自 config，本函数不定义、不修改任何参数。
%
%   映射字段（共 8 个）：
%       ukf_range_std_m       <- radar{N}_range_noise_std_m
%       ukf_azimuth_std_deg   <- radar{N}_azimuth_noise_std_deg
%       ukf_Q_scale           <- radar{N}_ukf_Q_scale
%       ukf_P_pos_std         <- radar{N}_ukf_P_pos_std
%       ukf_P_vel_std         <- radar{N}_ukf_P_vel_std
%       gate_sigma            <- radar{N}_gate_sigma
%       gate_vr_ms            <- radar{N}_gate_vr_ms
%       tracker_K_loss        <- radar{N}_tracker_K_loss
%
%   其他共用字段（ukf_alpha/beta/kappa、ukf_rv_std_ms、IMM、fuzzy 等）
%   原样透传。

    if radar_id ~= 1 && radar_id ~= 2
        error('radar_params:badId', 'radar_id 必须是 1 或 2，收到 %g', radar_id);
    end

    params_r = params;
    prefix = sprintf('radar%d_', radar_id);

    params_r.ukf_range_std_m     = params.([prefix 'range_noise_std_m']);
    params_r.ukf_azimuth_std_deg = params.([prefix 'azimuth_noise_std_deg']);
    params_r.ukf_Q_scale         = params.([prefix 'ukf_Q_scale']);
    params_r.ukf_P_pos_std       = params.([prefix 'ukf_P_pos_std']);
    params_r.ukf_P_vel_std       = params.([prefix 'ukf_P_vel_std']);
    params_r.gate_sigma          = params.([prefix 'gate_sigma']);
    params_r.gate_vr_ms          = params.([prefix 'gate_vr_ms']);
    params_r.tracker_K_loss      = params.([prefix 'tracker_K_loss']);
end
