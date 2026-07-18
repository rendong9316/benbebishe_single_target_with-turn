% =========================================================================
% post_init_multi.m — UKF 初始化后处理
% =========================================================================
% 【功能】
%   在 ukf_dispatch('init', ...) 之后统一注入以下字段：
%     1. dt — 时间步长（从 params.dt_sec 复制）
%     2. initialized — 标记为 true
%     3. nis_history — 清空（新航迹尚无 NIS 历史）
%     4. Q_base — 备份初始 Q（用于自适应 Q 的相对缩放）
%     5. Q_ema — 初始化为 1.0（自适应 Q 的 EMA 平滑因子）
%
% 【IMM 特殊处理】
%   如果 ukf 内部包含 ukf_cv 和 ukf_ct 子滤波器（IMM 模式），
%   需要对两个子 UKF 同样注入 dt 和 initialized 标志。
% =========================================================================
function ukf = post_init_multi(ukf, params)
    % 注入时间步长和初始化标志
    % dt 用于 UKF 预测时的时间积分，initialized 标记滤波器已完成初始化
    ukf.dt = params.dt_sec;
    ukf.initialized = true;

    % IMM 模式下，同步更新两个子 UKF 的 dt 和 initialized
    % IMM（Interactive Multi-Model）包含恒速(CV)和恒转弯(CT)两种运动模型，
    % 每个子滤波器都需要独立的时间步长和初始化标志
    if isfield(ukf, 'ukf_cv')
        ukf.ukf_cv.dt = params.dt_sec;
        ukf.ukf_cv.initialized = true;
        ukf.ukf_ct.dt = params.dt_sec;
        ukf.ukf_ct.initialized = true;
    end

    % 清空 NIS 历史（新航迹尚无观测数据，NIS 列表为空）
    ukf.nis_history = [];

    % 备份初始 Q 矩阵（用于自适应 Q 的相对缩放）
    % 自适应 Q 会根据 NIS 动态调整过程噪声协方差，
    % Q_base 作为参考基准，所有调整都是相对于 Q_base 的倍数
    if ~isfield(ukf, 'Q_base') || isempty(ukf.Q_base)
        if isfield(ukf, 'Q'), ukf.Q_base = ukf.Q; end
    end

    % 初始化 Q_EMA 为 1.0（自适应 Q 的平滑因子）
    % Q_ema 是自适应 Q 的指数移动平均系数，初始为 1.0 表示
    % 尚未开始自适应调整，使用原始 Q 矩阵
    if ~isfield(ukf, 'Q_ema') || isempty(ukf.Q_ema), ukf.Q_ema = 1.0; end
end
