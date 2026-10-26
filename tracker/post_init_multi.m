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
%   如果 ukf 为 IMM 模式，需要对 CV 和左右 CT 子 UKF 同步初始化。
% =========================================================================
function ukf = post_init_multi(ukf, params)
    % 注入时间步长和初始化标志
    % dt 用于 UKF 预测时的时间积分，initialized 标记滤波器已完成初始化
    ukf.dt = params.dt_sec;
    ukf.initialized = true;

    % IMM 模式下，同步更新所有子 UKF 的 dt 和 initialized。
    % 子模型列表由 IMM 模板提供，避免 tracker 硬编码模型数量和名称。
    if isfield(ukf, 'ukf_cv')
        if isfield(ukf, 'model_fields')
            model_fields = ukf.model_fields;
        else
            % 兼容旧三模型 IMM。
            model_fields = {'ukf_cv', 'ukf_ct', 'ukf_ct_right'};
        end
        for i = 1:numel(model_fields)
            name = model_fields{i};
            if isfield(ukf, name)
                ukf.(name).dt = params.dt_sec;
                ukf.(name).initialized = true;
            end
        end
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
