% =========================================================================
% adapt_q.m — 通用自适应 Q 调整函数
% =========================================================================
% 整合模糊自适应 Q 和机动自适应 Q，支持乘法叠加融合策略。
% 可被 ukf_zishiying 和 ukf_imm（CV/CT 子模型）复用。
%
% 输入:
%   ukf    - UKF 结构体（含 nis_history, Q_base, Q_ema, params 等字段）
%   params - 参数结构体
%   mode   - 'zishiying'（含机动检测）或 'fuzzy_only'（仅模糊）
%
% 输出:
%   ukf    - 更新后的 UKF 结构体（Q 和 Q_ema 已调整）
% =========================================================================

function ukf = adapt_q(ukf, params, mode)
    % 通用自适应 Q 调整函数
    %
    % 整合模糊自适应 Q 和机动自适应 Q，支持乘法叠加融合策略。
    % 模糊自适应：基于 NIS 滑动平均的三角形隶属函数推理，输出 Q 缩放因子
    % 机动自适应：基于短/长时 NIS 趋势检测机动，触发 Q 脉冲增强
    % 两种策略乘法叠加：factor = factor_fuzzy * factor_maneuver
    %
    % 输入:
    %   ukf    - UKF 结构体（含 nis_history, Q_base, Q_ema, params 等字段）
    %   params - 参数结构体
    %   mode   - 'zishiying'（含机动检测）或 'fuzzy_only'（仅模糊）
    %
    % 输出:
    %   ukf    - 更新后的 UKF 结构体（Q 和 Q_ema 已调整）

    % 支持 nargin < 3 的调用方式，缺省模式设为含机动检测的全模式
    if nargin < 3
        mode = 'zishiying';
    end

    % ---- 初始化字段（防御性编程） ----
    % 确保 ukf 结构体中存在机动检测所需的状态字段，避免后续访问报错
    if ~isfield(ukf, 'maneuver_active'), ukf.maneuver_active = false; end
    if ~isfield(ukf, 'maneuver_counter'), ukf.maneuver_counter = 0; end
    if ~isfield(ukf, 'maneuver_recovery'), ukf.maneuver_recovery = 0; end
    % Q_ema 是 EMA（指数移动平均）平滑后的 Q 缩放因子，初始为 1.0（无缩放）
    if ~isfield(ukf, 'Q_ema') || isempty(ukf.Q_ema)
        ukf.Q_ema = 1.0;
    end
    % Q_base 是基础过程噪声协方差（未自适应调整前的原始值）
    if ~isfield(ukf, 'Q_base') || isempty(ukf.Q_base)
        ukf.Q_base = ukf.Q;
    end

    % ---- 基础检查：NIS 历史不足 3 帧或滤波器未成熟则跳过 ----
    % 自适应 Q 需要足够的 NIS 样本来做统计推断，NIS 是新息归一化平方值
    % 反映滤波器预测与实际量测之间的不一致程度
    if ~isfield(ukf, 'nis_history') || isempty(ukf.nis_history) || length(ukf.nis_history) < 3
        return;
    end

    % 滤波器需要至少 12 帧才启用自适应 Q，避免启动瞬态阶段误判
    % mature_frames 可通过 params.maneuver_mature_frames 自定义
    mature_frames = 12;
    if isfield(params, 'maneuver_mature_frames')
        mature_frames = params.maneuver_mature_frames;
    end
    % life_count 是滤波器存活帧数，未达成熟期直接返回
    if ~isfield(ukf, 'life_count') || ukf.life_count < mature_frames
        return;
    end

    nis_history = ukf.nis_history;
    if isfield(params, 'fuzzy_window_size') && ...
            numel(nis_history) > params.fuzzy_window_size
        nis_history = nis_history(end-params.fuzzy_window_size+1:end);
    end

    % ================================================================
    % 模糊自适应 Q：基于 NIS 滑动平均的三角形隶属函数推理
    % ================================================================
    % NIS 理论期望等于量测维数。
    nis_avg = mean(nis_history);
    measurement_dim = 3.0;
    if isfield(ukf, 'm') && isfinite(ukf.m) && ukf.m > 0
        measurement_dim = double(ukf.m);
    end
    nis_ratio = nis_avg / measurement_dim;

    % 五个三角形隶属函数：VerySmall, Small, Medium, Large, VeryLarge
    % 每个隶属函数将 nis_ratio 映射到 [0,1] 的置信度
    % trimf(a,b,c)：a 为起点（隶属度0），b 为峰值点（隶属度1），c 为终点（隶属度0）
    % 例如 mu_VS: nis_ratio ∈ [0, 0.4] 之间上升，>0.4 后降为 0
    mu_VS = trimf_val_adaptq(nis_ratio, 0.0, 0.0, 0.4);
    mu_S  = trimf_val_adaptq(nis_ratio, 0.2, 0.5, 0.8);
    mu_M  = trimf_val_adaptq(nis_ratio, 0.6, 1.0, 1.5);
    mu_L  = trimf_val_adaptq(nis_ratio, 1.3, 2.0, 3.0);
    mu_VL = trimf_val_adaptq(nis_ratio, 2.5, 4.0, 4.0);

    % 每个模糊输出对应的 Q 缩放因子
    % 这些值是领域专家根据经验设定的：NIS 越小说明滤波器过于保守，应减小 Q
    % NIS 越小说明滤波器过于激进，应增大 Q 以增加过程噪声容错
    out_Decrease       = 0.6;   % NIS 很小 → 降低 Q（信任滤波器预测准确）
    out_SlightDecrease = 0.8;   % NIS 偏小 → 小幅降低 Q
    out_Maintain       = 1.0;   % NIS 正常 → 保持 Q 不变
    out_Increase       = 1.8;   % NIS 偏大 → 提升 Q（增加过程噪声）
    out_RapidIncrease  = 3.0;   % NIS 很大 → 大幅提升 Q

    % 加权平均：Q_factor = Σ(μ_i * output_i) / Σ(μ_i)
    % 这是模糊推理的去模糊化步骤（重心法），将所有模糊规则的结论合并为一个 crisp 值
    total_mu = mu_VS + mu_S + mu_M + mu_L + mu_VL;
    if total_mu < 1e-10
        factor_fuzzy = 1.0;
    else
        factor_fuzzy = (mu_VS * out_Decrease + mu_S * out_SlightDecrease + ...
                       mu_M * out_Maintain + mu_L * out_Increase + ...
                       mu_VL * out_RapidIncrease) / total_mu;
    end

    % ---- 仅模糊模式：直接应用并返回 ----
    % 某些场景（如 IMM 中的 CT 模型）只需要模糊自适应，不需要机动检测
    if strcmp(mode, 'fuzzy_only')
        factor_raw = factor_fuzzy;
        ema_eta = 0.20;  % EMA 平滑系数，0.2 表示新值占 20%、旧值占 80%
        if isfield(params, 'fuzzy_ema_eta'), ema_eta = params.fuzzy_ema_eta; end
        % apply_q_factor_adaptq 将原始因子通过 EMA 平滑后应用到滤波器
        ukf = apply_q_factor_adaptq(ukf, factor_raw, ema_eta);
        return;
    end

    % ================================================================
    % 机动自适应 Q：短时 vs 长时 NIS 趋势检测
    % ================================================================
    % 机动检测通过比较"短时 NIS 均值"和"长时 NIS 均值"的趋势来判断
    % 是否发生了目标机动（加速度/转弯等），触发 Q 脉冲增强。
    % 原理：目标机动时，近期新息会突然增大，短时均值 >> 长时均值

    % 机动检测阈值（可调参数）
    nis_ratio_thresh = 1.10;    % 短时/长时 NIS 比率阈值
    nis_short_abs = 2.0;        % 短时 NIS 绝对阈值
    nis_long_abs = 2.5;         % 长时 NIS 绝对阈值
    recovery_frames = 6;        % 机动恢复帧数（连续无机动才退出）
    max_duration = 80;          % 机动窗口最大帧数（防卡死）

    % 从 params 覆盖默认阈值，允许外部配置
    if isfield(params, 'maneuver_nis_ratio'), nis_ratio_thresh = params.maneuver_nis_ratio; end
    if isfield(params, 'maneuver_nis_short_thresh'), nis_short_abs = params.maneuver_nis_short_thresh; end
    if isfield(params, 'maneuver_nis_long_thresh'), nis_long_abs = params.maneuver_nis_long_thresh; end
    if isfield(params, 'maneuver_recovery_frames'), recovery_frames = params.maneuver_recovery_frames; end
    if isfield(params, 'maneuver_max_duration'), max_duration = params.maneuver_max_duration; end

    % 计算短时和长时 NIS 均值
    % 短时窗口取最近 3 帧（捕捉突发机动），长时取全部历史（反映稳态水平）
    win_short = min(3, length(nis_history));
    nis_short = mean(nis_history(end-win_short+1:end));  % 最近 3 帧
    nis_long  = mean(nis_history);                        % 全部历史

    % 机动判定：两个条件满足其一即可
    % 条件1: 短时 >> 长时（趋势突变，说明刚发生机动）
    % 条件2: 长时绝对过大（持续不一致，说明目标一直在机动）
    maneuver_detected = false;
    if (nis_short > nis_long * nis_ratio_thresh && nis_short > nis_short_abs) || nis_long > nis_long_abs
        maneuver_detected = true;
    end

    % 状态机：maneuver_active 标记是否处于机动中
    % 这是一个有限状态机，防止抖动（短暂波动不触发机动）
    if ~ukf.maneuver_active
        % 未机动 → 检测到机动：进入机动状态
        if maneuver_detected
            ukf.maneuver_active = true;
            ukf.maneuver_counter = 0;      % 重置机动持续时间计数器
            ukf.maneuver_recovery = 0;     % 重置恢复帧计数器
        end
    else
        % 已在机动状态
        ukf.maneuver_counter = ukf.maneuver_counter + 1;
        if ~maneuver_detected
            % 机动消失，开始计数恢复帧
            ukf.maneuver_recovery = ukf.maneuver_recovery + 1;
        else
            % 机动仍在继续，重置恢复计数
            ukf.maneuver_recovery = 0;
        end
        % 连续 recovery_frames 帧无机动 → 退出机动状态（去抖）
        if ukf.maneuver_recovery >= recovery_frames
            ukf.maneuver_active = false;
            ukf.maneuver_counter = 0;
            ukf.maneuver_recovery = 0;
        end
        % 机动持续时间超过 max_duration → 强制退出（防卡死）
        if ukf.maneuver_counter > max_duration
            ukf.maneuver_active = false;
            ukf.maneuver_counter = 0;
            ukf.maneuver_recovery = 0;
        end
    end

    % 机动 Q 提升因子：渐进式增强（初始 → 中期 → 饱和）
    % 机动不是瞬间完成的，Q 的提升也应该渐进，避免帧间跳变
    q_boost_init = 2.0;  % 机动初始提升倍数
    q_boost_mid = 3.0;   % 机动中期提升倍数
    q_boost_max = 5.0;   % 机动饱和提升倍数（最大 5 倍）

    % 从 params 覆盖默认提升倍数
    if isfield(params, 'maneuver_q_boost_init'), q_boost_init = params.maneuver_q_boost_init; end
    if isfield(params, 'maneuver_q_boost_mid'), q_boost_mid = params.maneuver_q_boost_mid; end
    if isfield(params, 'maneuver_q_boost_max'), q_boost_max = params.maneuver_q_boost_max; end

    if ukf.maneuver_active
        % 渐进式增强：前 5 帧快速提升，5-15 帧缓慢提升，15 帧后饱和
        if ukf.maneuver_counter < 5
            maneuver_target = q_boost_init + ukf.maneuver_counter * 0.2;
        elseif ukf.maneuver_counter < 15
            maneuver_target = q_boost_mid + (ukf.maneuver_counter - 5) * 0.08;
        else
            maneuver_target = q_boost_max;
        end
        % 乘法叠加: 模糊负责基线校准, 机动负责脉冲增强
        % 两者相乘：如果模糊也认为 NIS 偏大，机动提升会被进一步放大
        factor_raw = factor_fuzzy * maneuver_target;
    else
        factor_raw = factor_fuzzy;
    end

    % EMA 平滑：避免 Q 因子帧间跳变
    % 使用指数移动平均将离散帧的 Q 因子平滑为连续变化的因子
    ema_eta = 0.20;
    if isfield(params, 'maneuver_ema_eta'), ema_eta = params.maneuver_ema_eta; end
    ukf = apply_q_factor_adaptq(ukf, factor_raw, ema_eta);
end


% =========================================================================
% apply_q_factor_adaptq — 将原始 Q 因子平滑落到滤波器状态
% =========================================================================
% 这个函数完成两件事：
%   1. 将原始因子限制在 [factor_min, factor_max] 范围内（防极端值）
%   2. 通过 EMA 平滑后更新 Q_ema，进而计算最终的 Q = Q_base * Q_ema
% =========================================================================
function ukf = apply_q_factor_adaptq(ukf, factor_raw, ema_eta)
    % 默认因子范围
    factor_min = 0.5;  % 最小降至原始值的 50%
    factor_max = 4.0;  % 最大升至原始值的 400%

    % 允许 params 覆盖默认范围
    if isfield(ukf, 'params')
        if isfield(ukf.params, 'adaptive_Q_min'), factor_min = ukf.params.adaptive_Q_min; end
        if isfield(ukf.params, 'adaptive_Q_max'), factor_max = ukf.params.adaptive_Q_max; end
    end

    % 钳位到允许范围
    factor_raw = max(factor_min, min(factor_max, factor_raw));
    % EMA 平滑：Q_ema 是平滑后的缩放因子，eta 控制响应速度
    % eta 越大，对新因子的响应越快；eta 越小，平滑效果越强
    ukf.Q_ema = ema_eta * factor_raw + (1 - ema_eta) * ukf.Q_ema;

    % 如果 Q_ema 接近 1.0（偏差 < 5%），说明自适应效果已回归基线
    % 此时直接将 Q 设回 Q_base，避免浮点累积误差
    if abs(ukf.Q_ema - 1.0) < 0.05
        ukf.Q = ukf.Q_base;
    else
        % 否则 Q = 基线 Q × 自适应缩放因子
        ukf.Q = ukf.Q_base * ukf.Q_ema;
    end
end


% =========================================================================
% trimf_val_adaptq — 三角形隶属函数求值
% =========================================================================
% 三角形隶属函数是模糊逻辑中最简单的隶属函数形式
% 参数 (a, b, c) 定义三角形的三个顶点：
%   a: 左底点（隶属度从 0 开始上升）
%   b: 顶点（隶属度 = 1.0）
%   c: 右底点（隶属度降回 0）
% 返回值 mu ∈ [0, 1]，表示输入 x 对该模糊集合的隶属度
% =========================================================================
function mu = trimf_val_adaptq(x, a, b, c)
    % 在三角形外部，隶属度为 0
    if x <= a || x >= c
        mu = 0;
    elseif x < b
        % 在上升沿：线性从 0 上升到 1
        mu = (x - a) / (b - a);
    else
        % 在下降沿：线性从 1 下降到 0
        mu = (c - x) / (c - b);
    end
end
