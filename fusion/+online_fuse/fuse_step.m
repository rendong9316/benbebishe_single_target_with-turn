% =========================================================================
% fuse_step.m — 逐帧航迹融合
% =========================================================================
%
% 【功能概述】
%   在线主链路 Stage 6 的逐帧融合入口。对一对同源局部航迹（站 1 / 站 2），
%   按 F.method 指定的算法（SCC / BC / CI / FCI）做单帧融合；
%   其中 BC 额外维护跨帧互协方差 P12（唯一的有状态部分）。
%
% 【调用方式】
%   [x, P, F] = online_fuse.fuse_step(F, est1, est2, k, params);
%
% 【输入参数】
%   F     — online_fuse.init 创建的融合状态
%   est1  — 站 1 的当前帧估计，字段：
%             x       — [4x1] 状态估计（必填）
%             P       — [4x4] 误差协方差（必填）
%             id      — 局部航迹 ID（可选，用于配对键）
%             P_pred  — [4x4] 预测协方差（可选，用于收缩因子 α）
%             Q       — [4x4] 过程噪声（可选，用于互协方差注入）
%   est2  — 站 2 的当前帧估计，字段同上
%   k     — 当前帧号
%   params— 参数结构体，额外可用字段：
%             dt_sec           — 帧间隔（s），默认 30
%             bc_min_p12_scale — BC 互协方差保底相关度，默认 0.05
%
% 【输出参数】
%   x — [4x1] 融合后的状态估计
%   P — [4x4] 融合后的误差协方差
%   F — 更新后的融合状态
%
% 【BC 的跨帧互协方差递推】（契约 4.2 节）
%   P12(k) = F·P12(k-1)·F' + 0.5·Q
%   P12(k) = α1·α2·P12(k)      αi = sqrt(tr(P_i) / tr(P_i,pred))
%   对角元钳制：P12(i,i) 不超过 0.8·min(P1(i,i), P2(i,i))
%   保底缩放：归一化相关度不得低于 bc_min_p12_scale，
%             防止 P12 被压到 0 而使 BC 退化成 SCC。
%
%   帧间断（k 与该配对上次融合帧号不连续）时，P12 直接置零：
%   断连期间两站的误差相关性无法延续，强行外推只会放大错误。
%
% 【逐配对状态】
%   融合中心同一时刻可能有多条系统航迹，每条对应一对局部航迹。
%   P12 是"这一对"的属性，不同配对之间不能共用，否则会互相污染，
%   因此按配对键分开存放（F.pairs）；F.P12 只是最近一次融合的快照，
%   供诊断与单元测试使用。
%
% 【相关文档】
%   docs/主链路在线化重构契约_20261028.md 第 4 节
%   双站航迹配对与融合在线处理模型_20261028.md
% =========================================================================

function [x, P, F] = fuse_step(F, est1, est2, k, params)
    % -----------------------------------------------------------------
    % 参数与状态兜底
    % -----------------------------------------------------------------
    if nargin < 5 || isempty(params)
        params = struct();
    end
    if isempty(F) || ~isfield(F, 'method')
        error('online_fuse:fuse_step:badState', ...
            'F 必须是 online_fuse.init 创建的融合状态');
    end
    if isempty(est1) || isempty(est2)
        error('online_fuse:fuse_step:badEstimate', ...
            'est1 与 est2 均不能为空（单源透传由调用方处理）');
    end

    % -----------------------------------------------------------------
    % 取出两站的状态与协方差
    % -----------------------------------------------------------------
    x1 = double(est1.x(:));
    P1 = double(est1.P);
    x2 = double(est2.x(:));
    P2 = double(est2.P);
    if numel(x1) < 4 || numel(x2) < 4
        error('online_fuse:fuse_step:badEstimate', '状态估计维度必须为 4');
    end
    if any(size(P1) < 4) || any(size(P2) < 4)
        error('online_fuse:fuse_step:badEstimate', '协方差矩阵必须为 4x4');
    end

    % -----------------------------------------------------------------
    % 按算法分派
    % -----------------------------------------------------------------
    switch F.method
        case 'SCC'
            [x, P] = online_fuse.algorithms.scc(x1, P1, x2, P2);

        case 'BC'
            [P12, F] = bc_p12_step(F, est1, est2, x1, x2, P1, P2, k, params);
            [x, P] = online_fuse.algorithms.bc(x1, P1, x2, P2, P12);

        case 'CI'
            [x, P] = online_fuse.algorithms.ci(x1, P1, x2, P2);

        case 'FCI'
            [x, P] = online_fuse.algorithms.fci(x1, P1, x2, P2);

        otherwise
            error('online_fuse:fuse_step:badMethod', ...
                '未知融合算法：%s', F.method);
    end

    F.n_fuse = F.n_fuse + 1;
end

% =====================================================================
% BC 的跨帧互协方差递推
% =====================================================================
function [P12, F] = bc_p12_step(F, est1, est2, x1, x2, P1, P2, k, params)
%BC_P12_STEP 按契约 4.2 节递推 BC 的互协方差 P12
%
%   帧间断判定用的是该配对自己的 last_k，而不是全局帧号：
%   某一对中断不代表其它对也中断。

    dt = online_fuse.get_param(params, 'dt_sec', 30);
    min_scale = online_fuse.get_param(params, 'bc_min_p12_scale', 0.05);

    % CV 状态转移（与 tracker / central_track_manager 保持一致）
    Fst = [1, dt, 0, 0; 0, 1, 0, 0; 0, 0, 1, dt; 0, 0, 0, 1];

    % -----------------------------------------------------------------
    % 定位该配对的状态
    % -----------------------------------------------------------------
    key = online_fuse.pair_key(est1, est2);
    idx = find_pair(F.pairs, key);
    if idx > 0
        st = F.pairs{idx};
    else
        st = struct();
        st.key = key;
        st.P12 = zeros(4, 4);
        st.last_k = NaN;
        st.P1_prev = [];
        st.P2_prev = [];
    end

    % -----------------------------------------------------------------
    % 帧间断 / 首帧判定
    % -----------------------------------------------------------------
    fresh = ~isfinite(st.last_k);
    broken = (~fresh) && (k ~= st.last_k + 1);

    if fresh || broken
        % 首帧没有可继承的互协方差；断连期间两站误差相关性无法延续。
        % 两种情况都从零开始重建：broken 时按契约要求把 P12 置零。
        P12 = zeros(4, 4);
        if broken
            F.n_reset = F.n_reset + 1;
        end
    else
        % -------------------------------------------------------------
        % 1) 过程噪声 Q：优先用估计自带的 Q，否则按参数推算
        % -------------------------------------------------------------
        Q = shared_process_noise(est1, est2, params, dt, x1, x2);

        % -------------------------------------------------------------
        % 2) 收缩因子 αi = sqrt(tr(P_i) / tr(P_i,pred))
        %    P_i,pred 优先取估计自带的 P_pred；没有时用上一帧协方差外推
        % -------------------------------------------------------------
        Ppred1 = predicted_cov(est1, st.P1_prev, Fst, Q);
        Ppred2 = predicted_cov(est2, st.P2_prev, Fst, Q);
        alpha1 = shrink_factor(P1, Ppred1);
        alpha2 = shrink_factor(P2, Ppred2);

        % -------------------------------------------------------------
        % 3) 递推：P12 = α1·α2·(F·P12·F' + 0.5·Q)
        % -------------------------------------------------------------
        P12 = alpha1 * alpha2 * (Fst * st.P12 * Fst' + 0.5 * Q);

        % -------------------------------------------------------------
        % 4) 归一化 + 保底缩放 + Cauchy–Schwarz 裁剪 + 对角钳制
        % -------------------------------------------------------------
        P12 = online_fuse.p12_finalize(P12, P1, P2, min_scale);
    end

    % -----------------------------------------------------------------
    % 写回配对状态
    % -----------------------------------------------------------------
    st.P12 = P12;
    st.last_k = k;
    st.P1_prev = P1;
    st.P2_prev = P2;
    if idx > 0
        F.pairs{idx} = st;
    else
        F.pairs{end + 1} = st;
    end

    F.P12 = P12;
    F.last_k = k;
end

% =====================================================================
% 辅助函数
% =====================================================================
function idx = find_pair(pairs, key)
%FIND_PAIR 在配对状态数组中查找配对键对应的下标
    idx = 0;
    for i = 1:numel(pairs)
        if strcmp(pairs{i}.key, key)
            idx = i;
            return;
        end
    end
end

function Q = shared_process_noise(est1, est2, params, dt, x1, x2)
%SHARED_PROCESS_NOISE 取两站共享的过程噪声
%   优先用估计自带的 Q（站 1 优先），否则按加速度谱密度与当前纬度推算。
    Q = [];
    if isfield(est1, 'Q') && ~isempty(est1.Q)
        Q = double(est1.Q);
    elseif isfield(est2, 'Q') && ~isempty(est2.Q)
        Q = double(est2.Q);
    else
        lat = 0.5 * (x1(3) + x2(3));
        Q = online_fuse.process_noise_geo(params, lat, dt);
    end
    Q = 0.5 * (Q + Q');
end

function Ppred = predicted_cov(est, P_prev, Fst, Q)
%PREDICTED_COV 取当前帧的预测协方差
%   估计自带 P_pred 时直接使用；否则用上一帧后验协方差外推加过程噪声。
    if isfield(est, 'P_pred') && ~isempty(est.P_pred)
        Ppred = double(est.P_pred);
    elseif ~isempty(P_prev)
        Ppred = Fst * P_prev * Fst' + Q;
    else
        Ppred = [];
    end
end

function alpha = shrink_factor(P, Ppred)
%SHRINK_FACTOR 量测更新对协方差的收缩因子 α = sqrt(tr(P)/tr(P_pred))
%   取值裁剪到 [0,1]：收缩因子不应大于 1。
    if isempty(Ppred) || ~all(isfinite(Ppred(:))) || trace(Ppred) <= 0
        alpha = 1;
        return;
    end
    alpha = sqrt(max(trace(P), 0) / trace(Ppred));
    alpha = min(max(alpha, 0), 1);
end
