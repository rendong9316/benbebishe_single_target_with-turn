% =========================================================================
% fuse_estimate_sequence.m — 动态航迹组的四算法融合评估模块
% =========================================================================
%
% 【功能概述】
%   针对一个动态航迹组（group），分别用 SCC / BC / CI / FCI 四种融合
%   算法执行逐帧融合，并统计每种方法的覆盖帧数和 RMSE，最终选出最佳
%   融合算法。
%
%   典型使用场景：片段研究（fragment study）中，同一个目标在不同雷达
%   上被切分为多个航迹片段，需要将这些片段合并为一条完整航迹。本函数
%   对每个片段组（group）依次调用四种融合算法，评估哪种算法给出的
%   融合结果最精确。
%
% 【输入参数】
%   group   — struct，航迹组信息，包含：
%             .group_id        — 组 ID
%             .segment_indices — 属于该组的片段索引列表
%   segments — struct 数组，每个元素是一个航迹片段：
%              .segment_id     — 片段 ID
%              .radar_id       — 所属雷达编号
%              .states         — [4 x n_frames] 状态序列
%              .covariances    — [4 x 4 x n_frames] 协方差序列
%              .pred_covariances — [4 x 4 x n_frames] 预测协方差
%              .process_noises   — [4 x 4 x n_frames] 过程噪声
%              .effective_frames — 有效帧号列表
%              .raw_frames       — 原始帧号映射
%              .online_end_frame — 在线结束帧号
%   params  — 仿真参数结构体，包含 .dt_sec 等字段
%
% 【输出】
%   result — struct，包含四种算法的融合结果和评估指标：
%     .group_id         — 组 ID
%     .methods.(i).method      — 算法名称 ('SCC'/'BC'/'CI'/'FCI')
%     .methods.(i).snapshots   — 融合后的逐帧快照
%     .methods.(i).coverage_frames — 覆盖帧数
%     .methods.(i).rmse_km     — 融合 RMSE（km）
%     .best_method             — 最佳算法名称
%     .best_rmse_km            — 最佳 RMSE
%
% 【调用关系】
%   被 fragment_study 或类似的上层评估脚本调用
%   内部调用: fuse_method(), estimates_at_frame(), track_fusion_algorithms()
% =========================================================================

function result = fuse_estimate_sequence(group, segments, params)
    % FUSE_ESTIMATE_SEQUENCE 对一个动态 group 分别执行 SCC/BC/CI/FCI。
    % 遍历四种算法名称，逐一调用 fuse_method 执行融合，
    % 并将结果存入 result.methods 结构体
    methods = {'SCC','BC','CI','FCI'};

    % 初始化结果结构体：
    % .group_id — 组 ID，透传输入
    % .methods  — 空结构体，后续填充四种算法的结果
    % .best_method — 空字符串，后续填入最佳算法名
    % .best_rmse_km — inf，后续被更小的 RMSE 替换
    result = struct('group_id', group.group_id, 'methods', struct([]), 'best_method', '', 'best_rmse_km', inf);

    % 遍历四种融合算法，逐一执行
    for m = 1:numel(methods)
        % 调用 fuse_method 执行具体融合，返回逐帧融合快照
        snapshots = fuse_method(group, segments, params, methods{m});

        % 将融合结果存入 result.methods(m)
        result.methods(m).method = methods{m};
        result.methods(m).snapshots = snapshots;

        % 统计有效融合帧数（有融合结果的帧数）
        result.methods(m).coverage_frames = count_fused_frames(snapshots);

        % RMSE 暂时设为 NaN，后续由评估脚本计算填充
        result.methods(m).rmse_km = NaN;
    end
end

% =========================================================================
% fuse_method — 对单个 group 执行指定融合算法的逐帧融合
% =========================================================================
% 根据给定的融合算法（method），遍历 group 中所有片段的在线帧范围，
% 逐帧提取各片段的 UKF 估计，执行融合，并输出逐帧融合快照。
%
% BC 方法需要跨帧维护互协方差 P12，因此函数内部包含 P12 的预测和更新逻辑。
% 单源帧（只有一个片段有数据）直接透传，不执行融合。
%
% 【输入参数】
%   group    — 航迹组信息
%   segments — 片段数组
%   params   — 仿真参数
%   method   — 融合算法名称字符串 ('SCC'/'BC'/'CI'/'FCI')
%
% 【输出】
%   snapshots — [n_frames x 1] cell，每帧一个融合快照
% =========================================================================

function snapshots = fuse_method(group, segments, params, method)
    % 找到该组所有片段中最大的在线结束帧号，确定需要融合的帧总数
    n_frames = max([segments(group.segment_indices).online_end_frame]);

    % 预分配输出 cell 数组
    snapshots = cell(n_frames, 1);

    % --- BC 方法专用状态初始化 ---
    % P12: [4x4] 互协方差矩阵，跨帧递归维护
    % has_bc: 布尔标志，指示 P12 是否已有有效值
    % last_frame: 上一帧的帧号，用于检测帧间断（断连后需重置 P12）
    P12 = zeros(4);
    has_bc = false;
    last_frame = NaN;

    % 获取属于该组的所有片段
    members = segments(group.segment_indices);

    % 逐帧执行融合
    for k = 1:n_frames
        % 从各片段中提取当前帧 k 的状态估计
        estimates = estimates_at_frame(members, k);

        % 初始化本帧融合快照
        snap = struct('frameID', k, 'trackList', {{}});

        % --- 根据本帧有效估计的数量分流处理 ---
        if numel(estimates) >= 2
            % 情况A：两源及以上有数据，执行融合

            if strcmp(method, 'BC')
                % --- BC 方法：维护互协方差 P12 ---

                % 检测帧间断：如果 has_bc 为假，或者当前帧不是上一帧的下一帧
                % （说明中间有漏检或断连），重置 P12 为零矩阵
                if ~has_bc || k ~= last_frame + 1
                    P12 = zeros(4);
                else
                    % 正常连续帧：更新 P12（预测 + 收缩 + 稳定性约束）
                    P12 = update_cross_covariance(P12, estimates, params);
                end

                % 调用 BC 融合算法，传入互协方差 P12
                % 注意：estimates 可能有 >2 个元素，但 BC 只取前两个
                [x, P] = track_fusion_algorithms(method, estimates(1).x, estimates(1).P, estimates(2).x, estimates(2).P, P12);
                has_bc = true;

            elseif strcmp(method, 'SCC')
                % SCC 简单凸组合：直接融合前两个估计
                % 假设两个传感器估计独立，信息矩阵相加
                [x, P] = track_fusion_algorithms(method, estimates(1).x, estimates(1).P, ...
                    estimates(2).x, estimates(2).P);

            else
                % CI 和 FCI：协方差交叉及其快速版本
                % CI 通过优化权重 w 最小化 det(P_fused)
                % FCI 用迹的倒数之比解析计算 w
                % 忽略第三个输出（权重），只取融合状态和协方差
                [x, P, ~] = track_fusion_algorithms(method, estimates(1).x, estimates(1).P, ...
                    estimates(2).x, estimates(2).P);
            end

            source = 'both';  % 标记为双源融合

        elseif numel(estimates) == 1
            % 情况B：仅单源有数据，直接透传该源估计
            % source 标记为 'R1_only' 或 'R2_only'，取决于雷达编号
            x = estimates(1).x;
            P = estimates(1).P;
            source = sprintf('R%d_only', estimates(1).radar_id);

            % 单源数据会中断 BC 的 P12 连续性，重置 has_bc 标志
            if strcmp(method, 'BC')
                has_bc = false;
            end

        else
            % 情况C：两源均无数据（都漏检了）
            % 跳过本帧融合，保存空快照
            if strcmp(method, 'BC')
                has_bc = false;  % 重置 BC 连续性标志
            end
            snapshots{k} = snap;
            continue;
        end

        % --- 构建融合后的航迹结构体 ---
        % 将融合结果打包为航迹结构体，写入本帧快照
        % 从状态向量 x=[lon; v_lon; lat; v_lat] 中提取经纬度
        trk = struct('id', group.group_id, 'group_id', group.group_id, 'lat', x(3), 'lon', x(1), ...
            'ukf', struct('x', x, 'P', P), 'source', source, 'segment_ids', [members.segment_id]);

        % 将融合航迹写入本帧快照的 trackList
        snap.trackList{1} = trk;
        snapshots{k} = snap;

        % 记录当前帧号，用于 BC 的帧连续性检测
        last_frame = k;
    end
end

% =========================================================================
% estimates_at_frame — 从各片段中提取指定帧的状态估计
% =========================================================================
% 遍历 segments 中的每个片段，检查当前帧 frame 是否在该片段的有效帧
% 范围内。如果是，则从该片段的 states/covariances 等字段中提取该帧
% 对应的状态估计、协方差、预测协方差和过程噪声。
%
% 【输入参数】
%   segments — 片段数组
%   frame    — 当前帧号
%
% 【输出】
%   estimates — struct 数组，每个元素包含：
%     .radar_id     — 雷达编号
%     .x            — [4x1] 状态向量
%     .P            — [4x4] 协方差矩阵
%     .P_pred       — [4x4] 预测协方差
%     .Q            — [4x4] 过程噪声
% =========================================================================

function estimates = estimates_at_frame(segments, frame)
    % 预定义输出结构体的字段模板
    estimates = struct('radar_id', {}, 'x', {}, 'P', {}, 'P_pred', {}, 'Q', {});

    % 遍历每个片段
    for i = 1:numel(segments)
        % 如果当前帧不在该片段的有效帧范围内，跳过
        if ~ismember(frame, segments(i).effective_frames), continue; end

        % 找到当前帧在该片段原始帧号列表中的索引
        raw_idx = find(segments(i).raw_frames == frame, 1);

        % 从片段数据中提取当前帧的状态估计和协方差
        estimates(end+1) = struct('radar_id', segments(i).radar_id, ...
            'x', segments(i).states(:, raw_idx), 'P', segments(i).covariances(:, :, raw_idx), ...
            'P_pred', segments(i).pred_covariances(:, :, raw_idx), ...
            'Q', segments(i).process_noises(:, :, raw_idx)); %#ok<AGROW>
    end
end

% =========================================================================
% update_cross_covariance — BC 方法中互协方差 P12 的跨帧更新
% =========================================================================
% 对互协方差 P12 执行三步操作：
%   1. 预测：用 CV 模型将 P12 从上一帧传播到当前帧
%   2. 收缩：用 UKF 量测更新的迹收缩比压缩 P12
%   3. 稳定性约束：钳制 P12 对角元，防止相关性过强导致数值不稳定
%
% 【输入参数】
%   P12      — [4x4] 上一帧的互协方差矩阵
%   estimates — struct 数组，包含两个及以上传感器的估计
%   params   — 仿真参数（包含 .dt_sec）
%
% 【输出】
%   P12 — [4x4] 更新后的互协方差矩阵
% =========================================================================

function P12 = update_cross_covariance(P12, estimates, params)
    % 从参数中获取时间步长
    dt = params.dt_sec;

    % 构造 CV 模型状态转移矩阵
    F = [1 dt 0 0; 0 1 0 0; 0 0 1 dt; 0 0 0 1];

    % 预测步：P12_pred = F * P12 * F' + 0.5 * Q
    % 0.5 是因为两个传感器共享同一过程噪声源，互协方差中只计入一半
    P12 = F * P12 * F' + 0.5 * estimates(1).Q;

    % 计算两个传感器的迹收缩比（UKF 量测更新对协方差的压缩程度）
    alpha1 = covariance_contraction(estimates(1));
    alpha2 = covariance_contraction(estimates(2));

    % 用收缩比压缩 P12：alpha1 * alpha2 是两个传感器各自的压缩效果相乘
    % 钳制 alpha 在 [0.1, 1.0] 范围内，防止过度收缩或不变
    P12 = max(0.1, min(1, alpha1 * alpha2)) * P12;

    % --- 稳定性约束 ---
    % 限制 P12 对角元不超过 0.8 * min(diag(P1), diag(P2))
    % 如果互协方差对角元过大，说明两个估计在该维度上高度相关，
    % 可能导致融合结果数值不稳定。对超标的行/列进行等比例缩放
    limit = 0.8 * min([diag(estimates(1).P), diag(estimates(2).P)], [], 2);

    for i = 1:4
        % 如果第 i 个对角元超出限制
        if P12(i,i) > limit(i)
            % 计算缩放比例：sqrt(限制值 / 当前值)
            % 使用 sqrt 是因为 P12 出现在二次型中，对角元的缩放比例
            % 是行/列缩放比例的平方
            scale = sqrt(limit(i) / max(P12(i,i), eps));
            % 对第 i 行和第 i 列同时进行等比例缩放
            P12(i,:) = P12(i,:) * scale;
            P12(:,i) = P12(:,i) * scale;
        end
    end
end

% =========================================================================
% covariance_contraction — 计算 UKF 量测更新后的协方差迹收缩比
% =========================================================================
% 迹收缩比 alpha = sqrt(trace(P_post) / trace(P_pred))
% 表示 UKF 量测更新将协方差"压缩"了多少倍。
% alpha < 1 表示量测更新降低了不确定度，alpha 越小压缩越剧烈。
%
% 【输入参数】
%   estimate — struct，包含 .P（后验协方差）和 .P_pred（预测协方差）
%
% 【输出】
%   alpha — 标量，迹收缩比
% =========================================================================

function alpha = covariance_contraction(estimate)
    % 如果预测协方差迹大于阈值，正常计算收缩比
    if trace(estimate.P_pred) > 1e-10
        % alpha = sqrt(trace(P_post) / trace(P_pred))
        % max(1e-6, ...) 防止 trace 为零时除零
        alpha = sqrt(max(1e-6, trace(estimate.P) / trace(estimate.P_pred)));
    else
        % 预测协方差迹过小（几乎确定），使用保守的 sqrt(0.5) 作为默认收缩比
        alpha = sqrt(0.5);
    end
end

% =========================================================================
% count_fused_frames — 统计融合快照中有数据的帧数
% =========================================================================
% 遍历 snapshots cell 数组，统计 trackList 非空的帧数。
% 用于评估每种融合算法的覆盖能力。
%
% 【输入参数】
%   snapshots — [n_frames x 1] cell，每帧一个融合快照
%
% 【输出】
%   n — 有融合结果的帧数
% =========================================================================

function n = count_fused_frames(snapshots)
    n = 0;
    % 遍历所有帧，统计有效融合帧数
    for k = 1:numel(snapshots)
        % 只有当本帧快照非空且 trackList 非空时才计数
        if ~isempty(snapshots{k}) && ~isempty(snapshots{k}.trackList), n = n + 1; end
    end
end
