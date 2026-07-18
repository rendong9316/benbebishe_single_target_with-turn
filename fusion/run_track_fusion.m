% =========================================================================
% run_track_fusion.m — 逐帧航迹融合主循环
% =========================================================================
%
% 【功能概述】
%   对已匹配的雷达1 (R1) 和雷达2 (R2) 航迹对，在统一时间网格上逐帧
%   执行融合。支持四种经典融合算法：SCC、BC、CI、FCI。
%
%   融合策略：
%     - 双源均有数据：执行所选融合算法，将两源估计合并为一条融合航迹
%     - 仅单源有数据：直接透传该源结果，不做融合（标记为 R1_only 或 R2_only）
%     - 两源均无数据：跳过本帧本对
%
% 【数学原理】
%   本函数是融合框架的主调度器，自身不执行具体的融合数学运算，而是按帧、
%   按时序调用各具体融合算法（通过 track_fusion_algorithms 调度）。
%
%   BC 方法的特殊性：需要在时间递推中维护互协方差矩阵 P12，包括：
%     1. 预测步：用 CV 模型对 P12 进行时间传播（计入 1/2 Q）
%     2. 更新步：用迹收缩比近似 UKF 更新中 (I-KH) 对 P12 的压缩
%     3. 稳定性约束：将 P12 的对角元钳制在 0.8*min(diag(P1)) 以内
%
%   状态空间模型：目标状态 x = [lon; v_lon; lat; v_lat]
%   即经度、经度方向速度、纬度、纬度方向速度的四维向量。
%
% 【输入参数】
%   matched_pairs     - 结构体数组，每项含 R1_track_id 和 R2_track_id
%   trackSnapshots_R1 - [n_frames x 1] cell，R1 每帧的航迹快照
%   aligned_R2        - [n_frames x 1] cell，时间对齐后的 R2 航迹快照
%   params            - 仿真参数结构体（至少包含 dt_sec 字段）
%   method            - 字符串，融合算法选择：
%                       'SCC' 简单凸组合    'BC'  Bar-Shalom-Campo
%                       'CI'  协方差交叉    'FCI' 快速协方差交叉
%
% 【输出】
%   fused_snapshots - [n_frames x 1] cell，每帧的融合航迹快照结构体
%     每个 fused_snap 包含：
%       frameID   - 帧编号
%       trackList - cell 数组，每项为融合后的单条航迹结构体，含字段：
%         id, r1_id, r2_id  — 融合航迹ID及源航迹ID
%         lat, lon          — 融合后的经纬度位置
%         ukf_x, ukf_P      — 融合后的 UKF 状态和协方差
%         source            — 数据来源: 'both'/'R1_only'/'R2_only'
%         w                 — 融合权重（仅 CI/FCI/SCC 有）
%         P12               — 互协方差矩阵（仅 BC 有）
%
% 【调用关系】
%   被仿真主程序调用（通常在 run_simulation.m 或等效的顶层脚本中）
%   内部调用: track_fusion_algorithms（调度 fuse_scc / fuse_ci / fuse_fci / fuse_bc）
%   内部调用: find_track() 辅助函数在航迹列表中按 ID 查找
%   内部调用: regularize_cov() 对协方差进行正则化
% =========================================================================

function fused_snapshots = run_track_fusion(matched_pairs, trackSnapshots_R1, ...
        aligned_R2, params, method)

    % -----------------------------------------------------------------
    % 获取基本参数
    % n_frames: 总帧数，由 R1 的快照数量决定（R2 已通过 time_align_tracks
    %           对齐到 R1 的时间网格，因此两者帧数一致）
    % n_pairs:  成功匹配的航迹对数量，决定了需要融合的航迹组数
    % -----------------------------------------------------------------
    n_frames = length(trackSnapshots_R1);  % 总帧数
    n_pairs = length(matched_pairs);       % 匹配的航迹对数

    % -----------------------------------------------------------------
    % 建立航迹ID到pair索引的快速查找映射
    % 使用 containers.Map 实现 O(1) 时间复杂度的查找，避免在每帧循环
    % 中逐条遍历 matched_pairs 数组来寻找对应关系
    %
    % r1_to_pair(r1_id) = p  表示 R1 中 ID 为 r1_id 的航迹属于第 p 对
    % r2_to_pair(r2_id) = p  表示 R2 中 ID 为 r2_id 的航迹属于第 p 对
    %
    % 注意：当前实现中这个映射实际上未被使用（主循环直接遍历 matched_pairs），
    % 但保留它为未来扩展（如单源航迹的自动配对）预留接口
    % -----------------------------------------------------------------
    r1_to_pair = containers.Map('KeyType', 'int32', 'ValueType', 'int32');
    r2_to_pair = containers.Map('KeyType', 'int32', 'ValueType', 'int32');
    for p = 1:n_pairs
        r1_to_pair(matched_pairs(p).R1_track_id) = p;
        r2_to_pair(matched_pairs(p).R2_track_id) = p;
    end

    % -----------------------------------------------------------------
    % BC 方法专用初始化：为每对航迹维护互协方差矩阵 P12
    %
    % Bar-Shalom-Campo (BC) 融合算法需要知道两个传感器估计之间的
    % 互协方差 P12，这是它区别于 SCC/CI/FCI 的核心特征。
    % P12 描述了两个传感器估计误差的相关性（源于共同的过程噪声、
    % 公共先验等）。
    %
    % P12 的维护是一个跨帧的递归过程：
    %   - 首帧：P12 = 0（无先验相关性信息）
    %   - 后续帧：P12 = f(P12_prev, F_cv, Q, α)
    %     其中 F_cv 是 CV 模型的状态转移矩阵，Q 是过程噪声，
    %     α 是 UKF 量测更新引起的迹收缩比
    %
    % P12_cell{p} — 第 p 对航迹的互协方差矩阵 (4x4)
    % has_prev(p) — 布尔标志，指示第 p 对是否已有前一帧的有效 P12
    %               初始为 false，首帧融合后设为 true
    % -----------------------------------------------------------------
    if strcmp(method, 'BC')
        P12_cell = cell(n_pairs, 1);   % 存储每对航迹的互协方差矩阵
        for p = 1:n_pairs
            P12_cell{p} = zeros(4, 4);  % 初始化为零矩阵（无先验相关性）
        end
        has_prev = false(n_pairs, 1);   % 初始均无历史 P12
    end

    % -----------------------------------------------------------------
    % 预分配融合快照输出 cell 数组
    % fused_snapshots{k} 存储第 k 帧的融合结果
    % 每帧包含一个结构体，其中有 frameID 和 trackList 字段
    % -----------------------------------------------------------------
    fused_snapshots = cell(n_frames, 1);

    % =================================================================
    % 主循环：逐帧、逐对执行融合
    % 外层循环遍历时间帧，内层循环遍历匹配的航迹对
    % 这种嵌套结构确保了每帧中对每对航迹独立执行融合
    % =================================================================
    for k = 1:n_frames
        % 获取本帧两个雷达的航迹快照
        snap_r1 = trackSnapshots_R1{k};
        snap_r2 = aligned_R2{k};

        % 初始化本帧融合快照结构体
        % trackList 初始为空 cell，后续逐对填充
        fused_snap = struct('frameID', k, 'trackList', {{}});

        % 遍历每对匹配航迹，执行融合
        for p = 1:n_pairs
            r1_id = matched_pairs(p).R1_track_id;
            r2_id = matched_pairs(p).R2_track_id;

            % ---------------------------------------------------------
            % 在本帧快照中按 ID 查找各传感器的航迹
            % find_track 遍历 snap.trackList 找到 id 匹配的航迹结构体
            % 如果未找到，返回空数组 []
            % ---------------------------------------------------------
            trk1 = find_track(snap_r1, r1_id);  % 查找 R1 航迹
            trk2 = find_track(snap_r2, r2_id);  % 查找 R2 航迹

            % ---------------------------------------------------------
            % 过滤无效 UKF 状态
            % 即使找到了航迹结构体，其 ukf 字段也可能为空（如航迹刚
            % 起始、正在终止、或数据损坏等情况）。只有 ukf.x 非空才
            % 视为有效的状态估计，才能参与融合
            %
            % 判断逻辑：
            %   1. trk 非空（找到了航迹结构体）
            %   2. trk.ukf 非空（有 UKF 对象）
            %   3. trk.ukf 包含 'x' 字段（有状态向量）
            %   4. trk.ukf.x 非空（状态向量有值）
            % 以上任一条件不满足，就将 trk 置为空数组
            % ---------------------------------------------------------
            if ~isempty(trk1) && (isempty(trk1.ukf) || ~isfield(trk1.ukf,'x') || isempty(trk1.ukf.x))
                trk1 = [];
            end
            if ~isempty(trk2) && (isempty(trk2.ukf) || ~isfield(trk2.ukf,'x') || isempty(trk2.ukf.x))
                trk2 = [];
            end

            % ---------------------------------------------------------
            % 两源均无有效数据，跳过本帧本对
            % 这种情况可能发生在：
            %   - 该航迹对在某帧中两个雷达都漏检了
            %   - 该航迹对在某帧中一个雷达刚起始（ukf.x 尚未填充）
            %   - 航迹正在终止过程中
            % ---------------------------------------------------------
            if isempty(trk1) && isempty(trk2)
                continue;
            end

            % ---------------------------------------------------------
            % 初始化融合航迹结构体
            % 先填充元数据字段（ID、来源标识），再根据实际数据情况
            % 填充融合结果
            % ---------------------------------------------------------
            fused_trk = struct();
            fused_trk.id = p;          % 融合航迹ID = pair 索引（全局唯一）
            fused_trk.r1_id = r1_id;   % 记录原始 R1 航迹 ID（溯源用）
            fused_trk.r2_id = r2_id;   % 记录原始 R2 航迹 ID（溯源用）

            if ~isempty(trk1) && ~isempty(trk2)
                % =================================================
                % 情况1：两源均有有效数据 —— 执行双源融合
                % =================================================
                % 这是融合的核心场景。从两个传感器的 UKF 中提取
                % 状态估计 (x1, x2) 和协方差 (P1, P2)，然后
                % 根据 method 参数选择对应的融合算法

                % 提取状态向量和协方差矩阵
                % x1/P1: R1 的状态估计和误差协方差
                % x2/P2: R2 的状态估计和误差协方差
                x1 = trk1.ukf.x;  P1 = trk1.ukf.P;
                x2 = trk2.ukf.x;  P2 = trk2.ukf.P;

                % 根据 method 参数选择融合算法
                switch upper(method)
                    case 'SCC'
                        % 简单凸组合 (Simple Convex Combination)
                        % 假设两个传感器估计独立（互协方差为零），
                        % 直接将信息矩阵相加：P_fused^{-1} = P1^{-1} + P2^{-1}
                        % 这是最简单、最快的融合方式，但前提是独立性假设成立
                        [x_f, P_f] = track_fusion_algorithms('SCC', x1, P1, x2, P2);
                        fused_trk.w = 0.5;  % SCC 隐含 w=0.5 等权假设

                    case 'CI'
                        % 协方差交叉 (Covariance Intersection)
                        % 不需要知道互协方差 P12，通过优化权重 w 最小化
                        % 融合协方差的行列式 det(P_fused)，保证融合结果
                        % 不会过度自信（conservative bound）
                        [x_f, P_f, w_opt] = track_fusion_algorithms('CI', x1, P1, x2, P2);
                        fused_trk.w = w_opt;

                    case 'FCI'
                        % 快速协方差交叉 (Fast Covariance Intersection)
                        % CI 的简化版本，用协方差迹的倒数之比直接计算权重，
                        % 无需迭代优化，计算效率高
                        [x_f, P_f, w_fci] = track_fusion_algorithms('FCI', x1, P1, x2, P2);
                        fused_trk.w = w_fci;

                    case 'BC'
                        % -------------------------------------------------
                        % Bar-Shalom-Campo (BC) 融合
                        % 这是四种算法中最复杂的一种，因为它需要维护
                        % 跨帧的互协方差矩阵 P12
                        % -------------------------------------------------

                        % --- BC 互协方差预测步 ---
                        % 如果已有前一帧的 P12，需要用 CV 模型将其传播
                        % 到当前时刻。预测公式：
                        %   P12_pred = F_cv * P12_prev * F_cv' + 0.5 * Q
                        %
                        % 为什么只加 0.5*Q 而不是 Q？
                        %   因为两个传感器共享同一个过程噪声源（目标运动
                        %   产生的不确定性），在互协方差中只计入一次。
                        %   而各自单独的协方差 P1、P2 中各计入完整的 Q。
                        if has_prev(p)
                            dt = params.dt_sec;
                            F_cv_dt = [1, dt, 0, 0;
                                       0,  1, 0, 0;
                                       0,  0, 1, dt;
                                       0,  0, 0,  1];

                            % 过程噪声：两传感器共享同一过程
                            Q_half = trk1.ukf.Q * 0.5;
                            P12_pred = F_cv_dt * P12_cell{p} * F_cv_dt' + Q_half;

                            % --- BC 互协方差更新步 ---
                            % UKF 的量测更新会使协方差收缩：
                            %   P_post = (I - KH) * P_pred
                            % 其中 K 是卡尔曼增益，H 是量测矩阵。
                            % 收缩因子 (I-KH) 的"强度"可以用迹来近似：
                            %   alpha = sqrt(trace(P_post) / trace(P_pred))
                            %
                            % 对于互协方差 P12，两个传感器的收缩效果相乘：
                            %   P12_new = alpha1 * alpha2 * P12_pred
                            %
                            % 这样做的原因：P12 描述的是两个估计误差的
                            % 相关性，当两个估计各自被"收紧"时，它们之间
                            % 的相关性也应该成比例缩小
                            if isfield(trk1, 'P_pred') && isfield(trk2, 'P_pred') ...
                                    && ~isempty(trk1.P_pred) && ~isempty(trk2.P_pred) ...
                                    && trace(trk1.P_pred) > 1e-10 && trace(trk2.P_pred) > 1e-10
                                % 迹收缩比：量测更新后协方差迹与更新前迹的比值开方
                                % 这个比值反映了 UKF 量测更新对不确定度的压缩程度
                                alpha1 = sqrt(max(1e-6, trace(trk1.ukf.P) / trace(trk1.P_pred)));
                                alpha2 = sqrt(max(1e-6, trace(trk2.ukf.P) / trace(trk2.P_pred)));
                                % 合成收缩因子，钳制在 [0.1, 1.0]
                                % 下限 0.1 防止过度收缩导致 P12 接近零矩阵
                                alpha = max(0.1, min(1.0, alpha1 * alpha2));
                                P12_new = alpha * P12_pred;
                            else
                                % 无 P_pred 信息时用保守的 0.5 折半
                                % 这是一种安全降级策略：不确定时宁可保守
                                P12_new = 0.5 * P12_pred;
                            end

                            % --- 稳定性约束 ---
                            % 限制 P12 对角元不超过 0.8 * min(diag(P1), diag(P2))
                            % 如果互协方差的某个对角元超过这个界限，说明 P12
                            % 在该维度上的相关性太强，可能导致融合结果振荡
                            % 或数值不稳定。处理方式是对该行/列进行等比例缩放
                            max_p12 = 0.8 * min(diag(P1)) * eye(4);
                            for ii = 1:4
                                if P12_new(ii,ii) > max_p12(ii,ii)
                                    % 对第 ii 行/列进行等比例缩放
                                    % 使用 sqrt 是因为 P12 出现在二次型中，
                                    % 对角元的缩放比例是行缩放比例的平方
                                    scale = sqrt(max_p12(ii,ii) / max(1e-10, P12_new(ii,ii)));
                                    P12_new(ii,:) = P12_new(ii,:) * scale;
                                    P12_new(:,ii) = P12_new(:,ii) * scale;
                                end
                            end
                        else
                            % 首帧或断连后重新开始，P12 设为零矩阵
                            % 零矩阵表示"不知道两个传感器估计之间的相关性"，
                            % 此时 BC 退化为 SCC（独立假设）
                            P12_new = zeros(4, 4);
                        end

                        % 调用 BC 融合算法，传入互协方差 P12_new
                        [x_f, P_f] = track_fusion_algorithms('BC', x1, P1, x2, P2, P12_new);

                        % 保存互协方差供下一帧使用
                        % 这是 BC 方法跨帧记忆的关键：当前帧的 P12 成为
                        % 下一帧预测步的输入
                        P12_cell{p} = P12_new;
                        has_prev(p) = true;
                        fused_trk.P12 = P12_new;

                    otherwise
                        error('Unknown fusion method: %s', method);
                end

                % 提取融合后的经纬度（从状态向量中取出对应位置）
                % 状态顺序：[lon; v_lon; lat; v_lat]
                % 所以 lon = x_f(1), lat = x_f(3)
                fused_trk.lat = x_f(3);
                fused_trk.lon = x_f(1);

                % 保存融合后的 UKF 状态和协方差
                fused_trk.ukf_x = x_f;
                fused_trk.ukf_P = P_f;

                fused_trk.source = 'both';  % 标记为双源融合
                fused_trk.life = max(trk1.life, trk2.life);  % 取较长寿命

            elseif ~isempty(trk1)
                % =================================================
                % 情况2：仅 R1 有数据 —— 直接透传 R1 结果
                % =================================================
                % 当 R2 在本帧漏检或航迹未起始时，无法执行融合。
                % 此时直接传递 R1 的估计，不做任何修改。
                % 这是一种保守策略：不融合时宁可不融合，也不要用
                % 过时/错误的 R2 数据强行融合
                fused_trk.lat = trk1.ukf.x(3);
                fused_trk.lon = trk1.ukf.x(1);
                fused_trk.ukf_x = trk1.ukf.x;
                fused_trk.ukf_P = trk1.ukf.P;
                fused_trk.source = 'R1_only';  % 标记为仅 R1
                fused_trk.life = trk1.life;
                if strcmp(method, 'BC')
                    % BC 方法中单源数据会导致下一帧无有效 P12 历史
                    % 因为 P12 描述的是两个传感器估计之间的相关性，
                    % 只有一个传感器时 P12 失去意义，需要重置
                    has_prev(p) = false;
                end

            else  % 仅 R2 有数据
                % =================================================
                % 情况3：仅 R2 有数据 —— 直接透传 R2 结果
                % =================================================
                % 与情况2对称，只是源换成了 R2
                fused_trk.lat = trk2.ukf.x(3);
                fused_trk.lon = trk2.ukf.x(1);
                fused_trk.ukf_x = trk2.ukf.x;
                fused_trk.ukf_P = trk2.ukf.P;
                fused_trk.source = 'R2_only';  % 标记为仅 R2
                fused_trk.life = trk2.life;
                if strcmp(method, 'BC')
                    has_prev(p) = false;
                end
            end

            % 将本对融合结果添加入本帧航迹列表
            % fused_snap.trackList 是一个 cell 数组，逐对追加融合结果
            fused_snap.trackList{end+1} = fused_trk;
        end

        % 保存本帧融合快照到输出
        fused_snapshots{k} = fused_snap;
    end
end

% =========================================================================
% find_track — 在航迹快照中按 ID 查找航迹（局部辅助函数）
% =========================================================================
% 遍历 snap.trackList 中各航迹的 id 字段，找到匹配 ID 后返回该航迹
% 结构体。若未找到或 snap 为空，返回空数组 []。
%
% 输入:
%   snap     - 单帧航迹快照结构体（含 trackList 字段）
%   track_id - 要查找的航迹 ID（整数）
%
% 输出:
%   trk - 匹配的航迹结构体，未找到时为空数组
% =========================================================================
function trk = find_track(snap, track_id)
    trk = [];
    % 空快照或无 trackList 字段则直接返回空
    if isempty(snap) || ~isfield(snap, 'trackList'), return; end
    % 线性扫描 trackList（航迹数量一般很小，通常 < 10，线性搜索足够高效）
    for t = 1:length(snap.trackList)
        if snap.trackList{t}.id == track_id
            trk = snap.trackList{t};
            return;
        end
    end
end
