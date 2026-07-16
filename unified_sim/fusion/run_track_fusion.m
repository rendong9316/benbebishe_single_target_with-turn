% =========================================================================
% run_track_fusion.m — 逐帧航迹融合主循环
% =========================================================================
%
% 【功能概述】
%   对已匹配的雷达1 (R1) 和雷达2 (R2) 航迹对，在统一时间网格上逐帧
%   执行融合。支持四种经典融合算法：SCC、BC、CI、FCI。对于两源均有
%   数据的帧执行双源融合；对于仅单源有数据的帧直接透传该源的结果。
%
% 【数学原理】
%   本函数是融合框架的主调度器，自身不执行融合数学运算，而是按帧、
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
%         id, r1_id, r2_id  — 融合ID及源航迹ID
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
    % -----------------------------------------------------------------
    n_frames = length(trackSnapshots_R1);  % 总帧数
    n_pairs = length(matched_pairs);       % 匹配的航迹对数

    % -----------------------------------------------------------------
    % 建立航迹ID到pair索引的快速查找映射
    % 使用 containers.Map 实现 O(1) 查找，避免在循环中逐条遍历
    % -----------------------------------------------------------------
    r1_to_pair = containers.Map('KeyType', 'int32', 'ValueType', 'int32');
    r2_to_pair = containers.Map('KeyType', 'int32', 'ValueType', 'int32');
    for p = 1:n_pairs
        r1_to_pair(matched_pairs(p).R1_track_id) = p;
        r2_to_pair(matched_pairs(p).R2_track_id) = p;
    end

    % -----------------------------------------------------------------
    % BC 方法专用初始化：为每对航迹维护互协方差矩阵 P12
    %   P12_cell{p} — 第 p 对航迹的互协方差矩阵 (4x4)
    %   has_prev(p) — 布尔标志，指示已有前一帧的有效 P12
    % -----------------------------------------------------------------
    if strcmp(method, 'BC')
        P12_cell = cell(n_pairs, 1);   % 存储互协方差
        for p = 1:n_pairs
            P12_cell{p} = zeros(4, 4);  % 初始化为零矩阵（无先验相关性）
        end
        has_prev = false(n_pairs, 1);   % 初始均无历史 P12
    end

    % -----------------------------------------------------------------
    % 预分配融合快照输出 cell
    % -----------------------------------------------------------------
    fused_snapshots = cell(n_frames, 1);

    % =================================================================
    % 主循环：逐帧、逐对执行融合
    % =================================================================
    for k = 1:n_frames
        % 获取本帧两个雷达的航迹快照
        snap_r1 = trackSnapshots_R1{k};
        snap_r2 = aligned_R2{k};

        % 初始化本帧融合快照结构体
        fused_snap = struct('frameID', k, 'trackList', {{}});

        % 遍历每对匹配航迹，执行融合
        for p = 1:n_pairs
            r1_id = matched_pairs(p).R1_track_id;
            r2_id = matched_pairs(p).R2_track_id;

            % ---------------------------------------------------------
            % 在本帧快照中按 ID 查找各传感器的航迹
            % ---------------------------------------------------------
            trk1 = find_track(snap_r1, r1_id);  % 查找 R1 航迹
            trk2 = find_track(snap_r2, r2_id);  % 查找 R2 航迹

            % ---------------------------------------------------------
            % 过滤无效 UKF 状态（航迹起始前或终止后的空状态）
            % UKF 状态字段 ukf.x 必须非空才视为有效估计
            % ---------------------------------------------------------
            if ~isempty(trk1) && (isempty(trk1.ukf) || ~isfield(trk1.ukf,'x') || isempty(trk1.ukf.x))
                trk1 = [];
            end
            if ~isempty(trk2) && (isempty(trk2.ukf) || ~isfield(trk2.ukf,'x') || isempty(trk2.ukf.x))
                trk2 = [];
            end

            % ---------------------------------------------------------
            % 两源均无有效数据，跳过本帧本对
            % ---------------------------------------------------------
            if isempty(trk1) && isempty(trk2)
                continue;
            end

            % ---------------------------------------------------------
            % 初始化融合航迹结构体
            % ---------------------------------------------------------
            fused_trk = struct();
            fused_trk.id = p;          % 融合航迹ID = pair 索引
            fused_trk.r1_id = r1_id;   % 记录原始 R1 航迹 ID
            fused_trk.r2_id = r2_id;   % 记录原始 R2 航迹 ID

            if ~isempty(trk1) && ~isempty(trk2)
                % =================================================
                % 情况1：两源均有有效数据 —— 执行双源融合
                % =================================================

                % 提取状态向量和协方差矩阵
                x1 = trk1.ukf.x;  P1 = trk1.ukf.P;
                x2 = trk2.ukf.x;  P2 = trk2.ukf.P;

                % 根据 method 参数选择融合算法
                switch upper(method)
                    case 'SCC'
                        % 简单凸组合：假设独立，直接信息融合
                        [x_f, P_f] = track_fusion_algorithms('SCC', x1, P1, x2, P2);
                        fused_trk.w = 0.5;  % SCC 等效 w=0.5 等权

                    case 'CI'
                        % 协方差交叉：优化 w 最小化 det(P_fused)
                        [x_f, P_f, w_opt] = track_fusion_algorithms('CI', x1, P1, x2, P2);
                        fused_trk.w = w_opt;

                    case 'FCI'
                        % 快速协方差交叉：用迹估计权重
                        [x_f, P_f, w_fci] = track_fusion_algorithms('FCI', x1, P1, x2, P2);
                        fused_trk.w = w_fci;

                    case 'BC'
                        % -------------------------------------------------
                        % Bar-Shalom-Campo：维护互协方差 P12
                        % -------------------------------------------------
                        if has_prev(p)
                            % --- BC 互协方差预测步 ---
                            % 用 CV 模型对 P12 进行时间传播
                            % F_cv_dt: CV 模型状态转移矩阵
                            dt = params.dt_sec;
                            F_cv_dt = [1, dt, 0, 0;
                                       0,  1, 0, 0;
                                       0,  0, 1, dt;
                                       0,  0, 0,  1];

                            % 过程噪声：两传感器共享同一过程，互协方差
                            % 中仅计入 1/2 Q（因为各传感器分别承担一部
                            % 分不确定度）
                            Q_half = trk1.ukf.Q * 0.5;
                            P12_pred = F_cv_dt * P12_cell{p} * F_cv_dt' + Q_half;

                            % --- BC 互协方差更新步 ---
                            % 实际 UKF 更新使协方差从 P_pred 收缩到 P
                            % 用迹收缩比 sqrt(trace(P)/trace(P_pred)) 近似
                            % (I-KH) 的压缩效果，两传感器的压缩比例相乘
                            % 作为 P12 的近似收缩因子 alpha
                            if isfield(trk1, 'P_pred') && isfield(trk2, 'P_pred') ...
                                    && ~isempty(trk1.P_pred) && ~isempty(trk2.P_pred) ...
                                    && trace(trk1.P_pred) > 1e-10 && trace(trk2.P_pred) > 1e-10
                                % 迹收缩比：量测更新后迹与更新前迹的比值开方
                                alpha1 = sqrt(max(1e-6, trace(trk1.ukf.P) / trace(trk1.P_pred)));
                                alpha2 = sqrt(max(1e-6, trace(trk2.ukf.P) / trace(trk2.P_pred)));
                                % 合成收缩因子，钳制在 [0.1, 1.0]
                                alpha = max(0.1, min(1.0, alpha1 * alpha2));
                                P12_new = alpha * P12_pred;
                            else
                                % 无 P_pred 信息时用保守的 0.5 折半
                                P12_new = 0.5 * P12_pred;
                            end

                            % --- 稳定性约束 ---
                            % 限制 P12 对角元不超过 0.8 * min(diag(P1), diag(P2))
                            % 防止互协方差过大导致融合结果不稳定
                            max_p12 = 0.8 * min(diag(P1)) * eye(4);
                            for ii = 1:4
                                if P12_new(ii,ii) > max_p12(ii,ii)
                                    % 对第 ii 行/列进行等比例缩放
                                    scale = sqrt(max_p12(ii,ii) / max(1e-10, P12_new(ii,ii)));
                                    P12_new(ii,:) = P12_new(ii,:) * scale;
                                    P12_new(:,ii) = P12_new(:,ii) * scale;
                                end
                            end
                        else
                            % 首帧或断连后重新开始，P12 设为零
                            P12_new = zeros(4, 4);
                        end

                        % 调用 BC 融合
                        [x_f, P_f] = track_fusion_algorithms('BC', x1, P1, x2, P2, P12_new);

                        % 保存互协方差供下一帧使用
                        P12_cell{p} = P12_new;
                        has_prev(p) = true;
                        fused_trk.P12 = P12_new;

                    otherwise
                        error('Unknown fusion method: %s', method);
                end

                % 提取融合后的经纬度（状态向量的第3和第1元素）
                % 状态顺序：[lon; v_lon; lat; v_lat]
                fused_trk.lat = x_f(3);
                fused_trk.lon = x_f(1);

                % 保存融合后的 UKF 状态和协方差
                fused_trk.ukf_x = x_f;
                fused_trk.ukf_P = P_f;

                fused_trk.source = 'both';  % 标记为双源融合
                fused_trk.life = max(trk1.life, trk2.life);

            elseif ~isempty(trk1)
                % =================================================
                % 情况2：仅 R1 有数据 —— 直接透传 R1 结果
                % =================================================
                fused_trk.lat = trk1.ukf.x(3);
                fused_trk.lon = trk1.ukf.x(1);
                fused_trk.ukf_x = trk1.ukf.x;
                fused_trk.ukf_P = trk1.ukf.P;
                fused_trk.source = 'R1_only';  % 标记为仅 R1
                fused_trk.life = trk1.life;
                if strcmp(method, 'BC')
                    % BC 方法中单源数据会导致下一帧无有效 P12 历史
                    has_prev(p) = false;
                end

            else  % 仅 R2 有数据
                % =================================================
                % 情况3：仅 R2 有数据 —— 直接透传 R2 结果
                % =================================================
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
            fused_snap.trackList{end+1} = fused_trk;
        end

        % 保存本帧融合快照
        fused_snapshots{k} = fused_snap;
    end
end

% =========================================================================
% find_track — 在航迹快照中按 ID 查找航迹（局部辅助函数）
% =========================================================================
% 遍历 snap.trackList 中各航迹的 id 字段，找到匹配 ID 后返回。
% 若未找到或 snap 为空，返回空数组 []。
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
    % 线性扫描 trackList（航迹数量一般很小）
    for t = 1:length(snap.trackList)
        if snap.trackList{t}.id == track_id
            trk = snap.trackList{t};
            return;
        end
    end
end
