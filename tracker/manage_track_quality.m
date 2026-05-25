% =========================================================================
% manage_track_quality.m
% =========================================================================
% 【功能概述】
%   航迹质量状态机，管理航迹在四种状态之间的转移。采用对称计分机制
%  （关联+1分，漏检-1分），配合降级阈值缓冲，适配 Pd=60% 低检出率
%   场景下航迹不容易因偶然漏检而被误删。
%
% 【数学原理】
%   状态转移逻辑基于自适应评分（Adaptive Scoring）:
%     quality 初始值 = 9（TEMPORARY创建时设定，由track_starter_mofn设定）
%     quality ∈ [0, 15]，超出范围截断
%
%   四种航迹类型:
%     TYPE_RELIABLE  (1) : 置信航迹，quality ≥ 10 且连续关联良好
%     TYPE_MAINTAIN  (2) : 维持航迹，RELIABLE降级后的中间态
%     TYPE_TEMPORARY (6) : 暂定航迹，M/N起始后新创建，待确认
%     TYPE_HISTORY   (7) : 历史/死亡航迹，不再参与关联
%
%   状态转移规则（对称±1计分）:
%     TEMPORARY → 关联+1 → quality≥10 → 升级 RELIABLE
%     TEMPORARY → 漏检-1 → quality<3  → 降级 HISTORY（死亡）
%     RELIABLE  → 漏检-1 → quality<8  → 降级 MAINTAIN（缓冲态）
%     MAINTAIN  → 关联+1 → quality≥10 → 恢复 RELIABLE
%     MAINTAIN  → 漏检-1 → quality<3  → 降级 HISTORY
%     HISTORY   → 保持不变（不再参与跟踪）
%
%   特例: TEMPORARY航迹若连续漏检达到K_loss帧，强制降为HISTORY
%         （防止未确认航迹长期占用资源）
%
%   设计考量:
%     - RELIABLE→MAINTAIN 的降级阈值(quality<8)比 MAINTAIN→HISTORY
%       的死亡阈值(quality<3)更早触发，形成"缓冲带"
%     - 这保证了可靠航迹不会因为少量漏检直接死亡，而是先进入维持
%       状态，还有机会通过后续关联恢复
%
% 【输入参数】
%   trackList - cell数组，所有航迹结构体（原地修改）
%   active_idx - 向量，活跃航迹索引（type≠7）
%   params    - 参数结构体，需包含:
%               .tracker_K_loss: TEMPORARY航迹最大连续漏检帧数
%   frame_id  - 当前帧编号
%
% 【输出】
%   trackList - 更新后的航迹列表（原地修改后返回）
%
% 【调用关系】
%   被 multi_track_manager.m 在Step 7调用
%   被 single_track_runner.m 间接使用（单目标模式质量逻辑内联）
% =========================================================================

function trackList = manage_track_quality(trackList, active_idx, params, frame_id)
    % ---- 航迹类型常量定义 ----
    TYPE_RELIABLE   = 1;   % 置信航迹：quality≥10，跟踪可靠
    TYPE_MAINTAIN   = 2;   % 维持航迹：曾被确认为RELIABLE但近期漏检
    TYPE_TEMPORARY  = 6;   % 暂定航迹：M/N起始后新创建，待确认
    TYPE_HISTORY    = 7;   % 历史/死亡航迹：不再参与处理

    % ---- 遍历每条活跃航迹，更新质量状态 ----
    for i = 1:length(active_idx)
        t = active_idx(i);
        trk = trackList{t};

        % 判断本帧是否有量测关联（assoc_det非空表示关联成功）
        was_associated = ~isempty(trk.assoc_det);

        % ---- 根据航迹类型执行对应的状态转移逻辑 ----
        switch trk.type
            case TYPE_TEMPORARY
                % ---- 暂定航迹状态转移 ----
                if was_associated
                    % 关联成功: quality +1，上限15
                    trk.quality = min(trk.quality + 1, 15);
                    if trk.quality >= 10
                        % 质量达到阈值：升级为置信航迹
                        trk.type = TYPE_RELIABLE;
                    end
                else
                    % 漏检: quality -1
                    trk.quality = trk.quality - 1;
                    if trk.quality < 3
                        % 质量过低：标记为死亡航迹
                        trk.type = TYPE_HISTORY;
                        trk.death_frame = frame_id;
                    end
                end

            case TYPE_RELIABLE
                % ---- 置信航迹状态转移 ----
                if was_associated
                    % 关联成功: quality +1，上限15
                    trk.quality = min(trk.quality + 1, 15);
                else
                    % 漏检: quality -1
                    trk.quality = trk.quality - 1;
                    if trk.quality < 8
                        % 注意：RELIABLE的降级阈值(8)高于TEMPORARY的
                        % 死亡阈值(3)，为可靠航迹提供更大的容错空间
                        trk.type = TYPE_MAINTAIN;
                    end
                end

            case TYPE_MAINTAIN
                % ---- 维持航迹状态转移 ----
                if was_associated
                    % 关联成功: quality +1，可以恢复到RELIABLE
                    trk.quality = min(trk.quality + 1, 15);
                    if trk.quality >= 10
                        % 质量恢复：重新升级为置信航迹
                        trk.type = TYPE_RELIABLE;
                    end
                else
                    % 漏检: quality -1
                    trk.quality = trk.quality - 1;
                    if trk.quality < 3
                        % 持续恶化：标记为死亡
                        trk.type = TYPE_HISTORY;
                        trk.death_frame = frame_id;
                    end
                end

            case TYPE_HISTORY
                % ---- 历史航迹保持状态不变 ----
                % HISTORY状态是吸收态，一旦进入就不再自动退出
                % 唯一的例外是track_starter_mofn中的航迹复活机制
        end

        % ---- 暂定航迹超时强制终止 ----
        % K_loss帧连续漏检后，即使quality尚未降到3也强制死亡
        % 这防止未确认航迹在低质量徘徊时仍占用计算资源
        if trk.type == TYPE_TEMPORARY && trk.missed >= params.tracker_K_loss
            trk.type = TYPE_HISTORY;
            trk.death_frame = frame_id;
        end

        % 将更新后的航迹写回cell数组
        trackList{t} = trk;
    end
end
