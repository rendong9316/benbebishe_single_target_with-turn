function [candidate_tracks, asscPointsIndex] = fun_check_35logic_points_improved(curPoint, tempTrackList, QUALIFY_NUM, TOLERANT_NUM, sysPara)
% 3/5逻辑航迹起始函数（优化版）
% 在连续TOLERANT_NUM帧中，如果有QUALIFY_NUM帧存在运动一致的点迹，则起始航迹
% 保持原框架结构，但优化逻辑

candidate_tracks = [];
asscPointsIndex = [];

% 获取当前点信息（终点）
curFrameID = curPoint.frameID;
curRange = curPoint.prange;
curVr = curPoint.pvr;
curAz = curPoint.paz;

% 波门参数 - 根据数据特点调整
gateRange = 20;   % km
gateVr = 10;     % m/s
gateAz = 1.6;    % 度

% 1. 寻找可能的起点（在当前帧之前的TOLERANT_NUM-1帧内）
startFrame = curFrameID - (TOLERANT_NUM - 1);
if startFrame < 211
    startFrame = 211;
end

% 获取起点候选（当前帧之前的点）
start_candidate_indices = find([tempTrackList(:).frameID] >= startFrame & ...
                               [tempTrackList(:).frameID] < curFrameID);

% 优化：先对起点按时间排序，从最近的开始尝试
[~, sorted_start_idx] = sort([tempTrackList(start_candidate_indices).frameID], 'descend');
start_candidate_indices = start_candidate_indices(sorted_start_idx);

for i = 1:length(start_candidate_indices)
    startIdx = start_candidate_indices(i);
    startPoint = tempTrackList(startIdx);
    
    % 2. 检查起点和终点的运动一致性
    % 计算时间差（秒）
    timeDiff = (curPoint.time - startPoint.time) * 24 * 3600;
    
    % 基于起点预测终点位置（匀速直线运动）
    predictedRange = startPoint.prange + (startPoint.pvr * timeDiff / 1000);
    
    % 计算归一化距离
    dR = abs(curRange - predictedRange) / gateRange;
    dV = abs(curVr - startPoint.pvr) / gateVr;
    dA = abs(curAz - startPoint.paz) / gateAz;
    
    dist = sqrt(dR^2 + dV^2 + dA^2);
    
    % 如果起点和终点匹配（放宽匹配条件）
    if dist < 1.2  % 从1.0放宽到1.2，增加匹配机会
        % 3. 检查中间帧
        intermediate_frames = startPoint.frameID + 1 : curFrameID - 1;
        intermediate_indices = [];
        
        % 关键优化：不要求每个中间帧都有点，允许有缺失
        for frame = intermediate_frames
            % 获取该帧的所有点迹
            frame_indices = find([tempTrackList(:).frameID] == frame);
            
            if isempty(frame_indices)
                continue;
            end
            
            % 预测该帧点的位置（线性插值）
            alpha = (frame - startPoint.frameID) / (curFrameID - startPoint.frameID);
            predRange = startPoint.prange + alpha * (curRange - startPoint.prange);
            predVr = startPoint.pvr + alpha * (curVr - startPoint.pvr);
            predAz = startPoint.paz + alpha * (curAz - startPoint.paz);
            
            % 寻找该帧中最匹配的点
            bestDist = inf;
            bestIdx = 0;
            
            for j = 1:length(frame_indices)
                idx = frame_indices(j);
                point = tempTrackList(idx);
                
                % 跳过已经被选中的点
                if ~isempty(asscPointsIndex) && any(asscPointsIndex(:) == idx)
                    continue;
                end
                
                dR = abs(point.prange - predRange) / gateRange;
                dV = abs(point.pvr - predVr) / gateVr;
                dA = abs(point.paz - predAz) / gateAz;
                
                dist = sqrt(dR^2 + dV^2 + dA^2);
                
                if dist < bestDist && dist < 1.2  % 放宽匹配条件
                    bestDist = dist;
                    bestIdx = idx;
                end
            end
            
            % 如果找到满足条件的点
            if bestDist < 1.2 && bestIdx > 0
                intermediate_indices = [intermediate_indices, bestIdx];
            end
        end
        
        % 4. 统计匹配的帧数（起点 + 中间点 + 终点）
        matched_frames = [startPoint.frameID];
        if ~isempty(intermediate_indices)
            % 获取中间点的frameID，去重
            intermediate_frames_list = unique([tempTrackList(intermediate_indices).frameID]);
            matched_frames = [matched_frames, intermediate_frames_list];
        end
        matched_frames = [matched_frames, curFrameID];
        
        % 去重并统计
        total_frames = length(unique(matched_frames));
        
        % 5. 检查是否满足3/5条件
        if total_frames >= QUALIFY_NUM
            % 构建完整的点迹序列
            all_indices = [startIdx];
            if ~isempty(intermediate_indices)
                all_indices = [all_indices, intermediate_indices];
            end
            
            % 按frameID排序
            frameIDs = [tempTrackList(all_indices).frameID];
            [~, sort_idx] = sort(frameIDs);
            all_indices = all_indices(sort_idx);
            
            % 添加当前点
            curIdx = find([tempTrackList(:).frameID] == curFrameID & ...
                         [tempTrackList(:).prange] == curRange & ...
                         [tempTrackList(:).pvr] == curVr & ...
                         [tempTrackList(:).paz] == curAz, 1);
            all_indices = [all_indices, curIdx];
            
            % 获取所有点迹
            all_points = tempTrackList(all_indices);
            
            % 检查是否重复
            if ~is_duplicate_track(all_indices, asscPointsIndex)
                candidate_tracks = [candidate_tracks, struct('asscPointList', all_points)];
                
                % 存储索引
                if isempty(asscPointsIndex)
                    asscPointsIndex = all_indices;
                else
                    % 确保维度一致
                    if size(all_indices, 1) > size(all_indices, 2)
                        all_indices = all_indices';
                    end
                    
                    current_cols = size(asscPointsIndex, 2);
                    new_cols = length(all_indices);
                    
                    if current_cols < new_cols
                        padding = zeros(size(asscPointsIndex, 1), new_cols - current_cols);
                        asscPointsIndex = [asscPointsIndex, padding];
                    elseif current_cols > new_cols
                        all_indices = [all_indices, zeros(1, current_cols - new_cols)];
                    end
                    
                    asscPointsIndex = [asscPointsIndex; all_indices];
                end
                
                % 关键优化：找到一个符合条件的航迹后，不立即继续查找
                % 避免找到重复或相似的航迹
                break;  % 跳出起点循环，处理下一个当前点
            end
        end
    end
end
end