% =========================================================================
% sortTrackList_oracle.m — 航迹列表按 ID 排序
% =========================================================================
% 【功能】
%   将航迹列表按航迹 ID（track.id）从小到大排序。
%   这是 Oracle 模式下的标准排序函数，确保航迹列表的顺序与
%   创建顺序一致，便于后续的 ID 映射和诊断。
%
% 【输入】
%   trackList — 航迹结构体数组（无序）
%
% 【输出】
%   trackList — 按 ID 升序排列的航迹结构体数组
% =========================================================================
function trackList = sortTrackList_oracle(trackList)
    % 空列表直接返回，避免后续索引操作报错
    if isempty(trackList)
        return;
    end

    % 提取所有航迹 ID 到数值数组
    % 遍历 trackList 元胞数组，取出每条航迹的 id 字段
    ids = zeros(1, length(trackList));
    for i = 1:length(trackList)
        ids(i) = trackList{i}.id;
    end

    % 按 ID 排序并重新排列航迹列表
    % sort 返回排序后的值和索引 order，用 order 对 trackList 重排
    [~, order] = sort(ids);
    trackList = trackList(order);
end
