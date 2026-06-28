%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description:
% assign new track a batchNo
% ----------------------------------------------------------------------
% Modified: 2025-10-07
% Author : Jun @ HIT
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [trackList, BatchNoInfo] = fun_assign_batchNo_to_new_track(trackList, BatchNoInfo)
header; 

if isempty(trackList)
    return;
end

% step 1: sort tracks according to their type
tracks_type = [trackList(:).Type];
ind = find(tracks_type == TEMPORARY_TRACK); 
for ii = 1:length(ind) 
    cur_index = ind(ii); 
    if isempty(trackList(cur_index).BatchNo)
        trackList(cur_index).BatchNo = BatchNoInfo.cur;
        BatchNoInfo.cur = BatchNoInfo.cur + 1;
    end
end
end
