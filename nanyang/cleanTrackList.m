%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description:
% scan over the whole track list, and remove the one with bad quality
%
% Input:
% trackList: the track list with unqualified tracks 
% 
% Output:
% trackList: the clean track list
% ----------------------------------------------------------------------
% Date: 2022-03-31
% Author : Jun @ HIT
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function trackList = cleanTrackList(trackList)

global gTotalTrackCnt; % add a global varable for track batch no update @ 2023-04-14

trackNum = length(trackList);
ind_del = []; % create an empty list to store the index of removing items 

for tt = 1:trackNum
    curTrack = trackList(tt);
    duration = tool_get_time_difference(curTrack.asscPointList(end).time, ...
        curTrack.asscPointList(1).time, MATLAB_TIME_IN_MIN); % in min

    % renumber the track batch No use the global variable @ 2023-04-14
    if isempty(curTrack.BatchNo) && (curTrack.AsscPointCnt >= 5)
        % if the track is long enough, and has no batch no
        trackList(tt).BatchNo = gTotalTrackCnt; 
        gTotalTrackCnt = gTotalTrackCnt + 1; 
    end

    % if (curTrack.Type == 7) && (curTrack.TotalPointCnt < 15)
    invalid_rule1 = (curTrack.Type == HISTORY_TRACK) && (duration < MIN_TRACK_TIME_LEN);
    invalid_rule2 = (curTrack.Type == HISTORY_TRACK) && (curTrack.AsscPointCnt < MIN_TRACK_ASSC_LEN);
    if invalid_rule1 || invalid_rule2
        % case 1: remove the 'dead' and 'short' track
        ind_del = [ind_del, tt];
    else
        continue;
    end
end

trackList(ind_del) = [];

end