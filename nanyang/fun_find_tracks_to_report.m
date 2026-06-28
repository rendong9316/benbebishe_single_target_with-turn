function [reportTracks, reportPoints] = fun_find_tracks_to_report(trackList, sysPara)
% run('header.m');
header; 

% initialize the ouput variable
reportTracks = [];
reportPoints = [];

if isempty(trackList)
    return;
end

% 1. locate the candidate tracks: the track that being updated in this
% round
updateFlag = [trackList(:).updateFlag];
quality = [trackList(:).Quality]; 
% trackVaildLen = [trackList(:).AsscPointCnt];
ind1 = find(updateFlag == 1);
ind2 = find(quality == NEW_TRACK_QUALITY);
% ind3 = find(trackVaildLen <= MIN_REPORT_LEN);
ind12 = intersect(ind1, ind2); 
% ind13 = intersect(ind12, ind3); 

% 2. reported tracks: two conditions: track has been updated and the length
% just meet the minimum requirement;
reportTracks = trackList(ind12); % report all history associated points
% you can add more code here to smooth the header of this track. 

% 3. reported points: the track has been update, and only report the most
% recent point. 
trackCandidates = trackList; 
trackCandidates(ind12) = []; % remove all reported tracks
trackType = [trackCandidates(:).Type]; 
ind7 = find(trackType == HISTORY_TRACK);
ind6 = find(trackType == TEMPORARY_TRACK); 
ind = [ind6, ind7]; 
trackCandidates(ind) = []; % remove dead or unborn tracks
% report all history associated points
trackNum = length(trackCandidates);
cnt = 0; 
for tt = 1:trackNum
    cnt = cnt + 1;
%     reportPoints(tt).time = trackCandidates(1).time;
    curTrack = trackCandidates(tt);
    smoothPointList = [curTrack.smoothPointList(:)]; 
    reportPoints(cnt).time = smoothPointList(1).time;
    reportPoints(cnt).lat = smoothPointList(end).lat; %     % DONG_202512_v1
    reportPoints(cnt).lon = smoothPointList(end).lon;     % DONG_202512_v1
    reportPoints(cnt).v_x = smoothPointList(end).v_x;
    reportPoints(cnt).v_y = smoothPointList(end).v_y;
    reportPoints(cnt).az = smoothPointList(end).paz;
    reportPoints(cnt).vr = smoothPointList(end).pvr;
    reportPoints(cnt).travelLen = curTrack.travelLen;
    reportPoints(cnt).predictRes = curTrack.predictRes(end);
    reportPoints(cnt).asscPoint = curTrack.asscPointList(end);
    reportPoints(cnt).smoothPoint= curTrack.smoothPointList(end);
    reportPoints(cnt).outputPoint = curTrack.outputPointList(end);
    reportPoints(cnt).TotalPointCnt = curTrack.TotalPointCnt;
    reportPoints(cnt).AsscPointCnt = curTrack.AsscPointCnt;
    reportPoints(cnt).TotalLostPointCnt = curTrack.TotalLostPointCnt;
    reportPoints(cnt).SuccLossPointCnt = curTrack.SuccLossPointCnt;
    reportPoints(cnt).Quality = curTrack.Quality;
    reportPoints(cnt).BatchNo = curTrack.BatchNo;
end

end


