%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description:
% start a track via M out of N logic method. This method is realized by 
% Iteratively calling this function. 
% 
% Input:
% tempTrackList: All Existing temporary tracks.  
% DetPoint: currently detected points
% M: validation number
% N: total number
% Output:
% tempTrackList: updated existing temporary tracks
% valid_tracks: newly started tracks
% -------------------------------------------------------------------
% Author: Jun Geng
% Date: 2022/12/28
% -------------------------------------------------------------------
% Modification:
% Considering the velocity ambiguity, for each started track, provide two
% branches: one for no ambiguity, the other for ambiguity number 1. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [tempTrackList, valid_tracks] = trackStarter_logic( tempTrackList, pointList, sysPara, QUALIFY_NUM, TOLERANT_NUM)

% run('header.m');
header; 

%% part I: process special cases
% if there has no temp trackList
if isempty(tempTrackList)
    tempTrackList = pointList; 
    valid_tracks = [];
    return;
end

% if there has no detected point
if isempty(pointList)
    valid_tracks = []; 
    return
end

%% part II: selected valid temporary track points
curFrameID = pointList(1).frameID; 
prevFrameIDList = [tempTrackList(:).frameID]; 
ind = find( prevFrameIDList >= curFrameID - TOLERANT_NUM + 1 ); % valid temp tracks
tempTrackList = tempTrackList(ind); % remove the invalid ones

%% part III: scan over the current detected points
prevFrameIDList = [tempTrackList(:).frameID];
if length(unique(prevFrameIDList)) < QUALIFY_NUM - 1
    % do not have enough frames to determine a new track, update the temp
    % track list and quit
    tempTrackList = [tempTrackList, pointList]; 
    valid_tracks = []; 
    return;
end

% if have enough frames, then scan over all new single points for starting
% a new track
ptsNum = length(pointList);
valid_tracks = [];  % initialization the output
remove_pool_pts_index = []; 
remove_cur_pts_index = []; 
for pp = 1:ptsNum
    curPoint = pointList(pp); 
    % find all possible points in a line
%    try
    [candidate_tracks, asscPointsIndex] = fun_find_best_asscpoints_NN(curPoint, tempTrackList, QUALIFY_NUM, sysPara); 
%     catch
%         disp(1);
%     end
    % check valid checks 
    for cc = 1:length(candidate_tracks)
        % candidate_tracks = checkTrackValidation(candidate_tracks);
        isValid = fun_check_track_validation(candidate_tracks(cc)); 
        if isValid
            % create a new and fillout track
            newTrack = fun_create_new_track(candidate_tracks(cc), sysPara);
            
            % put the new track on the output
            valid_tracks = [valid_tracks, newTrack];
            
            % remove related points
            remove_pool_pts_index = [remove_pool_pts_index, asscPointsIndex(cc, :)];  
            remove_cur_pts_index = [remove_cur_pts_index, pp]; 
        else
            % not a good track, check the next one
            continue; 
        end
    end
    % (1) remove the associated ones
    tempTrackList = fun_remove_assc_pts_from_pointlist(tempTrackList, remove_pool_pts_index);
    remove_pool_pts_index = []; 
end


%% Part IV: update the temp track list
% (2) append the new ones
% pointList(remove_cur_pts_index) = []; 
pointList = fun_remove_assc_pts_from_pointlist(pointList, remove_cur_pts_index);
tempTrackList = [tempTrackList, pointList];
% (3) remove the old ones if already has enough frames
frameIDList = [tempTrackList(:).frameID];
minFrameID = min(frameIDList);
maxFrameID = max(frameIDList);
if (maxFrameID - minFrameID + 1) >= TOLERANT_NUM
    ind = find( frameIDList <= maxFrameID - TOLERANT_NUM +1);
    tempTrackList(ind) = []; 
    return;
end

end



% sub functions of track starter
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description:
% find the best point from the PointList to match the track
% 
% input
% track: current track
% pointlist: point list
% sysPara: system parameter
% 
% output:
% bestPoint: the best matched point
% PointList: the one has deleted the bestPoint from the input PointList
% if no point has been associated, bestPoint = []; output PointList  =
% input PointList
% ---------------------------------------------------------------------
% Author: Jun Geng
% Date: 2022 Mar. 17th
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [candidate_tracks, asscPointsIndex] = fun_find_best_asscpoints_NN(curPoint, tempTrackList, QUALIFY_NUM, sysPara)

run('header.m'); 

% output initialization
candidate_tracks = [];
asscPointsIndex = [];

if isempty(curPoint)
    return;
end

% get useful data
% starting point info
start_point = [curPoint.prange, curPoint.pvr, curPoint.time];
curFrameID = curPoint.frameID;
curTime = curPoint.time; 

% points pool info
pool_points_frameID = [tempTrackList(:).frameID];

prevFrameIDList = unique(pool_points_frameID); 
maxFrameID = max(prevFrameIDList);
minFrameID = min(prevFrameIDList);

% put the last candidate points
candidate_tracks.asscPointList(1) = curPoint;

for ff = maxFrameID : -1 : minFrameID
    % step 1: find all the points on the frame
    ind = find(pool_points_frameID == ff); 
    if isempty(ind)
        continue;
    end
    
    pastPointList = tempTrackList(ind); 
    pastTime = pastPointList(1).time;
    
    % step 2: predict (retrospect) the location of the potential track
    reLocation = fun_retrospective_prediction(candidate_tracks, pastTime); 
    
    % step 3: calculate the distance between the predict location and
    % measured location, find the best point
    [pastPoint, minDist] = fun_find_the_nearest_point(reLocation, pastPointList, sysPara);
    
    % step 4: check if the best point qualified or not, update the track if
    % it is qualified. 
    range_diff = abs(pastPoint.prange - reLocation.range); 
    vr_diff = abs(pastPoint.pvr - reLocation.vr);
    az_diff = abs(pastPoint.paz - reLocation.az);
    
    % the size of neighborhood is defined in header 
    isRangeOK = (range_diff <= NN_RANGE_RADIUS); 
    isVrOK = (vr_diff <= NN_VR_RADIUS);
    isAzOK = (az_diff <= NN_AZ_RADIUS);
    isDistOK = (minDist <= NN_OVERALL); % probably 10 is OK 
    isQualified = isRangeOK & isVrOK & isAzOK & isDistOK; 
    
    if isQualified
        asscPtsNum = length([candidate_tracks.asscPointList(:)]); 
        candidate_tracks.asscPointList( asscPtsNum+1 ) = pastPoint;
    end 
end

% check if the track length good or not
asscPtsNum = length([candidate_tracks.asscPointList(:)]);
if asscPtsNum < QUALIFY_NUM
    % this track is not long enough
    candidate_tracks = []; 
    asscPointsIndex = [];  
else
    % resort the order of associated points (from small to large)
    candidate_tracks.asscPointList = candidate_tracks.asscPointList(asscPtsNum:-1:1);  
    asscPointsIndex = fun_find_point_index_from_list(candidate_tracks, tempTrackList); 
end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function reLocation = fun_retrospective_prediction(candidate_tracks, pastTime)
run('header.m');

asscPointList = candidate_tracks.asscPointList; 
ptsNum = length(asscPointList); 
if 1 == ptsNum
    % only has one point, then use it as predict location
    reLocation.range = asscPointList.prange; 
    reLocation.vr = asscPointList.pvr; 
    reLocation.az = asscPointList.paz; 
else
    % has more than one point, then use linear regression to estimate the
    % location
    assc_points_range = [asscPointList(:).prange]; 
    assc_points_vr = [asscPointList(:).pvr];
    assc_points_az = [asscPointList(:).paz];
    assc_points_time = [asscPointList(:).time];
    
    ref_time = assc_points_time(1); 
    assc_time = tool_get_time_difference(assc_points_time, ref_time, MATLAB_TIME_IN_SEC);
    time_interval = tool_get_time_difference(pastTime, ref_time, MATLAB_TIME_IN_SEC);

    coef = polyfit(assc_time, assc_points_range, 1);
    reLocation.range = coef(1) * time_interval + coef(2); 
    coef = polyfit(assc_time, assc_points_vr, 1);
    reLocation.vr = coef(1) * time_interval + coef(2);
    reLocation.az = median(assc_points_az); 
end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [pastPoint, minDist] = fun_find_the_nearest_point(reLocation, pastPointList, sysPara)

run('header.m'); 

pointNum = length(pastPointList);  % the number of points on the list 
dist_p2t = zeros(3, pointNum); % distance between the point and the target point

pool_points_range = [pastPointList(:).prange]; 
pool_points_vr = [pastPointList(:).pvr];
pool_points_az = [pastPointList(:).paz];

dist_p2t(1, :) = pool_points_range - reLocation.range;  % the first row is about the range
dist_p2t(2, :) = pool_points_az - reLocation.az; % the second row is about the azimuth
dist_p2t(3, :) = pool_points_vr - reLocation.vr; % the third row is about the velocity

% calculate the distance between the track and the point. Note that the
% distance should be normalized resolution
dist_total = sqrt(NN_WEIGHT_R * abs(dist_p2t(1, :)/(sysPara.deltaR/1e3)).^2 + ...
    NN_WEIGHT_A * abs(dist_p2t(2, :)/sysPara.deltaAz).^2 +...
    NN_WEIGHT_V * abs(dist_p2t(3, :)/sysPara.deltaV).^2); 

% prepare the output
[minDist, ind] = min(dist_total); % find the one with the minimum distance
pastPoint = pastPointList(ind);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function pointIndex = fun_find_point_index_from_list(candidate_tracks, pointList)
pool_points_Rbin = [pointList(:).Rbin]; 
pool_points_Dbin = [pointList(:).Dbin];
pool_points_Abin = [pointList(:).Abin];
pool_points_ambg = [pointList(:).ambgNum];
pool_points_frameID = [pointList(:).frameID];

ptsNum = length([candidate_tracks.asscPointList(:)]); 
pointIndex = zeros(1, ptsNum);
for pp = 1:ptsNum - 1
    % NOTE: the loop above does not mark the last associated point because
    % the last one is from curPoint, which does not on the previous track
    % list
    curPoint = candidate_tracks.asscPointList(pp);
    curRbin = curPoint.Rbin;
    curDbin = curPoint.Dbin;
    curAbin = curPoint.Abin;
    curAmbgNum = curPoint.ambgNum;
    curFrameID = curPoint.frameID;
    
    ind1 = find(pool_points_Rbin == curRbin);
    ind2 = find(pool_points_Dbin == curDbin);
    ind3 = find(pool_points_Abin == curAbin);
    ind4 = find(pool_points_ambg == curAmbgNum);
    ind5 = find(pool_points_frameID == curFrameID); 
    
    ind12 = intersect(ind1, ind2);
    ind34 = intersect(ind3, ind4); 
    ind1234 = intersect(ind12, ind34); 
    
    ind = intersect(ind1234, ind5); 
    
    pointIndex(pp) = ind; 
end
end
