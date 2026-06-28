%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Discription
% find all the points that are colinear with respect to the input curPoint
% by searching over all the tempTrackList. Determine the qualification of
% the searching line by counting the number on line
% Input:
% curPoint: current Point, the start point of the line
% tempTrackList: 
% QUALIFY_NUM: a threshold, only when the points on the line is larger than
% QUALIFY_NUM, we can determine it is a qualification line
% Output:
% candidate_tracks: a group of possible associated points (INCLUDE THE
% INITIAL POINT)
% asscPointsIndex: the index of candidate_tracks on the tempTrackList, if
% the line if confirmed, it will be used to remove these associated points
% (and their shadows) frome the tempTrackList (DOES NOT INCLUDE THE INITIAL
% POINT).
% -------------------------------------------------------------------------
% Author: Jun Geng @ 2025-10-06
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [candidate_tracks, asscPointsIndex] = fun_check_colinear_points(curPoint, tempTrackList, QUALIFY_NUM, sysPara)

% output initialization
candidate_tracks = [];
asscPointsIndex = [];

% get useful data
% starting point info
start_point = [curPoint.prange, curPoint.pvr, curPoint.time];
curFrameID = curPoint.frameID;

% points pool info
pool_points_range = [tempTrackList(:).prange]; 
pool_points_vr = [tempTrackList(:).pvr]; 
pool_points_time = [tempTrackList(:).time];
pool_points_frameID = [tempTrackList(:).frameID];

% get some useful parameters
minFrameID = min(pool_points_frameID);
maxFrameID = max(pool_points_frameID);
if maxFrameID - minFrameID > 15
    warning('Points have stored more than 15 frames! A possible ERROR may occur in data preparation!');
end
frameID_List = maxFrameID : -1 : minFrameID;
[xx, ia] = unique(pool_points_frameID);
yy = pool_points_time(ia); 
time_List = interp1(xx, yy, frameID_List); 

% begin to search over the pool
group_cnt = 0; % the number of different direction vectors
stCluster = []; % the points with similar derection vector within the same cluster

% scan over each frame to find possible lines
vr_fixed = 10; % m/s
vr_float = 3; % m/s
range_fixed = 30; % km
range_float = 10; % km
for ff = 1 : length(frameID_List) - QUALIFY_NUM + 2
    % the ff th frame is activated
    % (1) get the neighborhood points
    nFrame = curFrameID - frameID_List(ff); 
    ind1 = find( pool_points_frameID == frameID_List(ff)); % only get the points within the activated frame
    ind2 = find( abs(pool_points_vr - curPoint.pvr) < vr_fixed + (nFrame - 1)*vr_float ); % get the points with valid velocity
    ind3 = find( abs(pool_points_range - curPoint.prange) < range_fixed + (nFrame - 1)*range_float ); % get the points with valid range
    ind12 = intersect(ind1, ind2);
    ind = intersect(ind12, ind3);
    neighbor_points = tempTrackList(ind); 
    
    % (2) find the line parameter & find possible points online
    ptsNum = length(neighbor_points); 
    for pp = 1:ptsNum
        % find the line paramter
        end_point = [neighbor_points(pp).prange, neighbor_points(pp).pvr, neighbor_points(pp).time]; 
        direct_vec = (end_point - start_point)/(end_point(3) - start_point(3)); 
        
        % get cluster ready
        group_cnt = group_cnt + 1; 
        stCluster(group_cnt).direct_vec = direct_vec;
        stCluster(group_cnt).pointID = fun_get_point_index_on_pointlist(neighbor_points(pp), tempTrackList);
        
        % put the points near the line into the cluster
        for gg = 1 : length(frameID_List)
            if gg == ff
                % this frame has been processed!
                continue;
            end
            
            % find the ones within the neighborhood
            time_interval = time_List(gg) - start_point(3); 
            predict_location =  direct_vec * time_interval; 
            predict_location =  predict_location + start_point;
            
            ind1 = find( pool_points_frameID == frameID_List(gg)); % only get the points within the activated frame
            ind2 = find( abs(pool_points_vr - predict_location(2)) < vr_fixed); % get the points with valid velocity
            ind3 = find( abs(pool_points_range - predict_location(1)) < range_fixed); % get the points with valid range
            ind12 = intersect(ind1, ind2);
            ind = intersect(ind12, ind3);
            prediction_neighbors = tempTrackList(ind); 
            
            % get the nearest one
            if isempty(prediction_neighbors)
                % no points assoicated, go to see the next one
                continue; 
            elseif length(prediction_neighbors) == 1
                % only one associated point
                point_index = fun_get_point_index_on_pointlist(prediction_neighbors(1), tempTrackList);
                stCluster(group_cnt).pointID = [stCluster(group_cnt).pointID, point_index]; 
            else
                % get the nearest one
                dist_all = sqrt(1 * (abs([prediction_neighbors(:).prange] - predict_location(1))/sysPara.deltaR).^2 + ...
                    0 * (abs([prediction_neighbors(:).paz] - 0)/sysPara.deltaAz).^2 +...
                    1 * (abs([prediction_neighbors(:).pvr] - predict_location(2))/sysPara.deltaV).^2); 
                
                [~, ind] = min(dist_all); % find the one with the minimum distance

                point_index = fun_get_point_index_on_pointlist(prediction_neighbors(ind), tempTrackList);
                stCluster(group_cnt).pointID = [stCluster(group_cnt).pointID, point_index];
            end
        end
    end  
end


% now many clutsers have been obtained, get rid of the reptiited ones and 
trk_count = 0;
for gg = 1:group_cnt
    % 1. check if the line has enough points
    pointIDs = sort([stCluster(gg).pointID], 'ascend'); 
    if length(pointIDs) < QUALIFY_NUM - 1 % notice here should be QUALIFY_NUM - 1, since you already have one at hand
        % too short, not enough to be a qualified, check next
        continue
    else
        % compare with existing tracks, if coincide with previous ones,
        % check the next
        flag_exist = 0;
        for tt = 1:trk_count
            exist_pointID = sort([asscPointsIndex(tt, :)], 'ascend');
            if length(exist_pointID) ~= length(pointIDs)
                continue;
            else
                temp = sum(abs(pointIDs - exist_pointID));
                if temp == 0
                    flag_exist = 1;
                    break;
                end
            end
        end
        if 1 == flag_exist
            % check the next
            continue;
        end
    end
        
    % has enough point, make it to be a candidate track!
    trk_count = trk_count + 1; 
    for pp = 1:length(pointIDs)
        % check if the following logic is correct
        candidate_tracks(trk_count).asscPointList(pp) = tempTrackList(pointIDs(pp)); 
        asscPointsIndex(trk_count, pp) = pointIDs(pp); 
    end
    candidate_tracks(trk_count).asscPointList(pp+1) = curPoint;
end