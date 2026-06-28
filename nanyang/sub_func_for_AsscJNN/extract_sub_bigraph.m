%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description:
% sub-function for PointTrackAssociation_JNN
% extract the bigraph that containning trackList(trackID) from the
% connection relationship track_to_point, point_to_track
%
% input
% track_to_point: stores the connected point indices for each track
% point_to_track: stores the connected track indices fof each point
% trackID: the track that must be contained in output bigraph
% 
% output:
% vertexTrack: vertex of tracks, with a specific structure requirement
% vertexTrack: vertex of points, with a specific structure requirement
% track_to_point: the listFlag and procFlag have been updated
% point_to_track: the listFlag and procFlag have been updated
% ---------------------------------------------------------------------
% Author: Jun Geng
% Date: 2023 Jan. 8th
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [vertexTrack, vertexPoint, track_to_point, point_to_track] = extract_sub_bigraph(track_to_point, point_to_track, trackID)
%% initialization 
ll = 1; % loop variable
listLen = 1; % initialize the length of the list 
% initialize the list
list(1).type = 1; % 1 for track; 0 for point
list(1).listIndex = trackID; % the position in track_to_point 
list(1).matrixIndex = []; % the position in the association matrix 
track_to_point(trackID).listFlag = 1; % mark the flag, as it is already on the list 

%% loop, scan over all connected points and tracks
while(1)
    % for tracks...
    if 1 == list(ll).type
        % get the trackID, and mark it as processed
        curTrackID = list(ll).listIndex;
        track_to_point(curTrackID).procFlag = 1;
        % put all the points that connected with current Track in the list
        for ii = 1:track_to_point(curTrackID).pointCnt
            curPointID = track_to_point(curTrackID).pointIndex(ii); 
            if 0 == point_to_track(curPointID).listFlag
                % if current point is not on the list, put on the list
                listLen = listLen + 1; % length plus one as one more item is added to the list
                % put the point on the list
                list(listLen).type = 0; % add a point
                list(listLen).listIndex = curPointID; 
                list(listLen).matrixIndex = []; 
                % mark the point has been on the list
                point_to_track(curPointID).listFlag = 1;
            end
        end
    end
    % for points....
    if 0 == list(ll).type
        % get the pointID, and mark it as processed
        curPointID = list(ll).listIndex;
        point_to_track(curPointID).procFlag = 1;
        % put all the points that connected with current point in the list
        for ii = 1:point_to_track(curPointID).trackCnt
            curTrackID = point_to_track(curPointID).trackIndex(ii); 
            if 0 == track_to_point(curTrackID).listFlag
                % if current track is not on the list, put on the list
                listLen = listLen + 1; % length plus one as one more item is added to the list
                % put the track on the list
                list(listLen).type = 1; % add a track
                list(listLen).listIndex = curTrackID; 
                list(listLen).matrixIndex = [];
                % mark this track has on the list
                track_to_point(curTrackID).listFlag = 1;
            end
        end
    end
    
    % if all the elements on the list has been processed, quit the loop; 
    % otherwise, check the next item on the list
    if ll == listLen
        break;
    else
        ll=ll+1;
    end
end

% divide the list into two parts, namely the vertexPoint and the vertexTrack
% and assign the matrix index for each point and each track 
listType = [list(:).type]; 
ind = find(listType == 0); % find all the point index
vertexPoint = list(ind); 
pointNum = length(vertexPoint); 
for pp = 1:pointNum 
    vertexPoint(pp).matrixIndex = pp;
end
ind = find(listType == 1); % find all the track index
vertexTrack = list(ind);
trackNum = length(vertexTrack); 
for tt = 1:trackNum
    % starting from 2, as the first column will be assign for the false
    % alarm
    vertexTrack(tt).matrixIndex = tt+1;
end
