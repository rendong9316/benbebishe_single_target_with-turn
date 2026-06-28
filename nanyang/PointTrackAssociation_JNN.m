%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description:
% find the best points from the PointList for each track on the TrackList
% 
% input
% tracklist: current track list
% pointlist: current point list
% sysPara: system parameter
% 
% output:
% TPmatch_result: the track-point match pair. a two column matrix, the
% first column is the track ID, the second column is the point ID, if there
% has no points match the track, the second column is 0. 
% singlePointList: the index of the pointList, which marks the points that 
% has no track to associate. 
% ---------------------------------------------------------------------
% Author: Jun Geng
% Date: 2023 Jan. 5th
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [TPmatch_result, singlePointsIndex] = PointTrackAssociation_JNN(trackList, pointList, sysPara)
% initialize the outpout variable
trackNum = length(trackList);
pointNum = length(pointList); 

singlePointsIndex = 1:pointNum;
TPmatch_result = zeros(trackNum, 2); 
TPmatch_result(:, 1) = 1:trackNum; 

% input argument check
if trackNum * pointNum == 0
    % if either no track or no point
    return;
end

% initialize two important variables: track_to_point and  point_to_track
% will store the association between the trackList and pointList
for tt = 1:trackNum
    track_to_point(tt).pointCnt = 0;
    track_to_point(tt).pointIndex = [];
    track_to_point(tt).listFlag = 0;
    track_to_point(tt).procFlag = 0; 
end

for pp = 1:pointNum
    point_to_track(pp).trackCnt = 0;
    point_to_track(pp).trackIndex = [];
    point_to_track(pp).listFlag = 0;
    point_to_track(pp).procFlag = 0; 
end

% scan over all point-track pair
cost_matrix = zeros(pointNum, trackNum);
for tt = 1:trackNum
    for pp = 1:pointNum
        % scan over all the point-track pair, and 
        % 1. calculate the cost of each pair
        cost = calculate_cost_of_point_track_pair(pointList(pp), trackList(tt), sysPara); 
        cost_matrix(pp, tt) = cost; 
        
        % 2. establish the association relationship between each point and
        % each track
        bool_flag = determine_if_point_within_the_scope_of_track(pointList(pp), trackList(tt), sysPara);
        if bool_flag
            % if the point within the neighborhood of the track
            % put the point(pp) in the track(tt)
            track_to_point(tt).pointCnt = track_to_point(tt).pointCnt + 1; 
            track_to_point(tt).pointIndex = [track_to_point(tt).pointIndex, pp]; 
            % put the track(tt) in the point(pp)
            point_to_track(pp).trackCnt = point_to_track(pp).trackCnt + 1;
            point_to_track(pp).trackIndex = [point_to_track(pp).trackIndex, tt];
        end
    end
end
cost_fa = calculate_cost_of_point_track_pair([], trackList(1), sysPara); 

% for each track, set its associated point
for tt = 1:trackNum
    % if current track has been processed, jump over it
    if 1 == track_to_point(tt).procFlag
        % this track has been processed
        continue; 
    end
    
    % if current track has not been processed, then process it!
    % case 1: if there has no point associate with this track
    if 0 == track_to_point(tt).pointCnt
        % set the output
        TPmatch_result(tt, 2) = 0;
        % mark the track has been processed
        track_to_point(tt).listFlag = 1;
        track_to_point(tt).procFlag = 1;
        continue; 
    end
    
    % case 2: if point and track is one-to-one 
    if 1 == track_to_point(tt).pointCnt
        pp = track_to_point(tt).pointIndex; % get its index
        if point_to_track(pp).trackCnt == 1
            % if track-point is a one-to-one pair
            % set the output
            TPmatch_result(tt, 2) = pp;
            
            % mark the track and the point have been processed
            track_to_point(tt).listFlag = 1;
            track_to_point(tt).procFlag = 1;
            point_to_track(pp).listFlag = 1;
            point_to_track(pp).procFlag = 1;
            continue;
        end
    end
    
    % otherwise, we have to establish the associate matrix
    % build a bi-graph that containning the track(tt)
    [vertexTrack, vertexPoint, track_to_point, point_to_track] = extract_sub_bigraph(track_to_point, point_to_track, tt);  
    % convert the bi-gragh into an assocation matrix
    TP_assc_matrix = convert_bigraph_into_matrix(vertexTrack, vertexPoint, track_to_point, point_to_track); 
    % decomposite the assocation matrix into match plans£º
    candidate_matrices = mat_division(TP_assc_matrix);
    % calculate the best match plan, and choose the best plan
    opt_matrix = candidate_matrix_selection(candidate_matrices, vertexTrack, vertexPoint, cost_matrix, cost_fa); 
    
    % set the output result
    for point_matrix_index = 1:length(vertexPoint)
        % scan over the row of optimal match plan
        temp = opt_matrix(point_matrix_index, :); % get the row
        track_matrix_index = find(temp == 1); 
        if track_matrix_index == 1
            % this point is a false alarm! do nothing, check the next one!
            continue;
        else
            track_list_index = get_list_index_by_matrix_index_for_vertex_track(vertexTrack, track_matrix_index);
            point_list_index = get_list_index_by_matrix_index_for_vertex_point(vertexPoint, point_matrix_index); 
            TPmatch_result(track_list_index, 2) = point_list_index; 
        end
    end
end

% set the second outpout: singlePointList
asscPointIndexList = TPmatch_result(:, 2); 
ind = find(asscPointIndexList == 0);
asscPointIndexList(ind) = []; % remove all 0 items and get all assciated point index
% find the shadows of the associated points
[~, remove_index] = fun_remove_assc_pts_from_pointlist(pointList, asscPointIndexList); 
singlePointsIndex(remove_index) = []; 
% the following program has been replaced by the new funciton fun_remove_assc_pts_from_pointlist
% remove_index = []; 
% for aa = 1:length(asscPointIndexList)
%     ind = asscPointIndexList(aa);
%     curPoint = pointList(ind); 
%     
%     ind_r = find( [pointList(:).Rbin] == curPoint.Rbin );
%     ind_v = find( [pointList(:).Dbin] == curPoint.Dbin );
%     ind_a = find( [pointList(:).Abin] == curPoint.Abin );
%     ind_rv = intersect(ind_r, ind_v);
%     ind_rva = intersect(ind_rv, ind_a); 
%     
%     remove_index = [remove_index, ind_rva]; 
% end
% singlePointList(remove_index) = []; % remove all the associated point; 
end