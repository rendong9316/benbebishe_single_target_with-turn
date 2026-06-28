%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description:
% sub-function for PointTrackAssociation_JNN
% construct the association matrix from input arguments. In particular, 
% vertex Track and vertex Point contains the column and row information, 
% track_to_point and point_to_track contain the association relationship. 
%
% input
% track_to_point: stores the connected point indices for each track
% point_to_track: stores the connected track indices fof each point
% vertexTrack: the track being considered (in column)
% vertexPoint: the point being considered (in row)
% 
% output:
% TP_assc_matrix: the track-point(TP) assocation matrix, whose definition
% can be found in JPDA algorithm. 
% ---------------------------------------------------------------------
% Author: Jun Geng
% Date: 2023 Jan. 8th
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function TP_assc_matrix = convert_bigraph_into_matrix(vertexTrack, vertexPoint, track_to_point, point_to_track)
col_num = length(vertexTrack) + 1; % column number
row_num = length(vertexPoint); % row number
TP_assc_matrix = zeros(row_num, col_num); % initialize the output variable

% fillout the matrix
TP_assc_matrix (:, 1) = 1; % the first column shoud be one 
for cc = 2:col_num
    % scan over tracks, and fill the matrix column by column
    trackID = vertexTrack(cc-1).listIndex; % get the position of current Track in the trackList
    asscPointNum = track_to_point(trackID).pointCnt; % scan over all the associate points 
    for pp = 1:asscPointNum
        curPointIndex = track_to_point(trackID).pointIndex(pp); % the position of point in the pointlist
        rr = get_matrix_index_by_list_index_for_vertex_point(vertexPoint, curPointIndex); % get the matrix index of current point
        % fill the matrix
        TP_assc_matrix(rr, cc) = 1; 
    end
end

end