%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description:
% sub-function for PointTrackAssociation_JNN
% get the list index of the given matrix index for vertexTrack variable 
% interested track
%
% input
% vertexPoint: the collection of points with specific structure
% point_matrix_index: the matrix index that searching for
% 
% output:
% point_list_index: the list index that coresponding to the track_matrix_index
% ---------------------------------------------------------------------
% Author: Jun Geng
% Date: 2023 Jan. 8th
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function point_matrix_index = get_matrix_index_by_list_index_for_vertex_point(vertexPoint, point_list_index)

piontIndexList = [vertexPoint(:).listIndex]; 
ind = find(piontIndexList == point_list_index); 
if length(ind) > 1
    error(['error branch: list index should be unique. index: ', num2str(ind)]);
else
    point_matrix_index = vertexPoint(ind).matrixIndex;
end