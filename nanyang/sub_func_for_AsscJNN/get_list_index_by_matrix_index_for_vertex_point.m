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
function point_list_index = get_list_index_by_matrix_index_for_vertex_point(vertexPoint, point_matrix_index)

% matrixIndexList = [vertexPoint(:).matrixIndex]; 
% ind = find(matrixIndexList == point_matrix_index); 
% if length(ind) > 1
%     error(['error branch: matrix index should be unique. index: ', num2str(ind)]);
% else
%     point_list_index = vertexPoint(ind).listIndex;
% end

point_list_index = vertexPoint(point_matrix_index).listIndex; 