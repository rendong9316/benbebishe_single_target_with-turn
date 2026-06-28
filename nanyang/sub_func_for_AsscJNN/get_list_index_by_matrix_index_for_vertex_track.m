%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description:
% sub-function for PointTrackAssociation_JNN
% get the list index of the given matrix index for vertexTrack variable 
% interested track
%
% input
% vertexTrack: the collection of tracks with specific structure
% track_matrix_index: the matrix index that searching for
% 
% output:
% track_list_index: the list index that coresponding to the track_matrix_index
% ---------------------------------------------------------------------
% Author: Jun Geng
% Date: 2023 Jan. 8th
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function track_list_index = get_list_index_by_matrix_index_for_vertex_track(vertexTrack, track_matrix_index)

% matrixIndexList = [vertexTrack(:).matrixIndex]; 
% ind = find(matrixIndexList == track_matrix_index); 
% if length(ind) > 1
%     error(['error branch: matrix index should be unique. index: ', num2str(ind)]);
% else
%     track_list_index = vertexTrack(ind).listIndex;
% end

track_list_index = vertexTrack(track_matrix_index-1).listIndex; 