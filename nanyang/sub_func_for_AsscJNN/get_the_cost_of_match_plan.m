%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description:
% sub-function for PointTrackAssociation_JNN
% return the best match plan, the best match plan has the smallest cost
% over all the match plans. 
%
% input
% candidate_matrices: all possible match plans
% cost_matrix: the matrix created by the function
% 'calculate_cost_of_point_track_pair', which contains the cost of each
% track point pair
% cost_fa: cost of false alarm
% 
% output:
% opt_matrix: the optimal assocation matrix. row represents the point, column
% represents the track, elements being 1 indicates the row and the column are
% assocated. 
% ---------------------------------------------------------------------
% Author: Jun Geng
% Date: 2023 Jan. 8th
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function plan_cost = get_the_cost_of_match_plan(match_plan, vertexTrack, vertexPoint, cost_matrix, cost_fa)

plan_cost = 0; % initialize the output varible 
[ind_r, ind_c] = find(match_plan == 1); % get the matrix index of the matched track and point
% check the input argument
if length(ind_r) ~= length(vertexPoint)
    error('match plan is wrong! every point should have an association!')
end

% scan over all '1' elements
for ii = 1:length(ind_r)
    track_matrix_index = ind_c(ii);
    if 1 == track_matrix_index
        % this is a false alarm
        pair_cost = cost_fa; 
    else
        % this is not a false alarm, then find the match pair and lookup
        % the cost
        track_list_index = get_list_index_by_matrix_index_for_vertex_track(vertexTrack, track_matrix_index);
        
        point_matrix_index = ind_r(ii);
        point_list_index = get_list_index_by_matrix_index_for_vertex_point(vertexPoint, point_matrix_index); 
        
        pair_cost = cost_matrix(point_list_index, track_list_index); 
    end

    plan_cost = plan_cost + pair_cost; 
end