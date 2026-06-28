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
% 
% output:
% opt_matrix: the optimal assocation matrix. row represents the point, column
% represents the track, elements being 1 indicates the row and the column are
% assocated. 
% ---------------------------------------------------------------------
% Author: Jun Geng
% Date: 2023 Jan. 8th
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function opt_matrix = candidate_matrix_selection(candidate_matrices, vertexTrack, vertexPoint, cost_matrix, cost_fa)
[~, ~, plan_num] = size(candidate_matrices); % get the number of candidate matrix
opt_matrix = candidate_matrices(:, :, 1); % intialize the output variable
opt_cost = get_the_cost_of_match_plan(opt_matrix, vertexTrack, vertexPoint, cost_matrix, cost_fa); % find the cost of initial matrix
% scan over all possible 
for dd = 2: plan_num
    cur_matrix = candidate_matrices(:, :, dd);  % current plan
    cur_cost = get_the_cost_of_match_plan(cur_matrix, vertexTrack, vertexPoint, cost_matrix, cost_fa); % find the cost of current plan
    if cur_cost < opt_cost
        % if current plan is cheaper 
        opt_cost = cur_cost; 
        opt_matrix = cur_matrix; 
    end
end

end