%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description:
% get rid of the associated points and its shadows (ambiguity points) from
% the temporary detected points pools
% Input:
% pointList: all detected points
% asscIndex: the positions (or index) of the associated points in the
% pointList
% Output:
% pointList: the detected points with the removal of the assocated points
% and its shadows
% remove_index: the index of removed points in the input pointList
% ------------------------------------------------------------------------
% Jun Geng @ 2025.09.07
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [pointList, remove_index] = fun_remove_assc_pts_from_pointlist(pointList, asscIndex)

if isempty(asscIndex)
    remove_index = []; 
    return;
end

asscIndex_unique = unique(asscIndex); 
ind = find(asscIndex_unique == 0);
asscIndex_unique(ind) = [];

remove_index = []; 
for aa = 1:length(asscIndex_unique)
    ind = asscIndex_unique(aa);
    curPoint = pointList(ind); 
    
    ind_r = find( [pointList(:).Rbin] == curPoint.Rbin );
    ind_v = find( [pointList(:).Dbin] == curPoint.Dbin );
    ind_a = find( [pointList(:).Abin] == curPoint.Abin );
    ind_rv = intersect(ind_r, ind_v);
    ind_rva = intersect(ind_rv, ind_a); 
    
    remove_index = [remove_index, ind_rva]; 
end
pointList(remove_index) = []; % remove all the associated point;

end