function pointList_selected = fun_select_point_by_rd(pointList, max_range, min_range, max_vr, min_vr)

point_range = [pointList(:).prange];
point_vr = [pointList(:).pvr];

ind1 = find(point_range > min_range); 
ind2 = find(point_range <= max_range); 
ind3 = find(point_vr > min_vr); 
ind4 = find(point_vr <= max_vr); 
ind5 = intersect(ind1, ind2);
ind6 = intersect(ind3, ind4);
ind = intersect(ind5, ind6);
pointList_selected = pointList(ind); 
    
end