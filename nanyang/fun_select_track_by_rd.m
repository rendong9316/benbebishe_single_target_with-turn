function trackList_selected = fun_select_track_by_rd(trackList, max_range, min_range, max_vr, min_vr)

trackNum = length(trackList); 
track_range = zeros(1, trackNum);
track_vr = zeros(1, trackNum);
for tt = 1:trackNum
    track_range(tt) = trackList(tt).predictRes(end).prange;
    track_vr(tt) = trackList(tt).predictRes(end).pvr;
end

ind1 = find(track_range > min_range); 
ind2 = find(track_range <= max_range); 
ind3 = find(track_vr > min_vr); 
ind4 = find(track_vr <= max_vr); 
ind5 = intersect(ind1, ind2);
ind6 = intersect(ind3, ind4);
ind = intersect(ind5, ind6);
trackList_selected = trackList(ind); 
    
end