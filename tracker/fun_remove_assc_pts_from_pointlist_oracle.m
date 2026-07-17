function [remainingPointList, pointOriginalIndex] = fun_remove_assc_pts_from_pointlist_oracle(pointList, used_det)
    pointOriginalIndex = find(~used_det);
    remainingPointList = pointList(pointOriginalIndex);
end
