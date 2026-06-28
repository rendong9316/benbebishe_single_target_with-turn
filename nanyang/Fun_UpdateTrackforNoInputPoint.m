%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description:
% When there are no input plots, update all tracks using the predicted values.
% Input: 
% trackList: existing tracks
% sysPara: current system parameter
% Output:
% trackList: updated tracks
% ------------------------------------------------------------------------
% Author: Jun Geng
% Date: 2026-01-24
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function trackList = Fun_UpdateTrackforNoInputPoint(trackList, sysPara)

trackNum = length(trackList);
for tt = 1:trackNum
    curTrack = trackList(tt);
    curTrack = predictNextStep_cv(curTrack, sysPara); % predict the track
    curTrack = fun_fill_smooth_list_by_predict_result(curTrack, sysPara); % udpate the smooth point list by predict result 
    curTrack = fun_track_quality_management_and_info_completion(curTrack, [], sysPara); % update the other fields
    trackList(tt) = curTrack; 
end

end

