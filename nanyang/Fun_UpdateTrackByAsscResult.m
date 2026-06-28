%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description:
% Update the tracks information, including asscPointList, smoothPointList,
% etc., based on the track association results.
% Input:
% trackList: existing tracks
% pointList: detected point in current frame
% TPmatch_result: tracks - points matching result
% sysPara: system paramter
% ---------------------------------------------------------------------
% Author: Jun Geng
% Date: 2026-01-24
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function trackList = Fun_UpdateTrackByAsscResult(trackList, pointList, TPmatch_result, sysPara)

    trackNum = length(trackList); 
    for tt = 1:trackNum
        curTrack = trackList(tt); % get current track
        
%         % TEST POINT: debug for single track
%         if curTrack.BatchNo == 20016
%             disp('DEBUG: stop point at track update');
%         end
        
        if 0 == TPmatch_result(tt, 2)   
            % if current track has no assocation point
            
            % -------------------------------------------------------------
            % NOTICE: The calling order of the following two subfunctions 
            % is critical: the variables (totalPointCnt) updated in the 
            % first function (fun_track_quality_management_and_info_completion) 
            % are used in the second function. If the order is incorrect, 
            % it will lead to issues such as data overwriting, inconsistency 
            % between the number of predict points in the smoothed points, 
            % and other related problems. 
            % Remarked by Jun Geng @ 2026/01/24
            % ------------------------------------------------------------
            % 1. update the track with associated points
            curTrack = fun_track_quality_management_and_info_completion(curTrack, [], sysPara);
            
            % 2. also put the information in smoothed point list
            curTrack = fun_fill_smooth_list_by_predict_result(curTrack, sysPara);
            
            % add a new member @ July. 16, 2022
            curTrack.outputPointList = curTrack.smoothPointList; 
        else
            % there is an point associate with the track
            % get the associate point
            pIndex = TPmatch_result(tt, 2); 
            bestPoint = pointList(pIndex);  
            
            % -------------------------------------------------------------
            % NOTICE: The calling order of the following two subfunctions 
            % is critical: the variables (totalPointCnt) updated in the 
            % first function (fun_track_quality_management_and_info_completion) 
            % are used in the second function. If the order is incorrect, 
            % it will lead to issues such as data overwriting, inconsistency 
            % between the number of predict points in the smoothed points, 
            % and other related problems. 
            % Remarked by Jun Geng @ 2026/01/24
            % ------------------------------------------------------------
            % update track information
            curTrack = fun_track_quality_management_and_info_completion(curTrack, bestPoint, sysPara);
            
            % put the information in smoothed point list
            curTrack = fun_fill_smooth_list_by_alpha_beta_filter(curTrack, bestPoint, sysPara);
      
            % add a new member @ July. 16, 2022
            curTrack.outputPointList = curTrack.smoothPointList; 
            
        end
        trackList(tt) = curTrack; % put the proccessed track back to the list
    end

end