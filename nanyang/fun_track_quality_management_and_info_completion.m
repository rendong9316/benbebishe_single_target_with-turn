%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description:
% This function accomplishes two tasks: (1) Complete the track information 
% except for the predicted values and smoothing values; (2) Calculate the 
% track quality and perform track quality management.
% Input: 
% curTrack: the track requiring information and quality maintenance
% bestPoint: the associated point of curTrack, empty if no association
% sysPara: system parameter
% Output: 
% curTrack: the track with completed information and quality maintenance
% ------------------------------------------------------------------------
% Author: Jun Geng
% Date: 2026-01-24
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function curTrack = fun_track_quality_management_and_info_completion(curTrack, bestPoint, sysPara)

header; 

NO_ASSOCIATION = 0;
HAS_ASSOCIATION = 1; 

if isempty(bestPoint)
    isAssociated = NO_ASSOCIATION;
    curPoint = curTrack.predictRes(end); 
else
    isAssociated = HAS_ASSOCIATION;
    curPoint = bestPoint; 
end

% update the track information as follows
curTrack.time = sysPara.datenum;

curTrack.lat = curPoint.lat;  % km 
curTrack.lon = curPoint.lon;  % km
curTrack.v_x = 0; % km
curTrack.v_y = 0; % km
curTrack.az = curPoint.paz;  % deg 
curTrack.range = curPoint.prange; %km
curTrack.vr = curPoint.pvr; % m/s
            
curTrack.TotalPointCnt = curTrack.TotalPointCnt+1;

if isAssociated == NO_ASSOCIATION
    % if has no associated point
    curTrack.travelLen = curTrack.travelLen; % no association, no update
    curTrack.AsscPointCnt = curTrack.AsscPointCnt;
    curTrack.TotalLostPointCnt = curTrack.TotalLostPointCnt + 1;
    curTrack.SuccLossPointCnt = curTrack.SuccLossPointCnt + 1;
    curTrack.updateFlag = 0; % added at 2023-04-11
    
    switch(curTrack.Type)
        case RELIABLE_TRACK
            curTrack.Quality = curTrack.Quality - POINT_LOSS_FOR_GOOD_TRACK; % reduce quality
            curTrack.Type = MAINTAIN_TRACK; % change to maintain
        case MAINTAIN_TRACK
            curTrack.Quality = curTrack.Quality - POINT_LOSS_FOR_GOOD_TRACK; % reduce quality
            if curTrack.Quality <= QUALITY_MIN % if quality is too low
                curTrack.Type = HISTORY_TRACK; % believe the track is lost 
            else % otherwise
                curTrack.Type = MAINTAIN_TRACK; % give him one more chance
            end
%         case GOOD_TRACK
%             curTrack.Quality = curTrack.Quality - POINT_LOSS_FOR_GOOD_TRACK; % reduce quality
%             if curTrack.Quality <= QUALITY_MIN % if quality is too low
%                 curTrack.Type = HISTORY_TRACK; % believe the track is lost 
%             else % otherwise
%                 curTrack.Type = MAINTAIN_TRACK; % give him one more chance
%             end
%         case INSPECTING_TRACK
%             curTrack.Quality = curTrack.Quality - POINT_LOSS_FOR_GOOD_TRACK; % reduce quality
%             if curTrack.Quality <= QUALITY_MIN % if quality is too low
%                 curTrack.Type = HISTORY_TRACK; % believe the track is lost 
%             else % otherwise
%                 curTrack.Type = MAINTAIN_TRACK; % give him one more chance
%             end
        case TEMPORARY_TRACK
            curTrack.Quality = curTrack.Quality - POINT_LOSS_FOR_NEW_TRACK; % reduce quality
            if curTrack.Quality <= QUALITY_MIN % if quality is too low
                curTrack.Type = HISTORY_TRACK; % believe the track is lost
            else % otherwise
                curTrack.Type = TEMPORARY_TRACK; % give him one more chance
            end
        otherwise
            warning(['TRACK_MAIN: ERROR BRANCH: Track Type:', num2str(curTrack.Type)] );
    end
    
else
    % if has associated point
    travel_dist = tool_calculate_distance(curPoint.lat, curPoint.lon, curTrack.asscPointList(end).lat, curTrack.asscPointList(end).lon);  % DONG_202512_v1
    
    curTrack.travelLen = curTrack.travelLen + travel_dist;
    curTrack.AsscPointCnt = curTrack.AsscPointCnt+1;
    curTrack.TotalLostPointCnt = curTrack.TotalLostPointCnt;
    curTrack.SuccLossPointCnt = 0;
    curTrack.updateFlag = 1; % added at 2023-04-11

    asscPointCnt = curTrack.AsscPointCnt;
    if asscPointCnt <= ASSC_POINT_MAX
        curTrack.asscPointList(asscPointCnt) = curPoint;  % put the associate point on the list
    else
        curTrack.asscPointList(1) = []; % remove the first one
        curTrack.asscPointList(ASSC_POINT_MAX) = curPoint;  % put the associate point on the last location
    end

    % update track information
    curTrack.Quality = curTrack.Quality + ASSOCIATION_AWARD; 
    if curTrack.Quality > QUALITY_MAX
        curTrack.Quality = QUALITY_MAX;
    end
    
    switch(curTrack.Type)
        case RELIABLE_TRACK
            curTrack.Type = RELIABLE_TRACK; % change to maintain
        case MAINTAIN_TRACK
            % if current track is a maintain track
            if curTrack.Quality >= QUALITY_RELIABLE % if it quality is larget
                curTrack.Type = RELIABLE_TRACK; % change it to reliable
            else
                curTrack.Type = MAINTAIN_TRACK; % otherwise, wait to see next performance
            end 
%         case GOOD_TRACK
%             % if current track is a maintain track
%             if curTrack.Quality >= QUALITY_RELIABLE % if it quality is larget
%                 curTrack.Type = RELIABLE_TRACK; % change it to reliable
%             else
%                 curTrack.Type = MAINTAIN_TRACK; % otherwise, wait to see next performance
%             end
%         case INSPECTING_TRACK
%             % if current track is a maintain track
%             if curTrack.Quality >= QUALITY_RELIABLE % if it quality is larget
%                 curTrack.Type = RELIABLE_TRACK; % change it to reliable
%             else
%                 curTrack.Type = MAINTAIN_TRACK; % otherwise, wait to see next performance
%             end
        case TEMPORARY_TRACK
            if curTrack.Quality >= QUALITY_RELIABLE % if it quality is larget
                curTrack.Type = RELIABLE_TRACK; % believe this is a reliable track, to make it a high priority in association
            else
                curTrack.Type = TEMPORARY_TRACK; % otherwise, wait to see next performance
            end         
        otherwise
            warning(['TRACK_MAIN: ERROR BRANCH: Track Type:', num2str(curTrack.Type)] );
    end

                
end   
end
